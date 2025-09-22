/* hal_i2c_stm32f1.c — I2C1 on PB6/PB7: blocking transfers, a DMA write path, and the
 * bit-level bus recovery the FR-0.3 sequence is built from.
 *
 * Three things about this file are worth knowing before reading it:
 *
 *   - The arbiter (firmware/app/src/i2c_arbiter.cpp) owns *policy*: who may use the bus,
 *     how long to wait, when to give up and recover. This file owns *mechanism*: it
 *     never decides to recover, it only exposes the primitives. That split is what makes
 *     the 9-clock sequence host-testable.
 *
 *   - Between recover_begin() and recover_end() the I2C peripheral is de-initialised
 *     and PB6/PB7 are ordinary open-drain GPIO, so the caller's edges appear on the
 *     pins and nothing in the peripheral can interfere. That is the contract in
 *     hal/i2c.h, and it is the reason the recovery path does not need the peripheral to
 *     be sane.
 *
 *   - transfer_wait() blocks on a semaphore given by the transfer-complete ISR. This is
 *     the one place where the HAL layer uses FreeRTOS directly, and it is deliberate:
 *     FR-0.5 exists so that a 1024-byte framebuffer flush does not pin the CPU, and a
 *     polling wait would pin it just as effectively as a byte-at-a-time transfer.
 */

#include <string.h>

#include "hal_stm32f1.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32f1xx_hal.h"

/* PB6 = SCL, PB7 = SDA: the default I2C1 mapping. The alternate mapping exists on
 * PB8/PB9 and is explicitly disabled in MspInit rather than left at whatever the reset
 * default happens to be. */
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_GPIO_PORT GPIOB
#define I2C_BOTH_PINS (I2C_SCL_PIN | I2C_SDA_PIN)

/* Both I2C and its DMA channel are configured below FreeRTOS's
 * configMAX_SYSCALL_INTERRUPT_PRIORITY (5), because both call xSemaphoreGiveFromISR().
 * On a Cortex-M3 a numerically higher number is a lower priority, so 6 is legal for
 * FreeRTOS's API and still urgent enough for a bus transfer. */
#define I2C_IRQ_PRIORITY 6U

static I2C_HandleTypeDef s_i2c1;
static DMA_HandleTypeDef s_dma_tx;
static SemaphoreHandle_t s_dma_done = NULL;

static volatile bool s_dma_busy = false;
static volatile hal_i2c_status_t s_dma_status = HAL_I2C_OK;
static bool s_in_recovery = false;

/* ---------------------------------------------------------------------------
 * Status mapping
 * ---------------------------------------------------------------------------
 */
static hal_i2c_status_t status_from_hal(HAL_StatusTypeDef status, uint32_t error_code) {
  /* A NACK is not a fault: the device simply did not answer. It must be reported as
   * such, because the arbiter treats only bus errors and timeouts as recoverable
   * (i2c_arbiter.h) — clocking the bus cannot make an absent device reply. */
  if ((error_code & HAL_I2C_ERROR_AF) != 0U) {
    return HAL_I2C_NACK;
  }
  if ((error_code & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) != 0U) {
    return HAL_I2C_BUS_ERROR;
  }
  if ((error_code & HAL_I2C_ERROR_TIMEOUT) != 0U) {
    return HAL_I2C_TIMEOUT;
  }
  if (status == HAL_TIMEOUT) {
    return HAL_I2C_TIMEOUT;
  }
  if (status == HAL_BUSY) {
    /* The peripheral refused to start because BUSY never cleared, which on a working
     * bus means a slave is holding a line: a bus fault, and therefore recoverable. */
    return HAL_I2C_BUS_ERROR;
  }
  if (status != HAL_OK) {
    return HAL_I2C_BUS_ERROR;
  }
  return HAL_I2C_OK;
}

/* The F1 HAL takes a non-const data pointer for transmit even though it only reads it.
 * The cast is done through uintptr_t so that the const is dropped explicitly at one
 * place rather than by weakening the interface's signature. */
static uint8_t *unconst(const uint8_t *p) {
  // The round trip through uintptr_t is deliberate: a direct cast from const uint8_t* to
  // uint8_t* would remove the qualifier and be caught by -Wcast-qual, and weakening
  // hal/i2c.h's signature instead would give up that warning for the whole interface.
  // The integer hop is a compile-time style matter here, not a cost: nothing about the
  // generated code differs on this target.
  // NOLINTNEXTLINE(performance-no-int-to-ptr) -- see above
  return (uint8_t *)(uintptr_t)p;
}

/* ---------------------------------------------------------------------------
 * Low-level hardware init
 * ---------------------------------------------------------------------------
 */
static void i2c_gpio_to_alternate(void) {
  GPIO_InitTypeDef init = {0};

  init.Pin = I2C_BOTH_PINS;
  init.Mode = GPIO_MODE_AF_OD; /* open drain is not optional on I2C */
  init.Pull = GPIO_NOPULL;     /* the bus pull-ups, external, set the high level */
  init.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(I2C_GPIO_PORT, &init);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != I2C1) {
    return;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* PB6/PB7 is the *default* I2C1 mapping, so the remap bit must be 0. Setting it
   * explicitly means a stray write to AFIO_MAPR elsewhere cannot silently move the bus
   * to PB8/PB9 while the pin configuration here stays on PB6/PB7. */
  __HAL_AFIO_REMAP_I2C1_DISABLE();

  i2c_gpio_to_alternate();

  /* DMA1 Channel 6 is hardwired to the I2C1_TX request on the F1 (there is no channel
   * selection register to configure, unlike later families). */
  __HAL_RCC_DMA1_CLK_ENABLE();

  s_dma_tx.Instance = DMA1_Channel6;
  s_dma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_dma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_dma_tx.Init.MemInc = DMA_MINC_ENABLE;
  s_dma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_dma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_dma_tx.Init.Mode = DMA_NORMAL;
  s_dma_tx.Init.Priority = DMA_PRIORITY_HIGH;
  (void)HAL_DMA_Init(&s_dma_tx);
  __HAL_LINKDMA(hi2c, hdmatx, s_dma_tx);

  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, I2C_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != I2C1) {
    return;
  }

  HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

  (void)HAL_DMA_DeInit(&s_dma_tx);

  __HAL_RCC_I2C1_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();

  /* The reset is the point of the de-init: a peripheral whose state machine is wedged
   * (mid-byte, waiting on a clock it will never see) is not fixed by reconfiguring its
   * registers, it is fixed by resetting it. recover_begin() relies on this. */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
}

/* Bring the peripheral back to a known-good state. Used both by recover_end() and after
 * an interrupted transfer, because a state machine left half-way through a frame fails
 * every transfer that follows it. */
static bool i2c_peripheral_restart(void) {
  (void)HAL_I2C_DeInit(&s_i2c1);

  if (HAL_I2C_Init(&s_i2c1) != HAL_OK) {
    return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------
 * Blocking transfer
 * ---------------------------------------------------------------------------
 */
static bool wait_until_ready(uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();

  while (HAL_I2C_GetState(&s_i2c1) != HAL_I2C_STATE_READY) {
    if ((HAL_GetTick() - start) >= timeout_ms) {
      return false;
    }
  }
  return true;
}

static hal_i2c_status_t i2c_transfer(void *ctx, uint16_t addr7, const uint8_t *tx,
                                     size_t tx_len, uint8_t *rx, size_t rx_len,
                                     uint32_t timeout_ms) {
  (void)ctx;

  if (s_in_recovery) {
    /* The peripheral is detached and the pins are GPIO: a transfer in this state would
     * quietly do nothing, so it is refused instead. */
    return HAL_I2C_BUS_ERROR;
  }
  if ((tx_len > 0xFFFFU) || (rx_len > 0xFFFFU)) {
    return HAL_I2C_BUS_ERROR;
  }
  if ((tx_len > 0U && tx == NULL) || (rx_len > 0U && rx == NULL)) {
    return HAL_I2C_BUS_ERROR;
  }
  if (HAL_I2C_GetState(&s_i2c1) != HAL_I2C_STATE_READY) {
    /* Another transfer (or a wedged previous one) owns the peripheral. The arbiter
     * should have made this impossible; reporting it as a bus fault is what triggers
     * the recovery that clears a wedged state. */
    return HAL_I2C_BUS_ERROR;
  }

  const uint16_t dev = (uint16_t)(addr7 << 1U);

  if (tx_len == 0U && rx_len == 0U) {
    /* Address probe: send the address and inspect the acknowledge bit, which is what the
     * bring-up bus scan needs. It must genuinely touch the bus — returning OK without
     * transmitting would report every address in range as present, which is worse than
     * having no scan at all.
     *
     * HAL_I2C_IsDeviceReady is the ST driver's own address-only check and takes care of
     * the start, the address and the ack. HAL_ERROR from it is the no-acknowledge case,
     * which is a fact about the bus rather than a fault on it, so it maps to NACK and
     * does not spend the FR-0.3 recovery budget.
     */
    const HAL_StatusTypeDef probe =
        HAL_I2C_IsDeviceReady(&s_i2c1, dev, 1U, timeout_ms);

    if (probe == HAL_OK) {
      return HAL_I2C_OK;
    }
    if (probe == HAL_TIMEOUT) {
      return HAL_I2C_TIMEOUT;
    }
    return HAL_I2C_NACK;
  }

  if (rx_len == 0U) {
    HAL_StatusTypeDef status =
        HAL_I2C_Master_Transmit(&s_i2c1, dev, unconst(tx), (uint16_t)tx_len, timeout_ms);
    return status_from_hal(status, s_i2c1.ErrorCode);
  }

  if (tx_len == 0U) {
    HAL_StatusTypeDef status =
        HAL_I2C_Master_Receive(&s_i2c1, dev, rx, (uint16_t)rx_len, timeout_ms);
    return status_from_hal(status, s_i2c1.ErrorCode);
  }

  /* Register read: write the command, then a repeated START and read, with no STOP
   * between the two — which is what "write then read" means in hal/i2c.h, and what most
   * sensors (the AHT20 included) require.
   *
   * The F1 HAL can only express that with the sequential API, which is interrupt driven:
   * the polling API always terminates a frame with a STOP. Each phase is then waited for
   * with a bounded timeout, so a lost interrupt surfaces as HAL_I2C_TIMEOUT — which the
   * arbiter turns into the FR-0.3 recovery — instead of a task that never wakes up.
   */
  if (HAL_I2C_Master_Seq_Transmit_IT(&s_i2c1, dev, unconst(tx), (uint16_t)tx_len,
                                     I2C_FIRST_FRAME) != HAL_OK) {
    return status_from_hal(HAL_ERROR, s_i2c1.ErrorCode);
  }
  if (!wait_until_ready(timeout_ms)) {
    (void)i2c_peripheral_restart();
    return HAL_I2C_TIMEOUT;
  }
  if (s_i2c1.ErrorCode != HAL_I2C_ERROR_NONE) {
    return status_from_hal(HAL_ERROR, s_i2c1.ErrorCode);
  }

  if (HAL_I2C_Master_Seq_Receive_IT(&s_i2c1, dev, rx, (uint16_t)rx_len,
                                    I2C_LAST_FRAME) != HAL_OK) {
    return status_from_hal(HAL_ERROR, s_i2c1.ErrorCode);
  }
  if (!wait_until_ready(timeout_ms)) {
    (void)i2c_peripheral_restart();
    return HAL_I2C_TIMEOUT;
  }

  return status_from_hal(HAL_OK, s_i2c1.ErrorCode);
}

/* ---------------------------------------------------------------------------
 * DMA write path (FR-0.5)
 * ---------------------------------------------------------------------------
 */
static bool i2c_transfer_start_dma(void *ctx, uint16_t addr7, const uint8_t *tx,
                                   size_t tx_len) {
  (void)ctx;

  if (s_in_recovery || tx == NULL || tx_len == 0U || tx_len > 0xFFFFU) {
    return false;
  }
  if (s_dma_busy || s_dma_done == NULL) {
    /* "Returns false if a transfer is already in flight", which is also how the caller
     * finds out that its previous transfer_wait() never happened. */
    return false;
  }
  if (HAL_I2C_GetState(&s_i2c1) != HAL_I2C_STATE_READY) {
    return false;
  }

  /* Drop any completion left over from the previous transfer before arming this one, so
   * that a stale give cannot be mistaken for this transfer finishing. */
  (void)xSemaphoreTake(s_dma_done, 0);

  s_dma_status = HAL_I2C_OK;
  s_dma_busy = true;

  if (HAL_I2C_Master_Transmit_DMA(&s_i2c1, (uint16_t)(addr7 << 1U), unconst(tx),
                                  (uint16_t)tx_len) != HAL_OK) {
    s_dma_busy = false;
    s_dma_status = status_from_hal(HAL_ERROR, s_i2c1.ErrorCode);
    return false;
  }

  return true;
}

static hal_i2c_status_t i2c_transfer_wait(void *ctx, uint32_t timeout_ms) {
  (void)ctx;

  if (!s_dma_busy || s_dma_done == NULL) {
    return HAL_I2C_BUS_ERROR;
  }

  if (xSemaphoreTake(s_dma_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    /* The transfer did not complete in time. Stop the DMA and reset the peripheral:
     * leaving a channel armed against a buffer the caller may reuse is worse than the
     * transfer failing, and the arbiter treats HAL_I2C_TIMEOUT as a bus fault, so the
     * caller's next move is the FR-0.3 recovery anyway. */
    (void)HAL_DMA_Abort(&s_dma_tx);
    s_dma_busy = false;
    (void)i2c_peripheral_restart();
    return HAL_I2C_TIMEOUT;
  }

  s_dma_busy = false;
  return s_dma_status;
}

/* ---------------------------------------------------------------------------
 * Bus levels and the recovery takeover
 * ---------------------------------------------------------------------------
 */
static bool i2c_read_lines(void *ctx, bool *scl_high, bool *sda_high) {
  (void)ctx;

  if (scl_high == NULL || sda_high == NULL) {
    return false;
  }

  /* Reading the input data register works whichever mode the pins are in: in alternate
   * function open drain the input path is still connected, and in GPIO open drain it is
   * the only way to see the line. So the caller can check for an idle bus before
   * deciding whether to take the pins over at all.
   */
  const uint32_t idr = I2C_GPIO_PORT->IDR;
  *scl_high = (idr & (uint32_t)I2C_SCL_PIN) != 0U;
  *sda_high = (idr & (uint32_t)I2C_SDA_PIN) != 0U;
  return true;
}

static bool i2c_recover_begin(void *ctx) {
  (void)ctx;

  if (s_in_recovery) {
    return true; /* idempotent: the sequence may be re-entered after a partial run */
  }

  if (s_dma_busy) {
    (void)HAL_DMA_Abort(&s_dma_tx);
    s_dma_busy = false;
    s_dma_status = HAL_I2C_BUS_ERROR;
  }

  /* Detach and reset the peripheral first: while it is enabled it drives PB6/PB7, and
   * the whole point of the recovery is that nothing but the caller drives them. */
  (void)HAL_I2C_DeInit(&s_i2c1);

  /* Both lines released high *before* they become outputs, so switching the pins to
   * output mode cannot pull a line low even for the few nanoseconds between the mode
   * write and the first data write. The pull-ups define the high level from here on;
   * the F1 cannot provide an internal pull-up in open-drain output mode, which is why
   * the board needs external ones (see hal_stm32f1.h). */
  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_BOTH_PINS, GPIO_PIN_SET);

  GPIO_InitTypeDef init = {0};
  init.Pin = I2C_BOTH_PINS;
  init.Mode = GPIO_MODE_OUTPUT_OD;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_GPIO_PORT, &init);

  s_in_recovery = true;
  return true;
}

static bool i2c_line_write(uint16_t pin, bool level) {
  if (!s_in_recovery) {
    /* Refused rather than silently accepted: a "recovery" edge emitted while the
     * peripheral still owns the pins would go nowhere, and a caller that believes it
     * clocked the bus out of a stuck state would be reporting a false success. */
    return false;
  }
  HAL_GPIO_WritePin(I2C_GPIO_PORT, pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return true;
}

static bool i2c_scl_write(void *ctx, bool level) {
  (void)ctx;
  return i2c_line_write(I2C_SCL_PIN, level);
}

static bool i2c_sda_write(void *ctx, bool level) {
  (void)ctx;
  return i2c_line_write(I2C_SDA_PIN, level);
}

static bool i2c_sda_read(void *ctx, bool *level) {
  (void)ctx;

  if (level == NULL) {
    return false;
  }
  *level = (I2C_GPIO_PORT->IDR & (uint32_t)I2C_SDA_PIN) != 0U;
  return true;
}

static bool i2c_scl_read(void *ctx, bool *level) {
  (void)ctx;

  if (level == NULL) {
    return false;
  }
  *level = (I2C_GPIO_PORT->IDR & (uint32_t)I2C_SCL_PIN) != 0U;
  return true;
}

static void i2c_delay_us(void *ctx, uint32_t us) {
  (void)ctx;

  /* Real microseconds from the DWT cycle counter. HAL_Delay() would round every edge to
   * a millisecond, and the ~5 us half-period the recovery sequence is specified with
   * cannot be expressed in milliseconds at all. */
  hal_stm32f1_delay_us(us);
}

static bool i2c_recover_end(void *ctx) {
  (void)ctx;

  if (!s_in_recovery) {
    return true;
  }

  /* Release both lines, then hand them back to the peripheral. */
  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_BOTH_PINS, GPIO_PIN_SET);
  s_in_recovery = false;

  /* Drop any completion from a transfer that was in flight when the bus was taken over,
   * so the next transfer_start_dma() does not consume it as its own. */
  if (s_dma_done != NULL) {
    (void)xSemaphoreTake(s_dma_done, 0);
  }

  return i2c_peripheral_restart();
}

/* ---------------------------------------------------------------------------
 * Interrupt handlers and HAL callbacks
 * ---------------------------------------------------------------------------
 */
void I2C1_EV_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&s_i2c1);
}

void I2C1_ER_IRQHandler(void) {
  HAL_I2C_ER_IRQHandler(&s_i2c1);
}

void DMA1_Channel6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&s_dma_tx);
}

/* The completion callbacks are shared by the interrupt-driven sequential path and the
 * DMA path (the HAL routes both through these function pointers), so each one checks
 * s_dma_busy before signalling: only a DMA transfer has a waiter. */
static void dma_complete_from_isr(hal_i2c_status_t status) {
  if (!s_dma_busy) {
    return;
  }

  s_dma_status = status;

  BaseType_t higher_priority_woken = pdFALSE;
  if (s_dma_done != NULL) {
    (void)xSemaphoreGiveFromISR(s_dma_done, &higher_priority_woken);
  }
  portYIELD_FROM_ISR(higher_priority_woken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != I2C1) {
    return;
  }
  dma_complete_from_isr(status_from_hal(HAL_OK, hi2c->ErrorCode));
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance != I2C1) {
    return;
  }
  /* Signal the waiter even on failure, otherwise transfer_wait() would sit out its
   * whole timeout before reporting what the peripheral already knew. */
  dma_complete_from_isr(status_from_hal(HAL_ERROR, hi2c->ErrorCode));
}

/* ---------------------------------------------------------------------------
 * Interface instance
 * ---------------------------------------------------------------------------
 */
static hal_i2c_iface_t s_i2c = {
    .ctx = NULL,
    .transfer = i2c_transfer,
    .transfer_start_dma = i2c_transfer_start_dma,
    .transfer_wait = i2c_transfer_wait,
    .read_lines = i2c_read_lines,
    .recover_begin = i2c_recover_begin,
    .scl_write = i2c_scl_write,
    .sda_write = i2c_sda_write,
    .sda_read = i2c_sda_read,
    .scl_read = i2c_scl_read,
    .delay_us = i2c_delay_us,
    .recover_end = i2c_recover_end,
};

hal_i2c_iface_t *hal_i2c_stm32f1(void) {
  return &s_i2c;
}

void hal_i2c_stm32f1_init(void) {
  /* Made once, here, and never freed: the arbitrating mutex in the application layer
   * already serialises users of this one, so a second pair of hands on it cannot
   * happen while the scheduler is running. */
  if (s_dma_done == NULL) {
    s_dma_done = xSemaphoreCreateBinary();
  }

  s_i2c1.Instance = I2C1;
  s_i2c1.Init.ClockSpeed = 400000U; /* 400 kHz fast mode; PCLK1 is 36 MHz */
  s_i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  s_i2c1.Init.OwnAddress1 = 0U; /* master only */
  s_i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  s_i2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  s_i2c1.Init.OwnAddress2 = 0U;
  s_i2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  s_i2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; /* allow a slow slave to stretch */

  (void)HAL_I2C_Init(&s_i2c1);
}
