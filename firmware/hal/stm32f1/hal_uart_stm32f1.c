/* hal_uart_stm32f1.c — USART1 on PA9/PA10 at 115200 8N1.
 *
 * The receive path is interrupt driven into a fixed 128-byte ring, so rx_available()
 * reports a real count and read() never has to guess. A byte-at-a-time polling driver
 * would make rx_available() a lie ("1 if a byte happens to be in the data register"),
 * and Phase 2's framed update protocol needs a buffer it can drain either way.
 *
 * read() waits on a binary semaphore that USART1_IRQHandler gives. There is one
 * deliberate subtlety in that ISR: it does not call a ...FromISR() API before the
 * scheduler is running, because port.c only computes its interrupt-priority validation
 * inside xPortStartScheduler() and a ...FromISR() call before then trips a false
 * configASSERT. The ring keeps filling either way.
 */

#include "hal_stm32f1.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32f1xx_hal.h"

/* Power of two, so the producer/consumer arithmetic is a mask rather than a modulo —
 * the ISR runs at up to ~11.5 kHz at 115200 baud and should stay short. */
#define UART_RX_RING 128U
#define UART_RX_MASK (UART_RX_RING - 1U)

/* Below FreeRTOS's configMAX_SYSCALL_INTERRUPT_PRIORITY (5) because the ISR signals the
 * read semaphore; see FreeRTOSConfig.h. */
#define UART_IRQ_PRIORITY 6U

static UART_HandleTypeDef s_uart1;
static SemaphoreHandle_t s_rx_sem = NULL;

/* Single producer (the ISR) writes head, single consumer (a task) writes tail. Both are
 * 16-bit aligned loads and stores, so no critical section is needed to keep them
 * consistent with each other. */
static volatile uint8_t s_rx_ring[UART_RX_RING];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint32_t s_rx_overruns = 0U;

static uint16_t uart_rx_count(void) {
  return (uint16_t)((s_rx_head - s_rx_tail) & UART_RX_MASK);
}

/* ---------------------------------------------------------------------------
 * Low-level hardware init
 * ---------------------------------------------------------------------------
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) {
    return;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef init = {0};

  /* PA9 = TX, alternate function push-pull. */
  init.Pin = GPIO_PIN_9;
  init.Mode = GPIO_MODE_AF_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &init);

  /* PA10 = RX. The pull-up matters: a UART line idles high, and a disconnected RX pin
   * left floating produces a continuous stream of framing errors that look exactly like
   * a broken link during bring-up. */
  init.Pin = GPIO_PIN_10;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &init);

  HAL_NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) {
    return;
  }

  __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  __HAL_RCC_USART1_CLK_DISABLE();
}

/* ---------------------------------------------------------------------------
 * Interrupt handler
 * ---------------------------------------------------------------------------
 */
void USART1_IRQHandler(void) {
  const uint32_t sr = USART1->SR;

  /* Reading SR and then DR clears RXNE and, with it, ORE/FE/NE/PE. All of them are read
   * in one go: a driver that handles RXNE alone loses the overrun flag and then reports
   * a clean link that is silently dropping bytes. */
  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
    const uint8_t byte = (uint8_t)(USART1->DR & 0xFFU);

    if ((sr & USART_SR_RXNE) != 0U) {
      const uint16_t next = (uint16_t)((s_rx_head + 1U) & UART_RX_MASK);
      if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next;

        /* Wake a waiting reader. Skipped before the scheduler exists, see the file
         * comment: FreeRTOS's own interrupt-priority validation is only armed by
         * xPortStartScheduler(). */
        if (s_rx_sem != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
          BaseType_t higher_priority_woken = pdFALSE;
          (void)xSemaphoreGiveFromISR(s_rx_sem, &higher_priority_woken);
          portYIELD_FROM_ISR(higher_priority_woken);
        }
      } else {
        /* Ring full: drop the newest byte and count it. Dropping the oldest would
         * corrupt a frame that is already being reassembled. */
        ++s_rx_overruns;
      }
    }
  }
}

/* ---------------------------------------------------------------------------
 * Interface implementation
 * ---------------------------------------------------------------------------
 */
static bool uart_init(void *ctx, uint32_t baud) {
  (void)ctx;

  if (baud < 100U) {
    return false;
  }

  s_uart1.Instance = USART1;
  s_uart1.Init.BaudRate = baud;
  s_uart1.Init.WordLength = UART_WORDLENGTH_8B;
  s_uart1.Init.StopBits = UART_STOPBITS_1;
  s_uart1.Init.Parity = UART_PARITY_NONE;
  s_uart1.Init.Mode = UART_MODE_TX_RX;
  s_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  s_uart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&s_uart1) != HAL_OK) {
    return false;
  }

  /* Check what the divider actually produced instead of trusting the request: on the F1
   * the baud rate is PCLK2/BRR with integer rounding, so a rate that is not reachable
   * from 72 MHz has to be reported rather than silently approximated. The interface's
   * contract is to return false in that case, and 2% is the tolerance an 8N1 frame
   * tolerates (roughly 0.2 bit of drift across the ten bit times). */
  const uint32_t brr = USART1->BRR & 0xFFFFU;
  if (brr == 0U) {
    return false;
  }

  const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  const uint32_t actual = pclk2 / brr;
  const uint32_t diff = (actual > baud) ? (actual - baud) : (baud - actual);

  return (diff * 100U) <= (baud * 2U);
}

static size_t uart_write(void *ctx, const uint8_t *data, size_t len, uint32_t timeout_ms) {
  (void)ctx;

  if (data == NULL || len == 0U) {
    return 0U;
  }

  size_t sent = 0U;

  /* HAL_UART_Transmit() takes a 16-bit length, so a longer buffer goes out in chunks. */
  while (sent < len) {
    size_t chunk = len - sent;
    if (chunk > 0xFFFFU) {
      chunk = 0xFFFFU;
    }

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(&s_uart1, data + sent, (uint16_t)chunk, timeout_ms);

    if (status == HAL_OK) {
      sent += chunk;
      continue;
    }

    if (status == HAL_TIMEOUT) {
      /* The HAL does not report a byte count on timeout; TxXferCount is what is left of
       * the chunk, which is the count the interface asks for. */
      sent += chunk - (size_t)s_uart1.TxXferCount;
    }

    /* Abort so the peripheral is left READY: a half-finished transmit that is not
     * aborted makes every later write return HAL_BUSY. */
    (void)HAL_UART_AbortTransmit(&s_uart1);
    break;
  }

  return sent;
}

static size_t uart_read(void *ctx, uint8_t *data, size_t len, uint32_t timeout_ms) {
  (void)ctx;

  if (data == NULL || len == 0U) {
    return 0U;
  }

  if (uart_rx_count() == 0U) {
    /* Nothing buffered yet, so wait for the ISR to change that. Before the scheduler is
     * running nothing can block on a tick that is not ticking, so the wait degrades to
     * "return whatever is there", which is the honest answer at that point. */
    if (s_rx_sem == NULL || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
      return 0U;
    }
    if (xSemaphoreTake(s_rx_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
      /* A timeout is the idle case, not an error (hal/uart.h). */
      return 0U;
    }
  }

  /* Drain everything buffered, up to len, and return immediately: the contract is "as
   * soon as any byte arrives", and a caller that wants to wait for more calls again. */
  const uint16_t head = s_rx_head;
  size_t count = 0U;

  while ((s_rx_tail != head) && (count < len)) {
    data[count] = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & UART_RX_MASK);
    ++count;
  }

  /* If a byte arrived between the semaphore give and this drain, a second give may be
   * pending; leaving it would make the *next* read return 0 immediately instead of
   * waiting out its timeout. */
  (void)xSemaphoreTake(s_rx_sem, 0);

  return count;
}

static size_t uart_rx_available(void *ctx) {
  (void)ctx;
  return (size_t)uart_rx_count();
}

static hal_uart_iface_t s_uart = {
    .ctx = NULL,
    .init = uart_init,
    .write = uart_write,
    .read = uart_read,
    .rx_available = uart_rx_available,
};

hal_uart_iface_t *hal_uart_stm32f1(void) {
  return &s_uart;
}

void hal_uart_stm32f1_init(void) {
  if (s_rx_sem == NULL) {
    s_rx_sem = xSemaphoreCreateBinary();
  }

  /* 115200 so that the banner is readable immediately after hal_stm32f1_init(); the
   * interface's init() can re-programme it later. The result is deliberately ignored
   * here: the caller that cares about the rate gets it from init(). */
  (void)uart_init(NULL, 115200U);
}

/* Diagnostics used by the bring-up image. Not part of the frozen interface. */
uint32_t hal_uart_stm32f1_overruns(void) {
  return s_rx_overruns;
}
