/* hal_stm32f1.c — board bring-up: clocks, the HAL timebase, the cycle counter, and the
 * microsecond delay the I2C recovery path depends on.
 *
 * The hardware assignment this implements is documented in hal_stm32f1.h.
 */

#include "hal_stm32f1.h"

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * HAL timebase: TIM4 at 1 kHz
 * ---------------------------------------------------------------------------
 * SysTick belongs to FreeRTOS — the Cortex-M3 port installs its own tick handler in
 * that vector — so the HAL cannot use it. The HAL needs *some* timebase that advances,
 * because every HAL_..._Timeout() call (and HAL_Delay) waits on HAL_GetTick(): with
 * uwTick frozen, HAL_UART_Transmit() or HAL_I2C_Master_Transmit() on a stuck bus would
 * never time out, and a task that never returns never releases the I2C arbiter mutex.
 * That is the failure mode FR-0.3's recovery exists to survive, so the timebase is not
 * optional.
 *
 * TIM4 (a general-purpose APB1 timer) rather than TIM2/TIM3, which Phase 1 wants for
 * the timer-triggered ADC. This override replaces the weak default in stm32f1xx_hal.c,
 * which would otherwise program SysTick and fight FreeRTOS for it.
 */
static TIM_HandleTypeDef s_hal_tick_tim;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  RCC_ClkInitTypeDef clkconfig;
  uint32_t pflash_latency = 0U;
  uint32_t tim_clock_hz;

  if (TickPriority >= (1UL << __NVIC_PRIO_BITS)) {
    return HAL_ERROR;
  }

  __HAL_RCC_TIM4_CLK_ENABLE();

  /* Read the *current* clock tree rather than assuming one: HAL_RCC_ClockConfig()
   * calls HAL_InitTick() again after it changes SYSCLK, and the F1 rule that the APB
   * timer clock is doubled when the APB prescaler is not 1 has to be applied each time.
   */
  HAL_RCC_GetClockConfig(&clkconfig, &pflash_latency);
  tim_clock_hz = HAL_RCC_GetPCLK1Freq();
  if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1) {
    tim_clock_hz *= 2U;
  }

  s_hal_tick_tim.Instance = TIM4;
  s_hal_tick_tim.Init.Prescaler = (tim_clock_hz / 1000000U) - 1U; /* 1 MHz counter */
  s_hal_tick_tim.Init.Period = (1000000U / 1000U) - 1U;           /* 1 kHz update */
  s_hal_tick_tim.Init.ClockDivision = 0U;
  s_hal_tick_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_hal_tick_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&s_hal_tick_tim) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_TIM_Base_Start_IT(&s_hal_tick_tim) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  uwTickPrio = TickPriority;
  uwTickFreq = HAL_TICK_FREQ_1KHZ;
  return HAL_OK;
}

/* The update ISR is written out rather than handed to HAL_TIM_IRQHandler().
 *
 * HAL_TIM_IRQHandler() dispatches to every TIM callback kind (output compare, input
 * capture, PWM, break) and therefore keeps that whole dispatcher in the image for a
 * timer used only as a tick. Four lines here cost nothing and say exactly what the
 * interrupt does: increment the HAL tick.
 *
 * It calls no FreeRTOS API, so its priority is unconstrained; it is set to 6 in
 * stm32f1xx_hal_conf.h so that it stays out of the kernel's critical sections.
 */
void TIM4_IRQHandler(void) {
  if (__HAL_TIM_GET_FLAG(&s_hal_tick_tim, TIM_FLAG_UPDATE) != RESET) {
    if (__HAL_TIM_GET_IT_SOURCE(&s_hal_tick_tim, TIM_IT_UPDATE) != RESET) {
      __HAL_TIM_CLEAR_IT(&s_hal_tick_tim, TIM_IT_UPDATE);
      HAL_IncTick();
    }
  }
}

/* ---------------------------------------------------------------------------
 * Clock tree: 8 MHz HSE -> PLL x9 -> 72 MHz
 * ---------------------------------------------------------------------------
 */
static bool s_hse_ok = false;

static bool clock_config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1; /* 8 MHz into the PLL, not divided */
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9; /* 8 MHz x 9 = 72 MHz */

  s_hse_ok = (HAL_RCC_OscConfig(&osc) == HAL_OK);

  if (!s_hse_ok) {
    /* No crystal (wrong board, a dead resonator, or bring-up step 6 not yet passed).
     * Falling back to HSI/2 x 16 = 64 MHz keeps the node bootable and able to *say* so
     * over the UART, which is worth far more during bring-up than a silent hang.
     * hal_stm32f1_hse_ok() and the banner report the real SystemCoreClock, so this can
     * never be mistaken for a working 72 MHz setup.
     */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSEState = RCC_HSE_OFF;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16; /* 4 MHz x 16 = 64 MHz */

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
      return false;
    }
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                  RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* HCLK  = 72 MHz */
  clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 36 MHz (the F1 APB1 limit) */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 72 MHz */

  /* Two flash wait states and the prefetch buffer: the F1 part needs latency 2 above
   * 48 MHz, and HAL_RCC_ClockConfig() does not touch FLASH->ACR at all, so the prefetch
   * bit is asserted here as well as in HAL_Init(). HAL_RCC_ClockConfig() also updates
   * SystemCoreClock, which is what the banner and FreeRTOS's SysTick reload read.
   */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
    return false;
  }
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

  return true;
}

bool hal_stm32f1_hse_ok(void) {
  return s_hse_ok;
}

/* ---------------------------------------------------------------------------
 * DWT cycle counter and the microsecond delay
 * ---------------------------------------------------------------------------
 */
static bool s_dwt_ok = false;
static uint32_t s_fallback_loops_per_us = 0U;

static void dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Verify it actually counts. Some clone parts do not implement the DWT cycle
   * counter, and a delay loop that waits on a frozen counter never returns — which
   * would turn the FR-0.3 recovery path into a hang instead of a recovery.
   */
  const uint32_t start = DWT->CYCCNT;
  uint32_t spins = 0U;
  while ((DWT->CYCCNT == start) && (spins < 100000U)) {
    ++spins;
  }
  s_dwt_ok = (DWT->CYCCNT != start);
}

/* Fallback calibration. No assumption is made about the core clock, the flash wait
 * states or the compiler: the loop that delay_us() uses is timed against the 1 ms HAL
 * tick, so whatever it costs per iteration is measured rather than guessed.
 *
 * If the counter does not run the result is meaningless — but it errs *long*, and a
 * long recovery half-period still releases the bus while a short one would not.
 */
static void delay_calibrate_fallback(void) {
  const uint32_t t0 = HAL_GetTick();
  uint32_t iterations = 0U;

  do {
    volatile uint32_t chunk = 1000U;
    while (chunk > 0U) {
      --chunk;
    }
    iterations += 1000U;
    /* The tick is read once per chunk so that the read itself does not dominate the
     * measurement. */
  } while (HAL_GetTick() == t0);

  s_fallback_loops_per_us = iterations / 1000U;
  if (s_fallback_loops_per_us == 0U) {
    s_fallback_loops_per_us = 1U;
  }
}

void hal_stm32f1_delay_us(uint32_t us) {
  if (us == 0U) {
    return;
  }

  if (s_dwt_ok) {
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t remaining = us;

    /* Chunked so that `step * cycles_per_us` cannot overflow at long delays; the
     * unsigned subtraction of CYCCNT handles its own wrap. */
    while (remaining > 0U) {
      const uint32_t step = (remaining > 10000U) ? 10000U : remaining;
      const uint32_t ticks = step * cycles_per_us;
      const uint32_t start = DWT->CYCCNT;
      while ((DWT->CYCCNT - start) < ticks) {
      }
      remaining -= step;
    }
    return;
  }

  if (s_fallback_loops_per_us == 0U) {
    delay_calibrate_fallback();
  }
  for (volatile uint32_t spin = us * s_fallback_loops_per_us; spin > 0U; --spin) {
  }
}

/* ---------------------------------------------------------------------------
 * Low-level hardware init hook, called by HAL_Init()
 * ---------------------------------------------------------------------------
 * Only the always-needed clocks: the alternate-function IO block (for the I2C1 remap
 * bit) and the power/backup domain (for the RST/clock enables and the boot counter).
 * Peripheral and GPIO clocks are enabled in the module that owns them.
 */
void HAL_MspInit(void) {
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
}

/* ---------------------------------------------------------------------------
 * Bring-up sequence
 * ---------------------------------------------------------------------------
 */
void hal_stm32f1_init(void) {
  /* 1. Reset cause first, and before anything else touches RCC. The flags in RCC_CSR
   *    are cleared by reading them (the implementation samples and clears them once),
   *    and NFR-0.7's reset counter is only meaningful if that happens before the clock
   *    setup could conceivably disturb anything. */
  hal_system_stm32f1_sample_reset_cause();

  /* 2. HAL_Init(): prefetch, NVIC priority grouping 4 (all bits preemption, which is
   *    what FreeRTOS requires), the TIM4 timebase, and HAL_MspInit() above. */
  (void)HAL_Init();

  /* 3. 72 MHz from the 8 MHz crystal. */
  (void)clock_config();

  /* 4. Cycle counter: delay_us() for the I2C recovery edges, and FreeRTOS run-time
   *    statistics for NFR-1.10. */
  dwt_init();

  /* 5. Boot accounting in the backup domain: survives a watchdog or software reset,
   *    cleared only by removing power (or a backup-domain reset). */
  hal_system_stm32f1_count_boot();

  /* 6. Pins and peripherals. The watchdog is deliberately not started here: FR-0.4
   *    requires the supervisor task to own that decision. */
  hal_stm32f1_gpio_init();
  hal_i2c_stm32f1_init();
  hal_uart_stm32f1_init();
}
