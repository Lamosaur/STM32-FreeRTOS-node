/* hal_watchdog_stm32f1.c — IWDG, clocked from the independent LSI oscillator.
 *
 * "Independent" is the requirement, not a detail (FR-0.4): the IWDG's timebase is the
 * LSI, not the PLL, so a failed HSE, a PLL that never locks, or a wedged SysTick cannot
 * keep the watchdog quiet. That is also why the HAL timebase (TIM4, off the PLL) is
 * irrelevant to it and why nothing here depends on the system clock at all.
 *
 * The decision of *whether* to feed lives in the supervisor task
 * (firmware/app/src/supervisor.cpp in the node application), never in a timer callback
 * and never in an interrupt: this file only starts and reloads.
 */

#include "hal_stm32f1.h"

#include "stm32f1xx_hal.h"

/* The IWDG prescaler options, in the order the IWDG_PRESCALER_* values are defined. */
static const uint16_t kDividers[7] = {4U, 8U, 16U, 32U, 64U, 128U, 256U};
static const uint32_t kPrescalers[7] = {IWDG_PRESCALER_4,   IWDG_PRESCALER_8,
                                       IWDG_PRESCALER_16,  IWDG_PRESCALER_32,
                                       IWDG_PRESCALER_64,  IWDG_PRESCALER_128,
                                       IWDG_PRESCALER_256};

/* The reload register is 12 bits, so the longest timeout is 256 * 4096 / 40 kHz = 26.2 s
 * and the shortest is 4 / 40 kHz = 0.1 ms. */
#define IWDG_RELOAD_MAX 0xFFFU

/* Datasheet typical. The part's real LSI is anywhere from ~30 kHz to ~60 kHz, so the
 * achieved timeout has the same spread; this is why the requested timeout is mapped to
 * the *nearest* achievable pair and why the supervisor's feed period (250 ms,
 * app/board.h) stays an order of magnitude inside the requested 1000 ms.
 */
#define LSI_NOMINAL_HZ 40000U

static IWDG_HandleTypeDef s_iwdg;
static bool s_started = false;
static uint32_t s_programmed_ms = 0U;

static bool wd_start(void *ctx, uint32_t timeout_ms) {
  (void)ctx;

  if (timeout_ms == 0U) {
    return false;
  }
  if (s_started) {
    /* Once started the IWDG cannot be stopped — not by software, not by a reset of the
     * peripheral (there is no way to reset it short of a full MCU reset, and the
     * hardware watchdog option bit is what makes it start on its own). Re-starting is
     * therefore refused rather than silently pretending to re-arm it with a new
     * timeout: the first timeout is the one in force for the rest of this boot. */
    return false;
  }

  /* Enable the LSI and wait for it. Reported as ready or not, the IWDG cannot be made
   * dependable by assuming the oscillator came up. */
  RCC->CSR |= RCC_CSR_LSION;
  uint32_t spins = 0U;
  while (((RCC->CSR & RCC_CSR_LSIRDY) == 0U) && (spins < 100000U)) {
    ++spins;
  }

  /* Choose the prescaler/reload pair whose period is closest to the request. Both
   * limits are hard: the prescaler is one of seven values and the reload is 12 bits, so
   * most timeouts are not exactly achievable and the nearest one must be picked
   * deliberately rather than by rounding the divider and hoping.
   */
  int32_t best = -1;
  uint32_t best_reload = 0U;
  uint32_t best_error = 0xFFFFFFFFU;

  for (uint32_t i = 0U; i < 7U; ++i) {
    /* counts = timeout_ms * LSI / (1000 * divider); in 64-bit arithmetic because the
     * numerator alone overflows 32 bits for long timeouts. */
    const uint64_t counts =
        ((uint64_t)timeout_ms * (uint64_t)LSI_NOMINAL_HZ) / ((uint64_t)kDividers[i] * 1000ULL);

    if (counts == 0ULL || counts > (uint64_t)(IWDG_RELOAD_MAX + 1U)) {
      continue; /* this divider cannot express the request */
    }

    const uint32_t reload = (uint32_t)(counts - 1ULL);
    const uint32_t achieved_ms =
        (uint32_t)(((uint64_t)kDividers[i] * (uint64_t)(reload + 1U) * 1000ULL) /
                   (uint64_t)LSI_NOMINAL_HZ);

    const uint32_t error_ms = (achieved_ms > timeout_ms) ? (achieved_ms - timeout_ms)
                                                         : (timeout_ms - achieved_ms);
    if (error_ms < best_error) {
      best_error = error_ms;
      best = (int32_t)i;
      best_reload = reload;
    }
  }

  if (best < 0) {
    return false; /* outside what the hardware can produce */
  }

  s_iwdg.Instance = IWDG;
  s_iwdg.Init.Prescaler = kPrescalers[(uint32_t)best];
  s_iwdg.Init.Reload = best_reload;
  if (HAL_IWDG_Init(&s_iwdg) != HAL_OK) {
    return false;
  }

  s_programmed_ms = (uint32_t)(((uint64_t)kDividers[(uint32_t)best] *
                                (uint64_t)(best_reload + 1U) * 1000ULL) /
                               (uint64_t)LSI_NOMINAL_HZ);
  s_started = true;
  return true;
}

static void wd_feed(void *ctx) {
  (void)ctx;

  /* Refreshed unconditionally rather than behind a "started" flag: writing 0xAAAA to KR
   * before the watchdog is running is ignored by the hardware, whereas a software flag
   * that disagreed with the hardware would either stop feeding a running watchdog or
   * claim to feed one that is not running. The hardware is the source of truth. */
  (void)HAL_IWDG_Refresh(&s_iwdg);
}

static hal_watchdog_iface_t s_watchdog = {
    .ctx = NULL,
    .start = wd_start,
    .feed = wd_feed,
};

hal_watchdog_iface_t *hal_watchdog_stm32f1(void) {
  return &s_watchdog;
}

/* Diagnostics for the bring-up image: what the requested timeout was mapped to. */
uint32_t hal_watchdog_stm32f1_programmed_ms(void) {
  return s_programmed_ms;
}
