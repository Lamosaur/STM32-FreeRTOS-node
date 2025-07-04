/* hal_stm32f1.h — accessors for the STM32F103C8T6 implementation of the HAL interfaces.
 *
 * hal/ has no application knowledge: nothing here includes app/board.h, and nothing in
 * app/ includes a device header. The two sides meet at the opaque pin ids below, which
 * are the same numbers the application names (app/board.h kPin*), and at the interfaces
 * in the hal/ headers. main_target.cpp static_asserts that the two sets of numbers
 * still agree.
 *
 * ---------------------------------------------------------------------------
 * Hardware assignment (design-spec.md §5.2, FR-0.1..FR-0.6)
 * ---------------------------------------------------------------------------
 *
 *   MCU          STM32F103C8T6, medium density, Cortex-M3, no FPU.
 *                64 KB flash, 20 KB SRAM.
 *
 *   Clock        8 MHz HSE crystal -> PLL x9 -> SYSCLK 72 MHz.
 *                AHB 72 MHz, APB1 36 MHz (HCLK/2), APB2 72 MHz.
 *                Flash latency 2 with the prefetch buffer enabled — the F1 HAL's
 *                HAL_RCC_ClockConfig() does *not* touch FLASH->ACR, so the prefetch bit
 *                is set explicitly in hal_stm32f1.c.
 *                SystemCoreClock ends at 72000000. If the crystal never starts, the
 *                clock falls back to HSI/2 x16 = 64 MHz so that a board with a dead
 *                crystal still boots and can say so over the UART (the banner prints
 *                the real SystemCoreClock and a warning; see hal_stm32f1_hse_ok()).
 *
 *   HAL timebase TIM4 update interrupt at 1 kHz feeding HAL_IncTick(). SysTick belongs
 *                to FreeRTOS (the port's tick), so the HAL cannot have it; without a
 *                timebase that advances, every HAL timeout would hang forever, which is
 *                precisely the failure the bus recovery in FR-0.3 has to survive.
 *                TIM1/TIM2/TIM3 are left unclaimed for Phase 1's ADC trigger and PWM.
 *                Interrupt priority 6: numerically below FreeRTOS's
 *                configMAX_SYSCALL_INTERRUPT_PRIORITY (5) so it never preempts a
 *                kernel critical section, and it calls no FreeRTOS API.
 *
 *   DWT          Cycle counter, enabled in hal_stm32f1_init(). Two users: the microsecond
 *                delay (delay_us for the I2C recovery edges), and FreeRTOS run-time
 *                statistics for NFR-1.10.
 *
 *   I2C1         PB6 = SCL, PB7 = SDA, alternate function open drain, 400 kHz, 7-bit
 *                addressing, no remap (PB6/PB7 is the default mapping; the alternate
 *                PB8/PB9 mapping is explicitly disabled rather than left to the reset
 *                default). DMA1 Channel 6 = I2C1_TX for the framebuffer path (FR-0.5).
 *                Interrupts: I2C1_EV / I2C1_ER at priority 6, DMA1_Channel6 at priority
 *                6, because both the DMA-complete and error paths call
 *                xSemaphoreGiveFromISR() for transfer_wait() to consume.
 *                Requires external pull-ups on SCL and SDA (4.7 k to 3V3): the recovery
 *                path releases both lines and lets the pull-ups define the high level,
 *                and the F1 GPIO cannot provide an internal pull-up in open-drain
 *                output mode. Without them a recovery reads SDA as stuck low.
 *
 *   USART1       PA9 = TX (alternate function push-pull), PA10 = RX (input with a
 *                pull-up: a UART line idles high, and a floating RX pin turns a
 *                disconnected cable into a flood of framing errors), 115200 8N1. RX is
 *                interrupt driven into a 128-byte ring so that rx_available() is a real
 *                count; USART1_IRQn at priority 6.
 *
 *   IWDG         Clocked from LSI (the independent oscillator), pre-warmed before the
 *                kernel starts, fed only by the supervisor task (FR-0.4).
 *
 *   GPIO pins    Application ids 0..3 map to:
 *                  0 kPinStatusLed    PC13  push-pull, ACTIVE LOW (Blue Pill LED).
 *                                           Speed deliberately LOW: PC13 sits in the
 *                                           backup domain and the datasheet limits it.
 *                  1 kPinTraceControl PB0   push-pull, 50 MHz. NFR-1.8 jitter marker.
 *                  2 kPinTraceIsr     PB1   push-pull, 50 MHz. NFR-1.9 ISR marker.
 *                  3 kPinTraceTask    PB5   push-pull, 50 MHz. NFR-1.9 wake marker.
 *                PB0/PB1/PB5 are free on this board and are not ADC channels in the
 *                Phase 1 analog plan (which uses PA0..PA7), so wiring a logic analyzer
 *                to them costs the analog path nothing. PA13/PA14 (SWD), PA8 (MCO, used
 *                for the crystal measurement in bring-up step 6), PA11/PA12 (USB) and
 *                PA15/PB3/PB4 (JTAG) are deliberately untouched.
 */
#ifndef HAL_STM32F1_H
#define HAL_STM32F1_H

#include <stdbool.h>
#include <stdint.h>

#include "hal/gpio.h"
#include "hal/i2c.h"
#include "hal/system.h"
#include "hal/uart.h"
#include "hal/watchdog.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * The five interface instances.
 * ---------------------------------------------------------------------------
 * Each is a singleton with static storage: the objects are created once during
 * hal_stm32f1_init() and never freed (NFR-0.7 forbids run-time allocation churn).
 */
hal_gpio_iface_t *hal_gpio_stm32f1(void);
hal_i2c_iface_t *hal_i2c_stm32f1(void);
hal_uart_iface_t *hal_uart_stm32f1(void);
hal_watchdog_iface_t *hal_watchdog_stm32f1(void);
hal_system_iface_t *hal_system_stm32f1(void);

/* Bring up clocks, the HAL timebase, the DWT, the backup-domain boot counter, the GPIO
 * table, I2C1 and USART1 (at 115200). Call once, before anything that uses the
 * interfaces above; every accessor returns a usable instance afterwards.
 *
 * The watchdog is deliberately NOT started here: FR-0.4 requires a task to own that
 * decision, so start() is the supervisor's call.
 */
void hal_stm32f1_init(void);

/* ---------------------------------------------------------------------------
 * Pin ids
 * ---------------------------------------------------------------------------
 * These are the application-level numbers from app/board.h (kPinStatusLed = 0 ...).
 * They are repeated rather than included because hal/ must not depend on app/; the
 * static_asserts in main_target.cpp are what keeps the two definitions from drifting.
 */
enum {
  HAL_PIN_STATUS_LED = 0,
  HAL_PIN_TRACE_CONTROL = 1,
  HAL_PIN_TRACE_ISR = 2,
  HAL_PIN_TRACE_TASK = 3,
  HAL_PIN_COUNT = 4
};

/* ---------------------------------------------------------------------------
 * Helpers shared between the stm32f1 translation units. Not part of any frozen
 * interface; nothing outside firmware/hal/stm32f1/ should need them.
 * ---------------------------------------------------------------------------
 */

/* Busy-wait for `us` real microseconds, from the DWT cycle counter.
 *
 * Explicitly not HAL_Delay(): that is millisecond-granular, and the I2C recovery in
 * FR-0.3 needs a ~5 us half-period. If the DWT is not counting (no cycle counter on
 * some clone silicon, or a debug unit that will not enable), this falls back to a
 * loop calibrated against the 1 ms HAL tick — a longer recovery half-period is
 * harmless, a shorter one is not.
 */
void hal_stm32f1_delay_us(uint32_t us);

/* True if the HSE crystal started and SYSCLK is the 72 MHz PLL. False means the
 * 64 MHz HSI fallback is in use, which the bring-up banner reports.
 */
bool hal_stm32f1_hse_ok(void);

/* Per-peripheral bring-up, called in order by hal_stm32f1_init(). */
void hal_stm32f1_gpio_init(void);
void hal_i2c_stm32f1_init(void);
void hal_uart_stm32f1_init(void);

/* Reset-cause sampling (sample and clear the RCC flags) and the backup-register boot
 * counter. Split because the flags must be read before anything else touches RCC,
 * while the boot counter needs the PWR/BKP clocks.
 */
void hal_system_stm32f1_sample_reset_cause(void);
void hal_system_stm32f1_count_boot(void);

/* Bring-up diagnostics for the self-test image. Both report something the frozen
 * interfaces in the hal/ headers have no way to express: the receive-path drop count (which
 * hal/uart.h's rx_available does not cover) and the timeout the watchdog was actually
 * programmed with (hal/watchdog.h's start() is bool).
 */
uint32_t hal_uart_stm32f1_overruns(void);
uint32_t hal_watchdog_stm32f1_programmed_ms(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_STM32F1_H */
