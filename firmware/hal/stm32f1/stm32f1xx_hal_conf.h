/* stm32f1xx_hal_conf.h — HAL configuration for the Phase 0 node (STM32F103C8T6).
 *
 * Derived from the vendored stm32f1xx_hal_conf_template.h, trimmed to the modules this
 * firmware actually builds, with the oscillator values and the tick interrupt priority
 * set for this board. The template is left untouched in third_party/.
 */
#ifndef __STM32F1xx_HAL_CONF_H
#define __STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Module selection
 * ---------------------------------------------------------------------------
 * Only the modules compiled into node_hal_stm32f1 (see firmware/hal/stm32f1/
 * CMakeLists.txt) are enabled, because the enable is also what makes the HAL's own
 * headers available.
 *
 * HAL_FLASH_MODULE_ENABLED is here for the header alone: FLASH_LATENCY_2 and
 * __HAL_FLASH_PREFETCH_BUFFER_ENABLE() live in stm32f1xx_hal_flash.h, while the clock
 * setup uses the register macros directly and never calls into the flash driver, so
 * stm32f1xx_hal_flash.c is not compiled.
 */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ---------------------------------------------------------------------------
 * Oscillators
 * ---------------------------------------------------------------------------
 * 8 MHz crystal (HSE) and the 40 kHz internal LSI, which is the IWDG's timebase.
 * The LSI figure is the datasheet's typical value; the part's real range is roughly
 * 30-60 kHz, which is why the supervisor feeds well inside the requested timeout.
 */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE 8000000U
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE 40000U
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif

/* ---------------------------------------------------------------------------
 * Flash
 * ---------------------------------------------------------------------------
 * The prefetch buffer is on (also set by HAL_Init(), and restated in the clock setup
 * because a latency change without it is the classic way to lose most of the
 * performance a 72 MHz part was chosen for).
 */
#define PREFETCH_ENABLE 1U

/* ---------------------------------------------------------------------------
 * Timebase
 * ---------------------------------------------------------------------------
 * Priority 6, not the template's 0x0F and not 0.
 *
 * SysTick is FreeRTOS's tick, so the HAL timebase is TIM4 (hal_stm32f1.c overrides
 * HAL_InitTick). Numerically 6 is documented as being *below*
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5), so the timebase ISR is masked
 * inside FreeRTOS critical sections and cannot lengthen their worst case. It calls no
 * FreeRTOS API, so 5 would also be legal; 6 is the more conservative choice.
 */
#define TICK_INT_PRIORITY 6U

/* The HAL is used from tasks that own their own synchronisation (the I2C arbiter's
 * mutex); letting the HAL take an RTOS lock of its own would nest two locks on the same
 * peripheral for no benefit. */
#define USE_RTOS 0U

/* USE_FULL_ASSERT stays off: assert_param() is a no-op, which keeps a 64 KB part's
 * flash for code. The runtime invariants that matter here (interrupt priorities,
 * scheduler state, stack overflow) are covered by FreeRTOS's configASSERT instead,
 * which is compiled live — see FreeRTOSConfig.h.
 */
/* #define USE_FULL_ASSERT 1U */

/* ---------------------------------------------------------------------------
 * Module headers
 * ---------------------------------------------------------------------------
 */
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f1xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f1xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f1xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f1xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f1xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32f1xx_hal_i2c.h"
#endif
#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32f1xx_hal_iwdg.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f1xx_hal_pwr.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f1xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f1xx_hal_uart.h"
#endif

/* ---------------------------------------------------------------------------
 * Parameter checking
 * ---------------------------------------------------------------------------
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_CONF_H */
