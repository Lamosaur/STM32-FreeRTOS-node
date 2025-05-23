/* gpio.h — pin-level digital I/O.
 *
 * Used for the status LED, and for the instrumentation the measurement plan depends
 * on: NFR-1.8 and NFR-1.9 are both "toggle a GPIO and look at it on a logic
 * analyzer", so the toggle has to be reachable from the control task and from an ISR.
 *
 * Pins are named by an opaque application-level id (see firmware/app/include/app/board.h),
 * not by port and bit. The implementation owns the mapping from id to port/pin, which
 * is what lets the application core compile on the host without a device header.
 */
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t hal_pin_t;

typedef enum {
  HAL_GPIO_OUT_PUSH_PULL = 0,
  HAL_GPIO_OUT_OPEN_DRAIN,
} hal_gpio_out_mode_t;

typedef struct hal_gpio_iface {
  void *ctx;

  /* Configure `pin` as an output. Returns false for an unknown pin id. */
  bool (*configure_output)(void *ctx, hal_pin_t pin, hal_gpio_out_mode_t mode);

  /* Configure `pin` as an input with an optional internal pull-up. */
  bool (*configure_input)(void *ctx, hal_pin_t pin, bool pull_up);

  /* Drive an output. Ignored (and returns false) if `pin` is not an output. */
  bool (*write)(void *ctx, hal_pin_t pin, bool level);

  /* Sample an input into `level`. */
  bool (*read)(void *ctx, hal_pin_t pin, bool *level);

  /* Invert an output, for instrumentation markers. */
  bool (*toggle)(void *ctx, hal_pin_t pin);
} hal_gpio_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_GPIO_H */
