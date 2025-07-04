/* hal_gpio_stm32f1.c — pin-level digital I/O for the F103C8T6.
 *
 * The mapping from application pin id to port/bit lives here and nowhere else, which is
 * what lets the application layer name a pin without a device header. The pin
 * assignment itself, with the reasoning for each choice, is documented in
 * hal_stm32f1.h.
 */

#include "hal_stm32f1.h"

#include "stm32f1xx_hal.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t mask;
  bool is_output;
} node_pin_t;

/* The pin table. Ids are the application's (app/board.h kPin*); see the static_asserts
 * in app/src/main_target.cpp that fail the build if the two ever disagree. */
static node_pin_t s_pins[HAL_PIN_COUNT] = {
    {GPIOC, GPIO_PIN_13, false}, /* 0 kPinStatusLed   — active low */
    {GPIOB, GPIO_PIN_0, false},  /* 1 kPinTraceControl — NFR-1.8 */
    {GPIOB, GPIO_PIN_1, false},  /* 2 kPinTraceIsr     — NFR-1.9 */
    {GPIOB, GPIO_PIN_5, false},  /* 3 kPinTraceTask    — NFR-1.9 */
};

static bool gpio_configure_output(void *ctx, hal_pin_t pin, hal_gpio_out_mode_t mode) {
  (void)ctx;

  if (pin >= (hal_pin_t)HAL_PIN_COUNT) {
    return false;
  }

  node_pin_t *p = &s_pins[pin];
  GPIO_InitTypeDef init = {0};

  init.Pin = p->mask;
  init.Mode = (mode == HAL_GPIO_OUT_OPEN_DRAIN) ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;

  /* PC13/PC14/PC15 sit in the backup domain and the datasheet limits them to 2 MHz, so
   * the status LED is configured slow even though the LED itself could not care. The
   * three trace pins are the opposite: NFR-1.8/NFR-1.9 read *edges* off a logic
   * analyzer, so they want the fastest output driver available.
   */
  init.Speed = (pin == (hal_pin_t)HAL_PIN_STATUS_LED) ? GPIO_SPEED_FREQ_LOW
                                                      : GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(p->port, &init);
  p->is_output = true;
  return true;
}

static bool gpio_configure_input(void *ctx, hal_pin_t pin, bool pull_up) {
  (void)ctx;

  if (pin >= (hal_pin_t)HAL_PIN_COUNT) {
    return false;
  }

  node_pin_t *p = &s_pins[pin];
  GPIO_InitTypeDef init = {0};

  init.Pin = p->mask;
  init.Mode = GPIO_MODE_INPUT;
  init.Pull = pull_up ? GPIO_PULLUP : GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(p->port, &init);
  p->is_output = false;
  return true;
}

static bool gpio_write(void *ctx, hal_pin_t pin, bool level) {
  (void)ctx;

  if (pin >= (hal_pin_t)HAL_PIN_COUNT) {
    return false;
  }

  node_pin_t *p = &s_pins[pin];
  if (!p->is_output) {
    /* The interface documents this as ignored-and-false rather than an error: driving a
     * pin the caller never declared as an output is a bug in the caller, and silently
     * switching the pin's mode here would hide it. */
    return false;
  }

  HAL_GPIO_WritePin(p->port, p->mask, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return true;
}

static bool gpio_read(void *ctx, hal_pin_t pin, bool *level) {
  (void)ctx;

  if (pin >= (hal_pin_t)HAL_PIN_COUNT || level == NULL) {
    return false;
  }

  node_pin_t *p = &s_pins[pin];
  *level = (HAL_GPIO_ReadPin(p->port, p->mask) != GPIO_PIN_RESET);
  return true;
}

static bool gpio_toggle(void *ctx, hal_pin_t pin) {
  (void)ctx;

  if (pin >= (hal_pin_t)HAL_PIN_COUNT) {
    return false;
  }

  node_pin_t *p = &s_pins[pin];
  if (!p->is_output) {
    return false;
  }

  /* Toggling is the instrumentation primitive (NFR-1.8/NFR-1.9): an odd number of
   * toggles per event is unambiguous on a logic analyzer, where a level would not be.
   * HAL_GPIO_TogglePin() reads and rewrites ODR, which is fine here because these pins
   * are only ever touched by the tasks and ISR being measured. */
  HAL_GPIO_TogglePin(p->port, p->mask);
  return true;
}

static hal_gpio_iface_t s_gpio = {
    .ctx = NULL,
    .configure_output = gpio_configure_output,
    .configure_input = gpio_configure_input,
    .write = gpio_write,
    .read = gpio_read,
    .toggle = gpio_toggle,
};

hal_gpio_iface_t *hal_gpio_stm32f1(void) {
  return &s_gpio;
}

void hal_stm32f1_gpio_init(void) {
  /* GPIOA for USART1 (PA9/PA10) and for MCO on PA8 later, GPIOB for I2C1 and the trace
   * pins, GPIOC for the status LED. AFIO is needed for the I2C1 remap bit. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* Both trace pins and the LED start low, and the LED is *active low*, so its initial
   * "off" state is a high output — write the output data register before switching the
   * pin to output so that it never glitches low on the way. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5, GPIO_PIN_RESET);

  hal_gpio_iface_t *gpio = hal_gpio_stm32f1();

  (void)gpio->configure_output(gpio->ctx, HAL_PIN_STATUS_LED, HAL_GPIO_OUT_PUSH_PULL);
  (void)gpio->configure_output(gpio->ctx, HAL_PIN_TRACE_CONTROL, HAL_GPIO_OUT_PUSH_PULL);
  (void)gpio->configure_output(gpio->ctx, HAL_PIN_TRACE_ISR, HAL_GPIO_OUT_PUSH_PULL);
  (void)gpio->configure_output(gpio->ctx, HAL_PIN_TRACE_TASK, HAL_GPIO_OUT_PUSH_PULL);

  (void)gpio->write(gpio->ctx, HAL_PIN_STATUS_LED, true); /* LED off: active low */
}
