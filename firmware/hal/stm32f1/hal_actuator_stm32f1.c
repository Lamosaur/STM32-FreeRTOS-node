/* hal_actuator_stm32f1.c — PA1, the low-side MOSFET gate of the Phase 1 actuator.
 *
 * The interface's whole argument (hal/actuator.h) is that "off" is the state this system is
 * allowed to fail into, and that the firmware can only guarantee it while the CPU is
 * running. This file is the part of that guarantee the firmware *can* make, and it comes
 * down to one ordering in init():
 *
 *     THE PIN IS DRIVEN LOW BEFORE IT IS CONFIGURED AS AN OUTPUT.
 *
 * Why that ordering and not the other way round, since on the F103 the "obvious" order —
 * configure the pin as a push-pull output, then write it low — compiles, runs, and works on
 * a bench with a MOSFET that is slow to turn on. It is a defect because of what the pin is
 * between those two statements: after the mode register is written the pin is an *output
 * driving whatever the output data register happened to hold*, and ODR's reset value is 0
 * only for as long as nothing has written it. Two cases where it is not:
 *
 *   - A watchdog or software reset that does not clear ODR (a warm reset does not reload
 *     the GPIO registers' state on every path, and a bootloader or earlier image that drove
 *     the pin high leaves it high), so the pin comes up driving the gate on for the whole
 *     of the boot — seconds, with an ST-Link attached.
 *   - init() being called twice, or after off(), where a stale write in the data register
 *     is exactly the level a previous on() left behind.
 *
 * A store to BSRR/ODR before the mode change is harmless while the pin is still an input
 * (it just sets the latch), and it means the first instant the pin becomes an output it is
 * already low. That is the whole fix, it costs two instructions, and reversing the two
 * statements is the one edit in this file that can damage hardware: an uncommanded pump run
 * is a flooded pot at best and a dry-run pump at worst.
 *
 * The other half of the guarantee is not in this file and cannot be: the external pull-down
 * (hal/actuator.h explains at length), and the fact that a MOSFET that has failed short, or
 * a gate driver whose 3V3 has collapsed while the pump's 12 V is still up, are failures no
 * firmware can see.
 *
 * The pin assignment — PA1, and the note that PA2 is reserved rather than driven — is in
 * app/board.h's Phase 1 pin table, with the reasoning for each pin.
 */

#include "hal_stm32f1.h"

#include "stm32f1xx_hal.h"

/* PA1. Push-pull output: the gate is driven actively in both directions through the
 * driver's own pull-down/pull-up network, and an open-drain output here would turn the
 * external pull-down into the only sink for the gate's charge — which is slower, and would
 * make the turn-off time a property of a resistor chosen for a different job. */
#define ACTUATOR_GPIO_PORT GPIOA
#define ACTUATOR_GPIO_PIN GPIO_PIN_1

/* Slow speed. Nothing about this output is fast: the shortest pulse the control loop may
 * command is kControlMinOnTimeMs (50 ms), and a gate of a few tens of nano-coulombs does
 * not need a 50 MHz driver. The slower slew rate is the smaller switching transient next to
 * the analog front end, which is the same argument hal/actuator.h makes against fast PWM,
 * applied to the one edge this design does produce. */
#define ACTUATOR_GPIO_SPEED GPIO_SPEED_FREQ_LOW

static void actuator_off_line(void) {
  HAL_GPIO_WritePin(ACTUATOR_GPIO_PORT, ACTUATOR_GPIO_PIN, GPIO_PIN_RESET);
}

static bool actuator_init(void *ctx) {
  (void)ctx;

  /* 1. The output latch is cleared while the pin is still an input. See the file comment:
   * this is the ordering that matters. */
  actuator_off_line();

  /* 2. Now the pin can become an output, and it starts low. */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef init = {0};
  init.Pin = ACTUATOR_GPIO_PIN;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  /* No internal pull: the F1's internal pull-up/pull-down resistors are only available in
   * input modes, so there is nothing for this field to do on an output — and the pull-down
   * that matters is the external one. Stated rather than left implicit because a reader
   * who expects a pull-down here would conclude the hardware needs none. */
  init.Pull = GPIO_NOPULL;
  init.Speed = ACTUATOR_GPIO_SPEED;
  HAL_GPIO_Init(ACTUATOR_GPIO_PORT, &init);

  /* 3. Restate the level after the mode change. Not redundant: HAL_GPIO_Init() configures a
   * port in one write per register but the CNF/MODE nibble for this pin is rewritten from
   * whatever the caller passed, and re-asserting the level here means the state of the gate
   * after init() is a fact this function establishes rather than one it inherits. */
  actuator_off_line();

  return true;
}

static void actuator_on(void *ctx) {
  (void)ctx;
  /* Deliberately without a "has init() run?" guard. on() before init() would leave the pin as
   * an input with a set latch — not an energised gate, because an input does not drive —
   * and refusing it here would mean the one caller that can call off() at any time could
   * not call on() at any time either, which is not a property the interface asks for. What
   * the interface does require is that off() is always safe, and it is: see below. */
  HAL_GPIO_WritePin(ACTUATOR_GPIO_PORT, ACTUATOR_GPIO_PIN, GPIO_PIN_SET);
}

static void actuator_off(void *ctx) {
  (void)ctx;
  /* Safe before init(), repeatedly, and while already off: it is a store to a register, and
   * clearing a latch that is already clear is a no-op. That property is what the fail-safe
   * decision rests on — a caller that cannot prove the pump should be running is always
   * allowed to say off — so this function must not grow a precondition. */
  actuator_off_line();
}

static hal_actuator_iface_t s_actuator = {
    .ctx = NULL,
    .init = actuator_init,
    .on = actuator_on,
    .off = actuator_off,
};

hal_actuator_iface_t *hal_actuator_stm32f1(void) {
  return &s_actuator;
}

/* Called by hal_stm32f1_init() early in the boot, before anything that could enable the
 * pump: leaving the gate in its reset state (a floating input on this part, which the
 * external pull-down holds off) for as short a time as possible is the point of doing it
 * here rather than in the node's init(). */
void hal_actuator_stm32f1_init(void) {
  (void)actuator_init(NULL);
}
