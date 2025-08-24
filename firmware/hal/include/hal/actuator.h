/* actuator.h — the switched output the control loop drives.
 *
 * Phase 1's output is time-proportioned (design-spec.md §6, decision 4 of the Phase 1
 * record): inside each control window the pump is on for a computed number of
 * milliseconds, and off for the rest. That is not PWM and this interface deliberately
 * cannot express PWM: there are two operations, on and off, and no duty register, no
 * frequency, and no partial state. A later change cannot quietly turn this into a PWM
 * abstraction, because there is nothing here to write a compare value into — the same
 * argument hal/adc.h makes about polling, applied to the output side.
 *
 * Why not PWM, recorded here because this is the header someone would change to get it:
 * a fast PWM edge train is a switching noise source that this board's analog front end
 * has to live next to, a soil-moisture loop has no bandwidth that needs it, and at a
 * 1000 ms window with a 50 ms minimum pulse the modulation is coarse enough that the
 * gate's own rise and fall times are a small part of the pulse. If a future phase needs
 * fast PWM it needs a different, richer interface and a different argument about the ADC
 * (FR-1.2, NFR-1.12), not a wider version of this one.
 *
 * THE PULL-DOWN, which is a hardware requirement this interface cannot enforce and
 * therefore documents instead. The gate must be pulled DOWN (or the driver's enable held
 * inactive) by a resistor, so that a dead MCU means a stopped pump. Everything the
 * firmware does about safety is conditional on the CPU still running:
 *
 *   - off() is only reached if the code that calls it runs:
 *   - the watchdog resets a hung CPU, but a reset does not stop a MOSFET that is already
 *     on — during the reset, and during the boot before init() runs, the gate is whatever
 *     the pin's default state is, which for an STM32 output is a floating input;
 *   - a MOSFET that has failed short, a board whose 3V3 rail has collapsed while the
 *     pump's 12 V rail is still up, and a gate driver whose input is left floating are
 *     all failures the firmware cannot see and cannot fix.
 *
 * A 10 k pull-down from gate to source makes all of those default to "pump off", which is
 * the state this system is allowed to fail into. It is also why off() is described below
 * as the fail-safe state rather than as the complement of on(): when a caller cannot
 * prove the pump should be running, off() is the answer, and it is always safe to call.
 */
#ifndef HAL_ACTUATOR_H
#define HAL_ACTUATOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_actuator_iface {
  void *ctx;

  /* Configure the gate as an output and leave it in the OFF state before returning.
   *
   * Returns false if the pin is unknown or the peripheral it needs is unavailable. A
   * caller that gets false must treat the pump as unavailable rather than as stopped and
   * forgotten: a failed init() is a reason to withhold control entirely, because a gate
   * driven through an unconfigured pin may sit high.
   *
   * Deliberately without an argument for the initial state: the only initial state this
   * system wants is off, and an interface that could be asked for anything else would
   * eventually be called with `true` by a caller that could not prove it. */
  bool (*init)(void *ctx);

  /* Energise the gate.
   *
   * The only place in the firmware that may ask for water. It is called for a bounded
   * number of milliseconds per control window and never from an interrupt. */
  void (*on)(void *ctx);

  /* De-energise the gate: the fail-safe state.
   *
   * Must be safe to call at any time — before init(), repeatedly, from an error path, and
   * while already off — and it returns nothing, because there is no failure to report: a
   * conforming implementation drives the pin low and, on the F103, that is a store to a
   * register that cannot fail. A caller that has nothing else it can do is always allowed
   * to call this, which is the property the fail-safe decision depends on. */
  void (*off)(void *ctx);

  /* No readback entry is provided. Reading the pin back would report the pin's electrical
   * state, which is not the question — the questions are "did we command on?" (firmware
   * state, tracked by the caller that commanded it) and "is water actually flowing?"
   * (unanswerable without a flow sensor, which this phase does not have). An interface
   * that offered a boolean would invite someone to treat a pin readback as proof that the
   * pump is running, which is the one conclusion it cannot support. */
} hal_actuator_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_ACTUATOR_H */
