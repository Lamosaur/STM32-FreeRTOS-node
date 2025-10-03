/* system.h — reset cause and persistent boot accounting.
 *
 * NFR-0.7 is verified by a 72 h unattended run with a reset counter. This interface
 * is that counter's source: the count lives in a backup register that survives a
 * watchdog or software reset (but not power removal), and the cause is decoded from
 * the RCC reset flags before they are cleared.
 *
 * The count is incremented exactly once per boot by the implementation, so the first
 * boot of a freshly powered board reads 1.
 */
#ifndef HAL_SYSTEM_H
#define HAL_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HAL_RESET_UNKNOWN = 0,
  HAL_RESET_POWER_ON,
  HAL_RESET_EXTERNAL_PIN,
  HAL_RESET_WATCHDOG,
  HAL_RESET_SOFTWARE,
  HAL_RESET_LOW_POWER,
} hal_reset_cause_t;

typedef struct hal_system_iface {
  void *ctx;

  /* Why the MCU last reset. Must be sampled once at start-up and cached by the
   * implementation: reading it clears the hardware flags.
   */
  hal_reset_cause_t (*reset_cause)(void *ctx);

  /* Boots recorded in backup RAM, this boot included. The diagnostic for NFR-0.7:
   * an unattended run is clean if this stays at 1.
   */
  uint32_t (*boot_count)(void *ctx);

  /* Phase 2. The application's side of the update handshake: write the update marker
   * into the backup domain (the same persistence the boot counter uses), so that after
   * the caller resets, the bootloader opens its update window (boot/update.h,
   * boot_update_marker_read). The write is separated from the reset so the caller can
   * report a failure honestly: false means the marker was NOT written, the node is
   * still running, and a reset right now would boot normally — the caller must not
   * reset on false.
   */
  bool (*update_request)(void *ctx);

  /* Reset the part now. Used for the update jump after a successful update_request;
   * the caller emits any farewell over the UART first, because the reset does not wait
   * for the port to drain. The marker in the backup domain is what survives the reset.
   */
  void (*system_reset)(void *ctx);
} hal_system_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_SYSTEM_H */
