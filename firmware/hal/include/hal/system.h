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
} hal_system_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_SYSTEM_H */
