/* update.h — the shared contract between the running application and the bootloader.
 *
 * Both images agree on two facts here: the backup register the application writes (with
 * backup-domain access enabled, exactly as it already does for its boot counter) and the
 * marker value that means "I, the app, ask for an update; stay in the bootloader after
 * the reset I am about to perform". The bootloader reads this register at every boot and,
 * if the marker is present, opens the update window (decision.h).
 *
 * Why a backup register rather than a flag in RAM or a message on the wire: RAM dies on
 * reset and the bootloader has no listener yet at reset time, while the backup domain
 * survives reset by design — the same persistence the application's 72 h reset counter
 * (NFR-0.7) relies on.
 */
#ifndef BOOT_UPDATE_H
#define BOOT_UPDATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ASCII "UP". 16-bit on purpose: the F1's BKP_DRx registers are 16-bit hardware
 * (the vendored CMSIS types them uint32_t, so the compiler cannot see the
 * truncation), and a marker wider than 16 bits can never compare equal after a
 * round trip through DR3 — which silently disables the whole update-requested rung.
 * Distinct from erased (0xFFFF), cleared (0x0000) and any residue of the old
 * 32-bit marker's low half, so a stale register cannot false-trigger. */
#define BOOT_UPDATE_MARKER 0x5055u

/* The marker must survive a round trip through a 16-bit BKP_DRx register. A wider
 * value truncates on write and can never compare equal on read, which silently
 * disables the update-requested rung — the compiler cannot see it (CMSIS types
 * the registers uint32_t), so the constraint is enforced here, where both images
 * and the host tests compile it. */
_Static_assert(BOOT_UPDATE_MARKER <= 0xFFFFu, "marker must fit a 16-bit BKP register");

/* BKP_DR3. BKP_DR1 and DR2 carry the application's boot counter and its validity marker;
 * DR3 is free, and both images know the number because both include this header. */
#define BOOT_UPDATE_BKP_SLOT 3u

#ifdef __cplusplus
}
#endif

#endif /* BOOT_UPDATE_H */