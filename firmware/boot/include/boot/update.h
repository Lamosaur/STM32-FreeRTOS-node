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

/* ASCII "UPD1" read little-endian: distinctive, and it displays readably in a debugger. */
#define BOOT_UPDATE_MARKER 0x31504455u

/* BKP_DR3. BKP_DR1 and DR2 carry the application's boot counter and its validity marker;
 * DR3 is free, and both images know the number because both include this header. */
#define BOOT_UPDATE_BKP_SLOT 3u

#ifdef __cplusplus
}
#endif

#endif /* BOOT_UPDATE_H */