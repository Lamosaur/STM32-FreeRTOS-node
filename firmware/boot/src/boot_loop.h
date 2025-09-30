/* boot_loop.h — the update loop, the image verifier and the marker contract's bootloader side. */
#ifndef BOOT_LOOP_H
#define BOOT_LOOP_H

#include <stdbool.h>

#include "boot/metadata.h"

#ifdef __cplusplus
extern "C" {
#endif

/* True when the application asked for an update: its marker sits in the backup register
 * named by BOOT_UPDATE_BKP_SLOT, read after enabling the BKP clock (reading needs the
 * clock; only writing needs the backup-domain write-protect lift). */
bool boot_update_marker_read(void);

/* Clear the marker. Called after a successful commit (the update is over) and when the
 * update window expires with a valid app present (a stale marker is a relic, not an
 * order). Writing needs PWR_CR.DBP, exactly as the application's boot counter does. */
void boot_update_marker_clear(void);

/* boot_decide's image checker: hash the target slot's live flash and compare with the
 * slip's recorded CRC-32. In its own translation unit because both the initial decision
 * and the update window's expiry re-decision use it. */
bool boot_verify_image(const boot_metadata_t *slip, void *ctx);

/* Run the update loop: consume frames from the polled UART, answer each with the framed
 * ACK/NAK of boot/frame.h, execute the commands, and measure the idle window against
 * BOOT_UPDATE_IDLE_TIMEOUT_MS. On window expiry with a valid app, jumps to it (the
 * anti-brick); otherwise keeps waiting — the recovery state. Never returns on its own. */
void boot_update_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_LOOP_H */