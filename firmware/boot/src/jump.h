/* jump.h — the FR-2.3 jump into an application slot. */
#ifndef BOOT_JUMP_H
#define BOOT_JUMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Run the application in `slot` and never return: load the application's initial stack
 * pointer from ITS vector table, relocate the vector table with SCB->VTOR, and branch to
 * the application's reset PC with the Thumb bit set. See jump.c for why the order is
 * what it is. */
void boot_jump(uint32_t slot);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_JUMP_H */