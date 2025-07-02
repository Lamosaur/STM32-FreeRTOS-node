/* rtos_freertos.h — the target binding of rtos_iface_t to FreeRTOS.
 *
 * The accessor is the only thing the target's entry point needs; everything else is
 * reached through the interface in rtos/rtos.h, which is what keeps the application core
 * free of FreeRTOS headers.
 */
#ifndef RTOS_FREERTOS_H
#define RTOS_FREERTOS_H

#include "rtos/rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The singleton interface. Its mutexes are created on demand and never freed: a 72 h
 * unattended run must not depend on the heap surviving that long (NFR-0.7). Safe to
 * call before the scheduler is started. */
rtos_iface_t *rtos_freertos_iface(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RTOS_FREERTOS_H */
