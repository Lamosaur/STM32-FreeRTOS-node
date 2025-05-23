/* uart.h — the host link.
 *
 * Carries telemetry and tooling for the comms task (FR-0.1's fifth task). In Phase 2
 * this same port carries the framed firmware-update protocol, which is why the framing
 * is kept out of this interface: length-prefixing, CRC-16 and ACK/retry live in the
 * application layer so the identical code runs on both sides of the link.
 *
 * All calls are blocking with an explicit timeout and return the number of bytes
 * actually transferred, so a host that stops reading cannot wedge a task.
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_uart_iface {
  void *ctx;

  /* Bring the port up at `baud` with 8N1. Returns false if the baud rate is not
   * reachable from the peripheral clock within tolerance.
   */
  bool (*init)(void *ctx, uint32_t baud);

  /* Transmit `len` bytes. Returns the number sent, which may be less than `len` on
   * timeout.
   */
  size_t (*write)(void *ctx, const uint8_t *data, size_t len, uint32_t timeout_ms);

  /* Receive up to `len` bytes, returning as soon as any byte arrives. Returns 0 on
   * timeout, which is the normal idle case, not an error.
   */
  size_t (*read)(void *ctx, uint8_t *data, size_t len, uint32_t timeout_ms);

  /* Bytes waiting in the receive path without blocking. */
  size_t (*rx_available)(void *ctx);
} hal_uart_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_UART_H */
