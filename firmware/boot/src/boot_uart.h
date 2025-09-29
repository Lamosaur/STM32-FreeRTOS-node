/* boot_uart.h — the bootloader's polled UART: USART1 on PA9 (TX) / PA10 (RX).
 *
 * Polled, not interrupt-driven, is the bootloader's whole style: nothing else runs
 * here, so an interrupt would have no competition to care about, and the update loop
 * wants to *block* on the wire anyway. The application later reconfigures the same
 * peripheral to its own liking, which is fine — the app owns the machine after the jump.
 */
#ifndef BOOT_UART_H
#define BOOT_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void boot_uart_init(void);
void boot_uart_putc(uint8_t c);
void boot_uart_puts(const char *s);

/* Bytes currently waiting in the receive register: 1 or 0. The updater uses this to
 * measure the idle window (decision.h's BOOT_UPDATE_IDLE_TIMEOUT_MS) without blocking. */
uint32_t boot_uart_rx_available(void);

/* Read one byte, or -1 if none arrived within timeout_ms. A polling loop with a cycle
 * count calibrated at build time; see the .c for the number. */
int boot_uart_getc(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_UART_H */