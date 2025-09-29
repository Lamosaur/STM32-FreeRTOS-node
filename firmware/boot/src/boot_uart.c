/* boot_uart.c — register-level USART1 for the bootloader.
 *
 * Why no HAL here, when the application uses the ST HAL everywhere: the bootloader is a
 * separate, freestanding program of its own (the "tiny program" discipline), and it
 * deliberately does not drag in the HAL's clock config, interrupt plumbing and config
 * header. Six registers is everything the wire needs, and a reader can audit every word.
 */
#include "boot_uart.h"

#include <stddef.h>

#include "stm32f1xx.h" /* the vendored CMSIS device header: registers, not the HAL */

/* The bootloader runs on the reset clock (HSI 8 MHz, no PLL): configuring the clock tree
 * is the application's job, and with nothing but a UART and some flash to touch, there is
 * nothing here that a higher clock buys. BRR for 115200 from 8 MHz is 8e6 / (16 * baud) =
 * 4.34, written as mantissa 4, fraction 5 -> 69. The integer lands at 115,942 baud,
 * +0.64%, comfortably inside the +-2% both ends of a UART link tolerate; the application,
 * which does configure 72 MHz, writes BRR 625 for exactly 115,200 — the same wire, and a
 * host cannot tell the two apart.
 */
void boot_uart_init(void) {
  /* Clock the port and the peripheral: RCC_APB2ENR bits IOPAEN (2) and USART1EN (14). */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

  /* PA9 = TX: alternate-function push-pull, 50 MHz (CRH nibble at bits 4..7 = 0xB).
   * PA10 = RX: input with pull-up (nibble at bits 8..11 = 0x8), and the ODR bit so the
   * line idles high when the far end says nothing. */
  GPIOA->CRH = (GPIOA->CRH & ~(0xFFu << 4)) | (0xBu << 4) | (0x8u << 8);
  GPIOA->ODR |= (1u << 10);

  /* 8N1, both ends on: UE, TE, RE. No interrupts, no FIFO — see the header. */
  USART1->BRR = 69u;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void boot_uart_putc(uint8_t c) {
  /* Wait for the transmit register to accept the previous byte (TXE). A blocking wait is
   * correct here: the wire is the bottleneck and there is nothing else to do. */
  while ((USART1->SR & USART_SR_TXE) == 0u) {
  }
  USART1->DR = c;
}

void boot_uart_puts(const char *s) {
  if (s == NULL) {
    return;
  }
  while (*s != '\0') {
    boot_uart_putc((uint8_t)*s++);
  }
}

uint32_t boot_uart_rx_available(void) {
  return (USART1->SR & USART_SR_RXNE) != 0u ? 1u : 0u;
}

int boot_uart_getc(uint32_t timeout_ms) {
  /* At 8 MHz a trivial loop is roughly 3 cycles per iteration, so about 2.6e6 iterations
   * per millisecond; 2000 is deliberately conservative (a slightly long timeout is fine,
   * a short one loses a frame). The updater's idle window is measured in these calls. */
  const uint32_t spins = timeout_ms * 2000u;
  for (uint32_t i = 0; i < spins; ++i) {
    if ((USART1->SR & USART_SR_RXNE) != 0u) {
      return (int)(USART1->DR & 0xFFu);
    }
  }
  return -1;
}