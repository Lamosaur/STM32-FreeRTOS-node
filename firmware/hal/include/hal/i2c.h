/* i2c.h — the I2C bus interface the application layer is written against.
 *
 * This interface is deliberately wider than "do a transfer". Bus recovery (FR-0.3)
 * is a bit-level sequence — nine clock pulses with SDA released, then a STOP — and
 * the requirement is that the *sequence* is correct, not that some vendor routine was
 * called. So the peripheral operations and the bit-level operations are separate
 * entries in one interface, and the sequencing itself lives in the application layer
 * (firmware/app/src/i2c_recovery.cpp) where a host test can drive it and assert on
 * exactly what was emitted.
 *
 * A conforming implementation must guarantee, between recover_begin() and
 * recover_end(), that scl_write/sda_write/sda_read operate on the physical bus lines
 * with the I2C peripheral detached, and that recover_end() restores the peripheral.
 */
#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  HAL_I2C_OK = 0,
  HAL_I2C_NACK,      /* a device or register address was not acknowledged */
  HAL_I2C_BUS_ERROR, /* arbitration lost, bus error, or a line stuck low */
  HAL_I2C_TIMEOUT,   /* the peripheral did not complete within timeout_ms */
} hal_i2c_status_t;

typedef struct hal_i2c_iface {
  void *ctx;

  /* Blocking transfer, 7-bit device address.
   *   tx_len == 0 && rx_len > 0  -> pure read
   *   rx_len == 0 && tx_len > 0  -> pure write
   *   both > 0                   -> write then repeated-start read (register read)
   *   both == 0                  -> address probe: the address is sent and the
   *                                 acknowledge bit is what decides the result. OK means
   *                                 a device answered, NACK means nothing is there. This
   *                                 is the bus scan of the bring-up log, so it must
   *                                 genuinely touch the bus rather than succeed trivially.
   * Returns HAL_I2C_OK only if every byte was acknowledged. `tx`/`rx` may be NULL
   * when their length is 0.
   */
  hal_i2c_status_t (*transfer)(void *ctx, uint16_t addr7, const uint8_t *tx, size_t tx_len,
                               uint8_t *rx, size_t rx_len, uint32_t timeout_ms);

  /* --- DMA-backed writes (FR-0.5) ---------------------------------------------
   * A 128x64 framebuffer flush is up to 1024 bytes. As a polled byte-at-a-time
   * transfer that pins the CPU for milliseconds; FR-0.5 requires the display path to
   * be DMA-driven instead, so the peripheral feeds itself from memory and the CPU is
   * free for the control and sensor work.
   *
   * Transfer is split into start and wait so the caller decides when to block. This is
   * a write-only path: reads are short register fetches where the setup cost of DMA
   * would exceed the transfer.
   *
   * transfer_start_dma() returns without waiting. `tx` must stay valid and unmodified
   * until transfer_wait() returns. Returns false if the transfer could not be started
   * (peripheral busy, or DMA unavailable).
   */
  bool (*transfer_start_dma)(void *ctx, uint16_t addr7, const uint8_t *tx, size_t tx_len);

  /* Wait for the write started by transfer_start_dma() and report its outcome. */
  hal_i2c_status_t (*transfer_wait)(void *ctx, uint32_t timeout_ms);

  /* Sample the raw bus levels. Returns false if the lines cannot be read at all.
   * Both-high means the bus is idle and usable.
   */
  bool (*read_lines)(void *ctx, bool *scl_high, bool *sda_high);

  /* Detach the I2C peripheral and take SCL and SDA over as open-drain GPIO, both
   * released high. Returns false if the pins cannot be taken over.
   */
  bool (*recover_begin)(void *ctx);

  /* Drive SCL / SDA while in recovery. false = pull low, true = release (the
   * external pull-ups set the high level). Return false only on a hard failure.
   */
  bool (*scl_write)(void *ctx, bool level);
  bool (*sda_write)(void *ctx, bool level);

  /* Read SDA (and, for a clock-stretch check, SCL) while in recovery. Writes the
   * sampled level through `level` and returns false only if the line is unreadable.
   */
  bool (*sda_read)(void *ctx, bool *level);
  bool (*scl_read)(void *ctx, bool *level);

  /* Bus-time delay for recovery edges, in microseconds. */
  void (*delay_us)(void *ctx, uint32_t us);

  /* Release SCL and SDA and re-initialise the I2C peripheral for normal transfers.
   * Returns false if the peripheral could not be brought back.
   */
  bool (*recover_end)(void *ctx);
} hal_i2c_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_I2C_H */
