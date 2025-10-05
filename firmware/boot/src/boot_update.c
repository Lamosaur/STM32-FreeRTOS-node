/* boot_update.c — the update loop: frames in, flash out, one command at a time.
 *
 * The stay path's job, in full (boot_update.h summarises the pieces). Every frame from
 * the host is answered with the framed ACK/NAK of boot/frame.h; the commands map to the
 * flash driver from Lesson 3, and four rules carry the whole thing:
 *
 *   1. Region safety (boot/region.h): ERASE and WRITE may touch only the two application
 *      slots. The host can never erase the program answering it, and metadata is written
 *      exclusively by COMMIT, which constructs the slip it programs.
 *   2. Erase-before-write: the F1 programs bits 1->0 only, so WRITE to a page that was
 *      not erased this session corrupts silently instead of failing. Each WRITE is
 *      bound to the currently-erased page, and the prepare records which slot the
 *      update is filling.
 *   3. COMMIT is the boot-0 moment: re-verify the image's CRC-32 over live flash (the
 *      slip is a promise, the flash is the evidence), build the next slip with
 *      boot_metadata_next (sequence + 1, active = the written slot, written to the
 *      metadata page that is NOT winning), clear the marker. A power cut anywhere before
 *      the new slip is whole leaves the previous slip intact — the old app still boots.
 *   4. The idle window re-decides: the marker is cleared at expiry and the ladder runs;
 *      a valid app boots (anti-brick), otherwise the loop waits on, which is the
 *      recovery state a field box should sit in.
 */
#include "boot_loop.h"

#include "boot/crc.h"
#include "boot/decision.h"
#include "boot/frame.h"
#include "boot/layout.h"
#include "boot/metadata.h"
#include "boot/region.h"
#include "boot/update.h"
#include "boot_uart.h"
#include "flash.h"
#include "jump.h"
#include "stm32f1xx.h"

#define BOOT_VERSION_STRING "0.1.0"

/* The session's prepared state: which slot and which page the update has been writing.
 * Reset by a commit; anything else that wants a fresh view reads these. */
static uint32_t s_written_slot = 0u;          /* 0 = nothing prepared this session */
static uint32_t s_erased_page = 0u;           /* the page open for programming */
static bool s_erased_page_valid = false;

static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

/* Send one reply frame. NAKs carry their reason in the status byte (frame.h). */
static void reply(uint8_t status, const uint8_t *payload, uint8_t len) {
  uint8_t buf[BOOT_FRAME_MAX_LEN + 8u];
  const uint16_t n = boot_frame_encode(buf, sizeof(buf), status, payload, len);
  if (n > 0u) {
    for (uint16_t i = 0; i < n; ++i) {
      boot_uart_putc(buf[i]);
    }
  }
}

static void nak(uint8_t status) {
  reply(status, NULL, 0u);
}

bool boot_update_marker_read(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  (void)RCC->APB1ENR; /* let the bus come up */
  return (BKP->DR3 & 0xFFFFu) == BOOT_UPDATE_MARKER;
}

void boot_update_marker_clear(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP; /* lift the backup-domain write protection, as the app does */
  BKP->DR3 = 0u;
  PWR->CR &= ~PWR_CR_DBP;
}

bool boot_verify_image(const boot_metadata_t *slip, void *ctx) {
  (void)ctx;
  const uint32_t base = boot_slot_base(slip->active_slot);
  if (base == 0u) {
    return false;
  }
  return boot_crc32((const uint8_t *)base, slip->image_size) == slip->image_crc32;
}

static void handle_hello(void) {
  reply(BOOT_STATUS_OK, (const uint8_t *)BOOT_VERSION_STRING,
        (uint8_t)(sizeof(BOOT_VERSION_STRING) - 1u));
}

static void handle_get_meta(const boot_metadata_t *slip_a, const boot_metadata_t *slip_b) {
  static const boot_metadata_t kZeroSlip = {0};
  const boot_metadata_t *winner = boot_metadata_newer(slip_a, slip_b);
  /* The 32-byte struct, verbatim: the host reads version, slot and CRC-32 (FR-2.6)
   * without booting the app. An empty slip (no valid metadata) is still a reply a host
   * can parse, and version 0 is distinguishable from a real build. */
  reply(BOOT_STATUS_OK, (const uint8_t *)(winner != NULL ? winner : &kZeroSlip),
        (uint8_t)sizeof(boot_metadata_t));
}

static void handle_erase(const uint8_t *payload, uint8_t len) {
  if (len != 4u) {
    nak(BOOT_STATUS_BAD_LEN);
    return;
  }
  const uint32_t addr = le32(payload);
  if ((addr & (BOOT_FLASH_PAGE_SIZE - 1u)) != 0u || /* the controller wants the page base */
      !boot_region_writable(addr, BOOT_FLASH_PAGE_SIZE)) {
    nak(BOOT_STATUS_BAD_ADDR);
    return;
  }
  /* Which slot does this page belong to? The update is bound to one slot per session. */
  uint32_t slot = 0u;
  if (addr >= BOOT_SLOT_A_BASE && addr < BOOT_SLOT_A_BASE + BOOT_SLOT_SIZE) {
    slot = BOOT_SLOT_A;
  } else {
    slot = BOOT_SLOT_B;
  }
  if (boot_flash_erase_page(addr) != 0) {
    nak(BOOT_STATUS_FLASH_ERROR);
    return;
  }
  s_written_slot = slot;
  s_erased_page = addr;
  s_erased_page_valid = true;
  reply(BOOT_STATUS_OK, NULL, 0u);
}

static void handle_write(const uint8_t *payload, uint8_t len) {
  if (len < 5u) {
    nak(BOOT_STATUS_BAD_LEN);
    return;
  }
  const uint32_t addr = le32(payload);
  const uint32_t data_len = (uint32_t)len - 4u;

  /* Bound to the erased page: programming without the page's erase this session would
   * silently clear bits the old image set (rule 2). The arithmetic cannot wrap because
   * `addr < s_erased_page + PAGE` is established before the subtraction. */
  if (!s_erased_page_valid || addr < s_erased_page ||
      addr - s_erased_page >= BOOT_FLASH_PAGE_SIZE ||
      data_len > BOOT_FLASH_PAGE_SIZE - (addr - s_erased_page)) {
    nak(BOOT_STATUS_BAD_STATE);
    return;
  }
  if ((addr & 1u) != 0u || (data_len & 1u) != 0u ||
      !boot_region_writable(addr, data_len)) {
    nak(BOOT_STATUS_BAD_ADDR);
    return;
  }
  if (boot_flash_program(addr, payload + 4, data_len) != 0) {
    nak(BOOT_STATUS_FLASH_ERROR);
    return;
  }
  reply(BOOT_STATUS_OK, NULL, 0u);
}

static void handle_commit(const uint8_t *payload, uint8_t len,
                          const boot_metadata_t *slip_a, const boot_metadata_t *slip_b) {
  if (len != 12u) { /* size(4) + crc32(4) + version(4) */
    nak(BOOT_STATUS_BAD_LEN);
    return;
  }
  if (s_written_slot == 0u || !s_erased_page_valid) {
    nak(BOOT_STATUS_BAD_STATE); /* commit without a prepared write: refuse */
    return;
  }
  const uint32_t size = le32(payload);
  const uint32_t crc = le32(payload + 4u);
  const uint32_t version = le32(payload + 8u);

  /* Re-verify what the host claims, against the flash that was actually written: the
   * decision to promote this image is made on evidence, not on the host's word. */
  const uint32_t base = boot_slot_base(s_written_slot);
  if (boot_crc32((const uint8_t *)base, size) != crc) {
    nak(BOOT_STATUS_VERIFY_FAIL);
    return;
  }

  boot_metadata_t next;
  const uint32_t write_page =
      boot_metadata_next(slip_a, slip_b, s_written_slot, size, crc, version, &next);
  if (write_page == 0u) {
    nak(BOOT_STATUS_BAD_LEN); /* the supplied size can never describe a valid slip */
    return;
  }
  if (boot_flash_erase_page(write_page) != 0 ||
      boot_flash_program(write_page, (const uint8_t *)&next, (uint32_t)sizeof(next)) != 0) {
    nak(BOOT_STATUS_FLASH_ERROR);
    return;
  }

  boot_update_marker_clear();
  s_written_slot = 0u;
  s_erased_page_valid = false;
  reply(BOOT_STATUS_OK, NULL, 0u);
}

static void handle_reset(void) {
  /* No marker to preserve: COMMIT cleared it. A reset boots the fresh image, or the
   * bootloader decides what it must if the commit did not happen. */
  reply(BOOT_STATUS_OK, NULL, 0u);
  NVIC_SystemReset();
  for (;;) { /* not reached */
  }
}

void boot_update_loop(void) {
  const boot_metadata_t slip_a = *(const boot_metadata_t *)BOOT_METADATA_A_BASE;
  const boot_metadata_t slip_b = *(const boot_metadata_t *)BOOT_METADATA_B_BASE;

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint32_t idle_ms = 0u;

  for (;;) {
    /* Poll with a 10 ms budget; the budget is the idle clock. Any byte resets the
     * window, so a slow host that keeps talking is never timed out. */
    const int got = boot_uart_getc(10u);
    if (got < 0) {
      idle_ms += 10u;
      if (idle_ms >= BOOT_UPDATE_IDLE_TIMEOUT_MS) {
        boot_update_marker_clear(); /* the marker was a relic, not an order */
        const boot_outcome_t outcome =
            boot_decide(&slip_a, &slip_b, false, true, boot_verify_image, NULL);
        if (outcome.decision == BOOT_DECISION_RUN_SLOT_A ||
            outcome.decision == BOOT_DECISION_RUN_SLOT_B) {
          boot_jump(outcome.decision == BOOT_DECISION_RUN_SLOT_A ? BOOT_SLOT_A
                                                                 : BOOT_SLOT_B);
        }
        idle_ms = 0u; /* recovery state: keep accepting frames */
      }
      continue;
    }

    idle_ms = 0u;
    uint8_t cmd = 0u, len = 0u;
    const uint8_t *payload = NULL;
    const boot_frame_result_t result =
        boot_frame_rx_byte(&rx, (uint8_t)got, &cmd, &len, &payload);
    if (result == BOOT_FRAME_COMPLETE) {
      switch (cmd) {
        case BOOT_CMD_HELLO:    handle_hello();
          break;
        case BOOT_CMD_GET_META: handle_get_meta(&slip_a, &slip_b);
          break;
        case BOOT_CMD_ERASE:    handle_erase(payload, len);
          break;
        case BOOT_CMD_WRITE:    handle_write(payload, len);
          break;
        case BOOT_CMD_COMMIT:   handle_commit(payload, len, &slip_a, &slip_b);
          break;
        case BOOT_CMD_RESET:    handle_reset();
          break;
        default:                nak(BOOT_STATUS_BAD_CMD);
          break;
      }
    } else if (result == BOOT_FRAME_BAD_CRC) {
      nak(BOOT_STATUS_BAD_CRC);
    } else if (result == BOOT_FRAME_BAD_LEN) {
      nak(BOOT_STATUS_BAD_LEN);
    }
  }
}