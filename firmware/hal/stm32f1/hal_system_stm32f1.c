/* hal_system_stm32f1.c — reset cause and the boot counter (NFR-0.7).
 *
 * NFR-0.7 is verified by running the node unattended for 72 h and checking that the
 * reset counter never moved. That makes this file the *instrument*, so both halves of it
 * are written to be honest about what they can and cannot see:
 *
 *   - The cause comes from the RCC reset flags, which the hardware sets and which are
 *     cleared by a single write. They are sampled once, at the very start of
 *     hal_stm32f1_init(), and cached, because reading them a second time can only
 *     return nothing.
 *
 *   - The count lives in a backup register, which survives a watchdog or software reset
 *     and is lost when the backup domain loses power. On a board whose VBAT is tied to
 *     VDD (the dev board, and the Phase 3 board as specified) that means a power cycle
 *     resets the count to 1 — which is exactly the property the requirement wants: a
 *     freshly powered board reads 1, and a node that resets itself while running does
 *     not.
 */

#include "hal_stm32f1.h"

// The update marker contract shared with the bootloader (Phase 2): the value and the
// backup register number, both defined once so the two images cannot disagree.
#include "boot/update.h"

#include "stm32f1xx_hal.h"

/* BKP_DR1 holds the count, BKP_DR2 the marker that says DR1 is meaningful. The marker
 * exists because the backup registers have no defined value at power-on: without it, a
 * first boot could read 0x0000, 0xFFFF or anything else and report it as history.
 * DR1 is 16-bit, so the count saturates and restarts rather than wrapping silently. */
#define NODE_BOOT_COUNT_REG BKP->DR1
#define NODE_BOOT_MARKER_REG BKP->DR2
#define NODE_BOOT_MARKER 0xB007U

static hal_reset_cause_t s_reset_cause = HAL_RESET_UNKNOWN;
static uint32_t s_boot_count = 0U;

void hal_system_stm32f1_sample_reset_cause(void) {
  /* One read, cleared immediately: the flags are sticky until RMVF is written, and
   * leaving them set would let a later reader attribute an old reset to this boot. */
  const uint32_t csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Order matters, because several flags can be set by one event.
   *
   * A watchdog reset sets only IWDGRSTF, and a software reset only SFTRSTF, so those are
   * unambiguous and are tested first: for NFR-0.7 they are the two that matter. A
   * power-on asserts PORRSTF *and* typically PINRSTF (the internal POR pulls NRST low),
   * so POWER_ON is tested before EXTERNAL_PIN — testing the pin flag first would label
   * every power-up as a button press, which is the one mistake that would make an
   * unattended-run report unreadable.
   */
  if ((csr & RCC_CSR_IWDGRSTF) != 0U) {
    s_reset_cause = HAL_RESET_WATCHDOG;
  } else if ((csr & RCC_CSR_SFTRSTF) != 0U) {
    s_reset_cause = HAL_RESET_SOFTWARE;
  } else if ((csr & RCC_CSR_PORRSTF) != 0U) {
    s_reset_cause = HAL_RESET_POWER_ON;
  } else if ((csr & RCC_CSR_PINRSTF) != 0U) {
    s_reset_cause = HAL_RESET_EXTERNAL_PIN;
  } else if ((csr & RCC_CSR_LPWRRSTF) != 0U) {
    s_reset_cause = HAL_RESET_LOW_POWER;
  } else {
    /* No flag set: a debugger-initiated reset, or the flags were cleared by a
     * bootloader running before this image. Unknown is the honest answer. */
    s_reset_cause = HAL_RESET_UNKNOWN;
  }
}

void hal_system_stm32f1_count_boot(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();

  /* The backup domain is write-protected by PWR_CR.DBP. Raising it here and lowering it
   * at the end keeps the window during which a stray write could corrupt the counter as
   * small as the counter itself. */
  HAL_PWR_EnableBkUpAccess();

  uint32_t count;

  if ((NODE_BOOT_MARKER_REG & 0xFFFFU) != NODE_BOOT_MARKER) {
    /* No marker: the backup domain lost power (or has never been written), so this is
     * the first boot of a freshly powered board and the count is 1 by definition. */
    count = 1U;
    NODE_BOOT_MARKER_REG = NODE_BOOT_MARKER;
  } else {
    count = (NODE_BOOT_COUNT_REG & 0xFFFFU) + 1U;
    if (count > 0xFFFFU) {
      /* Saturated at 16 bits. Restarting at 1 loses history but never wraps to a value
       * that would be mistaken for a fresh boot. */
      count = 1U;
    }
  }

  NODE_BOOT_COUNT_REG = (uint16_t)count;
  HAL_PWR_DisableBkUpAccess();

  s_boot_count = count;
}

static hal_reset_cause_t system_reset_cause(void *ctx) {
  (void)ctx;
  return s_reset_cause;
}

static uint32_t system_boot_count(void *ctx) {
  (void)ctx;
  return s_boot_count;
}

static bool system_update_request(void *ctx) {
  (void)ctx;
  /* The same backup-domain dance as count_boot(): clocks first, then the write-protect
   * lift, then the register. The marker is the one the bootloader reads at every boot
   * (boot/update.h); writing it is the whole request, and the caller decides when to
   * reset so it can say goodbye on the wire first. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  BKP->DR3 = BOOT_UPDATE_MARKER;

  HAL_PWR_DisableBkUpAccess();
  return true;
}

static void system_reset(void *ctx) {
  (void)ctx;
  NVIC_SystemReset();
  for (;;) { /* not reached */
  }
}

static hal_system_iface_t s_system = {
    .ctx = NULL,
    .reset_cause = system_reset_cause,
    .boot_count = system_boot_count,
    .update_request = system_update_request,
    .system_reset = system_reset,
};

hal_system_iface_t *hal_system_stm32f1(void) {
  return &s_system;
}
