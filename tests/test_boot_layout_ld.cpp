/* Tests that the linker scripts agree with boot/layout.h (NFR-2.9).
 *
 * The layout header is the single authority for the flash map, but a linker script
 * cannot include a C header, so each script carries its region as literals and this
 * test reads them back as text and checks them against the header — the same class of
 * drift that the application script's _estack bug was, caught at test time instead of
 * on a board. Three scripts are checked:
 *
 *   - boot/stm32f103c8t6_boot.ld      the bootloader's own 8 KB home
 *   - stm32f103c8t6_slot.ld           the application as installed in a slot (the
 *                                     update artifact)
 *   - stm32f103c8t6.ld                the application at the flash base (direct SWD
 *                                     flashing, bring-up)
 *
 * The two application scripts must also define _app_base = ORIGIN(FLASH): main_target
 * sets SCB->VTOR against it (FR-2.3), so a script that forgets the symbol would leave
 * the app with a relocated promise and no table movement.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <string>

#include "boot/layout.h"

namespace node {
namespace {

uint32_t parse_length(const std::string &raw) {
  std::string t = raw;
  while (!t.empty() && (t.back() == ' ' || t.back() == '\r' || t.back() == '\n')) {
    t.pop_back();
  }
  if (!t.empty() && (t.back() == 'K' || t.back() == 'k')) {
    const std::string number = t.substr(0, t.size() - 1);
    return static_cast<uint32_t>(std::stoul(number, nullptr, 10) * 1024u);
  }
  if (t.size() >= 2 && t.compare(0, 2, "0x") == 0) {
    return static_cast<uint32_t>(std::stoul(t, nullptr, 16));
  }
  return static_cast<uint32_t>(std::stoul(t, nullptr, 10));
}

// The FLASH region of one script: the ORIGIN and LENGTH from its "FLASH (rx)" line.
struct FlashRegion {
  uint32_t origin;
  uint32_t length;
};

FlashRegion read_flash_region(const std::string &path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    ADD_FAILURE() << "cannot open " << path;
    return {0u, 0u};
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("FLASH (rx)") == std::string::npos) {
      continue;
    }
    const size_t origin_at = line.find("ORIGIN = 0x");
    EXPECT_NE(origin_at, std::string::npos) << "no ORIGIN on the FLASH line: " << line;
    // "ORIGIN = 0x" is eleven characters; the eight hex digits follow.
    const uint32_t origin = static_cast<uint32_t>(
        std::stoul(line.substr(origin_at + 11, 8), nullptr, 16));
    const size_t length_at = line.find("LENGTH = ");
    EXPECT_NE(length_at, std::string::npos) << "no LENGTH on the FLASH line: " << line;
    return {origin, parse_length(line.substr(length_at + 9))};
  }
  ADD_FAILURE() << "no 'FLASH (rx)' line in " << path;
  return {0u, 0u};
}

bool script_defines(const std::string &path, const std::string &needle) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

TEST(BootLayoutTest, BootLinkerScriptMatchesTheLayoutHeader) {
  const FlashRegion region =
      read_flash_region(std::string(NODE_SOURCE_DIR) + "/firmware/boot/stm32f103c8t6_boot.ld");
  EXPECT_EQ(region.origin, BOOT_BOOTLOADER_BASE);
  EXPECT_EQ(region.length, BOOT_BOOTLOADER_SIZE);
}

TEST(BootLayoutTest, SlotLinkerScriptMatchesTheLayoutHeader) {
  const FlashRegion region =
      read_flash_region(std::string(NODE_SOURCE_DIR) + "/firmware/stm32f103c8t6_slot.ld");
  EXPECT_EQ(region.origin, BOOT_SLOT_A_BASE)
      << "the slot image is linked for slot A's base; the bootloader reads its vector "
         "table from wherever the metadata points, so the app must be linked there";
  EXPECT_EQ(region.length, BOOT_SLOT_SIZE);
}

TEST(BootLayoutTest, BaseLinkerScriptStartsAtTheFlashBase) {
  const FlashRegion region =
      read_flash_region(std::string(NODE_SOURCE_DIR) + "/firmware/stm32f103c8t6.ld");
  EXPECT_EQ(region.origin, BOOT_FLASH_BASE);
  // The direct-flash build keeps its datasheet-bounded 64 KB (the pre-bootloader home);
  // only the slot build is bounded by BOOT_SLOT_SIZE, which the test above checks.
}

TEST(BootLayoutTest, BothAppScriptsDefineAppBaseForTheVtorRelocation) {
  const std::string slot = std::string(NODE_SOURCE_DIR) + "/firmware/stm32f103c8t6_slot.ld";
  const std::string base = std::string(NODE_SOURCE_DIR) + "/firmware/stm32f103c8t6.ld";
  EXPECT_TRUE(script_defines(slot, "_app_base = ORIGIN(FLASH)"))
      << "main_target.cpp sets SCB->VTOR against _app_base (FR-2.3)";
  EXPECT_TRUE(script_defines(base, "_app_base = ORIGIN(FLASH)"));
}

} // namespace
} // namespace node