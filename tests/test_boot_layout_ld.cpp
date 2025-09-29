/* Tests that the bootloader's linker script agrees with boot/layout.h (NFR-2.9).
 *
 * The layout header is the single authority for the flash map, but a linker script
 * cannot include a C header, so the bootloader's script carries the bootloader's
 * ORIGIN and LENGTH as literals. This test reads the script as text and checks its
 * FLASH region against the header's BOOT_BOOTLOADER_BASE and BOOT_BOOTLOADER_SIZE —
 * the same class of drift that the application script's _estack bug was, caught at
 * test time instead of on a board. The application script gets the same check when
 * it is moved to the slot base (Lesson 5).
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

TEST(BootLayoutTest, BootLinkerScriptMatchesTheLayoutHeader) {
  const std::string path =
      std::string(NODE_SOURCE_DIR) + "/firmware/boot/stm32f103c8t6_boot.ld";
  std::ifstream in(path);
  ASSERT_TRUE(in.is_open()) << "cannot open " << path;

  std::string line;
  bool found_flash = false;
  uint32_t flash_origin = 0u;
  uint32_t flash_length = 0u;
  while (std::getline(in, line)) {
    if (line.find("FLASH (rx)") == std::string::npos) {
      continue;
    }
    const size_t origin_at = line.find("ORIGIN = 0x");
    ASSERT_NE(origin_at, std::string::npos) << "no ORIGIN on the FLASH line: " << line;
    /* "ORIGIN = 0x" is eleven characters; the eight hex digits follow. */
    flash_origin = static_cast<uint32_t>(
        std::stoul(line.substr(origin_at + 11, 8), nullptr, 16));

    const size_t length_at = line.find("LENGTH = ");
    ASSERT_NE(length_at, std::string::npos) << "no LENGTH on the FLASH line: " << line;
    flash_length = parse_length(line.substr(length_at + 9));
    found_flash = true;
    break;
  }
  ASSERT_TRUE(found_flash) << "no 'FLASH (rx)' line in " << path;

  EXPECT_EQ(flash_origin, BOOT_BOOTLOADER_BASE)
      << "bootloader linker script and layout.h disagree on the bootloader's origin";
  EXPECT_EQ(flash_length, BOOT_BOOTLOADER_SIZE)
      << "bootloader linker script and layout.h disagree on the bootloader's size";
}

} // namespace
} // namespace node