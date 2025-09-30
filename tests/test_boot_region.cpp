/* Tests for boot/region.h — the writable-region rule that keeps a host frame from ever
 * reaching the bootloader's own pages (the boot-0 rule's teeth).
 */
#include <gtest/gtest.h>

#include <cstdint>

#include "boot/layout.h"
#include "boot/region.h"

namespace node {
namespace {

TEST(BootRegionTest, BothSlotsAreWritable) {
  EXPECT_TRUE(boot_region_writable(BOOT_SLOT_A_BASE, 1u));
  EXPECT_TRUE(boot_region_writable(BOOT_SLOT_B_BASE, BOOT_SLOT_SIZE));
  // A single byte at the last address of a slot is inside it.
  EXPECT_TRUE(boot_region_writable(BOOT_SLOT_A_BASE + BOOT_SLOT_SIZE - 1u, 1u));
}

TEST(BootRegionTest, TheBootloaderRegionIsNeverWritable) {
  // The ruling case: a hostile frame must never erase the program answering it.
  EXPECT_FALSE(boot_region_writable(BOOT_BOOTLOADER_BASE, 1u));
  EXPECT_FALSE(boot_region_writable(BOOT_BOOTLOADER_BASE + BOOT_BOOTLOADER_SIZE - 1u, 1u));
  EXPECT_FALSE(boot_region_writable(BOOT_BOOTLOADER_BASE, BOOT_BOOTLOADER_SIZE));
}

TEST(BootRegionTest, MetadataAndSpareAreNotWritable) {
  EXPECT_FALSE(boot_region_writable(BOOT_METADATA_A_BASE, BOOT_METADATA_SIZE));
  EXPECT_FALSE(boot_region_writable(BOOT_METADATA_B_BASE, 1u));
  EXPECT_FALSE(boot_region_writable(BOOT_SPARE_BASE, BOOT_SPARE_SIZE));
}

TEST(BootRegionTest, LengthsCrossingABoundaryAreRefused) {
  // A length that starts inside a slot but spills past it must be refused whole.
  EXPECT_FALSE(boot_region_writable(BOOT_SLOT_A_BASE + BOOT_SLOT_SIZE - 1u, 2u));
  EXPECT_FALSE(boot_region_writable(BOOT_SLOT_B_BASE + BOOT_SLOT_SIZE - 1u, BOOT_SLOT_SIZE));
}

TEST(BootRegionTest, OutsideTheMapIsRefused) {
  EXPECT_FALSE(boot_region_writable(0u, 1u));
  EXPECT_FALSE(boot_region_writable(0x40000000u, 1u));          /* peripheral space */
  EXPECT_FALSE(boot_region_writable(BOOT_FLASH_END, 1u));       /* past the end */
}

TEST(BootRegionTest, ZeroLengthIsRefused) {
  EXPECT_FALSE(boot_region_writable(BOOT_SLOT_A_BASE, 0u));
}

} // namespace
} // namespace node