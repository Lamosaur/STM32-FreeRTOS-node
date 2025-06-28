/* Tests for the display module (FR-0.5).
 *
 * FR-0.5 says display updates must use DMA and partial refresh rather than blocking
 * full-frame writes, and gives the reason: a full 128x64 frame is ~23 ms of bus time on
 * a bus the sensor task also needs. So there are two claims to falsify here, not one:
 *
 *   1. The bytes. A flush must be proportional to what changed, and a repaint of
 *      identical content must cost nothing at all. The tests below measure bytes on the
 *      fake bus and compare them against Ssd1306::bytes_for(), so the partial-versus-
 *      full argument is a number from the real code path rather than a comment.
 *
 *   2. The path. Those bytes must go out over the arbiter's DMA transfer, and never
 *      over the polled one. The fake bus counts the two separately, which is what makes
 *      that checkable on the host instead of only on a scope.
 *
 * The font gets its own tests because a wrong glyph table is invisible in review and
 * looks entirely plausible on a panel: two glyphs are pinned pixel by pixel against
 * ASCII art, the 6 px advance and the case folding are asserted, and the box fallback
 * for an uncovered character is asserted to be a box rather than a blank.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "app/board.h"
#include "app/display.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "support/fake_i2c.h"
#include "support/fake_rtos.h"

namespace node {
namespace {

using test::BusOp;
using test::FakeI2c;
using test::FakeRtos;

class DisplayTest : public ::testing::Test {
 protected:
  FakeI2c bus;
  FakeRtos rtos;
  I2cRecovery recovery{bus.iface()};
  I2cArbiter arbiter{bus.iface(), rtos.iface(), &recovery};
  Ssd1306 panel{&arbiter};
};

constexpr Rect kFullPanel{0, 0, kDisplayWidth, kDisplayHeight};

// --- helpers ---------------------------------------------------------------

// Rect has no operator==, and adding one would be a change to the frozen header, so
// every rectangle assertion compares the four fields.
void expect_rect(const Rect &actual, const Rect &expected) {
  EXPECT_EQ(actual.x0, expected.x0);
  EXPECT_EQ(actual.y0, expected.y0);
  EXPECT_EQ(actual.x1, expected.x1);
  EXPECT_EQ(actual.y1, expected.y1);
}

// A region of the framebuffer as ASCII art, so a font assertion can be read against the
// bitmap it is checking instead of against a wall of hex.
std::vector<std::string> art(const Framebuffer &fb, uint16_t x, uint16_t y, uint16_t w,
                             uint16_t h) {
  std::vector<std::string> rows;
  for (uint16_t row = 0; row < h; ++row) {
    std::string line;
    for (uint16_t col = 0; col < w; ++col) {
      line.push_back(fb.pixel(static_cast<uint16_t>(x + col), static_cast<uint16_t>(y + row))
                         ? '#'
                         : '.');
    }
    rows.push_back(line);
  }
  return rows;
}

// Lit pixels across the whole framebuffer.
int pixel_count(const Framebuffer &fb) {
  int n = 0;
  for (size_t i = 0; i < Framebuffer::kBytes; ++i) {
    for (int bit = 0; bit < 8; ++bit) {
      if ((fb.bytes()[i] & static_cast<uint8_t>(1u << bit)) != 0) {
        ++n;
      }
    }
  }
  return n;
}

// Non-zero bytes across the whole framebuffer: a narrower check than pixel_count, and
// the one that catches a stray bit in an otherwise untouched byte.
int lit_byte_count(const Framebuffer &fb) {
  int n = 0;
  for (size_t i = 0; i < Framebuffer::kBytes; ++i) {
    if (fb.bytes()[i] != 0) {
      ++n;
    }
  }
  return n;
}

bool same_frame(const Framebuffer &a, const Framebuffer &b) {
  return std::equal(a.bytes(), a.bytes() + Framebuffer::kBytes, b.bytes());
}

// Payload length of every DMA write, in order. last_tx() only exposes the most recent
// transaction, which hides the per-page split of a multi-page flush.
std::vector<size_t> dma_lengths(const FakeI2c &fake) {
  std::vector<size_t> lengths;
  for (const test::BusLogEntry &entry : fake.log()) {
    if (entry.op == BusOp::TransferDma) {
      lengths.push_back(static_cast<size_t>(entry.arg));
    }
  }
  return lengths;
}

std::vector<uint16_t> dma_addrs(const FakeI2c &fake) {
  std::vector<uint16_t> addrs;
  for (const test::BusLogEntry &entry : fake.log()) {
    if (entry.op == BusOp::TransferDma) {
      addrs.push_back(entry.addr);
    }
  }
  return addrs;
}

// ---------------------------------------------------------------------------
// Rect
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, DefaultRectIsEmpty) {
  // The canonical "nothing has changed" value. Half-open bounds are what make a
  // default-constructed rectangle the empty one, which is why a zero-initialised
  // DamageTracker is already correct rather than a special case.
  const Rect none{};
  EXPECT_TRUE(none.empty());
  EXPECT_EQ(none.width(), 0);
  EXPECT_EQ(none.height(), 0);
  expect_rect(Rect::none(), none);
  EXPECT_TRUE(none.clamp_to_panel().empty());

  // A rectangle that is empty on one axis only is empty too.
  EXPECT_TRUE((Rect{5, 5, 5, 20}).empty());
  EXPECT_TRUE((Rect{5, 20, 20, 20}).empty());
  EXPECT_TRUE((Rect{5, 20, 5, 5}).empty()); // degenerate on both axes
}

TEST_F(DisplayTest, FromCornersNormalisesReversedCorners) {
  const Rect forward = Rect::from_corners(4, 2, 10, 8);
  const Rect reversed = Rect::from_corners(10, 8, 4, 2);
  const Rect mixed = Rect::from_corners(4, 8, 10, 2);

  expect_rect(forward, Rect{4, 2, 10, 8});
  expect_rect(reversed, Rect{4, 2, 10, 8});
  expect_rect(mixed, Rect{4, 2, 10, 8});
  EXPECT_EQ(forward.width(), 6);
  EXPECT_EQ(forward.height(), 6);

  // Two identical corners are the empty rectangle, not a one-pixel one: that is what
  // half-open bounds mean by the same corner twice, and Rect::pixel() is how a caller
  // asks for exactly one pixel.
  EXPECT_TRUE(Rect::from_corners(5, 5, 5, 5).empty());
}

TEST_F(DisplayTest, PixelIsExactlyOnePixel) {
  const Rect p = Rect::pixel(3, 5);
  EXPECT_FALSE(p.empty());
  EXPECT_EQ(p.width(), 1);
  EXPECT_EQ(p.height(), 1);
  EXPECT_TRUE(p.contains(3, 5));
  EXPECT_FALSE(p.contains(4, 5));
  EXPECT_FALSE(p.contains(3, 6));
}

TEST_F(DisplayTest, IncludeGrowsToCoverBothAndIgnoresEmpty) {
  Rect acc{10, 10, 20, 20};

  acc.include(Rect{15, 5, 18, 30});
  expect_rect(acc, Rect{10, 5, 20, 30});

  // An empty rectangle adds nothing, which is what lets damage accumulate without a
  // "first mark" special case at every call site.
  acc.include(Rect{});
  acc.include(Rect::none());
  acc.include(Rect{12, 12, 12, 12});
  expect_rect(acc, Rect{10, 5, 20, 30});

  // Including into an empty accumulator adopts the other rectangle wholesale.
  Rect fresh;
  fresh.include(Rect{7, 7, 9, 9});
  expect_rect(fresh, Rect{7, 7, 9, 9});
}

TEST_F(DisplayTest, IncludePointGrowsTheBoxOnePixelAtATime) {
  Rect acc{10, 10, 20, 20};

  acc.include_point(15, 15); // inside: no growth
  expect_rect(acc, Rect{10, 10, 20, 20});

  acc.include_point(5, 30);
  expect_rect(acc, Rect{5, 10, 20, 31});
  EXPECT_TRUE(acc.contains(5, 30));

  Rect fresh;
  fresh.include_point(2, 3);
  expect_rect(fresh, Rect{2, 3, 3, 4});
}

TEST_F(DisplayTest, ContainsIsHalfOpen) {
  const Rect r{0, 0, 4, 4};
  EXPECT_TRUE(r.contains(0, 0));
  EXPECT_TRUE(r.contains(3, 3));  // the last pixel inside
  EXPECT_FALSE(r.contains(4, 4)); // the far edges are outside
  EXPECT_FALSE(r.contains(4, 0));
  EXPECT_FALSE(r.contains(0, 4));

  const Rect empty{};
  EXPECT_FALSE(empty.contains(0, 0));
}

TEST_F(DisplayTest, IntersectsExcludesTouchingEdges) {
  const Rect a{0, 0, 4, 4};

  EXPECT_TRUE(a.intersects(a));
  EXPECT_TRUE(a.intersects(Rect{3, 3, 8, 8})); // one shared pixel
  EXPECT_TRUE(a.intersects(Rect{-0, 0, 4, 4}));

  // Under half-open bounds a rectangle starting where the other ends shares no pixel,
  // so it must not intersect. The inclusive-bounds habit (<=) would say otherwise, and
  // would make a flush redraw a boundary column nothing had changed.
  EXPECT_FALSE(a.intersects(Rect{4, 0, 8, 4}));
  EXPECT_FALSE(a.intersects(Rect{0, 4, 4, 8}));
  EXPECT_FALSE(a.intersects(Rect{4, 4, 8, 8}));

  // Nothing intersects an empty rectangle, itself included.
  EXPECT_FALSE(a.intersects(Rect{}));
  EXPECT_FALSE(Rect{}.intersects(a));
  EXPECT_FALSE(Rect{}.intersects(Rect{}));
}

TEST_F(DisplayTest, ClampToPanel) {
  // A rectangle entirely off the panel collapses to empty. Clamping each edge
  // independently is what makes the off-the-right and off-the-bottom cases fall out
  // with no separate branch.
  EXPECT_TRUE((Rect{200, 200, 300, 300}).clamp_to_panel().empty());
  EXPECT_TRUE((Rect{130, 0, 131, 8}).clamp_to_panel().empty());
  EXPECT_TRUE((Rect{0, 70, 8, 100}).clamp_to_panel().empty());
  EXPECT_TRUE((Rect{128, 64, 128, 64}).clamp_to_panel().empty());

  // Partly off the panel: the visible part survives, nothing grows.
  expect_rect((Rect{120, 50, 200, 100}).clamp_to_panel(), Rect{120, 50, 128, 64});
  expect_rect((Rect{0, 0, 10, 80}).clamp_to_panel(), Rect{0, 0, 10, 64});

  // Oversized: clamped to the whole panel.
  expect_rect((Rect{0, 0, 1000, 1000}).clamp_to_panel(), kFullPanel);

  // A rectangle already inside is returned unchanged.
  expect_rect((Rect{4, 4, 8, 8}).clamp_to_panel(), Rect{4, 4, 8, 8});
}

// ---------------------------------------------------------------------------
// Framebuffer
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, PageAndBitAddressingMatchesThePanel) {
  Framebuffer fb;

  // Byte index for page p, column c is p * 128 + c, and bit 0 is the top row of the
  // page. These four assertions are that sentence, checked against the specific bytes.
  EXPECT_FALSE(fb.pixel(0, 0));
  EXPECT_TRUE(fb.set_pixel(0, 0, true));
  EXPECT_EQ(fb.bytes()[0], 0x01);

  EXPECT_TRUE(fb.set_pixel(5, 3, true));
  EXPECT_EQ(fb.bytes()[5], 0x08); // page 0, column 5, bit 3

  EXPECT_TRUE(fb.set_pixel(0, 8, true));
  EXPECT_EQ(fb.bytes()[128], 0x01); // page 1 starts at byte 128

  EXPECT_TRUE(fb.set_pixel(127, 63, true));
  EXPECT_EQ(fb.bytes()[1023], 0x80); // last page, last column, bottom row of the page

  // Exactly four bytes hold a pixel: nothing was written to a neighbouring column or a
  // neighbouring page.
  EXPECT_EQ(lit_byte_count(fb), 4);
  EXPECT_EQ(pixel_count(fb), 4);

  // The accessors agree with the addressing.
  EXPECT_TRUE(fb.pixel(5, 3));
  EXPECT_FALSE(fb.pixel(5, 4));
  EXPECT_EQ(fb.byte_at(0, 5), 0x08);
  EXPECT_EQ(fb.byte_at(7, 127), 0x80);
}

TEST_F(DisplayTest, SetPixelReportsOnlyRealChanges) {
  Framebuffer fb;

  EXPECT_TRUE(fb.set_pixel(10, 20, true));
  EXPECT_FALSE(fb.set_pixel(10, 20, true)); // already set: no change
  EXPECT_TRUE(fb.set_pixel(10, 20, false));
  EXPECT_FALSE(fb.set_pixel(10, 20, false)); // already clear

  // This return value is the hook the whole dirty-rectangle design rests on: if it were
  // "true, I wrote something" the damage tracker would light up on every repaint and
  // the static screen would cost a full flush.
  EXPECT_EQ(pixel_count(fb), 0);

  // Out-of-panel writes change nothing, and report that they changed nothing.
  EXPECT_FALSE(fb.set_pixel(128, 0, true));
  EXPECT_FALSE(fb.set_pixel(0, 64, true));
  EXPECT_FALSE(fb.set_pixel(65535, 65535, true));
  EXPECT_FALSE(fb.pixel(128, 0));
  EXPECT_FALSE(fb.pixel(0, 64));
  EXPECT_EQ(pixel_count(fb), 0);
}

TEST_F(DisplayTest, FillAndClearCoverTheWholeFramebuffer) {
  Framebuffer fb;

  fb.fill(true);
  ASSERT_EQ(Framebuffer::kBytes, 1024u); // the FR-0.5 baseline: 128x64 at 1 bpp
  for (size_t i = 0; i < Framebuffer::kBytes; ++i) {
    ASSERT_EQ(fb.bytes()[i], 0xFF) << "byte " << i;
  }
  EXPECT_EQ(pixel_count(fb), 128 * 64);

  fb.clear();
  for (size_t i = 0; i < Framebuffer::kBytes; ++i) {
    ASSERT_EQ(fb.bytes()[i], 0x00) << "byte " << i;
  }
}

TEST_F(DisplayTest, ByteAtAndSetByteRoundTrip) {
  Framebuffer fb;

  fb.set_byte(3, 100, 0xA5); // page 3 covers rows 24..31
  EXPECT_EQ(fb.byte_at(3, 100), 0xA5);
  EXPECT_EQ(fb.byte_at(3, 99), 0x00);
  EXPECT_EQ(fb.byte_at(2, 100), 0x00);
  EXPECT_EQ(lit_byte_count(fb), 1);

  // 0xA5 = 1010 0101, bit 0 is the top row of the page.
  EXPECT_TRUE(fb.pixel(100, 24));
  EXPECT_FALSE(fb.pixel(100, 25));
  EXPECT_TRUE(fb.pixel(100, 26));
  EXPECT_FALSE(fb.pixel(100, 27));
  EXPECT_TRUE(fb.pixel(100, 29));
  EXPECT_TRUE(fb.pixel(100, 31));

  // Out-of-range access is ignored rather than wrapped onto another page's byte: a wrap
  // would light a pixel somewhere else on the panel, which is worse than a no-op.
  EXPECT_EQ(fb.byte_at(8, 0), 0x00);
  EXPECT_EQ(fb.byte_at(0, 128), 0x00);
  fb.set_byte(8, 0, 0xFF);
  fb.set_byte(0, 128, 0xFF);
  EXPECT_EQ(lit_byte_count(fb), 1);
}

// ---------------------------------------------------------------------------
// Canvas and damage
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, DrawingMarksExactlyThePixelsItChanged) {
  Canvas canvas;
  EXPECT_TRUE(canvas.damage().empty());

  EXPECT_TRUE(canvas.set_pixel(7, 9, true));
  expect_rect(canvas.damage().bounds(), Rect{7, 9, 8, 10});

  EXPECT_TRUE(canvas.set_pixel(20, 30, true));
  expect_rect(canvas.damage().bounds(), Rect{7, 9, 21, 31});

  canvas.clear_damage();
  EXPECT_TRUE(canvas.damage().empty());
  EXPECT_TRUE(canvas.set_pixel(7, 9, false)); // clearing it is a change too
  expect_rect(canvas.damage().bounds(), Rect{7, 9, 8, 10});
}

TEST_F(DisplayTest, RedrawingIdenticalContentMarksNothing) {
  // The property that makes a static screen cost zero bus bytes. It is asserted at the
  // damage tracker, at the framebuffer, and then on the bus, because the claim FR-0.5
  // makes is about bus time.
  Canvas canvas;

  canvas.draw_text(0, 0, "NODE");
  canvas.draw_text(0, 8, "T 21.50 C");
  canvas.draw_text(0, 16, "RH 45.00 %");
  canvas.draw_text(0, 24, "RH OK");
  ASSERT_FALSE(canvas.damage().empty());

  const std::vector<uint8_t> drawn(canvas.framebuffer().bytes(),
                                   canvas.framebuffer().bytes() + Framebuffer::kBytes);
  const Rect first_damage = canvas.damage().bounds();
  canvas.clear_damage();
  ASSERT_TRUE(canvas.damage().empty());

  // The same four rows again, character for character: every pixel write is a no-op.
  canvas.draw_text(0, 0, "NODE");
  canvas.draw_text(0, 8, "T 21.50 C");
  canvas.draw_text(0, 16, "RH 45.00 %");
  canvas.draw_text(0, 24, "RH OK");

  EXPECT_TRUE(canvas.damage().empty());
  ASSERT_EQ(drawn.size(), Framebuffer::kBytes);
  EXPECT_TRUE(std::equal(drawn.begin(), drawn.end(), canvas.framebuffer().bytes()));
  // Rows 0..30 of the four text rows, columns 0..58: the longest line is ten glyphs of
  // six pixels, less the trailing spacing column.
  expect_rect(first_damage, Rect{0, 0, 59, 31});

  // ...and a flush of that empty damage puts nothing on the bus at all.
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  EXPECT_EQ(bus.dma_writes(), 0);
  EXPECT_EQ(bus.dma_bytes(), 0);
  EXPECT_EQ(panel.stats().skipped, 1u);
  EXPECT_EQ(panel.stats().bytes_sent, 0u);
  EXPECT_EQ(panel.stats().flushes, 0u);
  EXPECT_EQ(panel.stats().last_bytes, 0u);

  // One digit of real change is the whole difference: the same screen with a new value
  // marks damage again, and only inside that digit's cell — the sixth glyph of the row,
  // at columns 30..34 of rows 8..14.
  canvas.draw_text(0, 8, "T 21.60 C");
  EXPECT_FALSE(canvas.damage().empty());
  const Rect changed = canvas.damage().bounds();
  EXPECT_GE(changed.x0, 30);
  EXPECT_LE(changed.x1, 35);
  EXPECT_GE(changed.y0, 8);
  EXPECT_LE(changed.y1, 15);

  // And the flush of that one digit is priced by where it landed, not by the row.
  const size_t one_digit = Ssd1306::bytes_for(canvas.damage().bounds());
  EXPECT_EQ(one_digit, static_cast<size_t>(canvas.damage().bounds().width()) + 5u);
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(one_digit));
  EXPECT_EQ(panel.stats().flushes, 1u);
  EXPECT_EQ(panel.stats().skipped, 1u); // the earlier free flush is still on the record
}

TEST_F(DisplayTest, DrawingOverDifferentTextLeavesTheOldPixelsLit) {
  // Canvas has no erase: drawing "1" over "8" leaves every pixel the '8' had and the '1'
  // does not. Stated as a test because the opposite is easy to assume, and the display
  // task's repaint is written around it — it fills the band before redrawing, which is
  // what removes the previous, possibly longer, text.
  Canvas canvas;
  canvas.draw_text(0, 0, "8");
  canvas.clear_damage();
  canvas.draw_text(0, 0, "1");

  // '8' row 0 is 01110 and '1' row 0 is 00100, so columns 1 and 3 of the '8' survive it.
  EXPECT_TRUE(canvas.framebuffer().pixel(1, 0));
  EXPECT_TRUE(canvas.framebuffer().pixel(3, 0));
  EXPECT_TRUE(canvas.framebuffer().pixel(2, 0));
  // Only the pixels that went from off to on are marked: overdrawing cannot mark the
  // remnants, because nothing was written there.
  EXPECT_FALSE(canvas.damage().empty());

  // Clearing the cell first is what erases them, and then the '1' stands alone.
  canvas.fill_rect(Rect{0, 0, 6, 8}, false);
  canvas.draw_text(0, 0, "1");
  EXPECT_FALSE(canvas.framebuffer().pixel(1, 0));
  EXPECT_FALSE(canvas.framebuffer().pixel(3, 0));
  EXPECT_TRUE(canvas.framebuffer().pixel(2, 0));

  Canvas reference;
  reference.draw_text(0, 0, "1");
  EXPECT_TRUE(same_frame(canvas.framebuffer(), reference.framebuffer()));
}

TEST_F(DisplayTest, ClearingABandThenRedrawingItIsNotFree) {
  // The counterpart to the test above, and the reason the display task compares its
  // state before repainting instead of relying on the framebuffer to notice nothing
  // changed. Clearing a band that already holds text erases it, and the redraw puts it
  // back: two real changes, so real damage, even though the frame ends up identical.
  // The zero-cost static screen comes from skipping the repaint, not from making the
  // repaint free.
  Canvas canvas;
  canvas.draw_text(0, 0, "RH OK");

  const std::vector<uint8_t> before(canvas.framebuffer().bytes(),
                                    canvas.framebuffer().bytes() + Framebuffer::kBytes);
  canvas.clear_damage();

  canvas.fill_rect(Rect{0, 0, kDisplayWidth, 8}, false);
  canvas.draw_text(0, 0, "RH OK");

  EXPECT_FALSE(canvas.damage().empty());
  EXPECT_TRUE(std::equal(before.begin(), before.end(), canvas.framebuffer().bytes()));

  // Filling a band that is already blank, on the other hand, costs nothing at all.
  canvas.clear_damage();
  canvas.fill_rect(Rect{0, 32, kDisplayWidth, 40}, false);
  EXPECT_TRUE(canvas.damage().empty());
}

TEST_F(DisplayTest, ClippingMarksNoDamageForPixelsThatDoNotExist) {
  Canvas canvas;

  EXPECT_FALSE(canvas.set_pixel(kDisplayWidth, 0, true));
  EXPECT_FALSE(canvas.set_pixel(0, kDisplayHeight, true));
  EXPECT_TRUE(canvas.damage().empty());

  canvas.draw_text(200, 0, "NODE");  // entirely off the right edge
  canvas.draw_text(0, 64, "NODE");   // entirely below the panel
  canvas.draw_text(0, 0, "");        // nothing at all
  canvas.draw_text(0, 0, nullptr);   // and no text at all
  canvas.fill_rect(Rect{200, 200, 300, 300}, true);
  canvas.draw_rect(Rect{200, 200, 300, 300}, true);
  canvas.draw_hline(200, 400, 0, true);
  canvas.draw_vline(0, 200, 400, true);
  EXPECT_TRUE(canvas.damage().empty());

  // A write that straddles the edge marks only the part that exists.
  EXPECT_TRUE(canvas.set_pixel(127, 63, true));
  expect_rect(canvas.damage().bounds(), Rect{127, 63, 128, 64});

  // Nothing leaked into the framebuffer either.
  EXPECT_EQ(lit_byte_count(canvas.framebuffer()), 1);
}

TEST_F(DisplayTest, ClearMarksTheWholePanel) {
  Canvas canvas;
  ASSERT_TRUE(canvas.set_pixel(3, 3, true));
  ASSERT_TRUE(canvas.set_pixel(100, 40, true));

  canvas.clear();
  expect_rect(canvas.damage().bounds(), kFullPanel);
  expect_rect(canvas.damage().bounds(), Rect{0, 0, kDisplayWidth, kDisplayHeight});
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 0);

  // Clearing is marked unconditionally, even when the framebuffer was already black:
  // the caller asked for the panel to end up black, and a flush is cheap enough that
  // being right beats saving one comparison.
  canvas.clear_damage();
  canvas.clear();
  expect_rect(canvas.damage().bounds(), kFullPanel);
}

TEST_F(DisplayTest, FillRectUsesHalfOpenBoundsAndClips) {
  Canvas canvas;

  canvas.fill_rect(Rect{2, 2, 5, 5}, true);
  expect_rect(canvas.damage().bounds(), Rect{2, 2, 5, 5});
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 9); // 3x3: the far edges are outside
  EXPECT_TRUE(canvas.framebuffer().pixel(2, 2));
  EXPECT_TRUE(canvas.framebuffer().pixel(4, 4));
  EXPECT_FALSE(canvas.framebuffer().pixel(5, 5));
  EXPECT_FALSE(canvas.framebuffer().pixel(2, 5));
  EXPECT_FALSE(canvas.framebuffer().pixel(5, 2));

  // Filling with black over the same region is also a change, so it also marks damage.
  canvas.clear_damage();
  canvas.fill_rect(Rect{2, 2, 5, 5}, false);
  expect_rect(canvas.damage().bounds(), Rect{2, 2, 5, 5});
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 0);

  // A region hanging off the corner is clipped, and the damage is the clipped part.
  canvas.clear_damage();
  canvas.fill_rect(Rect{120, 60, 200, 100}, true);
  expect_rect(canvas.damage().bounds(), Rect{120, 60, 128, 64});
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 8 * 4);

  // An empty rectangle is a no-op, and marks nothing.
  canvas.clear_damage();
  canvas.fill_rect(Rect{}, true);
  canvas.fill_rect(Rect{9, 9, 9, 20}, true);
  EXPECT_TRUE(canvas.damage().empty());
}

TEST_F(DisplayTest, DrawRectDrawsAnOutlineOnly) {
  Canvas canvas;

  canvas.draw_rect(Rect{1, 1, 6, 6}, true);
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 16); // 5x5 outline, 3x3 interior empty
  EXPECT_TRUE(canvas.framebuffer().pixel(1, 1));
  EXPECT_TRUE(canvas.framebuffer().pixel(5, 5));
  EXPECT_TRUE(canvas.framebuffer().pixel(3, 1));
  EXPECT_TRUE(canvas.framebuffer().pixel(1, 3));
  EXPECT_FALSE(canvas.framebuffer().pixel(3, 3));
  expect_rect(canvas.damage().bounds(), Rect{1, 1, 6, 6});

  // A one-pixel-wide rectangle is all edge, and must not be drawn twice into a thicker
  // shape or skip a row.
  Canvas thin;
  thin.draw_rect(Rect{3, 3, 4, 10}, true);
  EXPECT_EQ(pixel_count(thin.framebuffer()), 7);
  expect_rect(thin.damage().bounds(), Rect{3, 3, 4, 10});

  // A clipped outline draws only its visible edges. The clipped region is 3x4, which is
  // narrow enough that its vertical edges are its top and bottom rows too: 3 + 3 for
  // those, plus 2 rows of two vertical pixels.
  Canvas clipped;
  clipped.draw_rect(Rect{125, 60, 140, 70}, true);
  expect_rect(clipped.damage().bounds(), Rect{125, 60, 128, 64});
  EXPECT_EQ(pixel_count(clipped.framebuffer()), 10);

  // Redrawing the same outline marks nothing.
  clipped.clear_damage();
  clipped.draw_rect(Rect{125, 60, 140, 70}, true);
  EXPECT_TRUE(clipped.damage().empty());
}

TEST_F(DisplayTest, DrawHlineAndVlineSpanAndClip) {
  Canvas canvas;

  canvas.draw_hline(2, 6, 3, true); // half-open: columns 2..5
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 4);
  EXPECT_TRUE(canvas.framebuffer().pixel(2, 3));
  EXPECT_TRUE(canvas.framebuffer().pixel(5, 3));
  EXPECT_FALSE(canvas.framebuffer().pixel(6, 3));
  expect_rect(canvas.damage().bounds(), Rect{2, 3, 6, 4});

  canvas.clear_damage();
  Canvas vertical;
  vertical.draw_vline(9, 1, 5, true); // rows 1..4
  EXPECT_EQ(pixel_count(vertical.framebuffer()), 4);
  EXPECT_FALSE(vertical.framebuffer().pixel(9, 5));
  expect_rect(vertical.damage().bounds(), Rect{9, 1, 10, 5});

  // Endpoints may arrive in either order, as with Rect::from_corners.
  Canvas reversed;
  reversed.draw_hline(6, 2, 3, true);
  reversed.draw_vline(9, 5, 1, true);
  EXPECT_EQ(pixel_count(reversed.framebuffer()), 8);

  // Clipping: the visible part is drawn, the rest is not marked.
  Canvas offscreen;
  offscreen.draw_hline(120, 400, 0, true);
  EXPECT_EQ(pixel_count(offscreen.framebuffer()), 8);
  expect_rect(offscreen.damage().bounds(), Rect{120, 0, 128, 1});
  offscreen.clear_damage();
  offscreen.draw_hline(0, 400, 1, true); // spans past the edge, clipped, not wrapped
  EXPECT_EQ(pixel_count(offscreen.framebuffer()), 8 + 128);
  expect_rect(offscreen.damage().bounds(), Rect{0, 1, 128, 2});
}

TEST_F(DisplayTest, ScatteredDamageDegradesToOneBoundingBox) {
  Canvas canvas;
  canvas.set_pixel(0, 0, true);
  canvas.set_pixel(127, 63, true);

  // The documented trade of a bounding box rather than a tile map: two opposite corners
  // cost a whole-panel flush. Worth stating as a test rather than leaving as a surprise,
  // because the number is unflattering and it is the honest cost of O(1) marking with
  // no allocation.
  expect_rect(canvas.damage().bounds(), kFullPanel);
  EXPECT_EQ(Ssd1306::bytes_for(canvas.damage().bounds()),
            Ssd1306::bytes_for(kFullPanel));
}

// ---------------------------------------------------------------------------
// Canvas::draw_text — the font itself
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, DigitTwoMatchesTheFontTable) {
  Canvas canvas;
  canvas.draw_text(0, 0, "2");

  // Pinned pixel by pixel: a font table that is subtly wrong renders plausible garbage,
  // and only an exact comparison catches the difference.
  const std::vector<std::string> expected = {
      ".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"};
  EXPECT_EQ(art(canvas.framebuffer(), 0, 0, 5, 7), expected);
  expect_rect(canvas.damage().bounds(), Rect{0, 0, 5, 7});
}

TEST_F(DisplayTest, LetterOMatchesTheFontTable) {
  Canvas canvas;
  canvas.draw_text(3, 10, "O");

  const std::vector<std::string> expected = {
      ".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."};
  EXPECT_EQ(art(canvas.framebuffer(), 3, 10, 5, 7), expected);
  expect_rect(canvas.damage().bounds(), Rect{3, 10, 8, 17});

  // 'O' and '0' must not be the same glyph: on a panel that reports a reading, the
  // difference between a letter and a digit is the difference between a label and a
  // number.
  Canvas zero;
  zero.draw_text(0, 0, "0");
  EXPECT_NE(art(zero.framebuffer(), 0, 0, 5, 7), expected);
}

TEST_F(DisplayTest, GlyphAdvanceIsSixPixels) {
  Canvas single;
  single.draw_text(0, 0, "1");
  Canvas pair;
  pair.draw_text(0, 0, "11");

  // The second glyph is the first, shifted by exactly six columns...
  for (uint8_t row = 0; row < 7; ++row) {
    for (uint8_t col = 0; col < 5; ++col) {
      ASSERT_EQ(pair.framebuffer().pixel(static_cast<uint16_t>(6 + col), row),
                single.framebuffer().pixel(col, row))
          << "row " << static_cast<int>(row) << " col " << static_cast<int>(col);
    }
    // ...and column 5 is the spacing: no ink there for a glyph whose rightmost column is
    // empty.
    EXPECT_FALSE(pair.framebuffer().pixel(5, row));
  }
  EXPECT_EQ(pixel_count(pair.framebuffer()), 2 * pixel_count(single.framebuffer()));
  // The second glyph's rightmost ink is at column 9 ('1' reaches three columns in), so
  // the damage spans 1..9 across the two cells: a 6 px advance with a 5 px glyph.
  expect_rect(pair.damage().bounds(), Rect{1, 0, 10, 7});
}

TEST_F(DisplayTest, LowercaseFoldsToUppercase) {
  Canvas lower;
  lower.draw_text(0, 0, "node ok");
  Canvas upper;
  upper.draw_text(0, 0, "NODE OK");

  EXPECT_TRUE(same_frame(lower.framebuffer(), upper.framebuffer()));
  expect_rect(lower.damage().bounds(), upper.damage().bounds());
}

TEST_F(DisplayTest, UncoveredCharacterDrawsTheFilledBox) {
  // 'Q' is not in the table. A miss must be visible rather than silent: a blank glyph
  // and a legitimately empty value look identical on the panel.
  Canvas canvas;
  canvas.draw_text(0, 0, "Q");

  const std::vector<std::string> box = {
      "#####", "#####", "#####", "#####", "#####", "#####", "#####"};
  EXPECT_EQ(art(canvas.framebuffer(), 0, 0, 5, 7), box);
  EXPECT_EQ(pixel_count(canvas.framebuffer()), 35); // 5 x 7, filled

  // Lowercase folds first, so an uncovered lowercase letter boxes as its uppercase.
  Canvas lower;
  lower.draw_text(0, 0, "q");
  EXPECT_EQ(art(lower.framebuffer(), 0, 0, 5, 7), box);

  // Punctuation outside the set boxes as well.
  Canvas punct;
  punct.draw_text(0, 0, "@");
  EXPECT_EQ(art(punct.framebuffer(), 0, 0, 5, 7), box);

  // Space is covered, and renders as nothing at all — which is a different thing from an
  // uncovered character, and the reason the two cases are tested separately.
  Canvas space;
  space.draw_text(0, 0, " ");
  EXPECT_EQ(pixel_count(space.framebuffer()), 0);
  EXPECT_TRUE(space.damage().empty());
}

TEST_F(DisplayTest, EveryDocumentedCharacterHasARealGlyph) {
  // The declared character set, as documented in display.cpp. Every one of these must
  // render as something other than the 5x7 box, and none of them may be blank.
  const char *covered = " 0123456789.:-+%/ACDEHIKLMNOPRSTUW";

  for (const char *p = covered; *p != '\0'; ++p) {
    Canvas canvas;
    const char one[2] = {*p, '\0'};
    canvas.draw_text(0, 0, one);

    const int ink = pixel_count(canvas.framebuffer());
    if (*p == ' ') {
      EXPECT_EQ(ink, 0) << "space should render nothing";
    } else {
      EXPECT_GT(ink, 0) << "covered character '" << *p << "' rendered blank";
      EXPECT_NE(ink, 35) << "covered character '" << *p << "' rendered as the box";
    }
  }
}

TEST_F(DisplayTest, TextRunningOffTheRightEdgeIsClippedWithoutCorruption) {
  Canvas canvas;
  canvas.draw_text(120, 0, "NODE");

  // Only the first two glyphs fit: 'N' at 120..124 and the left two columns of 'O' at
  // 126..127. The remaining glyphs are past the edge and are not drawn at all.
  expect_rect(canvas.damage().bounds(), Rect{120, 0, 128, 7});

  // Nothing wrapped to column 0 of the same page, or to another page: every byte left of
  // the text is untouched. A byte-wise blit is what produces that corruption.
  for (uint8_t page = 0; page < Framebuffer::kPages; ++page) {
    for (uint16_t column = 0; column < 120; ++column) {
      ASSERT_EQ(canvas.framebuffer().byte_at(page, column), 0x00)
          << "page " << static_cast<int>(page) << " column " << column;
    }
  }
  for (uint8_t page = 1; page < Framebuffer::kPages; ++page) {
    for (uint16_t column = 120; column < kDisplayWidth; ++column) {
      ASSERT_EQ(canvas.framebuffer().byte_at(page, column), 0x00)
          << "page " << static_cast<int>(page) << " column " << column;
    }
  }

  // The partial glyph really is partial, not a squashed full glyph.
  const std::vector<std::string> visible = art(canvas.framebuffer(), 126, 0, 2, 7);
  EXPECT_EQ(visible, (std::vector<std::string>{".#", "#.", "#.", "#.", "#.", "#.", ".#"}));

  // And the flush of that clipped damage stays inside the panel.
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(Ssd1306::bytes_for(Rect{120, 0, 128, 7})));
}

TEST_F(DisplayTest, DrawU32RendersDecimalDigits) {
  const uint32_t values[] = {0u, 5u, 42u, 1000u, 65535u, 4294967295u};
  const char *expected[] = {"0", "5", "42", "1000", "65535", "4294967295"};

  for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    Canvas from_number;
    from_number.draw_u32(0, 0, values[i]);
    Canvas from_text;
    from_text.draw_text(0, 0, expected[i]);

    EXPECT_TRUE(same_frame(from_number.framebuffer(), from_text.framebuffer()))
        << "value " << values[i];
    expect_rect(from_number.damage().bounds(), from_text.damage().bounds());
  }

  // A readout drawn at a non-zero origin lands where asked, with no digits dropped.
  Canvas offset;
  offset.draw_u32(10, 24, 42);
  Canvas offset_text;
  offset_text.draw_text(10, 24, "42");
  EXPECT_TRUE(same_frame(offset.framebuffer(), offset_text.framebuffer()));
}

// ---------------------------------------------------------------------------
// Ssd1306 — the setup sequence
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, InitSendsThePanelSetupSequence) {
  ASSERT_TRUE(panel.init());

  const std::vector<uint8_t> tx = bus.last_tx();

  // The whole sequence, in order. One 0x00 control byte covers the entire run because
  // the panel stays in the mode the last control byte selected.
  const std::vector<uint8_t> expected = {
      0x00,       // the following bytes are commands
      0xAE,       // display off
      0xD5, 0x80, // clock divide / oscillator
      0xA8, 0x3F, // multiplex: 64 rows
      0xD3, 0x00, // display offset
      0x40,       // start line 0
      0x8D, 0x14, // charge pump enable
      0x20, 0x00, // horizontal addressing mode
      0xA1,       // segment remap
      0xC8,       // COM scan direction
      0xDA, 0x12, // COM pins for 128x64
      0x81, 0xCF, // contrast
      0xD9, 0xF1, // pre-charge
      0xDB, 0x40, // VCOMH deselect
      0xA4,       // follow RAM
      0xA6,       // normal display
      0xAF};      // display on
  EXPECT_EQ(tx, expected);

  // The significant ones called out individually, so a failure names the step rather
  // than printing two 26-byte vectors and leaving the reader to diff them.
  ASSERT_EQ(tx.size(), 26u);
  EXPECT_EQ(tx.front(), 0x00); // control byte: commands follow
  EXPECT_EQ(tx[1], 0xAE);      // display off first
  EXPECT_EQ(tx[9], 0x8D);      // charge pump command
  EXPECT_EQ(tx[10], 0x14);     // charge pump: enable
  EXPECT_EQ(tx[11], 0x20);     // memory addressing mode
  EXPECT_EQ(tx[12], 0x00);     // horizontal
  EXPECT_EQ(tx.back(), 0xAF);  // display on last

  // Sent to the panel's address, over the DMA path like every other write here.
  EXPECT_EQ(dma_addrs(bus), (std::vector<uint16_t>{kAddrSsd1306}));
  EXPECT_EQ(bus.dma_writes(), 1);
  EXPECT_EQ(bus.count(BusOp::Transfer), 0);

  // An init is not a flush, so it does not touch the flush counters.
  EXPECT_EQ(panel.stats().flushes, 0u);
  EXPECT_EQ(panel.stats().full_flushes, 0u);
  EXPECT_EQ(panel.stats().bytes_sent, 0u);
  EXPECT_EQ(panel.stats().failed, 0u);
}

TEST_F(DisplayTest, InitFailureIsReported) {
  bus.set_transfer_status(HAL_I2C_NACK);

  EXPECT_FALSE(panel.init());

  // Reported through the return value, not through `failed`, which counts rejected
  // flushes: folding an init into it would make that counter mean two things.
  EXPECT_EQ(panel.stats().failed, 0u);

  // A retry after the panel answers works.
  bus.set_transfer_status(HAL_I2C_OK);
  EXPECT_TRUE(panel.init());
}

// ---------------------------------------------------------------------------
// Ssd1306 — partial refresh
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, Fr05PartialUpdateIsAFractionOfAFullFrame) {
  // The FR-0.5 headline, measured rather than asserted in prose. An 8x8 update touches
  // one page and eight columns, so the payload is 8 data bytes + 5 command/control
  // bytes = 13. A full frame is 8 pages of (128 + 5) = 1064. Ratio 13/1064 = 1.2%: the
  // partial update costs about 82 times fewer bytes, and on a 400 kHz bus about 1.2%
  // of the ~23 ms a full frame would take.
  //
  // Both numbers come from the shipping code path: bytes_for() for the prediction, and
  // the fake bus's byte counter for what actually went out.
  Canvas canvas;
  canvas.fill_rect(Rect{0, 0, 8, 8}, true);

  const size_t partial = Ssd1306::bytes_for(canvas.damage().bounds());
  const size_t full = Ssd1306::bytes_for(kFullPanel);
  EXPECT_EQ(partial, 13u);
  EXPECT_EQ(full, 1064u);

  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(partial));
  EXPECT_EQ(panel.stats().bytes_sent, static_cast<uint32_t>(partial));
  EXPECT_EQ(bus.dma_bytes(), 13);
  EXPECT_EQ(bus.dma_writes(), 1);

  ASSERT_TRUE(panel.flush_full(canvas.framebuffer()));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(full));
  EXPECT_EQ(panel.stats().bytes_sent, static_cast<uint32_t>(partial + full));
  EXPECT_EQ(bus.dma_bytes(), static_cast<int>(partial + full));
  EXPECT_EQ(bus.dma_writes(), 1 + static_cast<int>(kDisplayPages));

  const double ratio = static_cast<double>(partial) / static_cast<double>(full);
  EXPECT_LT(ratio, 0.02) << "8x8 update vs full frame: " << partial << "/" << full;
  RecordProperty("partial_bytes_8x8", std::to_string(partial));
  RecordProperty("full_frame_bytes", std::to_string(full));
  RecordProperty("ratio", std::to_string(ratio));

  EXPECT_EQ(panel.stats().flushes, 1u);
  EXPECT_EQ(panel.stats().partial_flushes, 1u);
  EXPECT_EQ(panel.stats().full_flushes, 1u);
  EXPECT_EQ(panel.stats().skipped, 0u);
  EXPECT_EQ(panel.stats().failed, 0u);
}

TEST_F(DisplayTest, BytesForAgreesWithWhatFlushActuallySends) {
  // bytes_for() is what makes the partial-versus-full argument checkable without a
  // scope, so it has to be exactly right — including for the cases where it predicts
  // nothing.
  Framebuffer fb;
  fb.fill(true);

  const Rect cases[] = {
      Rect{0, 0, 8, 8},         // one page
      Rect{0, 0, 128, 64},      // the whole panel
      Rect{5, 3, 9, 20},        // three pages, four columns
      Rect{120, 60, 128, 64},   // the bottom-right corner
      Rect{0, 56, 1, 64},       // the last page, one column
      Rect{200, 200, 300, 300}, // entirely off the panel
      Rect{130, 0, 140, 8},     // entirely off the right edge
      Rect{},                   // empty
  };

  size_t expected_total = 0;
  for (const Rect &damage : cases) {
    const size_t predicted = Ssd1306::bytes_for(damage);
    const uint32_t before = panel.stats().bytes_sent;

    ASSERT_TRUE(panel.flush(fb, damage));
    EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(predicted))
        << "damage {" << damage.x0 << "," << damage.y0 << "," << damage.x1 << ","
        << damage.y1 << "}";
    EXPECT_EQ(panel.stats().bytes_sent - before, static_cast<uint32_t>(predicted));
    expected_total += predicted;
  }

  EXPECT_EQ(bus.dma_bytes(), static_cast<int>(expected_total));
  EXPECT_EQ(panel.stats().bytes_sent, static_cast<uint32_t>(expected_total));

  // The two off-panel cases and the empty one were skipped, not sent.
  EXPECT_EQ(panel.stats().skipped, 3u);
  EXPECT_EQ(panel.stats().flushes, 5u);
}

TEST_F(DisplayTest, EmptyDamageSendsNothingAndIsSkipped) {
  Framebuffer fb;

  EXPECT_TRUE(panel.flush(fb, Rect{}));
  EXPECT_TRUE(panel.flush(fb, Rect::none()));

  EXPECT_EQ(bus.dma_writes(), 0);
  EXPECT_EQ(bus.dma_bytes(), 0);
  EXPECT_EQ(bus.count(BusOp::Transfer), 0);
  EXPECT_EQ(dma_lengths(bus).size(), 0u);

  const DisplayStats stats = panel.stats();
  EXPECT_EQ(stats.skipped, 2u);
  EXPECT_EQ(stats.flushes, 0u);
  EXPECT_EQ(stats.full_flushes, 0u);
  EXPECT_EQ(stats.partial_flushes, 0u);
  EXPECT_EQ(stats.bytes_sent, 0u);
  EXPECT_EQ(stats.last_bytes, 0u);
  EXPECT_EQ(stats.failed, 0u);

  // Returning true is deliberate: the panel already shows the right thing, so a caller
  // must not read this as a failure and retry forever.
  EXPECT_TRUE(Ssd1306::bytes_for(Rect{}) == 0u);
}

TEST_F(DisplayTest, OffPanelDamageSendsNothingAndIsSkipped) {
  Framebuffer fb;

  EXPECT_TRUE(panel.flush(fb, Rect{200, 200, 300, 300}));
  EXPECT_TRUE(panel.flush(fb, Rect{0, 64, 128, 200}));
  EXPECT_TRUE(panel.flush(fb, Rect{128, 0, 200, 64}));

  EXPECT_EQ(bus.dma_writes(), 0);
  EXPECT_EQ(bus.dma_bytes(), 0);
  EXPECT_EQ(panel.stats().skipped, 3u);
  EXPECT_EQ(panel.stats().flushes, 0u);
  EXPECT_EQ(Ssd1306::bytes_for(Rect{200, 200, 300, 300}), 0u);
}

TEST_F(DisplayTest, DamageHangingOffTheEdgeIsClampedNotSent) {
  Framebuffer fb;
  fb.fill(true);

  // A rectangle that extends past the panel must cost what the visible part costs, not
  // what the rectangle claims: the flush works on the clamped region.
  const Rect hanging{120, 50, 200, 100};
  const size_t clamped = Ssd1306::bytes_for(Rect{120, 50, 128, 64});
  EXPECT_EQ(clamped, 2u * (8u + 5u)); // pages 6 and 7, columns 120..127 == 26

  EXPECT_EQ(Ssd1306::bytes_for(hanging), clamped);
  ASSERT_TRUE(panel.flush(fb, hanging));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(clamped));
  EXPECT_EQ(bus.dma_bytes(), static_cast<int>(clamped));

  // Both transactions carry the clamped column range, and the last one names page 7.
  const std::vector<uint8_t> tx = bus.last_tx();
  ASSERT_EQ(tx.size(), 13u);
  EXPECT_EQ(tx[0], 0x00);
  EXPECT_EQ(tx[1], 0xB7);           // page 7
  EXPECT_EQ(tx[2], 0x08);           // low nibble of column 120
  EXPECT_EQ(tx[3], 0x17);           // high nibble: 0x10 | (120 >> 4)
  EXPECT_EQ(tx[4], 0x40);           // the following bytes are data
  for (uint16_t i = 0; i < 8; ++i) {
    EXPECT_EQ(tx[5 + i], fb.byte_at(7, static_cast<uint16_t>(120 + i)));
  }

  EXPECT_EQ(panel.stats().partial_flushes, 1u);
}

TEST_F(DisplayTest, PixelsOutsideTheDamagedRegionAreNeverSent) {
  // Two pages tall, eight columns wide. The payload must be exactly those two pages'
  // worth of columns, with no full-width row and no untouched page.
  Framebuffer fb;
  fb.fill(true);

  const Rect damage{8, 8, 16, 24}; // pages 1 and 2, columns 8..15
  const size_t predicted = Ssd1306::bytes_for(damage);
  EXPECT_EQ(predicted, 2u * (8u + 5u)); // 26

  ASSERT_TRUE(panel.flush(fb, damage));

  // One transaction per page, each carrying five overhead bytes and eight data bytes.
  EXPECT_EQ(dma_lengths(bus), (std::vector<size_t>{13u, 13u}));
  EXPECT_EQ(bus.dma_writes(), 2);
  EXPECT_EQ(bus.dma_bytes(), static_cast<int>(predicted));

  // The last transaction is page 2, columns 8..15, and its data is those eight bytes of
  // the framebuffer — the other 120 columns are not in it.
  const std::vector<uint8_t> tx = bus.last_tx();
  ASSERT_EQ(tx.size(), 13u);
  EXPECT_EQ(tx[1], 0xB2);
  for (uint16_t i = 0; i < 8; ++i) {
    EXPECT_EQ(tx[5 + i], fb.byte_at(2, static_cast<uint16_t>(8 + i)));
  }

  // The contrast with the alternative is the point: 26 bytes here against 1064 for the
  // frame that a blocking full write would have pushed.
  EXPECT_LT(predicted * 40u, Ssd1306::bytes_for(kFullPanel));
}

TEST_F(DisplayTest, MultiPageDamageEmitsOneTransactionPerPage) {
  Framebuffer fb;

  const Rect damage{0, 16, 128, 40}; // pages 2, 3 and 4, full width
  ASSERT_EQ(Ssd1306::bytes_for(damage), 3u * 133u); // 399

  ASSERT_TRUE(panel.flush(fb, damage));

  EXPECT_EQ(bus.dma_writes(), 3);
  EXPECT_EQ(dma_lengths(bus), (std::vector<size_t>{133u, 133u, 133u}));
  EXPECT_EQ(bus.dma_bytes(), 399);
  EXPECT_EQ(panel.stats().last_bytes, 399u);
  EXPECT_EQ(panel.stats().flushes, 1u);
  EXPECT_EQ(panel.stats().partial_flushes, 1u); // three of eight pages is partial

  // Full width means the column window starts at 0 and the data is the whole row.
  const std::vector<uint8_t> tx = bus.last_tx();
  EXPECT_EQ(tx[1], 0xB4); // the last transaction is page 4
  EXPECT_EQ(tx[2], 0x00);
  EXPECT_EQ(tx[3], 0x10);

  // Fewer pages than the panel is still a partial flush, and a whole-panel damage region
  // through flush() is not counted as partial.
  Framebuffer blank;
  ASSERT_TRUE(panel.flush(blank, kFullPanel));
  EXPECT_EQ(panel.stats().partial_flushes, 1u);
  EXPECT_EQ(panel.stats().flushes, 2u);
}

TEST_F(DisplayTest, PageAndColumnCommandsNameTheRegion) {
  Framebuffer fb;

  // One page at a time, so each transaction's page command can be checked: last_tx()
  // only exposes the most recent one, and the page address is what ties a transaction to
  // the rows it carries.
  for (uint8_t page = 0; page < Framebuffer::kPages; ++page) {
    const uint16_t y = static_cast<uint16_t>(page) * 8u;
    ASSERT_TRUE(panel.flush(fb, Rect{0, y, 1, static_cast<uint16_t>(y + 8u)}));
    const std::vector<uint8_t> tx = bus.last_tx();
    ASSERT_EQ(tx.size(), 6u); // 5 overhead + 1 data
    EXPECT_EQ(tx[1], static_cast<uint8_t>(0xB0u | page)) << "page " << static_cast<int>(page);
  }
  EXPECT_EQ(bus.dma_writes(), static_cast<int>(kDisplayPages));

  // Column addressing: the start column is split across two nibble commands.
  bus.clear_log();
  const Rect off_origin[] = {Rect{0, 0, 1, 8}, Rect{15, 0, 16, 8}, Rect{16, 0, 17, 8},
                             Rect{100, 0, 101, 8}, Rect{127, 0, 128, 8}};
  for (const Rect &damage : off_origin) {
    ASSERT_TRUE(panel.flush(fb, damage));
    const std::vector<uint8_t> tx = bus.last_tx();
    const uint16_t x0 = damage.x0;
    EXPECT_EQ(tx[2], static_cast<uint8_t>(x0 & 0x0Fu)) << "x0 " << x0;
    EXPECT_EQ(tx[3], static_cast<uint8_t>(0x10u | ((x0 >> 4) & 0x0Fu))) << "x0 " << x0;
  }
}

TEST_F(DisplayTest, FlushFullWritesEveryPageAndColumn) {
  Framebuffer fb;
  fb.fill(true);

  const size_t full = Ssd1306::bytes_for(kFullPanel);
  EXPECT_EQ(full, static_cast<size_t>(kDisplayPages) * (kDisplayWidth + 5u)); // 1064

  ASSERT_TRUE(panel.flush_full(fb));

  EXPECT_EQ(bus.dma_writes(), static_cast<int>(kDisplayPages));
  EXPECT_EQ(bus.dma_bytes(), static_cast<int>(full));
  EXPECT_EQ(dma_lengths(bus), (std::vector<size_t>(kDisplayPages, 133u)));

  // Every data byte of the page is the framebuffer's own byte: the full flush is a
  // straight copy, with nothing transposed or reordered.
  const std::vector<uint8_t> tx = bus.last_tx();
  ASSERT_EQ(tx.size(), 133u);
  EXPECT_EQ(tx[1], 0xB7); // the last page
  EXPECT_EQ(tx[2], 0x00);
  EXPECT_EQ(tx[3], 0x10);
  for (uint16_t column = 0; column < kDisplayWidth; ++column) {
    ASSERT_EQ(tx[5 + column], fb.byte_at(7, column)) << "column " << column;
  }

  const DisplayStats stats = panel.stats();
  EXPECT_EQ(stats.full_flushes, 1u);
  EXPECT_EQ(stats.flushes, 0u); // flush_full is not a flush() call
  EXPECT_EQ(stats.partial_flushes, 0u);
  EXPECT_EQ(stats.bytes_sent, static_cast<uint32_t>(full));
  EXPECT_EQ(stats.last_bytes, static_cast<uint32_t>(full));
}

// ---------------------------------------------------------------------------
// Ssd1306 — the DMA path, and failure handling
// ---------------------------------------------------------------------------

TEST_F(DisplayTest, FlushUsesTheDmaPathAndNeverThePolledOne) {
  Canvas canvas;
  canvas.clear();

  // init() included on purpose: "the display driver never blocks on a byte loop" is
  // worth being true of the whole driver, not just of the flush.
  ASSERT_TRUE(panel.init());
  const int init_bytes = bus.dma_bytes();
  const int init_writes = bus.dma_writes();
  EXPECT_EQ(init_bytes, 26); // one control byte plus 25 command bytes
  EXPECT_EQ(init_writes, 1);

  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));

  EXPECT_GT(bus.dma_writes(), init_writes);
  EXPECT_EQ(bus.dma_writes(), 1 + static_cast<int>(kDisplayPages));
  EXPECT_EQ(bus.dma_bytes() - init_bytes, 1064);
  EXPECT_EQ(bus.count(BusOp::Transfer), 0);              // the polled path was never used
  EXPECT_EQ(bus.count(BusOp::TransferWait), 9);          // one init, eight pages

  const ArbiterStats arbiter_stats = arbiter.stats();
  EXPECT_EQ(arbiter_stats.transfers, 9u);
  EXPECT_EQ(arbiter_stats.dma_transfers, 9u);
  EXPECT_EQ(arbiter_stats.dma_bytes, static_cast<uint32_t>(init_bytes) + 1064u);
  EXPECT_EQ(arbiter_stats.failed_transfers, 0u);
  EXPECT_EQ(arbiter_stats.contended, 0u);
}

TEST_F(DisplayTest, RejectedFlushIsReportedAndLeavesTheFrameAlone) {
  Canvas canvas;
  canvas.fill_rect(Rect{0, 0, 32, 32}, true);

  const std::vector<uint8_t> before(canvas.framebuffer().bytes(),
                                    canvas.framebuffer().bytes() + Framebuffer::kBytes);

  // The panel stops answering. A NACK is not a bus fault, so the arbiter does not run
  // recovery and this fails on the first attempt.
  bus.set_transfer_status(HAL_I2C_NACK);

  EXPECT_FALSE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));

  const DisplayStats stats = panel.stats();
  EXPECT_EQ(stats.failed, 1u);
  EXPECT_EQ(stats.flushes, 0u);        // a rejected flush is not a successful one
  EXPECT_EQ(stats.partial_flushes, 0u);
  EXPECT_EQ(stats.bytes_sent, 0u);     // nothing completed on the bus
  EXPECT_EQ(stats.last_bytes, 0u);
  EXPECT_EQ(stats.skipped, 0u);
  EXPECT_EQ(bus.dma_writes(), 1); // stopped at the first rejected page, not all 4
  // The fake bus counts what it was handed, so its byte counter includes the rejected
  // page's 37 bytes; the driver's own counter counts only bytes that completed, which
  // is the number that has to agree with bytes_for().
  EXPECT_EQ(bus.dma_bytes(), 37);
  EXPECT_EQ(panel.stats().bytes_sent, 0u);

  // The framebuffer is untouched: the damage is still outstanding, so the caller can
  // retry the same rectangle rather than re-rendering.
  ASSERT_EQ(before.size(), Framebuffer::kBytes);
  EXPECT_TRUE(std::equal(before.begin(), before.end(),
                         canvas.framebuffer().bytes()));

  // The bus comes back, and the retry sends the whole region: four pages of 32 columns,
  // 4 x (32 + 5) = 148 bytes.
  bus.set_transfer_status(HAL_I2C_OK);
  const size_t predicted = Ssd1306::bytes_for(canvas.damage().bounds());
  EXPECT_EQ(predicted, 148u);
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  EXPECT_EQ(panel.stats().failed, 1u); // the earlier failure is still on the record
  EXPECT_EQ(panel.stats().flushes, 1u);
  EXPECT_EQ(panel.stats().bytes_sent, static_cast<uint32_t>(predicted));
  EXPECT_EQ(panel.stats().last_bytes, static_cast<uint32_t>(predicted));
  EXPECT_EQ(bus.dma_writes(), 5); // one rejected page, then four pages on the retry
}

TEST_F(DisplayTest, BusErrorIsRecoveredAndTheFlushStillSucceeds) {
  Canvas canvas;
  canvas.fill_rect(Rect{0, 0, 8, 8}, true);

  // A held bus is the FR-0.3 case, and the display must not be the one path that
  // silently gives up on it: the arbiter recovers, retries, and the flush reports what
  // actually happened.
  bus.hold_sda_low_for_clocks(2);
  bus.fail_transfers(1, HAL_I2C_BUS_ERROR);

  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));

  EXPECT_EQ(bus.recoveries_run(), 1);
  EXPECT_EQ(panel.stats().failed, 0u);
  EXPECT_EQ(panel.stats().flushes, 1u);
  EXPECT_EQ(panel.stats().bytes_sent, 13u);
  EXPECT_EQ(bus.count(BusOp::Transfer), 0); // still DMA on the retry
}

TEST_F(DisplayTest, StatsAccumulateAcrossFlushes) {
  Canvas canvas;
  Framebuffer fb;

  canvas.fill_rect(Rect{0, 0, 8, 8}, true);
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  canvas.clear_damage();

  canvas.fill_rect(Rect{0, 0, 128, 64}, true); // a full-panel change through flush()
  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds()));
  canvas.clear_damage();

  ASSERT_TRUE(panel.flush(canvas.framebuffer(), canvas.damage().bounds())); // skipped
  ASSERT_TRUE(panel.flush_full(fb));

  const DisplayStats stats = panel.stats();
  EXPECT_EQ(stats.flushes, 2u);
  EXPECT_EQ(stats.partial_flushes, 1u); // only the 8x8 one was partial
  EXPECT_EQ(stats.full_flushes, 1u);
  EXPECT_EQ(stats.skipped, 1u);
  EXPECT_EQ(stats.failed, 0u);
  EXPECT_EQ(stats.bytes_sent, 13u + 1064u + 1064u);
  EXPECT_EQ(stats.last_bytes, 1064u);
}

} // namespace
} // namespace node
