/* display.cpp — framebuffer, damage tracking and the SSD1306 driver (FR-0.5).
 *
 * The interface this implements is frozen in app/display.h, and the reasoning behind
 * its shapes — half-open rectangles, a bounding-box damage tracker, storage in the
 * panel's own page order — is in that header and is not repeated here. What this file
 * adds is the other half: which bytes actually go on the bus, and which are
 * deliberately left off it.
 *
 * Three properties are load-bearing, and each one is a host test:
 *
 *   1. Redrawing identical content costs nothing. Framebuffer::set_pixel reports
 *      whether the pixel really changed, Canvas::set_pixel marks damage only when it
 *      did, and DamageTracker accumulates nothing when nothing changed — so a static
 *      screen leaves an empty damage rectangle and flush() sends zero bytes.
 *
 *   2. A flush spends bytes in proportion to what changed. The unit of transfer is a
 *      page: the damaged column range of the affected pages, never a full 128-column
 *      row and never an untouched page. That is the whole point of FR-0.5, because the
 *      1024-byte full frame it replaces is ~23 ms of a bus the sensor task also needs.
 *
 *   3. Every write to the panel goes over the arbiter's DMA path, the init sequence
 *      included, so no code path here blocks the CPU while bytes are shifted.
 *
 * The driver owns no heap and no static buffers: the per-page transaction buffer is a
 * stack array, which is safe because the DMA contract requires the source to stay valid
 * only until the arbiter's transfer_wait() returns, and transfer_dma() does that wait
 * before it returns. On the target this file costs 1024 bytes of framebuffer and 133
 * bytes of stack, against 20 KB of RAM for everything.
 */
#include "app/display.h"

#include <cstring>

namespace node {
namespace {

// ---------------------------------------------------------------------------
// Panel geometry and coordinate helpers
// ---------------------------------------------------------------------------

constexpr uint16_t kPanelWidth = kDisplayWidth;
constexpr uint16_t kPanelHeight = kDisplayHeight;

constexpr Rect panel_rect() { return Rect{0, 0, kPanelWidth, kPanelHeight}; }

// `v + 1`, saturating at the top of uint16_t rather than wrapping to 0.
//
// A half-open rectangle stores one-past-the-last pixel, so the coordinate 65535 has no
// representation: wrapping would turn "this pixel changed" into an empty rectangle,
// which reads as "nothing changed". Saturating keeps the failure monotone — the
// rectangle is still wrong, but it never claims no damage. Nothing on a 128x64 panel
// can reach it; the helper exists so that the arithmetic does not depend on that
// remaining true.
constexpr uint16_t next_coord(uint16_t v) {
  return v == 0xFFFF ? v : static_cast<uint16_t>(v + 1u);
}

constexpr uint16_t clamp_coord(uint16_t v, uint16_t limit) {
  return v > limit ? limit : v;
}

// Byte index for a page and column in the panel's own order. Spelled as one helper
// because three separate call sites computing the same stride is three chances to get
// it wrong, and a wrong stride here is a wrong pixel on the panel.
constexpr size_t fb_index(uint8_t page, uint16_t column) {
  return static_cast<size_t>(page) * static_cast<size_t>(kDisplayWidth) +
         static_cast<size_t>(column);
}

// Whether a damaged region is the whole panel. Used only for the statistics: a flush
// that happens to cover everything is still a partial-refresh flush by construction,
// but reporting it as "partial" would misreport the FR-0.5 comparison.
constexpr bool covers_panel(const Rect &r) {
  return r.x0 == 0 && r.y0 == 0 && r.x1 >= kPanelWidth && r.y1 >= kPanelHeight;
}

// ---------------------------------------------------------------------------
// The font
// ---------------------------------------------------------------------------
//
// Row-major on purpose: each glyph is seven rows of five bits, and bit 4 is the
// leftmost pixel, so the table reads like the bitmap it is and can be checked by eye
// instead of trusted. (A column-major blob is what fonts are usually shipped as, and
// it is also why a wrong one produces plausible-looking garbage — every glyph is off
// in a way that resembles a glyph.)
//
// Character set — the whole of it:
//
//   space  '0'-'9'  '.'  ':'  '-'  '+'  '%'  '/'
//   'A' 'C' 'D' 'E' 'H' 'I' 'K' 'L' 'M' 'N' 'O' 'P' 'R' 'S' 'T' 'U' 'W'
//
// That is what this node's labels and readouts use: "NODE", "T 21.50 C", "RH 45.00 %",
// "SENSOR --", "NO DATA", "RH LO/OK/HI", "ERR", and the "-" of a negative reading.
// Lowercase folds to uppercase, so "ok" and "OK" render identically.
//
// Anything else — another letter, punctuation, a control byte — renders as a filled
// 5x7 box. Deliberately not blank: a missing glyph that renders as nothing looks
// exactly like a value that is legitimately empty, and on a panel with no other
// channel to ask, a visible box is the difference between "the font lacks this
// character" and an afternoon spent debugging a sensor.

constexpr uint8_t kGlyphWidth = 5;
constexpr uint8_t kGlyphHeight = 7;
// One pixel of spacing between glyphs, and the reason a text row costs 6 px per
// character rather than 5.
constexpr uint8_t kGlyphAdvance = kGlyphWidth + 1;

struct Glyph {
  char ch;
  uint8_t rows[kGlyphHeight];
};

constexpr Glyph kGlyphs[] = {
    {' ', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}},
    {'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
    {'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
    {'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
    // Rounded rather than flat-topped: the flat top of the alternative reads as a '7'
    // with a tail at this size, and a font whose digits can be confused is a font that
    // misreports a reading.
    {'3', {0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110}},
    {'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
    {'5', {0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110}},
    {'6', {0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
    {'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
    {'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
    {'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100}},
    {'.', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b00110}},
    {':', {0b00000, 0b00110, 0b00110, 0b00000, 0b00110, 0b00110, 0b00000}},
    {'-', {0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000, 0b00000}},
    {'+', {0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000}},
    {'%', {0b11001, 0b11001, 0b00010, 0b00100, 0b01000, 0b10011, 0b10011}},
    {'/', {0b00001, 0b00010, 0b00010, 0b00100, 0b01000, 0b01000, 0b10000}},
    {'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
    {'D', {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110}},
    {'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
    {'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'I', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111}},
    {'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
    {'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
    {'M', {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001}},
    {'N', {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}},
    {'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
    {'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
    {'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
    {'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
    {'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'W', {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010}},
};

// The fallback for a character the table does not cover: a solid 5x7 block.
constexpr uint8_t kBoxGlyph[kGlyphHeight] = {0b11111, 0b11111, 0b11111, 0b11111,
                                            0b11111, 0b11111, 0b11111};

constexpr char fold_case(char c) {
  return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}

// A linear scan, which at 34 entries is at most 34 comparisons per character. Worth
// stating why it is not a jump table: this runs once per character per repaint, a
// repaint is at most ~100 characters at 200 ms intervals, and the alternative — an
// indexed table with holes — trades that for a lookup that silently mis-maps if the
// index arithmetic is ever wrong.
const uint8_t *glyph_rows(char c) {
  const char key = fold_case(c);
  for (const Glyph &glyph : kGlyphs) {
    if (glyph.ch == key) {
      return glyph.rows;
    }
  }
  return kBoxGlyph;
}

} // namespace

// ---------------------------------------------------------------------------
// Rect
// ---------------------------------------------------------------------------

Rect Rect::from_corners(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by) {
  Rect r;
  r.x0 = ax < bx ? ax : bx;
  r.y0 = ay < by ? ay : by;
  r.x1 = ax < bx ? bx : ax;
  r.y1 = ay < by ? by : ay;
  // Two identical corners therefore give an empty rectangle, not a one-pixel one. That
  // is the half-open reading of the same corner twice, and Rect::pixel() is how a
  // caller says "exactly one pixel"; having the degenerate case mean the same thing in
  // both spellings would make `from_corners(x, y, x, y)` a silent off-by-one.
  return r;
}

Rect Rect::pixel(uint16_t x, uint16_t y) {
  return Rect{x, y, next_coord(x), next_coord(y)};
}

void Rect::include(const Rect &other) {
  if (other.empty()) {
    return; // the accumulator case: "nothing new" cannot shrink or extend the box
  }
  if (empty()) {
    *this = other;
    return;
  }
  if (other.x0 < x0) {
    x0 = other.x0;
  }
  if (other.y0 < y0) {
    y0 = other.y0;
  }
  if (other.x1 > x1) {
    x1 = other.x1;
  }
  if (other.y1 > y1) {
    y1 = other.y1;
  }
}

void Rect::include_point(uint16_t x, uint16_t y) {
  const uint16_t nx = next_coord(x);
  const uint16_t ny = next_coord(y);
  if (empty()) {
    x0 = x;
    y0 = y;
    x1 = nx;
    y1 = ny;
    return;
  }
  if (x < x0) {
    x0 = x;
  }
  if (y < y0) {
    y0 = y;
  }
  if (nx > x1) {
    x1 = nx;
  }
  if (ny > y1) {
    y1 = ny;
  }
}

bool Rect::contains(uint16_t x, uint16_t y) const {
  // Half-open: the far edges are outside. `empty()` is not needed — an empty rectangle
  // has x1 <= x0, so no x can satisfy both comparisons.
  return x >= x0 && x < x1 && y >= y0 && y < y1;
}

bool Rect::intersects(const Rect &other) const {
  // Strict comparisons, so rectangles that merely touch share no pixel and do not
  // intersect. The tempting `<=` version is the inclusive-bounds habit, and it would
  // make a flush redraw a boundary column that nothing changed.
  return x0 < other.x1 && other.x0 < x1 && y0 < other.y1 && other.y0 < y1;
}

Rect Rect::clamp_to_panel() const {
  // Clamping both edges of an axis independently is what makes this total: a rectangle
  // entirely off one side collapses to two equal bounds, which is empty, with no
  // special case for "off the left" versus "off the right".
  return Rect{clamp_coord(x0, kPanelWidth), clamp_coord(y0, kPanelHeight),
              clamp_coord(x1, kPanelWidth), clamp_coord(y1, kPanelHeight)};
}

// ---------------------------------------------------------------------------
// Framebuffer
// ---------------------------------------------------------------------------

void Framebuffer::fill(bool on) {
  std::memset(data_, on ? 0xFF : 0x00, sizeof(data_));
}

bool Framebuffer::pixel(uint16_t x, uint16_t y) const {
  if (x >= kWidth || y >= kHeight) {
    return false; // outside the panel there is no pixel, so it is not lit
  }
  const uint8_t page = static_cast<uint8_t>(y / 8u);
  const uint8_t bit = static_cast<uint8_t>(y % 8u);
  return (data_[fb_index(page, x)] & static_cast<uint8_t>(1u << bit)) != 0;
}

bool Framebuffer::set_pixel(uint16_t x, uint16_t y, bool on) {
  // An out-of-panel write reports "no change" rather than being an error the caller has
  // to check: it changed nothing, so no caller may mark damage for it, which is exactly
  // the clipping rule the Canvas relies on.
  if (x >= kWidth || y >= kHeight) {
    return false;
  }

  const uint8_t page = static_cast<uint8_t>(y / 8u);
  const uint8_t bit = static_cast<uint8_t>(y % 8u);
  const uint8_t mask = static_cast<uint8_t>(1u << bit);
  uint8_t &target = data_[fb_index(page, x)];

  if (((target & mask) != 0) == on) {
    return false;
  }
  target = on ? static_cast<uint8_t>(target | mask)
              : static_cast<uint8_t>(target & static_cast<uint8_t>(~mask));
  return true;
}

uint8_t Framebuffer::byte_at(uint8_t page, uint16_t column) const {
  if (page >= kPages || column >= kWidth) {
    // Out of range reads as blank rather than wrapping to another page's byte: a wrap
    // would put a real pixel in the wrong place on the panel, which is worse than a
    // wrong pixel in the right place.
    return 0;
  }
  return data_[fb_index(page, column)];
}

void Framebuffer::set_byte(uint8_t page, uint16_t column, uint8_t value) {
  if (page >= kPages || column >= kWidth) {
    return;
  }
  data_[fb_index(page, column)] = value;
}

// ---------------------------------------------------------------------------
// DamageTracker
// ---------------------------------------------------------------------------

void DamageTracker::mark_pixel(uint16_t x, uint16_t y) { bounds_.include_point(x, y); }

void DamageTracker::mark_rect(const Rect &r) {
  // Rect::include ignores an empty rectangle, so "mark nothing" needs no call-site
  // check. That is the property the whole zero-cost-static-screen behaviour rests on.
  bounds_.include(r);
}

// ---------------------------------------------------------------------------
// Canvas
// ---------------------------------------------------------------------------

void Canvas::clear() {
  fb_.clear();
  // Marked unconditionally, even if the framebuffer was already blank. The alternative
  // — compare first and mark nothing when it is already clear — saves one comparison
  // per repaint and buys a rule with two branches in it; more to the point, the caller
  // asking for a clear is saying the panel must end up black, and a flush is cheap
  // enough that being right matters more.
  damage_.mark_rect(panel_rect());
}

bool Canvas::set_pixel(uint16_t x, uint16_t y, bool on) {
  // Clipping happens inside the framebuffer, which reports "no change" for a pixel that
  // does not exist. Nothing off-panel can therefore reach the damage tracker, and no
  // coordinate can wrap: a write at x = 200 is discarded, not folded onto x = 72.
  if (!fb_.set_pixel(x, y, on)) {
    return false;
  }
  damage_.mark_pixel(x, y);
  return true;
}

void Canvas::fill_rect(const Rect &r, bool on) {
  const Rect region = r.clamp_to_panel();
  for (uint16_t y = region.y0; y < region.y1; ++y) {
    for (uint16_t x = region.x0; x < region.x1; ++x) {
      set_pixel(x, y, on);
    }
  }
  // Note what is *not* here: a bulk byte write. Filling whole bytes would be faster on
  // the target, but it would also mark damage for every byte it touched whether or not
  // the value changed, which is precisely the cost this module exists to avoid. A
  // 128x64 fill is 8192 pixel tests, a few hundred microseconds, against the ~23 ms of
  // bus time it replaces.
}

void Canvas::draw_rect(const Rect &r, bool on) {
  const Rect region = r.clamp_to_panel();
  if (region.empty()) {
    return;
  }

  draw_hline(region.x0, region.x1, region.y0, on);
  draw_hline(region.x0, region.x1, static_cast<uint16_t>(region.y1 - 1u), on);

  // The verticals cover the open interval (y0, y1-1) only: the corners already belong to
  // the horizontals, and a one- or two-pixel-tall rectangle has no interior rows at all
  // (the condition is false immediately), so a thin outline is not drawn twice.
  for (uint16_t y = static_cast<uint16_t>(region.y0 + 1u); y + 1u < region.y1; ++y) {
    set_pixel(region.x0, y, on);
    set_pixel(static_cast<uint16_t>(region.x1 - 1u), y, on);
  }
}

void Canvas::draw_hline(uint16_t x0, uint16_t x1, uint16_t y, bool on) {
  // Half-open [x0, x1), matching Rect, so a rectangle's top edge is exactly
  // draw_hline(r.x0, r.x1, r.y0) and there is one span convention in the module rather
  // than two. Endpoints may arrive in either order, as with Rect::from_corners.
  if (y >= kPanelHeight) {
    return;
  }
  const uint16_t last = x0 < x1 ? x1 : x0;
  for (uint16_t x = x0 < x1 ? x0 : x1; x < last && x < kPanelWidth; ++x) {
    set_pixel(x, y, on);
  }
}

void Canvas::draw_vline(uint16_t x, uint16_t y0, uint16_t y1, bool on) {
  if (x >= kPanelWidth) {
    return;
  }
  const uint16_t last = y0 < y1 ? y1 : y0;
  for (uint16_t y = y0 < y1 ? y0 : y1; y < last && y < kPanelHeight; ++y) {
    set_pixel(x, y, on);
  }
}

void Canvas::draw_text(uint16_t x, uint16_t y, const char *text) {
  if (text == nullptr || x >= kPanelWidth || y >= kPanelHeight) {
    return;
  }

  // A 32-bit pen rather than uint16_t: the advance would otherwise wrap after 10922
  // characters and start painting at the left edge again. The pen only ever grows, so
  // once it leaves the panel the rest of the string is invisible and the loop can stop
  // — worth doing because the alternative walks every remaining glyph and clips every
  // one of its pixels.
  uint32_t pen = x;
  for (const char *p = text; *p != '\0'; ++p) {
    const uint8_t *rows = glyph_rows(*p);

    for (uint8_t row = 0; row < kGlyphHeight; ++row) {
      const uint8_t bits = rows[row];
      if (bits == 0) {
        continue; // blank rows: nothing to draw, and nothing to mark as damaged
      }
      const uint16_t py = static_cast<uint16_t>(y + row);
      for (uint8_t col = 0; col < kGlyphWidth; ++col) {
        // Bit 4 is the leftmost of the five columns, so column 0 tests bit 4.
        const uint8_t mask = static_cast<uint8_t>(1u << (kGlyphWidth - 1u - col));
        if ((bits & mask) != 0) {
          set_pixel(static_cast<uint16_t>(pen + col), py, true);
        }
      }
    }

    pen += kGlyphAdvance;
    if (pen >= kPanelWidth) {
      break;
    }
  }
  // Clipping is per pixel through set_pixel, so text that runs off the right edge loses
  // only the pixels that do not exist. Nothing wraps to the next row or to column 0 of
  // the same page, which is the corruption a byte-wise blit would produce.
}

void Canvas::draw_u32(uint16_t x, uint16_t y, uint32_t value) {
  // 10 digits for the largest uint32_t, plus room for the terminator. Filled from the
  // back so the digits come out in order without a reversal pass.
  char digits[11];
  size_t begin = sizeof(digits);
  digits[--begin] = '\0';
  do {
    digits[--begin] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0);

  draw_text(x, y, &digits[begin]);
}

// ---------------------------------------------------------------------------
// Ssd1306
// ---------------------------------------------------------------------------

// The byte counters, so these numbers can be reconciled against a scope capture:
//
// A payload byte is one byte of the buffer handed to the arbiter — the 0x00 and 0x40
// control bytes, the page and column command bytes, and the display data. The 7-bit
// device address and the ACK bits are the HAL's contribution, added below this layer,
// so they are not counted here; bytes_for() predicts exactly the counted quantity.
//
// Only transactions that completed OK are counted, in bytes_sent and in last_bytes. A
// rejected transaction did not put its bytes on the bus, and counting an attempt would
// make the total disagree with the bytes_for() prediction for a flush that failed —
// which is the one case where the counter is worth having.
//
// A flush() the bus rejected increments `failed` and returns false without touching the
// framebuffer (which it could not anyway: the framebuffer arrives const). The panel's
// content is therefore unchanged and the damage is still outstanding, so the caller can
// simply flush the same rectangle again next cycle.
//
// init() failures are reported through init()'s return value only. An init is not a
// flush, and folding it into `failed` would make "flushes the bus rejected" untrue.

namespace {

// The panel's control bytes. Every transaction starts with one: 0x00 announces that the
// bytes which follow are commands, 0x40 that they are display data. The panel switches
// mode on each control byte, so a single transaction can carry a command run and a data
// run back to back — which is exactly how a page flush is shaped below.
constexpr uint8_t kCtrlCommand = 0x00;
constexpr uint8_t kCtrlData = 0x40;

// Per page transaction: control byte, page address, two column-address bytes, control
// byte. Spelled once because both the flush and bytes_for() must agree on it, and a
// disagreement would make bytes_for() a lie that only shows up on a scope.
constexpr size_t kPageOverheadBytes = 5;

// The setup sequence for a 128x64 module, sent as one transaction.
//
// One control byte is enough for the whole run: the panel stays in the mode the last
// control byte selected, so only the transition into the data phase needs its own 0x40.
//
// Ordering is part of the sequence. Display off is first and display on is last, so the
// panel is never visible in a half-configured state — during the charge-pump enable in
// particular, a panel that is already on can flash. The rest is the standard 128x64
// configuration: multiplex and COM pins for 64 rows, remap and scan direction so that
// (0,0) is the top-left as the framebuffer assumes, and the charge pump on because
// these modules have no separate panel supply.
const uint8_t kInitSequence[] = {
    kCtrlCommand,
    0xAE,       // display off while the panel is reconfigured
    0xD5, 0x80, // display clock divide ratio / oscillator frequency: reset default
    0xA8, 0x3F, // multiplex ratio: 63 + 1 = 64 rows
    0xD3, 0x00, // display offset 0: COM0 is the first row
    0x40,       // display start line 0
    0x8D, 0x14, // charge pump: enable
    0x20, 0x00, // memory addressing mode: horizontal
    0xA1,       // segment remap: column 0 is SEG0, so x grows left to right
    0xC8,       // COM scan remapped: y grows top to bottom in the usual mounting
    0xDA, 0x12, // COM pins: alternative (split) configuration, the 128x64 wiring
    0x81, 0xCF, // contrast
    0xD9, 0xF1, // pre-charge period
    0xDB, 0x40, // VCOMH deselect level
    0xA4,       // display follows RAM: not "entire display on"
    0xA6,       // normal display, not inverted
    0xAF,       // display on, last
};

// Writes the page-aligned region of `region` and returns the payload bytes that
// completed on the bus. `region` must already be clamped to the panel and non-empty;
// `*ok` is set false on the first rejected transaction, and the bytes after it are not
// attempted — there is no point pushing the rest of a frame onto a bus that has just
// refused a page, and the caller decides whether to retry the whole thing.
//
// The payload is built in one stack buffer reused per page. That is safe precisely
// because transfer_dma() waits for the transfer to finish before returning, which is
// the same property that makes a caller's buffer valid for the duration.
size_t write_region(I2cArbiter *bus, uint16_t addr7, const Framebuffer &fb,
                    const Rect &region, bool *ok) {
  uint8_t payload[kPageOverheadBytes + kDisplayWidth];

  const uint8_t first_page = static_cast<uint8_t>(region.y0 / 8u);
  const uint8_t last_page = static_cast<uint8_t>((region.y1 - 1u) / 8u);
  const uint16_t x0 = region.x0;
  const uint16_t width = region.width();

  size_t sent = 0;
  for (uint8_t page = first_page; page <= last_page; ++page) {
    payload[0] = kCtrlCommand;
    payload[1] = static_cast<uint8_t>(0xB0u | page); // set page start address
    payload[2] = static_cast<uint8_t>(x0 & 0x0Fu);   // set lower column start address
    payload[3] = static_cast<uint8_t>(0x10u | ((x0 >> 4) & 0x0Fu)); // upper nibble
    payload[4] = kCtrlData;
    for (uint16_t i = 0; i < width; ++i) {
      // Only the damaged columns of this page are copied, so the transaction length is
      // the damaged width and the panel's own column pointer never runs past it.
      payload[kPageOverheadBytes + i] =
          fb.byte_at(page, static_cast<uint16_t>(x0 + i));
    }

    const size_t length = kPageOverheadBytes + static_cast<size_t>(width);
    if (bus == nullptr || bus->transfer_dma(addr7, payload, length) != BusResult::Ok) {
      *ok = false;
      return sent;
    }
    sent += length;
  }
  return sent;
}

} // namespace

Ssd1306::Ssd1306(I2cArbiter *bus, uint16_t addr7) : bus_(bus), addr7_(addr7) {}

bool Ssd1306::init() {
  // Over DMA like every other write to this panel. The sequence is small enough that a
  // polled write would work, but "the display path never blocks on a byte loop" is a
  // property worth being absolute about: it can then be checked for the driver as a
  // whole rather than for the one entry point someone remembered.
  if (bus_ == nullptr) {
    return false;
  }
  return bus_->transfer_dma(addr7_, kInitSequence, sizeof(kInitSequence)) == BusResult::Ok;
}

bool Ssd1306::flush(const Framebuffer &fb, const Rect &damage) {
  const Rect target = damage.clamp_to_panel();
  if (target.empty()) {
    // Nothing to do, and the caller learns it cost nothing: `flushes` counts the flushes
    // that put bytes on the bus, `skipped` counts the ones that had nothing to send, and
    // the two are disjoint so a reader can add them up and get the number of calls.
    // Returning true is deliberate — the panel already shows the right thing, so a
    // caller must not treat this as a failure and retry.
    ++stats_.skipped;
    stats_.last_bytes = 0;
    return true;
  }

  bool ok = true;
  const size_t sent = write_region(bus_, addr7_, fb, target, &ok);
  stats_.last_bytes = static_cast<uint32_t>(sent);
  stats_.bytes_sent += static_cast<uint32_t>(sent);

  if (!ok) {
    ++stats_.failed;
    return false;
  }

  ++stats_.flushes;
  if (!covers_panel(target)) {
    ++stats_.partial_flushes;
  }
  return true;
}

bool Ssd1306::flush_full(const Framebuffer &fb) {
  bool ok = true;
  const size_t sent = write_region(bus_, addr7_, fb, panel_rect(), &ok);
  stats_.last_bytes = static_cast<uint32_t>(sent);
  stats_.bytes_sent += static_cast<uint32_t>(sent);

  if (!ok) {
    ++stats_.failed;
    return false;
  }

  // Counted as a full flush and nothing else: it is not a flush() call, and calling a
  // whole-panel refresh "partial" would corrupt the very comparison it exists for.
  ++stats_.full_flushes;
  return true;
}

size_t Ssd1306::bytes_for(const Rect &damage) {
  const Rect target = damage.clamp_to_panel();
  if (target.empty()) {
    return 0;
  }
  const size_t first_page = static_cast<size_t>(target.y0) / 8u;
  const size_t last_page = static_cast<size_t>(target.y1 - 1u) / 8u;
  const size_t pages = last_page - first_page + 1u;
  // +1 accounts for the width being a count of pixels rather than of the last index.
  return pages * (static_cast<size_t>(target.width()) + kPageOverheadBytes);
}

} // namespace node
