/* display.h — framebuffer, damage tracking and the SSD1306 driver (FR-0.5).
 *
 * FR-0.5 requires the display path to use DMA and partial refresh instead of blocking
 * full-frame writes, and gives a reason: a full 128x64 frame is 1024 bytes, which at
 * 400 kHz is about 23 ms of bus time on a bus the sensor task also needs. So the
 * requirement is really about bus occupancy, and the design that satisfies it has two
 * halves:
 *
 *   1. The application never hands the driver "the frame"; it hands it the bounding box
 *      of what actually changed. Redrawing identical content marks nothing, so a static
 *      screen costs zero bus bytes — that is the property DamageTracker plus
 *      Framebuffer::set_pixel returning "did this change" exist for.
 *
 *   2. The transfer itself goes over the arbiter's DMA path, so the CPU is not shifting
 *      bytes for the duration of the flush.
 *
 * The panel's memory is organised as 8 pages of 128 columns, one byte per column
 * holding 8 vertical pixels. A flush therefore works page by page, and the horizontal
 * extent is the damaged column range rather than the full width.
 */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/i2c_arbiter.h"

namespace node {

// A half-open pixel rectangle: [x0, x1) x [y0, y1).
//
// Half-open rather than inclusive because it makes the empty rectangle the natural
// default: a default-constructed Rect is {0,0,0,0}, which is empty, which is exactly
// what a "nothing has changed" tracker starts from and what a flush must treat as a
// no-op. With inclusive bounds the default value is a single pixel at the origin, and
// every "is there anything to do" check becomes a special case waiting to be got wrong.
struct Rect {
  uint16_t x0 = 0;
  uint16_t y0 = 0;
  uint16_t x1 = 0;
  uint16_t y1 = 0;

  // Builds a rectangle from two corners in any order.
  static Rect from_corners(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by);
  // A rectangle exactly one pixel wide and tall.
  static Rect pixel(uint16_t x, uint16_t y);
  // The empty rectangle.
  static Rect none() { return Rect{}; }

  bool empty() const { return x1 <= x0 || y1 <= y0; }
  uint16_t width() const { return empty() ? 0 : static_cast<uint16_t>(x1 - x0); }
  uint16_t height() const { return empty() ? 0 : static_cast<uint16_t>(y1 - y0); }

  // Grow to cover `other`. An empty `other` leaves this unchanged, so accumulating
  // damage works without a special first-case.
  void include(const Rect &other);
  void include_point(uint16_t x, uint16_t y);
  bool contains(uint16_t x, uint16_t y) const;
  bool intersects(const Rect &other) const;

  // Clamp to the panel. Returns an empty rectangle if nothing overlaps it.
  Rect clamp_to_panel() const;
};

// 1 bpp framebuffer in the panel's own page order, so a flush is a straight copy rather
// than a transpose.
class Framebuffer {
 public:
  static constexpr uint16_t kWidth = kDisplayWidth;
  static constexpr uint16_t kHeight = kDisplayHeight;
  static constexpr uint8_t kPages = kDisplayPages;
  static constexpr size_t kBytes = kDisplayFramebufferBytes;

  void clear() { fill(false); }
  void fill(bool on = true);

  bool pixel(uint16_t x, uint16_t y) const;

  // Returns true if the pixel actually changed. Callers use that to decide whether to
  // mark damage; a no-op write must not cost bus time.
  bool set_pixel(uint16_t x, uint16_t y, bool on);

  // Byte for a page and column, exactly as the panel wants it.
  uint8_t byte_at(uint8_t page, uint16_t column) const;
  void set_byte(uint8_t page, uint16_t column, uint8_t value);

  const uint8_t *bytes() const { return data_; }

 private:
  uint8_t data_[kBytes]{};
};

// The bounding box of everything drawn since the last flush.
//
// A bounding box rather than a tile list or a run-length map, deliberately: it is O(1)
// per draw with no allocation, which matters on a 20 KB part, and the screens this node
// shows are a handful of text rows, where one box is barely worse than an exact region.
// A scattered update degrades to a full-width flush, which is the honest trade.
class DamageTracker {
 public:
  void clear() { bounds_ = Rect{}; }
  void mark_pixel(uint16_t x, uint16_t y);
  void mark_rect(const Rect &r);
  bool empty() const { return bounds_.empty(); }
  Rect bounds() const { return bounds_; }

 private:
  Rect bounds_{};
};

// Drawing surface. Every mutation marks damage; nothing else does.
class Canvas {
 public:
  // Clears to black and marks the whole panel, because clearing changes everything.
  void clear();

  bool set_pixel(uint16_t x, uint16_t y, bool on);
  void fill_rect(const Rect &r, bool on);
  void draw_rect(const Rect &r, bool on); // outline only
  void draw_hline(uint16_t x0, uint16_t x1, uint16_t y, bool on);
  void draw_vline(uint16_t x, uint16_t y0, uint16_t y1, bool on);

  // 5x7 glyphs, one pixel of spacing, 6 px advance. Lowercase folds to uppercase.
  // A character the font does not cover renders as a filled box, so a missing glyph is
  // visible on the panel instead of silently blank.
  void draw_text(uint16_t x, uint16_t y, const char *text);

  // Right-aligned-free decimal readout: the common case on this screen.
  void draw_u32(uint16_t x, uint16_t y, uint32_t value);

  const Framebuffer &framebuffer() const { return fb_; }
  const DamageTracker &damage() const { return damage_; }
  void clear_damage() { damage_.clear(); }

 private:
  Framebuffer fb_{};
  DamageTracker damage_{};
};

struct DisplayStats {
  uint32_t flushes = 0;         // successful flush() calls
  uint32_t full_flushes = 0;    // successful flush_full() calls
  uint32_t partial_flushes = 0; // flushes that covered less than the panel
  uint32_t bytes_sent = 0;      // total payload bytes put on the bus
  uint32_t last_bytes = 0;      // payload bytes of the most recent flush
  uint32_t failed = 0;          // flushes the bus rejected
  uint32_t skipped = 0;         // flushes with nothing to do
};

class Ssd1306 {
 public:
  // `bus` is borrowed and must outlive this object.
  explicit Ssd1306(I2cArbiter *bus, uint16_t addr7 = kAddrSsd1306);

  Ssd1306(const Ssd1306 &) = delete;
  Ssd1306 &operator=(const Ssd1306 &) = delete;

  // Panel init sequence: display off, clock and multiplex for 128x64, charge pump on,
  // horizontal addressing mode, contrast, display on.
  bool init();

  // Write only the page-aligned intersection of `damage` with the panel: one I2C
  // transaction per affected page, carrying that page's column range only.
  bool flush(const Framebuffer &fb, const Rect &damage);

  // Refresh every page and every column. Exists to make the FR-0.5 comparison
  // measurable against flush() on the same panel, not as a normal path.
  bool flush_full(const Framebuffer &fb);

  // Payload bytes a flush of `damage` would put on the bus, without touching the bus.
  // Lets the partial-versus-full argument be checked in a host test rather than only on
  // a scope.
  static size_t bytes_for(const Rect &damage);

  const DisplayStats &stats() const { return stats_; }

 private:
  I2cArbiter *bus_;
  uint16_t addr7_;
  DisplayStats stats_{};
};

} // namespace node

#endif // APP_DISPLAY_H
