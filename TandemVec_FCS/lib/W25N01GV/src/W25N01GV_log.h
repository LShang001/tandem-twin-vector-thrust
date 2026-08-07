/*
 * W25N01GV_log.h - log-layer: RAM ring buffer + background page writes
 *
 * Design (see README.md for the full reasoning):
 *
 * - logPush() is called from a fast task (e.g. 200Hz black box); it ONLY
 *   copies a frame into the RAM ring buffer (a few us, never blocks).
 * - logService() is called from a low-priority task; per tick it writes
 *   up to W25N01GV_LOG_MAX_WRITES frames to flash (page program ~0.7ms
 *   each, busy-polled). This bounds the per-tick CPU cost (same budget
 *   pattern as CAN_RX_MAX_PER_TICK in the flight controller).
 * - Frames are stored as fixed-size records: magic + seq + t_ms + payload
 *   + CRC8. CRC uses lib/crc8's Crc8 class (frame < 255 bytes).
 * - A sequential cursor advances block/page linearly. If a page program
 *   fails (P-FAIL) or a read shows uncorrectable ECC, the whole block is
 *   skipped (simple bad-block avoidance; no BBM LUT remapping in v1).
 * - The cursor is persisted to a fixed "super" page near the end of the
 *   device so a power-cycle can resume appending (logBegin() reads it).
 *
 * License: MIT
 */

#ifndef W25N01GV_LOG_H
#define W25N01GV_LOG_H

#include "W25N01GV.h"

// ---- tunables ----
#ifndef W25N01GV_LOG_MAX_FRAMES
#define W25N01GV_LOG_MAX_FRAMES   32     // ring buffer depth (~3KB RAM)
#endif
#ifndef W25N01GV_LOG_MAX_WRITES
#define W25N01GV_LOG_MAX_WRITES   2      // max pages written per service tick
#endif
#ifndef W25N01GV_LOG_PAYLOAD
#define W25N01GV_LOG_PAYLOAD      84     // bytes of user payload per frame
#endif
#ifndef W25N01GV_LOG_SUPER_PAGE
#define W25N01GV_LOG_SUPER_PAGE   (W25N01GV_TOTAL_PAGES - 8U)  // cursor page
#endif

// one frame as stored on flash: magic(2) + seq(4) + t_ms(4) + payload + crc(1)
#define W25N01GV_LOG_FRAME_SIZE   (2U + 4U + 4U + W25N01GV_LOG_PAYLOAD + 1U)

/**
 * @brief Log layer over W25N01GV (sequential appender)
 *
 * Not thread-safe by itself: call logPush() from one task and
 * logService() from another; both are designed for the 2kHz FIFO
 * scheduler of the flight controller (no preemption between them).
 */
class W25N01GVLog {
public:
  explicit W25N01GVLog(W25N01GV &flash);

  /**
   * @brief Initialize: read cursor from super page, prepare ring buffer
   * @param erase  if true, erase whole device first (log begin fresh)
   * @return true if ready
   */
  bool begin(bool erase = false);

  /**
   * @brief Push one frame (non-blocking, copies into ring buffer)
   * @param payload pointer to W25N01GV_LOG_PAYLOAD bytes
   * @return true if accepted (false if ring full -> frame dropped)
   */
  bool logPush(const uint8_t *payload);

  /**
   * @brief Service the write queue (call periodically from a low-prio task)
   *        Writes up to W25N01GV_LOG_MAX_WRITES frames to flash.
   */
  void logService();

  /// frames currently buffered (awaiting flash write)
  uint16_t buffered() const { return _buffered; }

  /// total frames written since begin
  uint32_t written() const { return _written; }

  /// next page the cursor points at
  uint32_t cursorPage() const { return _cursorPage; }

  /// frames dropped because ring was full
  uint32_t dropped() const { return _dropped; }

  /// true while the device is being written (service busy)
  bool busy() const { return _serviceBusy; }

  /// erase the whole device (blocks ~2s; only for debug/format)
  bool format();

  /// dump a page's raw bytes via a debug callback (used by debug console)
  bool debugReadPage(uint32_t page, uint8_t *buf, uint16_t len);

private:
  W25N01GV &_flash;

  // ring buffer
  uint8_t _ring[W25N01GV_LOG_MAX_FRAMES][W25N01GV_LOG_FRAME_SIZE];
  uint16_t _head;        // next write index
  uint16_t _tail;        // next read index
  uint16_t _buffered;

  // cursor / stats
  uint32_t _cursorPage;
  uint32_t _written;
  uint32_t _dropped;
  bool _serviceBusy;

  // helpers
  void buildFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq);
  bool validateFrame(const uint8_t *src, uint32_t &seq) const;
  bool readCursor(uint32_t &page);
  void writeCursor(uint32_t page);
  void skipBadBlock();
};

#endif // W25N01GV_LOG_H
