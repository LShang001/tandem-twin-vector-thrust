/*
 * W25N01GV_log.h - log-layer v2: I/P delta frames + page packing + circular segments
 *
 * Design (see README.md for full reasoning; inspired by Betaflight flashfs
 * and ESP-IDF spi_nand_flash):
 *
 * Frame formats (v2):
 *   I-frame (full snapshot, every 32nd frame):
 *     magic(2)=0xAA55 + type(1)=0x49 + seq(2) + t_ms(4)
 *     + payload 21xfloat(84) + crc16(2)                 = 95 B
 *   P-frame (delta vs previous frame, other 31 frames):
 *     magic(2)=0xAA55 + type(1)=0x50 + seq(2) + dt_ms(2)
 *     + dpayload 21xint16(42) + crc16(2)               = 51 B
 *
 *   Delta quantization: (cur - prev) * 100, int16. I-frames reset
 *   the reference every 32 frames so quantization error never accumulates.
 *
 * Page packing: pages are filled with as many frames as fit (2048 B),
 * tail padded 0xFF. One page program writes ~39 frames (vs 1 in v1).
 *
 * Circular segments: the device is divided into segments (default 32
 * blocks = 2048 pages). Each segment starts with a segment-header page
 * (magic + seg number + start t_ms). When the last segment is full the
 * writer wraps to segment 0 (erasing it first) — the recorder never
 * fills up (Betaflight flashfs style).
 *
 * Cursor persistence: written once per BLOCK (not per frame) into the
 * segment header page → erase/write cost drops by 64x, wear is fine.
 *
 * Bad-block table: on begin(), spare-area bad-block markers of all 1024
 * blocks are scanned into a 128 B bitmap; runtime P-FAIL / uncorrectable
 * ECC marks a block bad and skips it.
 *
 * License: MIT
 */

#ifndef W25N01GV_LOG_H
#define W25N01GV_LOG_H

#include "W25N01GV.h"

// ---- tunables ----
#ifndef W25N01GV_LOG_MAX_FRAMES
#define W25N01GV_LOG_MAX_FRAMES   96     // ring buffer depth（攒页模式需 ≥ 一页量 39 + 余量）
#endif
#ifndef W25N01GV_LOG_MAX_WRITES
#define W25N01GV_LOG_MAX_WRITES   2      // max pages written per service tick
#endif
// ★ 页内打包：v2 调试时曾关闭（每页 1 帧），因当时误判"大页 SPI 传输
//   不稳定"卡死主循环。后定位真正根因是非对齐 float 访问 HardFault
//   （buildPFrame 的 (const float*) 强转，已用 memcpy 修复）。
//   重新启用打包：每页 2048B 填满 ~39 帧（I/P 混合），容量 ×40
//   （128MB ≈ 5-10 小时 @200Hz）。
#define W25N01GV_LOG_PACK_PAGE  1
#ifndef W25N01GV_LOG_PAYLOAD
#define W25N01GV_LOG_PAYLOAD      84     // bytes of user payload (21 floats)
#endif
#ifndef W25N01GV_LOG_I_INTERVAL
#define W25N01GV_LOG_I_INTERVAL   32     // one I-frame every N frames
#endif
#ifndef W25N01GV_LOG_SEG_BLOCKS
#define W25N01GV_LOG_SEG_BLOCKS   32     // segment = N blocks (default 32*64=2048 pages)
#endif

// ---- v2 frame constants ----
#define W25N01GV_LOG_MAGIC0       0xAA
#define W25N01GV_LOG_MAGIC1       0x55
#define W25N01GV_LOG_TYPE_I       0x49   // 'I'
#define W25N01GV_LOG_TYPE_P       0x50   // 'P'
#define W25N01GV_LOG_TYPE_S       0x53   // 'S': flight segment start
#define W25N01GV_LOG_TYPE_E       0x45   // 'E': flight segment end

#define W25N01GV_LOG_IFRAME_SIZE  95     // 2+1+2+4+84+2
#define W25N01GV_LOG_PFRAME_SIZE  51     // 2+1+2+2+42+2
#define W25N01GV_LOG_EFRAME_SIZE  15     // 2+1+2+4+4+2
// 通道名表最大长度（S 帧内 ASCII，21 通道名约 160B）
#define W25N01GV_LOG_CHNAME_MAX   160
// S 帧最大 = magic2+type1+seg2+t_ms4+names160+\0+crc2 = 172
#define W25N01GV_LOG_SFRAME_MAX   (W25N01GV_LOG_CHNAME_MAX + 12)

// delta quantization: (cur-prev)*100 stored as int16
#define W25N01GV_LOG_DELTA_SCALE  100

// segment header page content (persisted at segment start):
// magic(2) + seg(2) + start_t_ms(4) + reserved(6) + crc16(2) = 16 B
#define W25N01GV_LOG_SEG_HDR_SIZE 16

// page size used for packing (2048 = data area)
#define W25N01GV_LOG_PAGE_DATA    W25N01GV_PAGE_SIZE

/**
 * @brief Log layer v2 over W25N01GV
 *
 * Not thread-safe: logPush() from a fast task, logService() from a
 * low-priority task (2kHz FIFO scheduler, no preemption between them).
 */
class W25N01GVLog {
public:
  explicit W25N01GVLog(W25N01GV &flash);

  /**
   * @brief Initialize: scan bad blocks, locate latest segment + cursor
   * @param erase  if true, erase whole device (fresh start)
   * @return true if ready
   */
  bool begin(bool erase = false);

  /**
   * @brief Push one payload frame (non-blocking, encodes I/P delta)
   * @param payload pointer to W25N01GV_LOG_PAYLOAD bytes (21 floats)
   * @return true if accepted (false if ring full -> frame dropped)
   */
  bool logPush(const uint8_t *payload);

  /**
   * @brief Push flight-segment start (S frame) — arm edge
   * Embeds channel names for self-description (export tool reads them).
   */
  bool logFlightSegmentStart(uint16_t segNum, const char *chNames);

  /**
   * @brief Push flight-segment end (E frame) — disarm edge
   */
  bool logFlightSegmentEnd(uint16_t segNum, uint32_t durMs, uint32_t frames);

  /**
   * @brief Service write queue: pack ring frames into pages, write to NAND
   */
  void logService();

  /// frames currently buffered
  uint16_t buffered() const { return _buffered; }

  /// total frames written since begin
  uint32_t written() const { return _written; }

  /// current write page (absolute)
  uint32_t cursorPage() const { return _cursorPage; }

  /// current segment number
  uint16_t segment() const { return _segment; }

  /// frames dropped (ring full)
  uint32_t dropped() const { return _dropped; }

  /// pages written since begin
  uint32_t pagesWritten() const { return _pagesWritten; }

  /// bad blocks found
  uint16_t badBlocks() const { return _badBlocks; }

  /// true while a flash op is in progress
  bool busy() const { return _serviceBusy; }

  /// erase whole device (~2s; debug/format)
  bool format();

  /// read a raw page for debug/export
  bool debugReadPage(uint32_t page, uint8_t *buf, uint16_t len);

  /// number of usable (non-bad) blocks
  uint16_t usableBlocks() const { return W25N01GV_BLOCK_COUNT - _badBlocks; }

private:
  W25N01GV &_flash;

  // ring buffer of raw frames (I or P, variable length 51/95)
  uint8_t _ring[W25N01GV_LOG_MAX_FRAMES][W25N01GV_LOG_SFRAME_MAX];
  uint16_t _ringLen[W25N01GV_LOG_MAX_FRAMES];
  uint16_t _head, _tail, _buffered;

  // encode state
  uint8_t  _prevPayload[W25N01GV_LOG_PAYLOAD];  // reference for delta
  uint32_t _prevTms;
  uint16_t _frameCount;    // since last I-frame
  uint32_t _lastPushMs;    // 最近一次 logPush 时间（空闲冲刷判断）

  // storage state
  uint32_t _cursorPage;    // next page to write (absolute)
  uint16_t _segment;       // current segment number (0..SEG_COUNT-1)
  uint32_t _written, _dropped, _pagesWritten;
  uint32_t _globalSeq;     // ★ 全局帧序号（跨段/环形覆盖不重置，CSV 追踪唯一）
  bool _serviceBusy;

  // bad-block bitmap: 1 bit per block (1024 blocks -> 128 B)
  uint8_t _badMap[W25N01GV_BLOCK_COUNT / 8];
  uint16_t _badBlocks;

  // page assembly buffer（★ 32 字节对齐——D-Cache 行大小，SCB_CleanDCache 要求）
  alignas(32) uint8_t _pageBuf[W25N01GV_LOG_PAGE_DATA];

  // helpers
  void buildIFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq);
  void buildPFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq);
  void buildSFrame(uint8_t *dst, uint16_t segNum, const char *chNames);
  void buildEFrame(uint8_t *dst, uint16_t segNum, uint32_t durMs, uint32_t frames);
  bool pushRawFrame(const uint8_t *frame, uint16_t len);
  bool isBlockBad(uint32_t page) const;
  void markBlockBad(uint32_t page);
  void scanBadBlocks();
  uint32_t segmentStartPage(uint16_t seg) const;
  uint16_t segmentCount() const;
  bool writeSegmentHeader(uint16_t seg, uint32_t startTms);
  bool findLatestSegment(uint16_t &seg, uint32_t &cursor);
  void flushPage();        // encode ring frames into one page, write it
  void advanceCursor();    // page++ with segment wrap + header persist
};

#endif // W25N01GV_LOG_H
