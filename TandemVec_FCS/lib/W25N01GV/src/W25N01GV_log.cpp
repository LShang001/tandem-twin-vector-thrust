/*
 * W25N01GV_log.cpp - log-layer implementation
 *
 * License: MIT
 */

#include "W25N01GV_log.h"
#include <crc8.h>

// magic bytes at frame start (little-endian 0xAA55)
#define LOG_MAGIC0   0xAA
#define LOG_MAGIC1   0x55

// CRC-8 polynomial (CRC-8/MAXIM, same as 1-Wire; arbitrary but fixed)
#define LOG_CRC_POLY 0x31

W25N01GVLog::W25N01GVLog(W25N01GV &flash)
  : _flash(flash),
    _head(0), _tail(0), _buffered(0),
    _cursorPage(0), _written(0), _dropped(0), _serviceBusy(false)
{
}

bool W25N01GVLog::begin(bool erase)
{
  if (!_flash.isPresent()) {
    return false;
  }

  if (erase) {
    if (!format()) {
      return false;
    }
    _cursorPage = 0;
    writeCursor(_cursorPage);
  } else {
    // resume from persisted cursor
    uint32_t page;
    if (readCursor(page)) {
      _cursorPage = page;
    } else {
      _cursorPage = 0;
      writeCursor(_cursorPage);
    }
  }

  _head = _tail = _buffered = 0;
  _written = 0;
  _dropped = 0;
  _serviceBusy = false;
  return true;
}

// ---------------------------------------------------------------
// frame building / validation
// ---------------------------------------------------------------
void W25N01GVLog::buildFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq)
{
  dst[0] = LOG_MAGIC0;
  dst[1] = LOG_MAGIC1;
  dst[2] = (uint8_t)(seq & 0xFF);
  dst[3] = (uint8_t)((seq >> 8) & 0xFF);
  dst[4] = (uint8_t)((seq >> 16) & 0xFF);
  dst[5] = (uint8_t)((seq >> 24) & 0xFF);
  uint32_t t = millis();
  dst[6] = (uint8_t)(t & 0xFF);
  dst[7] = (uint8_t)((t >> 8) & 0xFF);
  dst[8] = (uint8_t)((t >> 16) & 0xFF);
  dst[9] = (uint8_t)((t >> 24) & 0xFF);
  memcpy(dst + 10, payload, W25N01GV_LOG_PAYLOAD);

  Crc8 crc(LOG_CRC_POLY);
  uint8_t crcLen = (uint8_t)(W25N01GV_LOG_FRAME_SIZE - 1);   // < 255
  dst[W25N01GV_LOG_FRAME_SIZE - 1] = crc.calc(dst, crcLen);
}

bool W25N01GVLog::validateFrame(const uint8_t *src, uint32_t &seq) const
{
  if (src[0] != LOG_MAGIC0 || src[1] != LOG_MAGIC1) {
    return false;
  }
  Crc8 crc(LOG_CRC_POLY);
  uint8_t crcLen = (uint8_t)(W25N01GV_LOG_FRAME_SIZE - 1);
  if (crc.calc((uint8_t*)src, crcLen) != src[W25N01GV_LOG_FRAME_SIZE - 1]) {
    return false;
  }
  seq = (uint32_t)src[2] |
        ((uint32_t)src[3] << 8) |
        ((uint32_t)src[4] << 16) |
        ((uint32_t)src[5] << 24);
  return true;
}

// ---------------------------------------------------------------
// cursor persistence (super page near end of device)
// ---------------------------------------------------------------
bool W25N01GVLog::readCursor(uint32_t &page)
{
  uint8_t buf[8];
  if (!_flash.pageRead(W25N01GV_LOG_SUPER_PAGE, buf, sizeof(buf))) {
    return false;
  }
  // magic + page + crc
  if (buf[0] != LOG_MAGIC0 || buf[1] != LOG_MAGIC1) {
    return false;
  }
  Crc8 crc(LOG_CRC_POLY);
  if (crc.calc(buf, 7) != buf[7]) {
    return false;
  }
  page = (uint32_t)buf[2] |
         ((uint32_t)buf[3] << 8) |
         ((uint32_t)buf[4] << 16) |
         ((uint32_t)buf[5] << 24);
  return (page < W25N01GV_TOTAL_PAGES);
}

void W25N01GVLog::writeCursor(uint32_t page)
{
  uint8_t buf[8];
  buf[0] = LOG_MAGIC0;
  buf[1] = LOG_MAGIC1;
  buf[2] = (uint8_t)(page & 0xFF);
  buf[3] = (uint8_t)((page >> 8) & 0xFF);
  buf[4] = (uint8_t)((page >> 16) & 0xFF);
  buf[5] = (uint8_t)((page >> 24) & 0xFF);
  buf[6] = 0x00;   // reserved
  Crc8 crc(LOG_CRC_POLY);
  buf[7] = crc.calc(buf, 7);

  // super page must be erased before program; erase its block first
  _flash.eraseBlock(W25N01GV_LOG_SUPER_PAGE);
  _flash.pageProgram(W25N01GV_LOG_SUPER_PAGE, buf, sizeof(buf));
}

// ---------------------------------------------------------------
// bad block avoidance
// ---------------------------------------------------------------
void W25N01GVLog::skipBadBlock()
{
  uint32_t blockBase = (_cursorPage / W25N01GV_PAGES_PER_BLOCK) * W25N01GV_PAGES_PER_BLOCK;
  _cursorPage = blockBase + W25N01GV_PAGES_PER_BLOCK;   // skip whole block
  if (_cursorPage >= W25N01GV_TOTAL_PAGES) {
    _cursorPage = 0;   // wrapped
  }
}

// ---------------------------------------------------------------
// public API
// ---------------------------------------------------------------
bool W25N01GVLog::logPush(const uint8_t *payload)
{
  if (_buffered >= W25N01GV_LOG_MAX_FRAMES) {
    _dropped++;
    return false;   // ring full
  }

  uint32_t seq = _written + (uint32_t)_buffered;
  uint8_t *slot = _ring[_head];
  buildFrame(slot, payload, seq);

  _head = (uint16_t)((_head + 1) % W25N01GV_LOG_MAX_FRAMES);
  _buffered++;
  return true;
}

void W25N01GVLog::logService()
{
  if (_serviceBusy) {
    return;   // previous long operation still finishing
  }

  uint16_t budget = W25N01GV_LOG_MAX_WRITES;
  while (budget-- > 0 && _buffered > 0) {
    uint8_t *frame = _ring[_tail];

    // frame fits one page (95 < 2048); write at cursor
    _serviceBusy = true;
    bool ok = _flash.pageProgram(_cursorPage, frame, W25N01GV_LOG_FRAME_SIZE);
    _serviceBusy = false;

    if (ok) {
      _cursorPage++;
      _tail = (uint16_t)((_tail + 1) % W25N01GV_LOG_MAX_FRAMES);
      _buffered--;
      _written++;
      writeCursor(_cursorPage);   // persist cursor (super page erase+prog)

      if (_cursorPage >= W25N01GV_TOTAL_PAGES) {
        _cursorPage = 0;          // wrap
      }
    } else {
      // page program failed -> skip whole block (bad block avoidance)
      skipBadBlock();
      // frame stays in ring; retry on next tick at new cursor
      break;
    }
  }
}

bool W25N01GVLog::format()
{
  uint8_t st;
  for (uint32_t block = 0; block < W25N01GV_BLOCK_COUNT; block++) {
    uint32_t page = block * W25N01GV_PAGES_PER_BLOCK;
    _flash.eraseBlock(page);
  }
  _cursorPage = 0;
  _written = 0;
  _dropped = 0;
  _buffered = 0;
  _head = _tail = 0;
  return true;
}

bool W25N01GVLog::debugReadPage(uint32_t page, uint8_t *buf, uint16_t len)
{
  return _flash.pageRead(page, buf, len);
}
