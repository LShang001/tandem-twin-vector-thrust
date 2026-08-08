/*
 * W25N01GV_log.cpp - log-layer v2 implementation
 *
 * License: MIT
 */

#include "W25N01GV_log.h"

// CRC-16/CCITT (poly 0x1021, init 0xFFFF) — simple bitwise impl
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

W25N01GVLog::W25N01GVLog(W25N01GV &flash)
  : _flash(flash),
    _head(0), _tail(0), _buffered(0),
    _prevTms(0), _frameCount(0), _lastPushMs(0),
    _cursorPage(0), _segment(0),
    _written(0), _dropped(0), _pagesWritten(0), _globalSeq(0), _serviceBusy(false),
    _badBlocks(0)
{
  memset(_prevPayload, 0, sizeof(_prevPayload));
  memset(_badMap, 0, sizeof(_badMap));
}

// ---------------------------------------------------------------
// geometry
// ---------------------------------------------------------------
uint32_t W25N01GVLog::segmentStartPage(uint16_t seg) const
{
  return (uint32_t)seg * W25N01GV_LOG_SEG_BLOCKS * W25N01GV_PAGES_PER_BLOCK;
}

uint16_t W25N01GVLog::segmentCount() const
{
  return W25N01GV_BLOCK_COUNT / W25N01GV_LOG_SEG_BLOCKS;   // 1024/32 = 32 segments
}

// ---------------------------------------------------------------
// bad-block bitmap
// ---------------------------------------------------------------
bool W25N01GVLog::isBlockBad(uint32_t page) const
{
  uint32_t block = page / W25N01GV_PAGES_PER_BLOCK;
  return (_badMap[block >> 3] & (1U << (block & 7))) != 0;
}

void W25N01GVLog::markBlockBad(uint32_t page)
{
  uint32_t block = page / W25N01GV_PAGES_PER_BLOCK;
  if (!(_badMap[block >> 3] & (1U << (block & 7)))) {
    _badMap[block >> 3] |= (1U << (block & 7));
    _badBlocks++;
  }
}

void W25N01GVLog::scanBadBlocks()
{
  _badBlocks = 0;
  memset(_badMap, 0, sizeof(_badMap));
  // 扫描每块第 0 页 spare 区：NAND 出厂坏块标记 = 该页 spare 首字节 0x00
  for (uint32_t block = 0; block < W25N01GV_BLOCK_COUNT; block++) {
    uint8_t spare[8];
    if (!_flash.readSpare(block * W25N01GV_PAGES_PER_BLOCK, spare, sizeof(spare))) {
      continue;
    }
    // Winbond: 坏块标记在 spare 区首 2 字节，全 0x00 为坏
    if (spare[0] == 0x00 && spare[1] == 0x00) {
      markBlockBad(block * W25N01GV_PAGES_PER_BLOCK);
    }
  }
}

// ---------------------------------------------------------------
// frame encoding
// ---------------------------------------------------------------
void W25N01GVLog::buildIFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq)
{
  dst[0] = W25N01GV_LOG_MAGIC0;
  dst[1] = W25N01GV_LOG_MAGIC1;
  dst[2] = W25N01GV_LOG_TYPE_I;
  dst[3] = (uint8_t)(seq & 0xFF);
  dst[4] = (uint8_t)((seq >> 8) & 0xFF);
  uint32_t t = millis();
  dst[5] = (uint8_t)(t & 0xFF);
  dst[6] = (uint8_t)((t >> 8) & 0xFF);
  dst[7] = (uint8_t)((t >> 16) & 0xFF);
  dst[8] = (uint8_t)((t >> 24) & 0xFF);
  memcpy(dst + 9, payload, W25N01GV_LOG_PAYLOAD);
  uint16_t crc = crc16_ccitt(dst, W25N01GV_LOG_IFRAME_SIZE - 2);
  dst[W25N01GV_LOG_IFRAME_SIZE - 2] = (uint8_t)(crc & 0xFF);
  dst[W25N01GV_LOG_IFRAME_SIZE - 1] = (uint8_t)((crc >> 8) & 0xFF);
}

void W25N01GVLog::buildPFrame(uint8_t *dst, const uint8_t *payload, uint32_t seq)
{
  dst[0] = W25N01GV_LOG_MAGIC0;
  dst[1] = W25N01GV_LOG_MAGIC1;
  dst[2] = W25N01GV_LOG_TYPE_P;
  dst[3] = (uint8_t)(seq & 0xFF);
  dst[4] = (uint8_t)((seq >> 8) & 0xFF);

  // delta time (ms, int16)
  uint32_t t = millis();
  int32_t dt = (int32_t)t - (int32_t)_prevTms;
  if (dt > 32767) dt = 32767;
  if (dt < -32768) dt = -32768;
  dst[5] = (uint8_t)(dt & 0xFF);
  dst[6] = (uint8_t)((dt >> 8) & 0xFF);

  // delta payload: 21 x int16 = 42 B (quantized *100)
  // ★ 2026-08-08 修正：循环写 22 个增量会把最后 2B 写到 CRC 槽位（dst[49..50]），
  //   被下方 crc 覆盖 → 实际存储恒为 21 个增量。此处显式收敛到 21：
  //   第 22 通道（p2）不参与差分，解码端保持最近 I 帧参考值。
  //   帧长/布局不变（51B），与既有导出工具兼容。
  // ★ 用 memcpy 逐元素读 float——payload/_prevPayload 是 uint8_t 数组，
  //   直接 (const float*) 强转会非对齐访问 → ARM HardFault（2026-08-08 实测复位）
  uint8_t *dp = dst + 7;
  const uint16_t N_DELTAS = (W25N01GV_LOG_PFRAME_SIZE - 9) / 2;   // 21
  float cur_f, prev_f;
  for (uint16_t i = 0; i < N_DELTAS; i++) {
    memcpy(&cur_f, payload + i * 4, 4);
    memcpy(&prev_f, _prevPayload + i * 4, 4);
    int32_t d = (int32_t)((cur_f - prev_f) * W25N01GV_LOG_DELTA_SCALE);
    if (d > 32767) d = 32767;
    if (d < -32768) d = -32768;
    int16_t v = (int16_t)d;
    dp[i * 2] = (uint8_t)(v & 0xFF);
    dp[i * 2 + 1] = (uint8_t)((v >> 8) & 0xFF);
  }

  uint16_t crc = crc16_ccitt(dst, W25N01GV_LOG_PFRAME_SIZE - 2);
  dst[W25N01GV_LOG_PFRAME_SIZE - 2] = (uint8_t)(crc & 0xFF);
  dst[W25N01GV_LOG_PFRAME_SIZE - 1] = (uint8_t)((crc >> 8) & 0xFF);
}

// ---------------------------------------------------------------
// segment header persistence
// ---------------------------------------------------------------
bool W25N01GVLog::writeSegmentHeader(uint16_t seg, uint32_t startTms)
{
  uint8_t hdr[W25N01GV_LOG_SEG_HDR_SIZE];
  memset(hdr, 0xFF, sizeof(hdr));
  hdr[0] = W25N01GV_LOG_MAGIC0;
  hdr[1] = W25N01GV_LOG_MAGIC1;
  hdr[2] = (uint8_t)(seg & 0xFF);
  hdr[3] = (uint8_t)((seg >> 8) & 0xFF);
  hdr[4] = (uint8_t)(startTms & 0xFF);
  hdr[5] = (uint8_t)((startTms >> 8) & 0xFF);
  hdr[6] = (uint8_t)((startTms >> 16) & 0xFF);
  hdr[7] = (uint8_t)((startTms >> 24) & 0xFF);
  uint16_t crc = crc16_ccitt(hdr, W25N01GV_LOG_SEG_HDR_SIZE - 2);
  hdr[14] = (uint8_t)(crc & 0xFF);
  hdr[15] = (uint8_t)((crc >> 8) & 0xFF);

  uint32_t page = segmentStartPage(seg);
  if (isBlockBad(page)) return false;
  if (!_flash.eraseBlock(page)) return false;         // segment header block
  if (!_flash.pageProgram(page, hdr, W25N01GV_LOG_SEG_HDR_SIZE)) {
    markBlockBad(page);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------
// resume: find latest segment + cursor
// ---------------------------------------------------------------
bool W25N01GVLog::findLatestSegment(uint16_t &seg, uint32_t &cursor)
{
  // 从段 0 顺序扫段头，找最后一个 magic 有效且段号连续的段
  // （环形覆盖场景下，最新段 = 段号最大且 header 有效）
  uint16_t count = segmentCount();
  uint8_t buf[W25N01GV_LOG_SEG_HDR_SIZE];
  int16_t lastValid = -1;
  uint32_t lastStart = 0;

  for (uint16_t s = 0; s < count; s++) {
    uint32_t page = segmentStartPage(s);
    if (isBlockBad(page)) continue;
    if (!_flash.pageRead(page, buf, sizeof(buf))) continue;
    if (buf[0] != W25N01GV_LOG_MAGIC0 || buf[1] != W25N01GV_LOG_MAGIC1) continue;
    uint16_t crc = crc16_ccitt(buf, W25N01GV_LOG_SEG_HDR_SIZE - 2);
    if ((uint8_t)(crc & 0xFF) != buf[14] || (uint8_t)((crc >> 8) & 0xFF) != buf[15]) continue;
    uint16_t sSeg = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    // 段号应等于 s（线性布局）或回绕后的值
    if (sSeg == s || (sSeg % count) == s) {
      lastValid = (int16_t)s;
      lastStart = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8) |
                  ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    }
  }

  if (lastValid < 0) {
    // 无有效段头：全新设备
    seg = 0;
    cursor = segmentStartPage(0);
    return false;
  }

  seg = (uint16_t)lastValid;
  // 游标 = 段内扫描：从段头后逐页读，找第一个 magic 无效的页（未写区域）
  cursor = segmentStartPage(seg);
  uint8_t probe[8];
  for (uint32_t page = cursor + 1; page < cursor + W25N01GV_LOG_SEG_BLOCKS * W25N01GV_PAGES_PER_BLOCK; page++) {
    if (isBlockBad(page)) continue;
    if (!_flash.pageRead(page, probe, sizeof(probe))) continue;
    if (probe[0] != W25N01GV_LOG_MAGIC0 || probe[1] != W25N01GV_LOG_MAGIC1) {
      break;   // 未写页 = 游标
    }
    cursor = page;
  }
  cursor++;   // 下一可写页
  return true;
}

// ---------------------------------------------------------------
// public API
// ---------------------------------------------------------------
bool W25N01GVLog::begin(bool erase)
{
  if (!_flash.isPresent()) return false;

  scanBadBlocks();

  if (erase) {
    if (!format()) return false;
    _segment = 0;
    _cursorPage = segmentStartPage(0);
    writeSegmentHeader(_segment, millis());
    _cursorPage++;   // header page consumed
  } else {
    if (!findLatestSegment(_segment, _cursorPage)) {
      // 全新设备：建段头
      writeSegmentHeader(0, millis());
      _cursorPage = segmentStartPage(0) + 1;
    }
  }

  _head = _tail = _buffered = 0;
  _written = 0;
  _dropped = 0;
  _pagesWritten = 0;
  _frameCount = 0;
  _prevTms = millis();
  _lastPushMs = _prevTms;
  memset(_prevPayload, 0, sizeof(_prevPayload));
  _serviceBusy = false;
  return true;
}

bool W25N01GVLog::pushRawFrame(const uint8_t *frame, uint16_t len)
{
  if (_buffered >= W25N01GV_LOG_MAX_FRAMES) {
    _dropped++;
    return false;
  }
  memcpy(_ring[_head], frame, len);
  _ringLen[_head] = len;
  _head = (uint16_t)((_head + 1) % W25N01GV_LOG_MAX_FRAMES);
  _buffered++;
  _lastPushMs = millis();
  return true;
}

bool W25N01GVLog::logPush(const uint8_t *payload)
{
  // ★ 全局帧序号（跨段/环形覆盖不重置）——导出 CSV 的 seq 列全局唯一
  uint32_t seq = _globalSeq++;

  uint8_t slot[W25N01GV_LOG_SFRAME_MAX];

  if ((_frameCount % W25N01GV_LOG_I_INTERVAL) == 0) {
    buildIFrame(slot, payload, seq);
    if (!pushRawFrame(slot, W25N01GV_LOG_IFRAME_SIZE)) return false;
  } else {
    buildPFrame(slot, payload, seq);
    if (!pushRawFrame(slot, W25N01GV_LOG_PFRAME_SIZE)) return false;
  }
  _frameCount++;

  // update delta reference
  memcpy(_prevPayload, payload, W25N01GV_LOG_PAYLOAD);
  _prevTms = millis();
  _lastPushMs = _prevTms;

  return true;
}

// ---------------------------------------------------------------
// flight segment control frames (S / E)
// ---------------------------------------------------------------
void W25N01GVLog::buildSFrame(uint8_t *dst, uint16_t segNum, const char *chNames)
{
  // ★ 固定长度 S 帧（W25N01GV_LOG_SFRAME_MAX）：通道名 pad 0x00，
  //   工具端按固定长度切分（不依赖变长 \0 搜索，避免误命中 I/P 帧数据）
  memset(dst, 0x00, W25N01GV_LOG_SFRAME_MAX);
  dst[0] = W25N01GV_LOG_MAGIC0;
  dst[1] = W25N01GV_LOG_MAGIC1;
  dst[2] = W25N01GV_LOG_TYPE_S;
  dst[3] = (uint8_t)(segNum & 0xFF);
  dst[4] = (uint8_t)((segNum >> 8) & 0xFF);
  uint32_t t = millis();
  dst[5] = (uint8_t)(t & 0xFF);
  dst[6] = (uint8_t)((t >> 8) & 0xFF);
  dst[7] = (uint8_t)((t >> 16) & 0xFF);
  dst[8] = (uint8_t)((t >> 24) & 0xFF);
  // 通道名表（ASCII，截断到 CHNAME_MAX，剩余 pad 0x00）
  uint16_t len = 0;
  if (chNames) {
    len = (uint16_t)strlen(chNames);
    if (len > W25N01GV_LOG_CHNAME_MAX) len = W25N01GV_LOG_CHNAME_MAX;
    memcpy(dst + 9, chNames, len);
  }
  // 帧尾 CRC（固定位置：SFRAME_MAX-2）
  uint16_t crc = crc16_ccitt(dst, W25N01GV_LOG_SFRAME_MAX - 2);
  dst[W25N01GV_LOG_SFRAME_MAX - 2] = (uint8_t)(crc & 0xFF);
  dst[W25N01GV_LOG_SFRAME_MAX - 1] = (uint8_t)((crc >> 8) & 0xFF);
}

void W25N01GVLog::buildEFrame(uint8_t *dst, uint16_t segNum, uint32_t durMs, uint32_t frames)
{
  memset(dst, 0xFF, W25N01GV_LOG_EFRAME_SIZE);
  dst[0] = W25N01GV_LOG_MAGIC0;
  dst[1] = W25N01GV_LOG_MAGIC1;
  dst[2] = W25N01GV_LOG_TYPE_E;
  dst[3] = (uint8_t)(segNum & 0xFF);
  dst[4] = (uint8_t)((segNum >> 8) & 0xFF);
  dst[5] = (uint8_t)(durMs & 0xFF);
  dst[6] = (uint8_t)((durMs >> 8) & 0xFF);
  dst[7] = (uint8_t)((durMs >> 16) & 0xFF);
  dst[8] = (uint8_t)((durMs >> 24) & 0xFF);
  dst[9] = (uint8_t)(frames & 0xFF);
  dst[10] = (uint8_t)((frames >> 8) & 0xFF);
  dst[11] = (uint8_t)((frames >> 16) & 0xFF);
  dst[12] = (uint8_t)((frames >> 24) & 0xFF);
  uint16_t crc = crc16_ccitt(dst, W25N01GV_LOG_EFRAME_SIZE - 2);
  dst[13] = (uint8_t)(crc & 0xFF);
  dst[14] = (uint8_t)((crc >> 8) & 0xFF);
}

bool W25N01GVLog::logFlightSegmentStart(uint16_t segNum, const char *chNames)
{
  uint8_t slot[W25N01GV_LOG_SFRAME_MAX];
  buildSFrame(slot, segNum, chNames);
  return pushRawFrame(slot, W25N01GV_LOG_SFRAME_MAX);   // 固定长度
}

bool W25N01GVLog::logFlightSegmentEnd(uint16_t segNum, uint32_t durMs, uint32_t frames)
{
  uint8_t slot[W25N01GV_LOG_EFRAME_SIZE];
  buildEFrame(slot, segNum, durMs, frames);
  return pushRawFrame(slot, W25N01GV_LOG_EFRAME_SIZE);
}

void W25N01GVLog::advanceCursor()
{
  _cursorPage++;
  _pagesWritten++;
  // 段内写满 → 回绕到下一段（或段 0 覆盖）
  uint32_t segPages = (uint32_t)W25N01GV_LOG_SEG_BLOCKS * W25N01GV_PAGES_PER_BLOCK;
  uint32_t segBase = segmentStartPage(_segment);
  if (_cursorPage >= segBase + segPages) {
    _segment = (uint16_t)((_segment + 1) % segmentCount());
    _cursorPage = segmentStartPage(_segment);
    // 覆盖前擦除新段 + 写段头（含游标持久化，每段一次而非每帧）
    if (!writeSegmentHeader(_segment, millis())) {
      // 段头写失败（坏块）→ 跳段
      _segment = (uint16_t)((_segment + 1) % segmentCount());
      _cursorPage = segmentStartPage(_segment);
      writeSegmentHeader(_segment, millis());
    }
    _cursorPage++;
  }
}

void W25N01GVLog::flushPage()
{
#if W25N01GV_LOG_PACK_PAGE
  // ★ 打包到页满或 ring 空（攒页判断在 logService 做）
  uint16_t used = 0;
  while (_buffered > 0 && used + _ringLen[_tail] <= W25N01GV_LOG_PAGE_DATA) {
    memcpy(_pageBuf + used, _ring[_tail], _ringLen[_tail]);
    used += _ringLen[_tail];
    _tail = (uint16_t)((_tail + 1) % W25N01GV_LOG_MAX_FRAMES);
    _buffered--;
    _written++;
  }
  if (used == 0) return;   // 无数据（防御）
#else
  // ★ 每页 1 帧（稳定模式，见 W25N01GV_LOG_PACK_PAGE 注释）
  uint16_t used = 0;
  if (_buffered > 0) {
    memcpy(_pageBuf, _ring[_tail], _ringLen[_tail]);
    used = _ringLen[_tail];
    _tail = (uint16_t)((_tail + 1) % W25N01GV_LOG_MAX_FRAMES);
    _buffered--;
    _written++;
  }
#endif
  // 页尾 pad 0xFF
  if (used < W25N01GV_LOG_PAGE_DATA) {
    memset(_pageBuf + used, 0xFF, W25N01GV_LOG_PAGE_DATA - used);
  }

  // 跳过坏块页（★ 限 64 次防死循环——若整片都坏则放弃本次写入）
  uint16_t skip = 0;
  while (isBlockBad(_cursorPage)) {
    advanceCursor();
    if (++skip > W25N01GV_PAGES_PER_BLOCK) {
      return;   // 连续 64 页坏块，放弃（设备已不可写）
    }
  }

  _serviceBusy = true;
  // ★ D-Cache 维护：H743 上 SPI 外设经 AHB 直读 SRAM（不经 CPU cache），
  //   CPU 刚写入的页数据若还在 cache 里，SPI 会读到旧值。
  //   每页写入前 Clean 页 buffer（2048B 大页必须，v1 小页碰巧没事）。
  SCB_CleanDCache_by_Addr((uint32_t*)_pageBuf, W25N01GV_LOG_PAGE_DATA);
  // 每页 1 帧时只写实际帧长（2048B 传输不稳定，95/51B 稳定）
  uint16_t writeLen = W25N01GV_LOG_PACK_PAGE ? W25N01GV_LOG_PAGE_DATA : used;
  bool ok = _flash.pageProgram(_cursorPage, _pageBuf, writeLen);
  _serviceBusy = false;

  if (ok) {
    advanceCursor();
  } else {
    // 页编程失败 → 该块坏，标记并跳过（帧留在 ring，下 tick 重试新页）
    markBlockBad(_cursorPage);
    advanceCursor();
  }
}

void W25N01GVLog::logService()
{
  if (_serviceBusy) return;

  uint16_t budget = W25N01GV_LOG_MAX_WRITES;
  uint32_t t0 = millis();
  while (budget-- > 0 && _buffered > 0) {
    // ★ 攒页判断：ring 占用 ≥ 半页（~20 帧）或 ring 满或空闲>200ms 才写，
    //   否则等攒够再打包（页利用率高）。空闲冲刷保证记录停止后残余帧落盘。
#if W25N01GV_LOG_PACK_PAGE
    uint16_t halfPage = W25N01GV_LOG_PAGE_DATA / 2 / W25N01GV_LOG_PFRAME_SIZE;  // ~20
    bool idle = (int32_t)(millis() - _lastPushMs) > 200;
    if (_buffered < halfPage && _buffered < W25N01GV_LOG_MAX_FRAMES && !idle) {
      break;   // 未攒够、ring 未满、非空闲 → 等
    }
#endif
    flushPage();
    // ★ 保护：单次 service 总耗时上限（页编程+擦除可能 100ms 级，
    //   防止异常时拖死主循环触发 IWDG）
    if ((int32_t)(millis() - t0) > 250) {
      break;
    }
  }
}

bool W25N01GVLog::format()
{
  for (uint32_t block = 0; block < W25N01GV_BLOCK_COUNT; block++) {
    if (isBlockBad(block * W25N01GV_PAGES_PER_BLOCK)) continue;
    _flash.eraseBlock(block * W25N01GV_PAGES_PER_BLOCK);
  }
  _cursorPage = 0;
  _segment = 0;
  _written = 0;
  _dropped = 0;
  _pagesWritten = 0;
  _buffered = 0;
  _head = _tail = 0;
  _frameCount = 0;
  return true;
}

bool W25N01GVLog::debugReadPage(uint32_t page, uint8_t *buf, uint16_t len)
{
  return _flash.pageRead(page, buf, len);
}
