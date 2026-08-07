/*
 * W25N01GV.cpp - command-layer implementation
 *
 * License: MIT
 */

#include "W25N01GV.h"

W25N01GV::W25N01GV(SPIClass &bus, uint8_t csPin)
  : _bus(bus), _cs(csPin),
    _manId(0), _devId(0), _devId2(0), _present(false), _lastEcc(0)
{
}

bool W25N01GV::begin()
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  _bus.begin();
  _bus.beginTransaction(SPISettings(W25N01GV_SPI_SPEED, MSBFIRST, SPI_MODE0));

  // ---- reset + read JEDEC ID ----
  reset();
  {
    uint8_t buf[6] = {W25N_CMD_JEDEC_ID, 0x00, 0x00, 0x00, 0x00, 0x00};
    noInterrupts();
    select();
    _bus.transfer(buf, sizeof(buf));   // tx buffer is overwritten with rx
    deselect();
    interrupts();
    _manId = buf[2];
    _devId = buf[3];
    _devId2 = buf[4];
  }

  if (_manId == W25N01GV_MANUFACTURER &&
      _devId == W25N01GV_DEVICE_ID &&
      _devId2 == W25N01GV_DEVICE_ID2) {
    _present = true;
  } else {
    return false;
  }

  // ---- clear block write protection (SR1 BP0-3) ----
  // W25N01GV powers up with all blocks write-protected; page program
  // fails (P-FAIL) until BP bits are cleared. Also leave ECC_E as-is
  // (default 1 = enabled).
  writeEnable();
  writeReg(W25N_REG_PROTECT, 0x00);

  return true;
}

void W25N01GV::reset()
{
  uint8_t cmd = W25N_CMD_RESET;
  noInterrupts();
  select();
  _bus.transfer(&cmd, 1);
  deselect();
  interrupts();
  delayMicroseconds(600);   // tRST max 500us
}

uint8_t W25N01GV::readReg(uint8_t regAddr)
{
  uint8_t buf[3] = {W25N_CMD_READ_STATUS, regAddr, 0x00};
  noInterrupts();
  select();
  _bus.transfer(buf, sizeof(buf));
  deselect();
  interrupts();
  return buf[2];
}

void W25N01GV::writeReg(uint8_t regAddr, uint8_t val)
{
  uint8_t buf[3] = {W25N_CMD_WRITE_STATUS, regAddr, val};
  noInterrupts();
  select();
  _bus.transfer(buf, sizeof(buf));
  deselect();
  interrupts();
}

void W25N01GV::writeEnable()
{
  uint8_t cmd = W25N_CMD_WRITE_ENABLE;
  noInterrupts();
  select();
  _bus.transfer(&cmd, 1);
  deselect();
  interrupts();
}

uint8_t W25N01GV::readStatus()
{
  return readReg(W25N_REG_STATUS);
}

bool W25N01GV::waitReady(uint32_t timeout_ms)
{
  uint32_t start = millis();
  do {
    uint8_t st = readStatus();
    if (!(st & W25N_SR3_BUSY)) {
      return true;
    }
  } while ((millis() - start) < timeout_ms);
  return false;
}

bool W25N01GV::eraseBlock(uint32_t pageAddr)
{
  if (pageAddr >= W25N01GV_TOTAL_PAGES) return false;
  if (!waitReady(100)) return false;

  uint8_t buf[4] = {
    W25N_CMD_BLOCK_ERASE,
    0x00,
    (uint8_t)((pageAddr >> 8) & 0xFF),
    (uint8_t)(pageAddr & 0xFF)
  };
  writeEnable();
  noInterrupts();
  select();
  _bus.transfer(buf, sizeof(buf));
  deselect();
  interrupts();

  if (!waitReady(100)) return false;   // block erase ~2ms

  uint8_t st = readStatus();
  return !(st & W25N_SR3_EFAIL);
}

bool W25N01GV::pageProgram(uint32_t pageAddr, const uint8_t *data, uint16_t len)
{
  if (pageAddr >= W25N01GV_TOTAL_PAGES) return false;
  if (len == 0 || len > W25N01GV_PAGE_SIZE) return false;
  if (!waitReady(100)) return false;

  // ---- 1. program data load (0x02, column 0) into cache ----
  {
    uint8_t cmdbuf[3] = {W25N_CMD_PROG_DATA_LOAD, 0x00, 0x00};
    writeEnable();
    noInterrupts();
    select();
    _bus.transfer(cmdbuf, sizeof(cmdbuf));
    _bus.transfer((void*)data, nullptr, len);   // tx only, don't corrupt data
    deselect();
    interrupts();
  }

  // ---- 2. program execute (0x10, page addr) ----
  {
    uint8_t buf[4] = {
      W25N_CMD_PROG_EXECUTE,
      0x00,
      (uint8_t)((pageAddr >> 8) & 0xFF),
      (uint8_t)(pageAddr & 0xFF)
    };
    writeEnable();
    noInterrupts();
    select();
    _bus.transfer(buf, sizeof(buf));
    deselect();
    interrupts();
  }

  if (!waitReady(100)) return false;   // page program ~0.7ms

  uint8_t st = readStatus();
  return !(st & W25N_SR3_PFAIL);
}

bool W25N01GV::pageRead(uint32_t pageAddr, uint8_t *data, uint16_t len)
{
  if (pageAddr >= W25N01GV_TOTAL_PAGES) return false;
  if (len == 0 || len > W25N01GV_PAGE_SIZE) return false;
  if (!waitReady(100)) return false;

  // ---- 1. page data read (0x13): flash page -> internal cache ----
  {
    uint8_t buf[4] = {
      W25N_CMD_PAGE_DATA_READ,
      0x00,
      (uint8_t)((pageAddr >> 8) & 0xFF),
      (uint8_t)(pageAddr & 0xFF)
    };
    noInterrupts();
    select();
    _bus.transfer(buf, sizeof(buf));
    deselect();
    interrupts();
  }

  if (!waitReady(100)) return false;

  // ---- 2. read from cache (0x03, column 0, dummy byte) ----
  {
    uint8_t cmdbuf[4] = {W25N_CMD_READ, 0x00, 0x00, 0x00};
    noInterrupts();
    select();
    _bus.transfer(cmdbuf, sizeof(cmdbuf));
    _bus.transfer(nullptr, data, len);
    deselect();
    interrupts();
  }

  _lastEcc = (readStatus() & W25N_SR3_ECC_MASK) >> W25N_SR3_ECC_SHIFT;
  return (_lastEcc != W25N_SR3_ECC_UNCORRECTABLE);
}
