/*
 * W25N01GV.h - Winbond W25N01GV 1Gbit SPI NAND Flash driver (command layer)
 *
 * Written for stm32duino (STM32 Arduino core). Works on any platform where
 * SPI is available; the bus instance is injected (same pattern as
 * arduino-CAN's CAN.setSPI()).
 *
 * ============ Why a custom driver ============
 *
 * 1. Winbond publishes no MCU driver for SPI NAND (their "Software and
 *    Drivers" downloads need an account and are not MCU driver code).
 * 2. The only Arduino-library candidate (squaresausage/WinbondW25N) has
 *    fatal defects: never enables ECC, no bad-block handling, hard-coded
 *    global SPI, blocking WIP waits with a 15ms timeout, and a
 *    send-buffer-corruption bug (SPI.transfer() overwrites the tx buffer).
 *
 * ============ W25N01GV facts (datasheet) ============
 *
 * - 1Gbit SLC NAND, page = 2048B data + 64B spare, block = 64 pages,
 *   1024 blocks total. Up to 20 factory bad blocks (BBM LUT can remap).
 * - ECC is ENABLED BY DEFAULT after power-up (ECC_E=1 in config reg SR2).
 *   ECC status is reported in SR3 bits 4:5 (00=no err, 01=corrected
 *   1-4bit, 10/11=uncorrectable).
 * - WARNING: blocks are WRITE-PROTECTED by default (SR1 BP bits) —
 *   must clear them first or Page Program sets P-FAIL.
 * - Command set: JEDEC ID 0x9F / Block Erase 0xD8 / Program Data Load
 *   0x02 + Program Execute 0x10 / Page Data Read 0x13 + Read 0x03 /
 *   Status registers: SR1(0xA0 write) SR2(0xB0 write) SR3(0xC0 read).
 *
 * License: MIT
 */

#ifndef W25N01GV_H
#define W25N01GV_H

#include <Arduino.h>
#include <SPI.h>

// ---- geometry ----
#define W25N01GV_PAGE_SIZE       2048U   // data bytes per page
#define W25N01GV_SPARE_SIZE       64U    // spare bytes per page (ECC/BBM)
#define W25N01GV_PAGES_PER_BLOCK   64U
#define W25N01GV_BLOCK_COUNT     1024U
#define W25N01GV_TOTAL_PAGES     (W25N01GV_BLOCK_COUNT * W25N01GV_PAGES_PER_BLOCK)

// ---- command codes ----
#define W25N_CMD_RESET              0xFF
#define W25N_CMD_JEDEC_ID           0x9F
#define W25N_CMD_READ_STATUS        0x05   // + reg addr (0xA0/0xB0/0xC0)
#define W25N_CMD_WRITE_STATUS       0x01   // + reg addr + value
#define W25N_CMD_WRITE_ENABLE       0x06
#define W25N_CMD_WRITE_DISABLE      0x04
#define W25N_CMD_BLOCK_ERASE        0xD8   // + 2-byte page addr (block base)
#define W25N_CMD_PROG_DATA_LOAD     0x02   // + 2-byte column + data (cache)
#define W25N_CMD_PROG_EXECUTE       0x10   // + 2-byte page addr
#define W25N_CMD_PAGE_DATA_READ     0x13   // + 2-byte page addr (to cache)
#define W25N_CMD_READ               0x03   // + 2-byte column + dummy + data
#define W25N_CMD_BB_MANAGE          0xA1
#define W25N_CMD_LAST_ECC_FAIL      0xA9

// ---- status register addresses (sent after 0x05/0x01) ----
#define W25N_REG_PROTECT            0xA0   // SR1: BP0-3 write protection
#define W25N_REG_CONFIG             0xB0   // SR2: ECC_E, BUFF, ...
#define W25N_REG_STATUS             0xC0   // SR3: BUSY, WEL, ECC-1/0, P-FAIL, E-FAIL, LUT-F

// SR3 bit masks
#define W25N_SR3_BUSY               (1U << 0)
#define W25N_SR3_WEL                (1U << 1)
#define W25N_SR3_ECC_SHIFT          4U
#define W25N_SR3_ECC_MASK           (0x3U << W25N_SR3_ECC_SHIFT)
#define W25N_SR3_ECC_NOERR          0x0U
#define W25N_SR3_ECC_CORRECTED      0x1U   // 1-4 bits corrected
#define W25N_SR3_ECC_UNCORRECTABLE  0x2U   // >4 bits, page lost
#define W25N_SR3_PFAIL              (1U << 6)
#define W25N_SR3_EFAIL              (1U << 7)
#define W25N_SR3_LUTF               (1U << 2)

// JEDEC ID expected bytes
#define W25N01GV_MANUFACTURER       0xEF   // Winbond
#define W25N01GV_DEVICE_ID          0xAA   // W25N01GV
#define W25N01GV_DEVICE_ID2         0x21

// default SPI speed for flash transactions
#ifndef W25N01GV_SPI_SPEED
#define W25N01GV_SPI_SPEED          25000000UL
#endif

/**
 * @brief W25N01GV command-layer driver
 *
 * All operations are BLOCKING with busy-polling (page program ~0.7ms,
 * block erase ~2ms). SPI transactions are protected with
 * noInterrupts()/interrupts() so a frame write can't be torn by the
 * 2kHz scheduler ISR (flash uses its own SPIClass instance, isolated
 * from SPI1/IMU).
 */
class W25N01GV {
public:
  /**
   * @param bus      SPI instance (e.g. a dedicated SPIClass for SPI3)
   * @param csPin    chip-select Arduino pin
   */
  W25N01GV(SPIClass &bus, uint8_t csPin);

  /**
   * @brief Initialize: verify JEDEC ID and clear write protection
   * @return true if a W25N01GV was detected and protection cleared
   */
  bool begin();

  /// JEDEC ID bytes (valid after begin)
  uint8_t manufacturerId() const { return _manId; }
  uint8_t deviceId() const { return _devId; }
  uint8_t deviceId2() const { return _devId2; }

  /// true after begin() detected the chip
  bool isPresent() const { return _present; }

  /// 最近一次 pageProgram 的错误类型（0=成功, 1=busy超时, 2=P-FAIL, 3=参数错误）
  uint8_t lastProgError() const { return _lastProgErr; }

  /// read current SR3 status byte
  uint8_t readStatus();

  /// wait until device not busy (poll BUSY bit), timeout ms
  bool waitReady(uint32_t timeout_ms);

  /**
   * @brief Erase one block (64 pages)
   * @param pageAddr any page inside the block
   * @return true on success (E-FAIL cleared)
   */
  bool eraseBlock(uint32_t pageAddr);

  /**
   * @brief Program one page from RAM buffer (max 2048 bytes)
   * @return true on success (P-FAIL cleared)
   */
  bool pageProgram(uint32_t pageAddr, const uint8_t *data, uint16_t len);

  /**
   * @brief Read one page (up to 2048 bytes) from flash into buffer
   * @return true on success (no uncorrectable ECC error)
   */
  bool pageRead(uint32_t pageAddr, uint8_t *data, uint16_t len);

  /// ECC status of the last pageRead (SR3 bits 4:5)
  uint8_t lastEccStatus() const { return _lastEcc; }

  /// true if the last pageRead had uncorrectable ECC error
  bool lastEccUncorrectable() const { return (_lastEcc == W25N_SR3_ECC_UNCORRECTABLE); }

  /**
   * @brief Read spare-area bytes of a page (bad-block markers live there)
   *
   * Uses Read From Cache with column = 2048 (spare area start).
   * @param pageAddr page address
   * @param data     output buffer
   * @param len      bytes to read (max W25N01GV_SPARE_SIZE)
   */
  bool readSpare(uint32_t pageAddr, uint8_t *data, uint16_t len);

  /// reset the device
  void reset();

private:
  SPIClass &_bus;
  uint8_t _cs;
  uint8_t _manId, _devId, _devId2;
  bool _present;
  uint8_t _lastEcc;
  uint8_t _lastProgErr;   // 最近一次 pageProgram 错误类型

  void select() { digitalWrite(_cs, LOW); }
  void deselect() { digitalWrite(_cs, HIGH); }
  void writeEnable();
  uint8_t readReg(uint8_t regAddr);
  void writeReg(uint8_t regAddr, uint8_t val);

};

#endif // W25N01GV_H
