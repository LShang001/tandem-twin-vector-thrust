# W25N01GV

Winbond **W25N01GV** 1Gbit SPI NAND Flash driver + sequential log layer for
**stm32duino** (STM32 Arduino core), usable on any platform with SPI.

## Why a custom driver

1. Winbond publishes **no MCU driver** for SPI NAND (their "Software and
   Drivers" downloads require an account and are not MCU driver code).
2. The only Arduino candidate (squaresausage/WinbondW25N) has fatal defects:
   never enables ECC, no bad-block handling, hard-coded global SPI, blocking
   WIP waits with 15ms timeout, and a tx-buffer-corruption bug.

## Features

- **Command layer** (`W25N01GV`): JEDEC ID check, block erase, page program,
  page read with **ECC status** (SR3), busy-polling with timeout.
- **Write-protection handling**: clears SR1 BP bits (default = all blocks
  write-protected; page program fails with P-FAIL otherwise).
- **ECC enabled by default** (power-on ECC_E=1); uncorrectable ECC errors are
  reported per read.
- **Log layer** (`W25N01GVLog`): RAM ring buffer + background page writes with
  per-tick budget (`W25N01GV_LOG_MAX_WRITES`), sequential cursor, **cursor
  persistence** across power cycles (super page), **bad-block skip** (whole
  block skipped on P-FAIL / uncorrectable ECC).
- Frame format: `magic(0xAA55) + seq(4) + t_ms(4) + payload(84) + CRC8`.

## Quick start

```cpp
#include <W25N01GV.h>
#include <W25N01GV_log.h>

// SPI3 on WeAct H743: PB2=MOSI, PC11=MISO, PC10=SCK, PA15=CS
// NOTE: SSEL omitted (NC) — hardware NSS conflicts with manual CS control
SPIClass FLASH_SPI(PB2, PC11, PC10);
W25N01GV flash(FLASH_SPI, PA15);
W25N01GVLog flashLog(flash);

void setup() {
  flash.begin();        // verify JEDEC ID + clear write protection
  flashLog.begin();     // resume cursor from super page
}

// fast task (e.g. 200Hz): copy frame into ring buffer (never blocks)
void fastTask() {
  uint8_t payload[W25N01GV_LOG_PAYLOAD];
  // ... fill payload ...
  flashLog.logPush(payload);
}

// low-priority task (e.g. 100Hz): flush ring to NAND (bounded budget)
void lowPrioTask() {
  flashLog.logService();
}
```

## Wiring (SPI3)

| Signal | MCU pin |
|---|---|
| MOSI | PB2 |
| MISO | PC11 |
| SCK  | PC10 |
| CS   | PA15 (manual, software NSS) |
| VDD  | 3.3V |
| GND  | GND |

## Known limitations

- Blocking busy-polling (page program ~0.7ms, block erase ~2ms) — fine for a
  bounded-per-tick background task, not for hard-real-time contexts.
- Bad blocks are **skipped** (simple, robust); the hardware BBM LUT remapping
  (0xA1) is not used yet.
- `Crc8::calc` length is `uint8_t` — frames must stay < 255 bytes (default 95).

## License

MIT — see [LICENSE](LICENSE).
