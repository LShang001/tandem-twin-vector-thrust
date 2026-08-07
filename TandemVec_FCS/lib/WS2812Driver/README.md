# WS2812Driver

Cross-platform WS2812 / NeoPixel LED driver for **stm32duino (STM32 Arduino core)** with
**STM32H7 TIM+DMA hardware acceleration**. Also compiles on any other Arduino-compatible
platform via a generic fallback.

This library solves two problems no other Arduino library handles correctly on STM32:

| Problem | Adafruit NeoPixel / others | **WS2812Driver** |
|---|---|---|
| **High-pin BSRR bug**: `portSetRegister()` returns the 32-bit BSRR, but the library writes it through a `volatile uint8_t*` → only GPIO 0-7 can be driven. Pins like PD15, PE9 never work. | ❌ broken on pins ≥ 8 | ✅ writes BSRR as full 32-bit with correct per-pin mask |
| **STM32H7 TIM4_CH4 missing DMA**: the H7 DMAMUX table has no `TIM4_CH4` request (only CH1/2/3/UP), so `HAL_TIM_PWM_Start_DMA(...CH4)` never triggers. | ❌ no workaround | ✅ TIM Update DMA bypass writes CCR4 directly |

## Features

- **Pixel API compatible with Adafruit NeoPixel** (`setPixelColor`, `fill`, `setBrightness`, `show`, ...)
- **Any pin** (bit-bang mode) — including high pins on stm32duino
- **Any LED count** — dynamic pixel buffer
- **Configurable color order** (GRB / RGB / BRG)
- **STM32H7 TIM+DMA mode** — CPU-free frame transmission, any timer + any channel
  (CH4 handled via the UP-DMA bypass, works on all H7 timers)
- **DWT-free timing** — pure NOP-loop bit-bang (DWT cycle-counter waits can deadlock
  under `-O3` + disabled interrupts on Cortex-M7)
- **D-Cache safe** — DMA buffer 32-byte aligned (`aligned_alloc`) + `SCB_CleanDCache_by_Addr`
- **Flicker-free** — the timer DMA request bit (DIER.UDE) is disabled right after
  each transfer (leaving it set wedges the DMA state machine, causing skipped
  frames). Transfers use `HAL_DMA_Start` + `HAL_DMA_PollForTransfer` (polling,
  ~130 µs per frame at 1 LED) — the IT path is NOT used because stm32duino's
  startup file leaves `DMA1_Stream6_IRQHandler` as a weak alias to
  `Default_Handler` (no interrupt ever fires).

## Platform matrix

| Platform | Bit-bang | TIM+DMA |
|---|---|---|
| stm32duino (all STM32 MCUs) | ✅ BSRR + NOP loop (precise) | ❌ (H7 only) |
| STM32H7 + HAL DMA | ✅ (same) | ✅ CPU-free |
| Other Arduino boards (AVR, ESP8266, ...) | ✅ digitalWrite fallback (functional) | ❌ |

## Quick start

```cpp
#include <WS2812Driver.h>

// single LED on PD15 (stm32duino pin number 30 on WeAct H743)
WS2812Driver strip(30, 1);   // (pin, numPixels)

// STM32H7: enable CPU-free DMA mode (optional, any timer + channel)
// strip.beginDma(TIM4, TIM_CHANNEL_4);   // TIM4_CH4 uses the UP-DMA bypass

void setup() {
  strip.begin();
}

void loop() {
  strip.setPixelColor(0, 255, 0, 0);   // red (r, g, b)
  strip.show();
  delay(500);
  strip.setPixelColor(0, 0, 0, 255);   // blue
  strip.show();
  delay(500);
}
```

### Wiring

```
WS2812 DIN  ──► MCU GPIO pin (any, e.g. PD15)
WS2812 VDD  ──► 5V (many strips need 5V; check your LED's spec)
WS2812 GND  ──► GND (common ground with MCU!)
```

> WS2812 data lines are 3.3V-logic compatible on most boards, but always
> verify against your LED's datasheet (some need a level shifter).

## STM32H7 DMA mode

```cpp
WS2812Driver strip(30, 8);              // 8 LEDs on PD15
strip.beginDma(TIM4, TIM_CHANNEL_4);    // TIM4_CH4 → UP-DMA bypass
strip.begin();
```

- The timer must be free (not used by anything else).
- CH1/2/3 also work — they use the same UP-DMA bypass writing their CCR.
- `MODE_TIM_DMA` is selected automatically when `beginDma()` succeeded;
  `setMode(MODE_BITBANG)` switches back at runtime.
- On non-H7 platforms `beginDma()` returns `false` and bit-bang is used.

### Why CH4 needs a bypass (STM32H7)

The H7 DMAMUX1 request table (RM0433) lists for TIM4 only:
`TIM4_CH1, TIM4_CH2, TIM4_CH3, TIM4_UP` — **no TIM4_CH4**. The HAL's
`HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_4, ...)` sets `DIER.CC4DE` but there is
no DMAMUX route, so no DMA transfer ever happens.

**Workaround:** every H7 timer *does* have a `TIMx_UP` DMA request. We program the
DMA to write the pre-encoded CCR value into `TIMx->CCR4` once per timer overflow
(800kHz), so each PWM period outputs the WS2812 bit pulse. Same trick applies to
any channel — the DMA destination is just `CCR1..CCR4`.

## Timing parameters (bit-bang mode)

Derived from `F_CPU` at compile time:

| Parameter | Value | Derivation |
|---|---|---|
| bit period | 1.25 µs | WS2812 800 kHz |
| T1H (bit=1 high) | ~0.8 µs | `F_CPU/3e6 * 4/5` NOP loops − calibration |
| T0H (bit=0 high) | ~0.4 µs | `F_CPU/3e6 * 2/5` NOP loops − calibration |
| reset pulse | ~50 µs | 50 × `F_CPU/3e6` loops |

The calibration constant (`WS2812_BSRR_CAL_LOOPS`, default 18) compensates for the
BSRR write + branch overhead. If your LED shows wrong colors, tune it:

```cpp
#define WS2812_BSRR_CAL_LOOPS 20   // before #include <WS2812Driver.h>
```

## Known limitations

- Generic fallback (non-STM32) uses `digitalWrite` — timing is not spec-grade,
  keep LED count small or use a hardware driver on those platforms.
- `MODE_TIM_DMA` requires a free timer and DMA1_Stream6 (hard-coded stream;
  any stream works on H7 thanks to DMAMUX1, only one is wired for simplicity).

## License

MIT — see [LICENSE](LICENSE).

## References / debug notes

This library was born from a real debugging session on an STM32H743 board
(2026-08-08). The full investigation — including the Adafruit high-pin bug,
the TIM4_CH4 missing-DMA discovery, the DWT deadlock, the ARR overflow
(`clk/800` vs `clk/800000`), and the DMA-UDE flicker fix — is documented in
the project's `docs/memory/2026-08-08-WS2812驱动调试.md`.
