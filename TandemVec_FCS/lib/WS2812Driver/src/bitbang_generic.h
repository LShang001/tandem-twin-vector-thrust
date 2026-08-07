/*
 * bitbang_generic.h - generic Arduino bit-bang back-end (fallback)
 *
 * Used on platforms without the STM32 BSRR fast path. Timing is built
 * from delayMicroseconds() (1us resolution) with a simple assembly-free
 * loop; this is functional on any Arduino-compatible board but is NOT
 * timing-critical-grade (the WS2812 spec tolerance is ~+/-150ns).
 *
 * Recommended use on these platforms: keep LED count small, or use a
 * hardware peripheral driver (ESP32 RMT, RP2040 PIO) instead.
 *
 * License: MIT
 */

#ifndef WS2812_BITBANG_GENERIC_H
#define WS2812_BITBANG_GENERIC_H

#if !WS2812_HAVE_STM32_BSRR

// pulse widths in microseconds (WS2812 800kHz)
#define WS2812_GEN_T1H_US  1   // ~0.8us (rounded to 1us granularity)
#define WS2812_GEN_T0H_US  0   // ~0.4us -> 0 rounds down, rely on loop
#define WS2812_GEN_BIT_US  1   // total bit period ~1.25us

inline bool WS2812Driver::beginBitbang()
{
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  return true;
}

inline void WS2812Driver::showBitbang()
{
  // reset pulse: low for ~50us
  digitalWrite(_pin, LOW);
  delayMicroseconds(50);

  for (uint32_t i = 0; i < (uint32_t)_numPixels * 3; i++) {
    uint8_t pix = _pixels[i];
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
      digitalWrite(_pin, HIGH);
      if (pix & mask) {
        delayMicroseconds(WS2812_GEN_T1H_US);
      } else {
        delayMicroseconds(WS2812_GEN_T0H_US);
      }
      digitalWrite(_pin, LOW);
      delayMicroseconds(WS2812_GEN_BIT_US);
    }
  }

  digitalWrite(_pin, LOW);
  delayMicroseconds(50);   // frame end reset
}

#endif // !WS2812_HAVE_STM32_BSRR
#endif // WS2812_BITBANG_GENERIC_H
