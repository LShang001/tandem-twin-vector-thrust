/*
 * bitbang_stm32.h - STM32 (stm32duino) BSRR bit-bang back-end
 *
 * Drives WS2812 via direct BSRR register writes with a NOP-loop delay.
 *
 * Why BSRR instead of digitalWrite / portSetRegister:
 * - digitalWrite is far too slow for the 800kHz WS2812 protocol.
 * - Adafruit NeoPixel writes BSRR through a `volatile uint8_t*` which
 *   only affects GPIO 0-7 on stm32duino -> high pins (PD8-15, PE8-15,
 *   ...) never work. Here the full 32-bit register is written with the
 *   correct per-pin bit mask.
 *
 * Why NOP loops instead of DWT cycle counter:
 * - DWT-based waits can deadlock under -O3 + disabled interrupts on
 *   Cortex-M7 (observed on STM32H743). NOP loops depend only on CPU
 *   pipeline cycles.
 *
 * Timing parameters are derived from F_CPU so the library works on any
 * STM32 clock without manual tuning. A calibration constant (in NOP-loop
 * iterations) accounts for BSRR write + branch overhead; it can be
 * overridden by defining WS2812_BSRR_CAL_LOOPS before including this
 * header.
 *
 * License: MIT
 */

#ifndef WS2812_BITBANG_STM32_H
#define WS2812_BITBANG_STM32_H

#if WS2812_HAVE_STM32_BSRR

// --- timing calibration ---
// The NOP delay loop consumes ~3 cycles per iteration (subs + bne).
// One WS2812 bit = 1.25us, T0H ~ 0.4us, T1H ~ 0.8us.
//
//   loops_per_us = F_CPU / 3e6
//   T1H loops    = 0.8 * loops_per_us - CAL   (CAL = BSRR+branch overhead)
//
// CAL was measured on STM32H743 @480MHz: T1H=110, T0H=45 loops,
//   T1H_ideal = 0.8*160 = 128 -> CAL ~ 18 loops (~0.11us overhead)
#ifndef WS2812_BSRR_CAL_LOOPS
#define WS2812_BSRR_CAL_LOOPS 18
#endif

// bit period and pulse lengths in NOP-loop iterations (derived from F_CPU)
// integer math only (compile-time constants):
//   loops_per_us = F_CPU / 3e6
//   T1H = 0.8us * loops_per_us - CAL ; T0H = 0.4us * loops_per_us - CAL
#define WS2812_LOOPS_PER_US   ((F_CPU) / 3000000UL)
#define WS2812_T1H_LOOPS      (((uint32_t)((WS2812_LOOPS_PER_US) * 4UL / 5UL) > (uint32_t)WS2812_BSRR_CAL_LOOPS) \
                               ? (uint32_t)((WS2812_LOOPS_PER_US) * 4UL / 5UL) - (uint32_t)WS2812_BSRR_CAL_LOOPS \
                               : 0)
#define WS2812_T0H_LOOPS      (((uint32_t)((WS2812_LOOPS_PER_US) * 2UL / 5UL) > (uint32_t)WS2812_BSRR_CAL_LOOPS) \
                               ? (uint32_t)((WS2812_LOOPS_PER_US) * 2UL / 5UL) - (uint32_t)WS2812_BSRR_CAL_LOOPS \
                               : 0)
#define WS2812_T1L_LOOPS      ((uint32_t)((WS2812_LOOPS_PER_US) * 45UL / 100UL))
#define WS2812_T0L_LOOPS      ((uint32_t)((WS2812_LOOPS_PER_US) * 85UL / 100UL))
#define WS2812_RESET_LOOPS    ((uint32_t)((WS2812_LOOPS_PER_US) * 50UL))

// --- back-end implementation (inline member functions) ---

inline bool WS2812Driver::beginBitbang()
{
  // pin masks were resolved in the constructor (needed by both modes)
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  return (_port != nullptr);
}

// precise delay: 3-cycle loop (subs + bne)
static inline void ws2812_delay_loops(uint32_t loops)
{
  __asm volatile(
    "1: subs %0, %0, #1 \n"
    "   bne 1b          \n"
    : "+r"(loops) : : "cc"
  );
}

inline void WS2812Driver::showBitbang()
{
  noInterrupts();

  // reset pulse: low for >50us
  _port->BSRR = _resetMask;
  ws2812_delay_loops(WS2812_RESET_LOOPS);

  // send all pixels, MSB first
  for (uint32_t i = 0; i < (uint32_t)_numPixels * 3; i++) {
    uint8_t pix = _pixels[i];
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
      // rising edge
      _port->BSRR = _setMask;

      if (pix & mask) {
        ws2812_delay_loops(WS2812_T1H_LOOPS);
      } else {
        ws2812_delay_loops(WS2812_T0H_LOOPS);
      }

      // falling edge
      _port->BSRR = _resetMask;

      // remaining bit period (low time)
      if (pix & mask) {
        ws2812_delay_loops(WS2812_T1L_LOOPS);
      } else {
        ws2812_delay_loops(WS2812_T0L_LOOPS);
      }
    }
  }

  // frame end reset
  _port->BSRR = _resetMask;
  interrupts();
}

#endif // WS2812_HAVE_STM32_BSRR
#endif // WS2812_BITBANG_STM32_H
