/*
 * WS2812Driver.h - Cross-platform WS2812/NeoPixel LED driver
 *
 * Written for stm32duino (STM32 Arduino core) with special handling for
 * STM32H7, but also compiles on any Arduino-compatible platform via a
 * generic fallback.
 *
 * ============ Why this library exists ============
 *
 * 1. Adafruit NeoPixel's `__arm__` bit-bang path writes the BSRR register
 *    through a `volatile uint8_t*`, which only affects GPIO 0-7 on
 *    stm32duino. High pins (PD8-15, PE8-15, ...) can never be driven.
 *    This library writes BSRR as a full 32-bit register with the correct
 *    per-pin bit mask.
 *
 * 2. On STM32H7, timer TIM4 has NO DMAMUX request for its CH4 output
 *    (only CH1/CH2/CH3/UP exist). `HAL_TIM_PWM_Start_DMA(htim,
 *    TIM_CHANNEL_4, ...)` therefore never triggers on TIM4. This library
 *    works around it by using the TIM Update (UP) DMA request to write
 *    the CCR4 register directly.
 *
 * 3. DWT cycle-counter based timing deadlocks under -O3 + disabled
 *    interrupts on Cortex-M7. This library uses pure NOP-loop delays for
 *    bit-bang mode (no DWT, no SysTick dependency).
 *
 * ============ Platform matrix (compile-time auto-selected) ============
 *
 *   Platform               | Bit-bang        | DMA acceleration
 *   -----------------------|-----------------|-------------------
 *   stm32duino (all MCUs)  | BSRR + NOP loop | - (H7 only, see below)
 *   other Arduino boards   | digitalWrite    | -
 *   STM32H7 + HAL DMA      | (same)          | TIM_UP DMA bypass
 *
 * See README.md for full documentation, wiring and timing details.
 *
 * License: MIT
 */

#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <Arduino.h>

// ---- platform feature flags (compile-time) ----

// STM32 (stm32duino): BSRR direct-write bit-bang available
#if defined(ARDUINO_ARCH_STM32)
#define WS2812_HAVE_STM32_BSRR 1
#else
#define WS2812_HAVE_STM32_BSRR 0
#endif

// STM32H7 + HAL DMA: TIM+DMA acceleration available
#if defined(ARDUINO_ARCH_STM32) && defined(STM32H7xx) && defined(HAL_DMA_MODULE_ENABLED)
#define WS2812_HAVE_H7_DMA 1
#include "stm32h7xx_hal.h"
#else
#define WS2812_HAVE_H7_DMA 0
#endif

class WS2812Driver {
public:
  // WS2812 pixel byte order (GRB is the standard for WS2812B)
  enum ColorOrder {
    ORDER_GRB,
    ORDER_RGB,
    ORDER_BRG
  };

  // Transmission back-end
  enum Mode {
    MODE_BITBANG,   // software bit-bang (any pin, any platform)
    MODE_TIM_DMA    // hardware TIM + DMA (STM32H7 only, if available)
  };

  /**
   * @brief Constructor
   * @param pin        Arduino pin number connected to WS2812 DIN
   * @param numPixels  number of LEDs in the strip
   * @param order      pixel byte order (default GRB)
   */
  WS2812Driver(uint32_t pin, uint16_t numPixels, ColorOrder order = ORDER_GRB);

  ~WS2812Driver();

  /**
   * @brief Initialize the driver
   *
   * Auto-selects the best available transmission mode:
   * - H7: tries TIM+DMA (if beginDma() was called first), falls back to
   *   bit-bang
   * - others: bit-bang only
   *
   * @return true on success
   */
  bool begin();

  /**
   * @brief Send the current pixel buffer to the strip
   *
   * Blocks until the frame has been fully transmitted.
   * On MODE_BITBANG this disables interrupts for the frame duration
   * (numPixels * 24 * 1.25us, e.g. ~30us for 1 LED).
   */
  void show();

  // ---- pixel API (Adafruit NeoPixel compatible) ----

  void setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b);
  void setPixelColor(uint16_t i, uint32_t color);  // 0xRRGGBB
  uint32_t getPixelColor(uint16_t i) const;
  void fill(uint8_t r, uint8_t g, uint8_t b);
  void clear();
  void setBrightness(uint8_t b);   // 0-255, scales the whole strip
  uint8_t getBrightness() const;
  uint16_t numPixels() const;

  // ---- transmission mode control ----

  /**
   * @brief Configure TIM+DMA mode on STM32H7
   *
   * Any timer channel can be used:
   * - CH1/CH2/CH3: standard capture/compare DMA
   * - CH4:         TIM Update DMA bypass writing CCR4 (works on all H7
   *                timers; required on TIM4 which lacks a CH4 request)
   *
   * @param tim     timer instance (e.g. TIM4)
   * @param channel TIM_CHANNEL_x
   * @return true if DMA mode is now active; false on non-H7 or failure
   */
  bool beginDma(TIM_TypeDef *tim, uint32_t channel);

  /// true when the TIM+DMA back-end is active
  bool isDmaActive() const;

  /// manual mode switch (MODE_TIM_DMA is ignored if DMA not ready)
  void setMode(Mode m);

  /// current transmission mode
  Mode getMode() const;

  // ---- debug helpers (used by the FCS debug console) ----

  /// DMA buffer address (0 if DMA not ready) — for register-level debugging
  uint32_t dmaBufAddr() const;

  /// DMA buffer value at index i (0..dmaBufLen-1)
  uint32_t dmaBufAt(uint32_t i) const;

  /// DMA buffer length in words
  uint32_t dmaBufLen() const;

  /// DMA transfer destination address (CCR register)
  uint32_t dmaDstAddr() const;

private:
  // ---- pixel state ----
  uint32_t _pin;
  uint16_t _numPixels;
  ColorOrder _order;
  uint8_t *_pixels;       // raw buffer, _numPixels*3 bytes (order-encoded)
  uint8_t _brightness;    // 0-255 global brightness
  Mode _mode;

  // ---- internal helpers ----
  void transmitFrame();
  void colorToRaw(uint8_t r, uint8_t g, uint8_t b,
                  uint8_t &b0, uint8_t &b1, uint8_t &b2) const;

  // ---- platform state (only present when feature compiled in) ----

#if WS2812_HAVE_STM32_BSRR
  GPIO_TypeDef *_port;    // GPIO port of the pin
  uint32_t _setMask;      // BSRR SET bit mask (STM_GPIO_PIN value, e.g. 0x8000)
  uint32_t _resetMask;    // BSRR RESET bit mask (_setMask << 16)
#endif

#if WS2812_HAVE_H7_DMA
  bool _dmaReady;             // DMA back-end initialized
  bool _dmaActive;            // DMA back-end currently selected
  TIM_HandleTypeDef *_htim;   // timer handle (owned by lib)
  DMA_HandleTypeDef _hdma;    // DMA handle (owned by lib)
  uint32_t *_dmaBuf;          // pre-encoded CCR values (aligned 32)
  uint16_t _dmaBufLen;        // capacity in uint32 words
  uint32_t _ccr1, _ccr0;      // CCR values for bit 1 / bit 0
  uint32_t _dmaChannel;       // TIM_CHANNEL_x used for DMA mode
  static WS2812Driver *_self; // DMA IRQ callback -> instance
  static void dmaXferComplete(DMA_HandleTypeDef *hdma);  // IRQ callback
#endif

  // ---- platform back-end declarations (implemented in platform .h) ----
  bool beginBitbang();
  void showBitbang();

#if WS2812_HAVE_H7_DMA
  bool beginDmaPlatform(TIM_TypeDef *tim, uint32_t channel);
  void showDmaPlatform();
  // switch the pin between GPIO output (bit-bang) and timer AF (DMA)
  void configurePinGpio();
  void configurePinAf(TIM_TypeDef *tim);
#endif
};

// ---- platform implementation headers ----
// Each file self-guards on platform macros and defines the back-end
// member functions as `inline` (safe to include from multiple TUs).
#include "bitbang_stm32.h"    // WS2812_HAVE_STM32_BSRR: BSRR + NOP loop
#include "bitbang_generic.h"  // !WS2812_HAVE_STM32_BSRR: digitalWrite
#include "dma_stm32h7.h"      // WS2812_HAVE_H7_DMA: TIM+DMA acceleration

#endif // WS2812_DRIVER_H
