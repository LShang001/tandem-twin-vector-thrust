/*
 * WS2812Driver.cpp - core implementation (platform-independent)
 *
 * License: MIT
 */

#include "WS2812Driver.h"
#include <cstdlib>   // aligned_alloc / free (DMA buffer)

// ------------------------------------------------------------------
// static member definition (only when H7 DMA compiled in)
// ------------------------------------------------------------------
#if WS2812_HAVE_H7_DMA
WS2812Driver *WS2812Driver::_self = nullptr;
#endif

// ------------------------------------------------------------------
// construction / destruction
// ------------------------------------------------------------------
WS2812Driver::WS2812Driver(uint32_t pin, uint16_t numPixels, ColorOrder order)
  : _pin(pin),
    _numPixels(numPixels),
    _order(order),
    _pixels(nullptr),
    _brightness(255),
    _mode(MODE_BITBANG)
#if WS2812_HAVE_STM32_BSRR
    , _port(nullptr), _setMask(0), _resetMask(0)
#endif
#if WS2812_HAVE_H7_DMA
    , _dmaReady(false), _dmaActive(false), _htim(nullptr)
    , _dmaBuf(nullptr), _dmaBufLen(0), _ccr1(0), _ccr0(0), _dmaChannel(0)
#endif
{
  if (_numPixels == 0) {
    _numPixels = 1;   // guard: at least one pixel
  }
  _pixels = new uint8_t[(uint32_t)_numPixels * 3];
  if (_pixels != nullptr) {
    memset(_pixels, 0, (size_t)_numPixels * 3);
  }

#if WS2812_HAVE_STM32_BSRR
  // Resolve pin -> port + bit masks ONCE here. This must NOT live only in
  // beginBitbang(): in DMA mode begin() skips beginBitbang() and a zero
  // mask would make showBitbang() write BSRR=0 (LED never lights).
  //
  // NOTE: STM_GPIO_PIN(pn) already returns the bit mask (1<<pin), NOT the
  // pin number — do not shift it again (1<<0x8000 is UB on 32-bit).
  PinName pn = digitalPinToPinName(_pin);
  _port = get_GPIO_Port(STM_PORT(pn));
  uint32_t pinMask = STM_GPIO_PIN(pn);    // e.g. PD15 -> 0x8000
  _setMask   = pinMask;                    // BSRR low 16 bits = SET
  _resetMask = pinMask << 16;              // BSRR high 16 bits = RESET
#endif

#if WS2812_HAVE_H7_DMA
  _self = this;      // for DMA IRQ callback
#endif
}

WS2812Driver::~WS2812Driver()
{
  if (_pixels != nullptr) {
    delete[] _pixels;
    _pixels = nullptr;
  }
#if WS2812_HAVE_H7_DMA
  if (_dmaBuf != nullptr) {
    free(_dmaBuf);
    _dmaBuf = nullptr;
  }
  if (_htim != nullptr) {
    delete _htim;
    _htim = nullptr;
  }
#endif
}

// ------------------------------------------------------------------
// initialization
// ------------------------------------------------------------------
bool WS2812Driver::begin()
{
  if (_pixels == nullptr) {
    return false;
  }

#if WS2812_HAVE_H7_DMA
  if (_dmaReady) {
    // DMA mode: pin was configured as AF by beginDma(); do NOT run
    // beginBitbang() here or it would switch the pin back to GPIO
    // (the PWM signal would never reach the pin).
    _mode = MODE_TIM_DMA;
    _dmaActive = true;
  } else {
    if (!beginBitbang()) {
      return false;
    }
  }
#else
  if (!beginBitbang()) {
    return false;
  }
#endif

  show();   // all off
  return true;
}

// ------------------------------------------------------------------
// pixel API
// ------------------------------------------------------------------
void WS2812Driver::colorToRaw(uint8_t r, uint8_t g, uint8_t b,
                              uint8_t &b0, uint8_t &b1, uint8_t &b2) const
{
  // apply global brightness (simple linear scale)
  if (_brightness != 255) {
    uint16_t br = _brightness;
    r = (uint8_t)(((uint16_t)r * br) >> 8);
    g = (uint8_t)(((uint16_t)g * br) >> 8);
    b = (uint8_t)(((uint16_t)b * br) >> 8);
  }

  switch (_order) {
    case ORDER_GRB: b0 = g; b1 = r; b2 = b; break;
    case ORDER_RGB: b0 = r; b1 = g; b2 = b; break;
    case ORDER_BRG: b0 = b; b1 = r; b2 = g; break;
    default:        b0 = g; b1 = r; b2 = b; break;
  }
}

void WS2812Driver::setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b)
{
  if (i >= _numPixels || _pixels == nullptr) {
    return;
  }
  uint8_t b0, b1, b2;
  colorToRaw(r, g, b, b0, b1, b2);
  uint8_t *p = _pixels + (uint32_t)i * 3;
  p[0] = b0; p[1] = b1; p[2] = b2;
}

void WS2812Driver::setPixelColor(uint16_t i, uint32_t color)
{
  setPixelColor(i,
                (uint8_t)(color >> 16),
                (uint8_t)(color >> 8),
                (uint8_t)(color));
}

uint32_t WS2812Driver::getPixelColor(uint16_t i) const
{
  if (i >= _numPixels || _pixels == nullptr) {
    return 0;
  }
  const uint8_t *p = _pixels + (uint32_t)i * 3;
  uint8_t r, g, b;
  switch (_order) {
    case ORDER_GRB: g = p[0]; r = p[1]; b = p[2]; break;
    case ORDER_RGB: r = p[0]; g = p[1]; b = p[2]; break;
    case ORDER_BRG: b = p[0]; r = p[1]; g = p[2]; break;
    default:        g = p[0]; r = p[1]; b = p[2]; break;
  }
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void WS2812Driver::fill(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint16_t i = 0; i < _numPixels; i++) {
    setPixelColor(i, r, g, b);
  }
}

void WS2812Driver::clear()
{
  if (_pixels != nullptr) {
    memset(_pixels, 0, (size_t)_numPixels * 3);
  }
}

void WS2812Driver::setBrightness(uint8_t b)
{
  _brightness = b;
}

uint8_t WS2812Driver::getBrightness() const
{
  return _brightness;
}

uint16_t WS2812Driver::numPixels() const
{
  return _numPixels;
}

// ------------------------------------------------------------------
// transmission dispatch
// ------------------------------------------------------------------
void WS2812Driver::show()
{
  transmitFrame();
}

void WS2812Driver::transmitFrame()
{
#if WS2812_HAVE_H7_DMA
  if (_mode == MODE_TIM_DMA && _dmaActive) {
    showDmaPlatform();
    return;
  }
#endif
  showBitbang();
}

// ------------------------------------------------------------------
// mode control
// ------------------------------------------------------------------
#if WS2812_HAVE_H7_DMA
bool WS2812Driver::beginDma(TIM_TypeDef *tim, uint32_t channel)
{
  bool ok = beginDmaPlatform(tim, channel);
  if (ok) {
    _dmaReady = true;
    _dmaActive = true;
    _mode = MODE_TIM_DMA;
  }
  return ok;
}

bool WS2812Driver::isDmaActive() const
{
  return _dmaActive;
}
#else
bool WS2812Driver::beginDma(TIM_TypeDef *, uint32_t)
{
  // DMA back-end not compiled in on this platform
  return false;
}

bool WS2812Driver::isDmaActive() const
{
  return false;
}
#endif

void WS2812Driver::setMode(Mode m)
{
  if (m == MODE_TIM_DMA) {
#if WS2812_HAVE_H7_DMA
    if (_dmaReady) {
      _mode = MODE_TIM_DMA;
      _dmaActive = true;
      configurePinAf(_htim->Instance);   // GPIO -> AF (PWM to pin)
    }
#else
    (void)m;  // TIM_DMA not available
#endif
  } else {
    _mode = MODE_BITBANG;
#if WS2812_HAVE_H7_DMA
    _dmaActive = false;
    configurePinGpio();                  // AF -> GPIO (bit-bang output)
#endif
  }
}

// ★2026-08-10 静态电平测试：强制数据引脚输出固定电平（绕过 WS2812 协议）。
// 先切 bitbang 确保引脚为 GPIO 输出，再写 BSRR。
void WS2812Driver::setStaticLevel(bool high)
{
  if (_mode != MODE_BITBANG)
  {
    setMode(MODE_BITBANG);
  }
  _port->BSRR = high ? _setMask : _resetMask;
}

WS2812Driver::Mode WS2812Driver::getMode() const
{
  return _mode;
}

// ------------------------------------------------------------------
// debug helpers
// ------------------------------------------------------------------
#if WS2812_HAVE_H7_DMA
uint32_t WS2812Driver::dmaBufAddr() const { return (uint32_t)_dmaBuf; }
uint32_t WS2812Driver::dmaBufAt(uint32_t i) const
{
  return (i < _dmaBufLen && _dmaBuf != nullptr) ? _dmaBuf[i] : 0xFFFFFFFFUL;
}
uint32_t WS2812Driver::dmaBufLen() const { return _dmaBufLen; }
uint32_t WS2812Driver::dmaDstAddr() const
{
  if (_htim == nullptr) return 0;
  uint32_t off = (_dmaChannel == TIM_CHANNEL_4) ? 12U :
                 (_dmaChannel == TIM_CHANNEL_3) ? 8U  :
                 (_dmaChannel == TIM_CHANNEL_2) ? 4U  : 0U;
  return (uint32_t)&_htim->Instance->CCR1 + off;
}
#else
uint32_t WS2812Driver::dmaBufAddr() const { return 0; }
uint32_t WS2812Driver::dmaBufAt(uint32_t) const { return 0xFFFFFFFFUL; }
uint32_t WS2812Driver::dmaBufLen() const { return 0; }
uint32_t WS2812Driver::dmaDstAddr() const { return 0; }
#endif
