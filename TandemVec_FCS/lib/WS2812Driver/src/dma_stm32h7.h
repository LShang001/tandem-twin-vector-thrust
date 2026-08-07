/*
 * dma_stm32h7.h - STM32H7 TIM + DMA back-end
 *
 * Drives WS2812 using a timer's PWM output updated via DMA, so the CPU
 * does no per-bit work and interrupts stay responsive.
 *
 * == The H7 problem ==
 * On STM32H743 the DMAMUX request table has NO entry for TIM4_CH4
 * (only TIM4_CH1/CH2/CH3/UP exist). `HAL_TIM_PWM_Start_DMA(htim,
 * TIM_CHANNEL_4, ...)` therefore never triggers on TIM4.
 *
 * == The bypass (this back-end) ==
 * Every H7 timer has a TIM Update (UP) DMA request. We use it for ALL
 * channels: the DMA writes the pre-encoded CCR value into the timer's
 * CCRy register once per PWM period (triggered by the UP event). The
 * PWM runs at 800kHz (1.25us period); each CCR value encodes T0H
 * (bit=0) or T1H (bit=1) as the pulse width.
 *
 *   - CH1..CH4: same mechanism, DMA destination = CCR1..CCR4
 *   - request source: DMA_REQUEST_TIMx_UP (exists on all H7 timers)
 *   - request enable bit: DIER.UDE (exists on all H7 timers)
 *
 * This works on TIM4_CH4 too, which is the whole point.
 *
 * == Key details ==
 * - Frame encoded into a 32-byte-aligned buffer; D-Cache flushed with
 *   SCB_CleanDCache_by_Addr before each transfer (H7 has D-Cache).
 * - A transfer-complete IRQ callback disables DIER.UDE immediately;
 *   leaving it set causes stray DMA requests that wedge the DMA state
 *   machine (observed: next HAL_DMA_Start_IT returns HAL_BUSY and the
 *   LED flickers).
 *
 * License: MIT
 */

#ifndef WS2812_DMA_STM32H7_H
#define WS2812_DMA_STM32H7_H

#if WS2812_HAVE_H7_DMA

// Frame layout: [40 reset words][24 words per pixel][40 reset words]
#define WS2812_DMA_RESET_WORDS  40U

// Enable the clock gate for the given timer instance (H7 RCC)
static inline void ws2812_tim_clk_enable(TIM_TypeDef *tim)
{
  if (tim == TIM1)  __HAL_RCC_TIM1_CLK_ENABLE();
  else if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
  else if (tim == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
  else if (tim == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

// DMAMUX1 request ID for the given timer's UPDATE event (all H7 timers have it)
static inline uint32_t ws2812_tim_up_request(TIM_TypeDef *tim)
{
  if (tim == TIM1)  return DMA_REQUEST_TIM1_UP;
  if (tim == TIM2)  return DMA_REQUEST_TIM2_UP;
  if (tim == TIM3)  return DMA_REQUEST_TIM3_UP;
  if (tim == TIM4)  return DMA_REQUEST_TIM4_UP;
  if (tim == TIM5)  return DMA_REQUEST_TIM5_UP;
  if (tim == TIM8)  return DMA_REQUEST_TIM8_UP;
  return 0;   // unsupported timer
}

// CCR register offset (bytes) for a TIM channel (CCR1..CCR4 are contiguous)
static inline uint32_t ws2812_ccr_offset(uint32_t channel)
{
  switch (channel) {
    case TIM_CHANNEL_2: return 4U;
    case TIM_CHANNEL_3: return 8U;
    case TIM_CHANNEL_4: return 12U;
    case TIM_CHANNEL_1:
    default:            return 0U;
  }
}

// GPIO alternate-function number for a timer on STM32H7
static inline uint32_t ws2812_tim_af(TIM_TypeDef *tim)
{
  if (tim == TIM1 || tim == TIM2) return GPIO_AF1_TIM1;   // AF1
  if (tim == TIM3 || tim == TIM4 || tim == TIM5) return GPIO_AF2_TIM3;  // AF2
  if (tim == TIM8) return GPIO_AF3_TIM8;                  // AF3
  return GPIO_AF2_TIM3;
}

// Pin -> GPIO output mode (bit-bang back-end)
inline void WS2812Driver::configurePinGpio()
{
  PinName pn = digitalPinToPinName(_pin);
  GPIO_InitTypeDef g;
  memset(&g, 0, sizeof(g));
  g.Pin = (uint16_t)STM_GPIO_PIN(pn);   // already a bit mask (1<<pin)
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(pn)), &g);
  _port->BSRR = _resetMask;   // start low
}

// Pin -> timer alternate function (DMA back-end)
inline void WS2812Driver::configurePinAf(TIM_TypeDef *tim)
{
  PinName pn = digitalPinToPinName(_pin);
  GPIO_InitTypeDef g;
  memset(&g, 0, sizeof(g));
  g.Pin = (uint16_t)STM_GPIO_PIN(pn);   // already a bit mask (1<<pin)
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = ws2812_tim_af(tim);
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(pn)), &g);
}

inline bool WS2812Driver::beginDmaPlatform(TIM_TypeDef *tim, uint32_t channel)
{
  if (tim == nullptr) {
    return false;
  }

  // 1. peripheral clocks (DMAMUX1 shares the DMA1 AHB1 clock gate on H7)
  ws2812_tim_clk_enable(tim);
  __HAL_RCC_DMA1_CLK_ENABLE();

  // 2. timer clock: APB1 timer clock = PCLK1 x2 when APB1 prescaler != 1
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t tim_clk = ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0) ? (pclk1 * 2) : pclk1;

  // 3. 800kHz PWM: ARR = tim_clk/800000 - 1, must fit 16-bit ARR
  uint32_t period = (tim_clk / 800000UL) - 1UL;
  if (period > 65535UL) {
    return false;   // cannot reach 800kHz with a 16-bit timer at this clock
  }

  _htim = new TIM_HandleTypeDef();
  if (_htim == nullptr) {
    return false;
  }
  memset(_htim, 0, sizeof(*_htim));
  _htim->Instance = tim;
  _htim->Init.Prescaler = 0;
  _htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  _htim->Init.Period = period;
  _htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  _htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(_htim) != HAL_OK) {
    return false;
  }

  TIM_OC_InitTypeDef oc;
  memset(&oc, 0, sizeof(oc));
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;                       // start low
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(_htim, &oc, channel) != HAL_OK) {
    return false;
  }

  // 4. CCR pulse widths (timer ticks)
  _ccr1 = tim_clk / 1250000UL;        // 0.80us (T1H)
  _ccr0 = tim_clk / 2500000UL;        // 0.40us (T0H)
  if (_ccr1 > period) _ccr1 = period;
  if (_ccr0 > period) _ccr0 = period;

  // 5. DMA buffer: [reset][pixels][reset], 32-byte aligned for D-Cache
  uint32_t words = 2U * WS2812_DMA_RESET_WORDS + 24U * (uint32_t)_numPixels;
  _dmaBuf = (uint32_t*)aligned_alloc(32, (size_t)words * sizeof(uint32_t));
  if (_dmaBuf == nullptr) {
    return false;
  }
  _dmaBufLen = words;

  // 6. DMA stream via DMAMUX1 (any stream can carry any request on H7)
  _hdma.Instance = DMA1_Stream6;
  _hdma.Init.Request = ws2812_tim_up_request(tim);
  _hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  _hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  _hdma.Init.MemInc = DMA_MINC_ENABLE;
  _hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  _hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  _hdma.Init.Mode = DMA_NORMAL;
  _hdma.Init.Priority = DMA_PRIORITY_HIGH;
  _hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (_hdma.Init.Request == 0 || HAL_DMA_Init(&_hdma) != HAL_OK) {
    return false;
  }

  // 7. link DMA into the timer handle's UPDATE slot (HAL looks it up there)
  __HAL_LINKDMA(_htim, hdma[TIM_DMA_ID_UPDATE], _hdma);

  // 8. transfer-complete IRQ: disable DIER.UDE to avoid stray-request wedging
  _hdma.XferCpltCallback = &WS2812Driver::dmaXferComplete;

  // 8b. switch the pin to timer AF so the PWM signal actually reaches it
  configurePinAf(tim);

  // 9. start PWM output (CCR=0 -> low)
  if (HAL_TIM_PWM_Start(_htim, channel) != HAL_OK) {
    return false;
  }

  _dmaChannel = channel;
  return true;
}

inline void WS2812Driver::dmaXferComplete(DMA_HandleTypeDef *)
{
  // Disable the UDE request bit immediately; leaving it set wedges the
  // DMA state machine (the "flicker" bug).
  if (_self != nullptr && _self->_htim != nullptr) {
    __HAL_TIM_DISABLE_DMA(_self->_htim, TIM_DMA_UPDATE);
  }
}

inline void WS2812Driver::showDmaPlatform()
{
  // 1. encode frame: [reset][pixels][reset]
  uint32_t *p = _dmaBuf;
  uint32_t idx = 0;
  for (uint32_t i = 0; i < WS2812_DMA_RESET_WORDS; i++) p[idx++] = 0;
  for (uint32_t i = 0; i < (uint32_t)_numPixels * 3; i++) {
    uint8_t pix = _pixels[i];
    for (int8_t bit = 7; bit >= 0; bit--) {
      p[idx++] = (pix & (1 << bit)) ? _ccr1 : _ccr0;
    }
  }
  for (uint32_t i = 0; i < WS2812_DMA_RESET_WORDS; i++) p[idx++] = 0;

  // 2. D-Cache: DMA reads this buffer, flush it first
  SCB_CleanDCache_by_Addr((uint32_t*)_dmaBuf, (uint32_t)idx * sizeof(uint32_t));

  // 3. safe start: disable UDE + abort stale DMA, then start and poll
  //    (destination = CCR1 + channel offset; source = encoded buffer)
  //
  // NOTE: we intentionally use HAL_DMA_Start() + PollForTransfer instead
  // of HAL_DMA_Start_IT(). The IT path requires a DMA1_Stream6_IRQHandler
  // that stm32duino's startup file does NOT provide (weak alias to
  // Default_Handler -> hardfault/hang or stuck-BUSY state). Polling is
  // fully self-contained: ~130us for a 1-LED frame, called at 100Hz.
  noInterrupts();
  __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
  if (HAL_DMA_GetState(&_hdma) != HAL_DMA_STATE_READY) {
    HAL_DMA_Abort(&_hdma);
  }
  uint32_t dst = (uint32_t)&_htim->Instance->CCR1 + ws2812_ccr_offset(_dmaChannel);
  if (HAL_DMA_Start(&_hdma, (uint32_t)_dmaBuf, dst, idx) == HAL_OK) {
    __HAL_TIM_ENABLE_DMA(_htim, TIM_DMA_UPDATE);
    // wait for the frame to be shifted out (max 50ms guard)
    HAL_DMA_PollForTransfer(&_hdma, HAL_DMA_FULL_TRANSFER, 50U);
    __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
  }
  interrupts();
}

#endif // WS2812_HAVE_H7_DMA
#endif // WS2812_DMA_STM32H7_H
