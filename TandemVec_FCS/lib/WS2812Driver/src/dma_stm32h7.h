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
 * - 轮询 DMA 完成与错误标志，不依赖 DMA 中断。
 * - 返回前关闭 TIM 更新 DMA 请求。发帧失败时切回 GPIO 低电平，等待下次调用恢复。
 *
 * License: MIT
 */

#ifndef WS2812_DMA_STM32H7_H
#define WS2812_DMA_STM32H7_H

#if WS2812_HAVE_H7_DMA

// 帧首低电平保留 40 个周期。帧尾多留两个周期，覆盖 CCR 预装载及完成标志的边界。
static constexpr uint32_t kWs2812ResetWords = 40U;
static constexpr uint32_t kWs2812TailWords = kWs2812ResetWords + 2U;

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
  _port->BSRR = _resetMask;   // 切到 GPIO 输出前先锁存低电平
  HAL_GPIO_Init(get_GPIO_Port(STM_PORT(pn)), &g);
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

  const uint32_t words = kWs2812ResetWords + 24U * static_cast<uint32_t>(_numPixels) + kWs2812TailWords;
  if (words > UINT16_MAX) {
    return false;  // DMA 的 NDTR 只有 16 位
  }
  if (_dmaBuf == nullptr) {
    const size_t bytes = (words * sizeof(uint32_t) + 31U) & ~size_t{31U};
    _dmaBuf = static_cast<uint32_t*>(aligned_alloc(32U, bytes));
    if (_dmaBuf == nullptr) {
      return false;
    }
  }
  _dmaBufLen = words;

  // 6. DMA stream via DMAMUX1（由本驱动独占 DMA1_Stream6）
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
  DMA1_Stream6->CR &= ~DMA_SxCR_EN;
  DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 |
                DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;

  // HAL 仅负责配置请求路由和数据宽度，发送阶段直接使用硬件寄存器。
  DMA_HandleTypeDef dma{};
  dma.Instance = DMA1_Stream6;
  dma.Init.Request = ws2812_tim_up_request(tim);
  dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  dma.Init.PeriphInc = DMA_PINC_DISABLE;
  dma.Init.MemInc = DMA_MINC_ENABLE;
  dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  dma.Init.Mode = DMA_NORMAL;
  dma.Init.Priority = DMA_PRIORITY_HIGH;
  dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (dma.Init.Request == 0 || HAL_DMA_Init(&dma) != HAL_OK) {
    return false;
  }
  // HAL_DMA_Init 已写入流和 DMAMUX 配置，DMA 句柄本身无需保留。

  configurePinAf(tim);
  if (HAL_TIM_PWM_Start(_htim, channel) != HAL_OK) {
    configurePinGpio();
    return false;
  }

  _dmaChannel = channel;
  return true;
}

inline void WS2812Driver::stopDmaOutput()
{
  // 先停请求源，再请求停流。流可能仍在收尾，下次确认 EN 已清才可改写缓冲。
  __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
  CLEAR_BIT(DMA1_Stream6->CR, DMA_SxCR_EN);
  configurePinGpio();
  _dmaNeedsRecovery = true;
}

inline bool WS2812Driver::showDmaPlatform()
{
  auto *const stream = DMA1_Stream6;
  __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
  if (stream->CR & DMA_SxCR_EN) {
    stopDmaOutput();
    return false;
  }

  auto *const ccr = &_htim->Instance->CCR1 + ws2812_ccr_offset(_dmaChannel) / sizeof(uint32_t);
  if (_dmaNeedsRecovery) {
    // 引脚仍为 GPIO 低电平。先装载 CCR=0，再切回复用，避免残留占空比打出脉冲。
    *ccr = 0;
    _htim->Instance->EGR = TIM_EGR_UG;
    configurePinAf(_htim->Instance);
    _dmaNeedsRecovery = false;
  }

  // 编码帧首复位、像素数据和帧尾复位，再将缓冲写回内存供 DMA 读取。
  uint32_t *const buffer = _dmaBuf;
  uint32_t words = 0;
  for (uint32_t i = 0; i < kWs2812ResetWords; ++i) {
    buffer[words++] = 0;
  }
  for (uint32_t i = 0; i < static_cast<uint32_t>(_numPixels) * 3U; ++i) {
    const uint8_t pixel = _pixels[i];
    for (int8_t bit = 7; bit >= 0; --bit) {
      buffer[words++] = (pixel & (1U << bit)) ? _ccr1 : _ccr0;
    }
  }
  for (uint32_t i = 0; i < kWs2812TailWords; ++i) {
    buffer[words++] = 0;
  }
  SCB_CleanDCache_by_Addr(buffer, words * sizeof(uint32_t));

  constexpr uint32_t errorFlags = DMA_HISR_FEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_TEIF6;
  constexpr uint32_t clearFlags = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 |
                                  DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
  DMA1->HIFCR = clearFlags;
  stream->PAR = reinterpret_cast<uintptr_t>(ccr);
  stream->M0AR = reinterpret_cast<uintptr_t>(buffer);
  stream->NDTR = words;

  // 每字至多 1.25 µs，另留 50 µs 启动及总线余量。单像素的轮询预算为 183 µs。
  // micros() 使用 SysTick，全程保持全局中断开启。减法兼容 32 位时间戳回绕。
  const uint32_t timeoutUs = (words * 5U + 3U) / 4U + 50U;
  const uint32_t startUs = micros();
  SET_BIT(stream->CR, DMA_SxCR_EN);
  __HAL_TIM_ENABLE_DMA(_htim, TIM_DMA_UPDATE);
  do {
    const uint32_t status = DMA1->HISR;
    if (status & errorFlags) {
      break;
    }
    if ((status & DMA_HISR_TCIF6) && !(stream->CR & DMA_SxCR_EN)) {
      __HAL_TIM_DISABLE_DMA(_htim, TIM_DMA_UPDATE);
      DMA1->HIFCR = clearFlags;
      return true;
    }
  } while (static_cast<int32_t>(micros() - startUs) < static_cast<int32_t>(timeoutUs));

  stopDmaOutput();
  return false;
}

#endif // WS2812_HAVE_H7_DMA
#endif // WS2812_DMA_STM32H7_H
