/*
 * basic.ino - minimal WS2812Driver example
 *
 * Connect:
 *   WS2812 DIN  -> any GPIO pin (change PIN below)
 *   WS2812 VDD  -> 5V
 *   WS2812 GND  -> GND
 *
 * On STM32H7 you can enable CPU-free DMA mode:
 *   strip.beginDma(TIM4, TIM_CHANNEL_4);   // TIM4_CH4 -> UP-DMA bypass
 */

#include <WS2812Driver.h>

// 1 = one LED (change for your strip length)
#define NUM_LEDS  1
// stm32duino pin number (PD15 = 30 on WeAct H743; change for your board)
#define LED_PIN   30

WS2812Driver strip(LED_PIN, NUM_LEDS);

void setup() {
  // Optional, STM32H7 only: enable TIM+DMA acceleration (CPU-free)
  // strip.beginDma(TIM4, TIM_CHANNEL_4);

  strip.begin();
}

void loop() {
  // red
  strip.fill(255, 0, 0);
  strip.show();
  delay(500);

  // green
  strip.fill(0, 255, 0);
  strip.show();
  delay(500);

  // blue
  strip.fill(0, 0, 255);
  strip.show();
  delay(500);

  // white
  strip.fill(255, 255, 255);
  strip.show();
  delay(500);

  // off
  strip.clear();
  strip.show();
  delay(500);
}
