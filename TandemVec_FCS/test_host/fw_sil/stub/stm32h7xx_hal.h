/**
 * @file    stm32h7xx_hal.h
 * @module  固件/宿主 SIL 桩（fw-tests：STM32H7 HAL 替身）
 * @brief   为链接真实通信层/主程序提供寄存器级类型与常量，不做任何硬件访问
 * @details 被测：无（替身层）。真实固件里 `communication.cpp` 会直接操作
 *          `GPIOD->BSRR`、`UART4->CR1`、`NVIC_GetPriority()` 等寄存器/内核接口，
 *          `main.cpp` 读 `F_CPU` 并调用 `NVIC_SystemReset()`。宿主没有这些外设，
 *          但源码必须能编译，因此这里提供**形状一致**的类型与实例。
 *
 *          设计原则：结构体字段名与真实 CMSIS 一致（`ODR`/`IDR`/`BSRR`/`CR1`），
 *          值是纯内存、不映射任何物理地址。这样源码里的寄存器读写语法原样成立，
 *          只是写进了一块普通内存——既不会崩，也**不能**据此推断硬件行为。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units  N/A
 * @note    ★这是"数字孪生"的边界所在：任何依赖寄存器副作用的行为（引脚电平、
 *          串口 FIFO 使能、中断优先级）在宿主都**不成立**。能验证的是
 *          "代码路径是否走对"，不是"硬件是否配置对"。后者只能上板。
 * @see     test_host/fw_sil/stub/Arduino.h、fw_sil_stubs.cpp（实例定义）
 */
#pragma once

#include <cstdint>

// ---- GPIO ----
// 字段顺序与 CMSIS 的 GPIO_TypeDef 对齐（只保留固件实际用到的）。
typedef struct
{
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;

// 8 个端口实例（A..H）。inline 变量：每个 TU 一份，互不干扰，也无需额外 .cpp。
inline GPIO_TypeDef sil_gpio_port[8] = {};
#define GPIOA (&sil_gpio_port[0])
#define GPIOB (&sil_gpio_port[1])
#define GPIOC (&sil_gpio_port[2])
#define GPIOD (&sil_gpio_port[3])
#define GPIOE (&sil_gpio_port[4])
#define GPIOF (&sil_gpio_port[5])
#define GPIOG (&sil_gpio_port[6])
#define GPIOH (&sil_gpio_port[7])

// ---- 定时器（WS2812 的 beginDma 形参类型）----
typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR4;
} TIM_TypeDef;
inline TIM_TypeDef sil_tim[8] = {};
#ifndef TIM4
#define TIM4 (&sil_tim[3])
#endif

// ---- UART ----
typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t BRR;
} USART_TypeDef;

typedef struct
{
  USART_TypeDef *Instance;
  uint32_t ErrorCode;
  uint32_t FifoMode;  ///< 固件回读 FIFO 配置时读取
} UART_HandleTypeDef;

// ---- DMA / RCC ----
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t NDTR;
  volatile uint32_t PAR;
  volatile uint32_t M0AR;
  volatile uint32_t M1AR;
} DMA_Stream_TypeDef;
inline DMA_Stream_TypeDef sil_dma_stream[8][8] = {};
#define DMA1_Stream6 (&sil_dma_stream[0][6])

typedef struct
{
  volatile uint32_t CCR;
} DMAMUX_Channel_TypeDef;
inline DMAMUX_Channel_TypeDef sil_dmamux[16] = {};
#define DMAMUX1_Channel6 (&sil_dmamux[6])

typedef struct
{
  volatile uint32_t D2CFGR;
} RCC_TypeDef;
inline RCC_TypeDef sil_rcc = {};
#define RCC (&sil_rcc)
#ifndef RCC_D2CFGR_D2PPRE1
#define RCC_D2CFGR_D2PPRE1 (7UL << 4)
#endif
inline uint32_t SystemCoreClock = 480000000UL;
inline uint32_t HAL_RCC_GetPCLK1Freq() { return 120000000UL; }

inline USART_TypeDef sil_usart[8] = {};
#define UART4 (&sil_usart[3])

// 固件读回 FIFO 使能位时用到的常量（手册 RM0433 USART_CR1）
#ifndef USART_CR1_FIFOEN
#define USART_CR1_FIFOEN (1UL << 29)
#endif
#ifndef UART_FIFOMODE_ENABLE
#define UART_FIFOMODE_ENABLE 1UL
#endif

// ---- 内核/NVIC ----
#ifndef UART4_IRQn
#define UART4_IRQn 52
#endif
#ifndef UART5_IRQn
#define UART5_IRQn 53
#endif
inline uint32_t NVIC_GetPriority(int) { return 0; }
inline void NVIC_SetPriority(int, int) {}
/// 上板时这是真复位；宿主直接结束进程，避免"复位后继续跑"制造假象。
[[noreturn]] inline void NVIC_SystemReset() { __builtin_trap(); }

// ---- 时钟 ----
// STM32H743 主频；固件用它做波特率/延时换算。
#ifndef F_CPU
#define F_CPU 480000000UL
#endif