
# LQ-S48 光流测距一体模块 Arduino 驱动库

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](https://c.liaox.ai/chat/LICENSE)
[![Platform](https://img.shields.io/badge/platform-Arduino%20%7C%20ESP32%20%7C%20STM32-orange.svg)](https://www.arduino.cc/)

基于武汉凌启科技 LQ-S48 光流测距一体模块官方协议开发的 Arduino 驱动库，支持自定义协议的完整解析。

---

## 📋 目录

* [产品简介](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E4%BA%A7%E5%93%81%E7%AE%80%E4%BB%8B)
* [功能特性](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E5%8A%9F%E8%83%BD%E7%89%B9%E6%80%A7)
* [硬件参数](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E7%A1%AC%E4%BB%B6%E5%8F%82%E6%95%B0)
* [硬件连接](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E7%A1%AC%E4%BB%B6%E8%BF%9E%E6%8E%A5)
* [安装方法](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E5%AE%89%E8%A3%85%E6%96%B9%E6%B3%95)
* [快速开始](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E5%BF%AB%E9%80%9F%E5%BC%80%E5%A7%8B)
* [API 参考](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#api-%E5%8F%82%E8%80%83)
* [数据说明](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E6%95%B0%E6%8D%AE%E8%AF%B4%E6%98%8E)
* [配置指令](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E9%85%8D%E7%BD%AE%E6%8C%87%E4%BB%A4)
* [使用示例](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E4%BD%BF%E7%94%A8%E7%A4%BA%E4%BE%8B)
* [常见问题](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98)
* [注意事项](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E6%B3%A8%E6%84%8F%E4%BA%8B%E9%A1%B9)
* [更新日志](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E6%9B%B4%E6%96%B0%E6%97%A5%E5%BF%97)
* [参考资料](https://c.liaox.ai/chat/14496fc4-5619-4665-99e7-a1945f07fc44#%E5%8F%82%E8%80%83%E8%B5%84%E6%96%99)

---

## 产品简介

LQ-S48 是武汉凌启科技有限公司生产的一款 **光流测距一体定位模块** ，集成了：

* **双目视觉测距** - 0.25m~16m 量程，精度 2%~3%
* **高精度光流** - 可实现百米级高空视觉绝对定位
* **角度测量** - 可测量自身旋转角度

主要应用于无人机室内外定点悬停、自主导航、位置保持等场景。

### 支持的飞控平台

| 平台          | 协议        | 支持情况  |
| ------------- | ----------- | --------- |
| APM/ArduPilot | MAVLINK_APM | ✅        |
| PX4           | MAVLINK_PX4 | ✅        |
| INAV          | MSP_V2      | ✅        |
| 自定义开发    | 自定义协议  | ✅ (本库) |
| 上位机调试    | VOFA+       | ✅        |

---

## 功能特性

### ✅ 本库实现的功能

* **非阻塞数据解析** - 基于状态机，不影响主循环
* **CRC8-DVB-S2 校验** - 使用官方校验算法，确保数据可靠
* **完整数据获取** - 光流、高度、速度、角度全覆盖
* **自动数据累加** - 使用 double 精度，支持长时间运行
* **丢帧检测统计** - 实时监控通信质量
* **数据有效性判断** - 自动过滤不可信数据
* **模块配置指令** - 高度校准、波特率设置、协议切换

### 📊 可获取的数据

| 类别               | 数据项             | 单位        | 说明          |
| ------------------ | ------------------ | ----------- | ------------- |
| **光流增量** | X/Y 角位移         | rad         | 单帧增量      |
|                    | X/Y 线位移         | mm          | 单帧增量      |
|                    | 旋转角             | °          | 单帧增量      |
| **速度**     | X/Y 线速度         | mm/s, m/s   | 实时速度      |
|                    | 合成速度           | mm/s, m/s   | √(Vx²+Vy²) |
|                    | 角速度             | rad/s, °/s | 旋转速度      |
| **累加值**   | X/Y 累计位移       | rad, mm     | 从起点累加    |
|                    | 累计旋转           | °          | 从起点累加    |
| **测距**     | 对地高度           | mm          | 0.25m~16m     |
| **状态**     | 数据质量           | 0/1/2/3/23  | 可信度标志    |
|                    | 光照强度           | 0~100       | 环境亮度      |
| **统计**     | 有效帧/错误帧/丢帧 | 帧数        | 通信质量      |

---

## 硬件参数

> 摘自官方手册第 3 页

| 参数               | 规格              | 备注         |
| ------------------ | ----------------- | ------------ |
| 型号               | LQ-S48            |              |
| 尺寸               | 60 × 20 × 10 mm |              |
| 重量               | 7g                |              |
| 工作电压           | 4.0V ~ 5.5V       |              |
| 功耗               | 820mW             | @3.3V        |
| 通信接口           | UART 串口         | TTL 3.3V     |
| 默认波特率         | 115200 bps        | 可调         |
| 数据帧率           | 40~50 Hz          |              |
| 视场角             | 20°              | 水平/垂直    |
| 环境照度           | >20 Lux           | 无惧室外强光 |
| 工作温度           | -10°C ~ 50°C    |              |
| **测距量程** | 0.25m ~ 16m       | 精度 2%~3%   |
| **光流精度** | 1%                | @1m, 0.5m/s  |
| **最大测速** | 8 m/s             | @2m 高度     |

---

## 硬件连接

### 模块接口定义

> 模块正面朝上，从左到右依次为：

```
┌─────────────────────────────────────┐
│  [5V]   [RX]   [TX]   [GND]        │
│    │      │      │      │          │
│    红     黄     绿     黑          │
└─────────────────────────────────────┘
```

### 接线示意图

```
LQ-S48 模块                    Arduino / ESP32
═══════════════════════════════════════════════
    5V   ──────────────────►   5V (或 VIN)
    GND  ──────────────────►   GND
    TX   ──────────────────►   RX (串口接收)
    RX   ──────────────────►   TX (串口发送)
```

### 常用开发板接线参考

| 开发板       | 串口    | RX 引脚     | TX 引脚     |
| ------------ | ------- | ----------- | ----------- |
| Arduino Mega | Serial1 | 19          | 18          |
| Arduino Mega | Serial2 | 17          | 16          |
| ESP32        | Serial1 | 16 (可配置) | 17 (可配置) |
| ESP32        | Serial2 | 16 (可配置) | 17 (可配置) |
| STM32F4      | Serial1 | PA10        | PA9         |

### 安装方向

> 参考手册第 11 页

**重要：模块的 Y 轴正方向必须指向机头！**

```
              ↑ 机头方向 (Y+)
              │
         ┌────┴────┐
         │ LQ-S48  │
    X- ←─┤ [镜头]  ├─► X+
         │         │
         └─────────┘
              │
              ↓ 机尾方向 (Y-)
```

---

## 安装方法

### 方法 1：手动安装

1. 下载本仓库的 `LQS48_Flow.h` 和 `LQS48_Flow.cpp`
2. 将两个文件复制到你的 Arduino 项目目录
3. 在代码中 `#include "LQS48_Flow.h"`

### 方法 2：Arduino 库管理器

```
即将支持...
```

### 文件结构

```
你的项目/
├── LQS48_Flow.h          # 头文件
├── LQS48_Flow.cpp        # 实现文件
├── your_sketch.ino       # 你的主程序
└── README.md             # 说明文档
```

---

## 快速开始

### 最小示例

```cpp
#include "LQS48_Flow.h"

LQS48_Flow sensor;

void setup() {
    Serial.begin(115200);      // 调试串口
    Serial1.begin(115200);     // 模块串口
    sensor.begin(Serial1);     // 初始化
}

void loop() {
    if (sensor.update()) {
        // 获取高度
        Serial.print("高度: ");
        Serial.print(sensor.getHeight_mm());
        Serial.println(" mm");
      
        // 获取速度
        if (sensor.isAllValid()) {
            Serial.print("速度: ");
            Serial.print(sensor.getVelocity_mm_s());
            Serial.println(" mm/s");
        }
    }
}
```

### ESP32 完整示例

```cpp
#include "LQS48_Flow.h"

LQS48_Flow sensor;

void setup() {
    Serial.begin(115200);
  
    // ESP32 指定串口引脚
    Serial1.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17
  
    sensor.begin(Serial1);
    sensor.resetAccumulators();  // 起飞前重置累加器
  
    Serial.println("LQ-S48 初始化完成");
}

void loop() {
    if (sensor.update()) {
        // 获取数据
        LQS48_Data_t data = sensor.getData();
      
        Serial.printf("高度: %d mm, 质量: %d, 光照: %d\n", 
                      data.height_mm, data.quality, data.lux);
      
        if (sensor.isAllValid()) {
            Serial.printf("速度: Vx=%.1f Vy=%.1f mm/s\n",
                          sensor.getVelocityX_mm_s(),
                          sensor.getVelocityY_mm_s());
          
            Serial.printf("累计位移: X=%.1f Y=%.1f mm\n",
                          sensor.getAccumX_mm(),
                          sensor.getAccumY_mm());
        }
      
        Serial.println();
        delay(100);  // 控制打印频率
    }
}
```

---

## API 参考

### 类：LQS48_Flow

#### 初始化

| 方法                           | 说明             |
| ------------------------------ | ---------------- |
| `LQS48_Flow()`               | 构造函数         |
| `void begin(Stream &serial)` | 初始化，绑定串口 |

#### 数据更新

| 方法              | 返回值            | 说明                               |
| ----------------- | ----------------- | ---------------------------------- |
| `bool update()` | `true`=有新数据 | **必须在 loop() 中频繁调用** |

#### 单帧数据获取

| 方法                            | 返回值 | 单位 | 说明          |
| ------------------------------- | ------ | ---- | ------------- |
| `LQS48_Data_t getData()`      | 结构体 | -    | 获取完整数据  |
| `float getFlowX_rad()`        | float  | rad  | X轴角位移增量 |
| `float getFlowY_rad()`        | float  | rad  | Y轴角位移增量 |
| `float getRotation_deg()`     | float  | °   | 旋转角增量    |
| `uint32_t getHeight_mm()`     | uint32 | mm   | 对地高度      |
| `float getDisplacementX_mm()` | float  | mm   | X轴线位移增量 |
| `float getDisplacementY_mm()` | float  | mm   | Y轴线位移增量 |

#### 速度获取

| 方法                             | 返回值 | 单位  | 说明       |
| -------------------------------- | ------ | ----- | ---------- |
| `float getVelocityX_mm_s()`    | float  | mm/s  | X轴线速度  |
| `float getVelocityY_mm_s()`    | float  | mm/s  | Y轴线速度  |
| `float getVelocityX_m_s()`     | float  | m/s   | X轴线速度  |
| `float getVelocityY_m_s()`     | float  | m/s   | Y轴线速度  |
| `float getVelocity_mm_s()`     | float  | mm/s  | 合成线速度 |
| `float getVelocity_m_s()`      | float  | m/s   | 合成线速度 |
| `float getAngularVelX_rad_s()` | float  | rad/s | X轴角速度  |
| `float getAngularVelY_rad_s()` | float  | rad/s | Y轴角速度  |
| `float getRotationVel_deg_s()` | float  | °/s  | 旋转角速度 |

#### 累加数据获取

| 方法                         | 返回值 | 单位 | 说明           |
| ---------------------------- | ------ | ---- | -------------- |
| `double getAccumX_rad()`   | double | rad  | X轴累计角位移  |
| `double getAccumY_rad()`   | double | rad  | Y轴累计角位移  |
| `double getAccumX_mm()`    | double | mm   | X轴累计线位移  |
| `double getAccumY_mm()`    | double | mm   | Y轴累计线位移  |
| `double getAccumRot_deg()` | double | °   | 累计旋转角     |
| `void resetAccumulators()` | void   | -    | 重置所有累加值 |

#### 状态信息

| 方法                         | 返回值     | 说明         |
| ---------------------------- | ---------- | ------------ |
| `uint8_t getQuality()`     | 0/1/2/3/23 | 数据质量标志 |
| `uint8_t getLux()`         | 0~100      | 光照强度     |
| `uint8_t getDtFlow_ms()`   | ms         | 光流时间间隔 |
| `uint8_t getDtHeight_ms()` | ms         | 高度时间间隔 |
| `uint8_t getSequence()`    | 0~255      | 包序列号     |

#### 数据有效性判断

| 方法                     | 返回值 | 说明                             |
| ------------------------ | ------ | -------------------------------- |
| `bool isFlowValid()`   | bool   | 光流数据是否有效 (quality=0,1)   |
| `bool isHeightValid()` | bool   | 高度数据是否有效 (quality≠3,23) |
| `bool isAllValid()`    | bool   | 所有数据是否有效 (quality=0,1)   |

#### 统计信息

| 方法                          | 返回值 | 说明             |
| ----------------------------- | ------ | ---------------- |
| `uint32_t getValidFrames()` | 帧数   | 校验通过的有效帧 |
| `uint32_t getErrorFrames()` | 帧数   | 校验失败的错误帧 |
| `uint32_t getLostFrames()`  | 帧数   | 检测到的丢帧数   |
| `float getLostRate()`       | %      | 丢帧率百分比     |
| `void resetStatistics()`    | void   | 重置所有统计     |

#### 模块配置

| 方法                                                | 说明                |
| --------------------------------------------------- | ------------------- |
| `void calibrateHeight(uint32_t actual_height_mm)` | 高度校准            |
| `void setBaudRate(LQS48_BaudRate baud)`           | 设置波特率 (需重启) |
| `void setProtocol(LQS48_Protocol protocol)`       | 切换通信协议        |

---

## 数据说明

### 数据质量标志 (Quality)

| 值 | 枚举常量                  | 含义       | 处理建议           |
| -- | ------------------------- | ---------- | ------------------ |
| 0  | `LQS48_QUALITY_STRONG`  | 强可信     | 直接使用           |
| 1  | `LQS48_QUALITY_WEAK`    | 弱可信     | 可用，精度略低     |
| 2  | `LQS48_QUALITY_BAD_XYR` | 光流不可信 | 丢弃光流，高度有效 |
| 3  | `LQS48_QUALITY_BAD_H`   | 高度不可信 | 丢弃高度，光流有效 |
| 23 | `LQS48_QUALITY_BAD_ALL` | 全部不可信 | 全部丢弃           |

### 数据帧格式 (17 字节)

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│Byte 0│Byte 1│Byte2-3│Byte4-5│Byte6-7│Byte8-10│Byte11│...   │Byte16│
├──────┼──────┼───────┼───────┼───────┼────────┼──────┼──────┼──────┤
│ 0xFA │ 0xAA │ X位移 │ Y位移 │ 旋转R │ 高度H  │ 质量 │ ...  │ CRC  │
│ 帧头1│ 帧头2│ ÷200000│ ÷200000│ ÷1000 │  mm   │      │      │校验位│
└──────┴──────┴───────┴───────┴───────┴────────┴──────┴──────┴──────┘
```

### 重要：数据是帧间增量！

模块输出的光流数据是 **两帧之间的增量** ，不是累计值。

```
帧1: dx = +0.00001 rad  ──┐
帧2: dx = +0.00002 rad    ├──► 累加得到总位移
帧3: dx = -0.00001 rad  ──┘
```

本库会自动累加有效数据，可通过 `getAccumX_mm()` 等函数获取。

---

## 配置指令

### 高度校准

> 安装后必须进行高度校准！

```cpp
// 将模块正对地面，测量实际距离
// 例如实际距离 1500mm
sensor.calibrateHeight(1500);

// 等待约 5 秒，校准完成
```

**校准步骤：**

1. 将模块正对一个有纹理的平面
2. 用尺子测量模块到平面的实际距离
3. 调用 `calibrateHeight(距离mm)`
4. 等待约 5 秒，观察模块状态
5. 若精度不理想，可增加距离后再次校准

### 波特率设置

```cpp
// 可用波特率
LQS48_BAUD_19200   // 19200 bps
LQS48_BAUD_38400   // 38400 bps
LQS48_BAUD_57600   // 57600 bps
LQS48_BAUD_115200  // 115200 bps (默认)
LQS48_BAUD_460800  // 460800 bps
LQS48_BAUD_921600  // 921600 bps

// 设置波特率 (设置后需重启模块！)
sensor.setBaudRate(LQS48_BAUD_115200);
```

### 协议切换

```cpp
// 可用协议
LQS48_PROTOCOL_CUSTOM      // 自定义协议 (本库使用)
LQS48_PROTOCOL_MSP_V2      // INAV 飞控
LQS48_PROTOCOL_MAVLINK_PX4 // PX4 飞控
LQS48_PROTOCOL_MAVLINK_APM // APM 飞控
LQS48_PROTOCOL_VOFA        // VOFA+ 上位机

// 切换协议
sensor.setProtocol(LQS48_PROTOCOL_VOFA);
```

---

## 使用示例

### 示例 1：基础数据读取

```cpp
#include "LQS48_Flow.h"

LQS48_Flow sensor;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    sensor.begin(Serial1);
}

void loop() {
    if (sensor.update()) {
        Serial.print("高度: ");
        Serial.print(sensor.getHeight_mm());
        Serial.print(" mm, 质量: ");
        Serial.println(sensor.getQuality());
    }
}
```

### 示例 2：速度控制

```cpp
void loop() {
    if (sensor.update() && sensor.isAllValid()) {
        float vx = sensor.getVelocityX_m_s();
        float vy = sensor.getVelocityY_m_s();
      
        // 速度环控制
        float error_vx = target_vx - vx;
        float error_vy = target_vy - vy;
      
        // PID 计算...
    }
}
```

### 示例 3：位置累加

```cpp
void setup() {
    // ...
    sensor.resetAccumulators();  // 起飞前重置
}

void loop() {
    if (sensor.update()) {
        // 获取相对起点的位移
        double x_mm = sensor.getAccumX_mm();
        double y_mm = sensor.getAccumY_mm();
      
        Serial.printf("位置: X=%.1f Y=%.1f mm\n", x_mm, y_mm);
    }
}

void onReturnToHome() {
    sensor.resetAccumulators();  // 回到起点时重置
}
```

### 示例 4：通信质量监控

```cpp
void printStats() {
    Serial.println("===== 通信统计 =====");
    Serial.printf("有效帧: %lu\n", sensor.getValidFrames());
    Serial.printf("错误帧: %lu\n", sensor.getErrorFrames());
    Serial.printf("丢帧数: %lu\n", sensor.getLostFrames());
    Serial.printf("丢帧率: %.2f%%\n", sensor.getLostRate());
}
```

### 示例 5：补光灯控制

```cpp
const int LED_PIN = 13;

void checkLighting() {
    uint8_t lux = sensor.getLux();
  
    if (lux < 30) {
        // 光照不足，开启补光灯
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
```

---

## 常见问题

### Q1: 没有数据输出？

**检查项：**

* [ ] 接线是否正确 (TX→RX, RX→TX)
* [ ] 电源是否 4.0V~5.5V
* [ ] 波特率是否 115200
* [ ] `update()` 是否在 loop() 中调用

### Q2: 校验失败率高？

**可能原因：**

* 串口干扰，尝试降低波特率
* 接线过长或不稳定
* 电源噪声

**解决方案：**

```cpp
// 查看错误统计
Serial.printf("错误帧: %lu\n", sensor.getErrorFrames());
```

### Q3: 高度数据不准？

**解决方案：**

1. 确认已进行高度校准
2. 检查模块是否安装牢固
3. 避免剧烈振动
4. 检查镜头是否遮挡

### Q4: 光流漂移严重？

**可能原因：**

* 地面纹理不清晰
* 光照不足 (lux < 20)
* 振动过大

**解决方案：**

```cpp
// 检查光照
if (sensor.getLux() < 30) {
    // 开启补光灯
}
```

### Q5: 累加数据精度丢失？

本库已使用 `double` 类型累加，可支持长时间运行。如仍有问题：

* 定期重置累加器
* 检查是否有大量无效数据跳过

---

## 注意事项

### ⚠️ 使用注意

1. **地面要求** - 需要有清晰纹理，光照 > 20Lux
2. **安装方向** - Y 轴正方向必须指向机头
3. **安装牢固** - 避免形变影响测距精度
4. **高度校准** - 安装后必须进行高度校准
5. **振动控制** - 剧烈振动会影响数据精度

### ⚠️ 数据处理

1. **增量数据** - 光流输出是帧间增量，需累加
2. **有效性检查** - 使用前检查 `isFlowValid()` / `isHeightValid()`
3. **丢帧处理** - 监控丢帧率，必要时做插值补偿
4. **精度问题** - 累加时使用 double 类型

### ⚠️ 红外补光灯

1. **供电电压** - 9V~30V，注意正负极！
2. **散热** - 必须安装散热片
3. **红外特性** - 红外光对颜色不敏感，只对材质和距离敏感

---

## 更新日志

### v2.0.0 (2025-xx-xx)

* ✅ 修正校验算法为官方 CRC8-DVB-S2
* ✅ 新增线速度获取函数
* ✅ 新增丢帧检测和统计
* ✅ 新增数据累加功能 (double 精度)
* ✅ 扩展高度校准支持 24 位距离
* ✅ 完善中文注释和文档

### v1.0.0

* 初始版本

---

## 参考资料

* [LQ-S48 产品使用说明书 V3.3](https://www.lingqi-tech.com/) - 武汉凌启科技
* [APM 光流配置指南](https://ardupilot.org/copter/docs/common-optical-flow-sensors-landingpage.html)
* [PX4 光流配置指南](https://docs.px4.io/main/en/sensor/optical_flow.html)
* [VOFA+ 上位机](https://www.vofa.plus/)

---

## 许可证

MIT License

---

## 联系方式

* **模块厂商** : 武汉凌启科技有限公司
* **技术文章** : CSDN 搜索 "凌启科技-1"

---

**如有问题或建议，欢迎提交 Issue！**
