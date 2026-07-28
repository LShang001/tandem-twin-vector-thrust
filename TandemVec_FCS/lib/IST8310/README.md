
这是一个为 **iSentek IST8310** 磁力计深度定制的高精度 Arduino 驱动库说明文档。

这份文档不仅包含基础的使用说明，还详细解释了驱动内部针对 Datasheet 进行的深度优化逻辑，帮助开发者理解为何以此方式实现。

---

# IST8310 High-Precision Arduino Driver

[![Arduino](https://img.shields.io/badge/framework-Arduino-blue.svg)](https://www.arduino.cc/)
[![Sensor](https://img.shields.io/badge/sensor-IST8310-orange.svg)](http://www.isentek.com/en/product_detail.php?id=38)
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()

这是一个针对 **iSentek IST8310 三轴数字磁力计** 的高性能 Arduino 驱动库。本驱动完全基于 Datasheet V1.5 规范编写，修复了市面上常见驱动的初始化缺陷，针对噪声控制、自检逻辑和物理量转换进行了深度优化。

## 🌟 核心特性 (Key Features)

与通用驱动相比，本库具有以下显著优势：

1. **底噪深度优化**：
   * 在初始化阶段强制配置 `PDCNTL (0x42)` 寄存器为 `0xC0`。这是 Datasheet 第 3.1.1 节强烈建议的设置，用于优化脉冲持续时间，显著降低传感器底噪。大多数开源驱动忽略了这一点。
2. **严格的自检流程**：
   * 修正了自检逻辑。在执行硬件自检时，驱动会自动**禁用温度补偿** (TCCNTL)，确保自检磁场不受补偿电路干扰（遵循 Datasheet 3.1.3 节规范）。
3. **稳健的时序控制**：
   * 抛弃了不准确的 `delay` 估算，采用轮询 `DRDY` (Data Ready) 标志位 + 超时机制，确保在数据准备就绪的瞬间读取，避免读到旧数据或脏数据。
4. **标准的物理单位**：
   * 直接输出 **微特斯拉 ($\mu T$)**。转换系数严格基于官方参数 **330 LSB/Gauss** 计算，无需用户手动换算。
5. **低功耗设计**：
   * 利用 IST8310 的 "Single Measurement Mode" 特性，测量完成后传感器自动进入待机模式，适合电池供电设备。

---

## 🛠 硬件连接 (Hardware Setup)

### 引脚定义 (Pinout)

IST8310 通常通过 I2C 接口通信。

| IST8310 Pin        | Arduino Pin | 说明                                              |
| :----------------- | :---------- | :------------------------------------------------ |
| **VCC/AVDD** | 3.3V        | **警告：不可接 5V**，否则会损坏芯片         |
| **GND**      | GND         | 共地                                              |
| **SCL**      | SCL         | I2C 时钟线 (需上拉电阻)                           |
| **SDA**      | SDA         | I2C 数据线 (需上拉电阻)                           |
| **RSTN**     | IO / 3.3V   | 复位引脚 (低电平复位，悬空内部拉高，建议接 3.3V)  |
| **DRDY**     | IO (可选)   | 数据就绪中断引脚 (本驱动主要使用软件轮询，可不接) |

### ⚠️ 关键硬件警告 (Critical Hardware Note)

**Pin 10 (C1) 电容至关重要！**
IST8310 内部的 Set/Reset 磁畴复位电路需要瞬间大电流。务必在 **C1 引脚** 和 **GND** 之间连接一个 **4.7 $\mu F$** 的陶瓷电容。

* **现象**：如果该电容缺失、容值不足或走线过长，传感器读数将表现为全 0、极高噪声或严重的迟滞（Hysteresis）。

---

## 📥 安装 (Installation)

1. 下载本项目的 `IST8310.h` 和 `IST8310.cpp` 文件。
2. 将这两个文件放入你的 Arduino 项目文件夹中（与 `.ino` 文件同级）。
3. 或者，将其放入 `Documents/Arduino/libraries/IST8310/` 目录下作为全局库使用。

---

## 🚀 快速开始 (Quick Start)

```cpp
#include <Wire.h>
#include "IST8310.h"

IST8310 mag;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // 初始化传感器
    // 默认 I2C 地址为 0x0E
    if (!mag.begin()) {
        Serial.println("初始化失败！请检查接线。");
        while (1);
    }
  
    Serial.println("IST8310 就绪");
}

void loop() {
    // 触发并读取一次数据
    if (mag.read()) {
        Serial.print("X: "); Serial.print(mag.get_x_uT());
        Serial.print(" Y: "); Serial.print(mag.get_y_uT());
        Serial.print(" Z: "); Serial.print(mag.get_z_uT());
        Serial.println(" uT");
    } else {
        Serial.println("读取超时");
    }
  
    delay(100); // 10Hz 采样率
}
```

---

## 📚 API 文档 (API Reference)

### 1. 初始化

```cpp
bool begin(TwoWire *wire = &Wire, uint8_t addr = 0x0E);
```

* **功能**：复位传感器，检查 Chip ID，配置低噪声模式 (`PDCNTL=0xC0`) 和平均采样 (`AVG=16x`)。
* **参数**：
  * `wire`: I2C 总线指针 (如 `&Wire`, `&Wire1`)。
  * `addr`: I2C 地址。默认为 `0x0E` (CAD0=DVDD, CAD1=GND)。
* **返回**：成功返回 `true`，失败返回 `false`。

### 2. 读取数据

```cpp
bool read();
```

* **功能**：发送单次测量指令 -> 等待数据完成 -> 读取寄存器 -> 转换单位。
* **说明**：IST8310 每次测量后会自动休眠，因此每次读取前都需要重新触发。函数内部包含约 15ms 的超时保护。

### 3. 获取物理数据

```cpp
IST8310_Vector get_data_uT();
float get_x_uT();
float get_y_uT();
float get_z_uT();
```

* **返回**：单位为微特斯拉 ($\mu T$) 的磁场强度。

### 4. 设置平均采样 (降噪)

```cpp
bool set_average(IST8310_AvgSample avg_y, IST8310_AvgSample avg_xz);
```

* **参数**：`IST8310_AVG_1` 到 `IST8310_AVG_16`。
* **建议**：推荐使用 `IST8310_AVG_16`。虽然这会将最大输出速率 (ODR) 限制在约 160Hz，但能提供最稳定的数据。

### 5. 硬件自检

```cpp
bool start_self_test();
bool stop_self_test();
```

* **功能**：开启/关闭内置激磁电流。开启后，读数会发生剧烈变化。这用于验证传感器是否损坏或焊接不良。

---

## ⚙️ 技术细节深度解析 (Technical Deep Dive)

### 为什么必须写入 `PDCNTL (0x42) = 0xC0`？

IST8310 是一款 AMR（各向异性磁阻）传感器。为了消除磁滞，它在每次测量前会进行 **Set/Reset** 操作（强磁脉冲复位磁畴）。`PDCNTL` 寄存器控制这个脉冲的持续时间。Datasheet 明确指出，初始上电后该寄存器必须被配置为 `0xC0`，否则 Set/Reset 脉冲能量可能不理想，导致传感器噪声变大或零偏不稳定。

### 分辨率是如何计算的？

Datasheet 给出的灵敏度是 **330 LSB/Gauss**。

* $1 \text{ Gauss} = 100 \mu T$
* Sensitivity $= \frac{330 \text{ LSB}}{100 \mu T} = 3.3 \text{ LSB} / \mu T$
* Resolution (1 LSB 代表多少 uT) $= \frac{1}{3.3} \approx 0.303 \mu T$

### 关于航向角 (Heading) 的说明

本库提供的 `get_heading_degrees()` 仅使用简单的 `atan2(y, x)` 计算。这只有在传感器**绝对水平**放置时才准确。
在实际应用（如无人机、手持设备）中，必须结合加速度计（Accelerometer）进行**倾斜补偿 (Tilt Compensation)**，并进行**硬铁/软铁校准 (Hard/Soft Iron Calibration)**，才能得到可用的电子罗盘数据。

---

## ❓ 常见问题 (Troubleshooting)

**Q: 初始化总是返回 false。**

* 检查 I2C 地址是否正确 (0x0E, 0x0C, 0x0D, 0x0F)。
* 检查 SDA/SCL 是否接了上拉电阻 (4.7kΩ 或 10kΩ)。
* 检查供电是否为 3.3V。

**Q: 读数全是 0.00。**

* **重点检查 C1 电容**。如果 Pin 10 没有接 4.7uF 电容，内部电荷泵无法工作，ADC 将读不到数据。

**Q: 数据抖动很大。**

* 确保调用了 `set_average(IST8310_AVG_16, IST8310_AVG_16)`。
* 检查电源纹波。
* 检查周围是否有强磁干扰（电机、扬声器、大电流导线）。

---

## 📄 License

MIT License.
Feel free to use and modify for your projects.
