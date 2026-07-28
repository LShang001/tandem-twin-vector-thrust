/**
 * @file main.ino
 * @brief STM32 IST8310 模块化示例
 * @details 将初始化逻辑封装独立函数，提高代码可维护性
 */

#include <Arduino.h>
#include <Wire.h>
#include "IST8310.h"

// ============================================================================
// 全局对象定义
// ============================================================================

// 1. 定义自定义 I2C 对象 (SDA=PB7, SCL=PB6)
// 注意：对象必须在全局定义，以便在 setup 和 loop 中都能访问
TwoWire Wire1(PB7, PB6);

// 2. 实例化传感器驱动
IST8310 compass;

// ============================================================================
// 独立初始化函数
// ============================================================================

/**
 * @brief 初始化磁力计硬件子系统
 * @return true 初始化成功
 * @return false 初始化失败 (I2C错误或ID不匹配)
 */
bool initMagnetometer()
{
    Serial.println("----------------------------------------");
    Serial.println("[Init] Starting Magnetometer Setup...");

    // 1. 启动 I2C 总线硬件
    //    在 STM32Duino 中，这一步会配置 GPIO 复用功能
    Wire1.begin();
    Wire1.setClock(400000); // 400kHz Fast Mode
    Serial.println("[Init] I2C Bus (Wire1) Started at 400kHz");

    // 2. 初始化传感器驱动
    //    传入 &Wire1 指针，指定使用 PB7/PB6 端口
    if (!compass.begin(&Wire1))
    {
        Serial.println("[Error] IST8310 Not Found!");
        Serial.println("[Error] Please check wiring (SDA/SCL) and Power.");
        return false;
    }

    // 3. 配置传感器参数 (可选)
    //    设置长沙的磁偏角: -4.13度 ≈ -0.072 弧度
    compass.set_declination(-0.072);
    Serial.println("[Init] Declination set to -0.072 rad");

    Serial.println("[Init] IST8310 Setup Finished Successfully.");
    Serial.println("----------------------------------------");

    return true;
}

// ============================================================================
// Arduino 标准 Setup
// ============================================================================
void setup()
{
    // 系统级初始化 (串口、LED等)
    Serial.begin(115200);
    // while (!Serial); // 如果是 USB CDC 设备，建议开启此行等待连接
    delay(1000);

    // 调用独立的初始化函数
    if (!initMagnetometer())
    {
        // 初始化失败的处理逻辑
        Serial.println("SYSTEM HALTED: Sensor Init Failed.");
        while (1)
        {
            // 可以在这里加入 LED 快闪代码提示错误
            delay(100);
        }
    }

    Serial.println("System Ready. Loop starting...");
}

// ============================================================================
// Arduino 标准 Loop
// ============================================================================
void loop()
{
    // 尝试读取数据
    if (compass.read())
    {
        // 获取物理量
        IST8310_Vector mag = compass.get_data_uT();
        float heading = compass.get_heading_degrees();

        // 打印数据
        Serial.print("Mag: [");
        Serial.print(mag.x, 2);
        Serial.print(", ");
        Serial.print(mag.y, 2);
        Serial.print(", ");
        Serial.print(mag.z, 2);
        Serial.print("] uT   Heading: ");
        Serial.print(heading, 1);
        Serial.println(" deg");
    }
    else
    {
        // 读取失败或数据未就绪 (正常现象，取决于采样率)
    }

    delay(100);
}
