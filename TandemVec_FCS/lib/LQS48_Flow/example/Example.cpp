/**
 * @file Example.ino
 * @brief LQ-S48 传感器测试程序
 */

#include "LQS48_Flow.h"

// 实例化驱动对象
LQS48_Flow flowSensor;

// 根据你的硬件选择串口
// 对于 Arduino Uno/Nano: 使用软串口 (Pin 10 RX, Pin 11 TX)
// 对于 Mega2560/ESP32/STM32: 建议使用 Serial1, Serial2 等硬串口
#if defined(ESP32) || defined(ARDUINO_AVR_MEGA2560)
#define FLOW_SERIAL Serial1
#else
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
#define FLOW_SERIAL mySerial
#endif

void setup()
{
    // 调试串口
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("LQ-S48 Flow Sensor Demo Start");

// 传感器串口初始化 (默认波特率 115200)
#if defined(ESP32)
    // ESP32 硬串口引脚定义 (RX, TX)
    FLOW_SERIAL.begin(115200, SERIAL_8N1, 16, 17);
#else
    FLOW_SERIAL.begin(115200);
#endif

    // 初始化库
    flowSensor.begin(FLOW_SERIAL);
}

void loop()
{
    // 1. 必须在 loop 中频繁调用 update
    // 该函数是非阻塞的，会自动从串口缓冲区读取数据解析
    if (flowSensor.update())
    {

        // 2. 只有当 update 返回 true 时，说明解析到了一帧新数据
        LQS48_Data_t data = flowSensor.getData();

        // 3. 安全性检查 (飞控逻辑核心)
        if (flowSensor.isFlowValid())
        {
            Serial.print("[Flow] X_Rad: ");
            Serial.print(data.flow_x_rad, 4); // 打印4位小数
            Serial.print(" | Y_Rad: ");
            Serial.print(data.flow_y_rad, 4);

            // 计算角速度 (用于 EKF 融合)
            Serial.print(" | Vel_X: ");
            Serial.print(flowSensor.getVelAngX(), 2);
            Serial.println(" rad/s");
        }
        else
        {
            Serial.print("[Warning] Flow Data Unreliable! Code: ");
            Serial.println(data.quality);
        }

        if (flowSensor.isHeightValid())
        {
            Serial.print("[Height] Dist: ");
            Serial.print(data.height_mm);
            Serial.println(" mm");
        }

        Serial.println("-----------------------------");
    }

    // 模拟飞控的其他任务...
}