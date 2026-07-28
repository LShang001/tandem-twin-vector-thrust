/**
 * @file MTF02P_BasicRead.ino
 * @author 高级AI
 * @brief MTF-02P传感器库基础使用示例
 * @version 1.0
 * @date 2023-10-27
 *
 * @details
 *  此示例演示了如何使用重构后的 MTF02P 库从传感器读取数据。
 *  库名现在直接反映了硬件型号，更加直观。
 *
 *  硬件连接 (以Arduino Mega为例):
 *  - 传感器 VCC -> Arduino 5V
 *  - 传感器 GND -> Arduino GND
 *  - 传感器 TX  -> Arduino RX1 (Pin 19)
 *  - 传感器 RX  -> Arduino TX1 (Pin 18)
 *
 *  传感器的默认波特率通常是9600或115200，请根据您的传感器手册进行确认。
 */

#include "MTF02P.h"

// 实例化一个 MTF02P 对象
MTF02P mtf02p;

// 假设传感器连接到Arduino Mega的Serial1
// 如果您使用其他串口，请修改这里
#define SENSOR_SERIAL Serial1

// 传感器串口的波特率，请根据实际情况修改
const long SENSOR_BAUDRATE = 9600;

void setup()
{
    // 启动用于调试输出的串口 (连接到电脑USB)
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("--- MTF-02P 传感器库示例 ---");

    // 启动用于连接传感器的串口
    SENSOR_SERIAL.begin(SENSOR_BAUDRATE);

    // 初始化MTF02P库，并传入传感器串口对象
    mtf02p.begin(SENSOR_SERIAL);

    Serial.println("MTF02P库已初始化，等待传感器数据...");
}

void loop()
{
    // 在主循环中持续调用update()函数
    // 当它返回true时，表示接收到了一帧完整有效的数据
    if (mtf02p.update())
    {

        Serial.println("----------------------------------------");
        Serial.print("接收到新数据！ 时间戳: ");
        Serial.print(mtf02p.getTimestamp_ms());
        Serial.println(" ms");

        // --- 处理和打印测距数据 ---
        if (mtf02p.isRangeDataValid())
        {
            Serial.print("  [测距] 距离: ");
            Serial.print(mtf02p.getDistance_mm());
            Serial.print(" mm, ");
            Serial.print("信号强度: ");
            Serial.println(mtf02p.getSignalStrength());
        }
        else
        {
            Serial.println("  [测距] 数据无效");
        }

        // --- 处理和打印光流数据 ---
        if (mtf02p.isFlowDataValid())
        {
            Serial.print("  [光流] 速度 X: ");
            Serial.print(mtf02p.getFlowVelX_cms());
            Serial.print(" cm/s@1m, ");
            Serial.print("速度 Y: ");
            Serial.print(mtf02p.getFlowVelY_cms());
            Serial.print(" cm/s@1m, ");
            Serial.print("质量: ");
            Serial.println(mtf02p.getFlowQuality());

            // 示例：计算实际速度 (需要一个高度值)
            float current_height_m = (float)mtf02p.getDistance_mm() / 1000.0;
            if (current_height_m > 0)
            {
                float actual_speed_x = (float)mtf02p.getFlowVelX_cms() * current_height_m;
                float actual_speed_y = (float)mtf02p.getFlowVelY_cms() * current_height_m;
                Serial.print("  [计算] 实际速度 X: ");
                Serial.print(actual_speed_x);
                Serial.print(" cm/s, Y: ");
                Serial.print(actual_speed_y);
                Serial.println(" cm/s");
            }
        }
        else
        {
            Serial.println("  [光流] 数据无效");
        }
    }
}
