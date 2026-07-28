/**
 * =============================================================================
 * @file    LQS48_Example.ino
 * @brief   LQ-S48 光流测距模块 Arduino 使用示例
 * =============================================================================
 *
 * 【硬件连接】
 *
 *   LQ-S48 模块         Arduino (以 ESP32 为例)
 *   ─────────────────────────────────────────
 *   5V      ────────►   5V (或 VIN)
 *   GND     ────────►   GND
 *   TX      ────────►   RX1 (GPIO 16)
 *   RX      ────────►   TX1 (GPIO 17)  // 发送配置指令时需要
 *
 * 【接口线序】(手册第7页)
 *
 *   模块正面朝上，从左到右依次为:
 *   [5V] [RX] [TX] [GND]
 *
 * 【模块安装方向】(手册第11页)
 *
 *   模块的 Y 轴正方向应指向机头
 *
 *           ↑ 机头 (Y+)
 *           │
 *      ┌────┴────┐
 *      │ LQ-S48  │
 *      │ [镜头]  │
 *      └─────────┘
 *
 * =============================================================================
 */

#include "LQS48_Flow.h"

// =============================================================================
//                              全局对象
// =============================================================================

LQS48_Flow sensor; // 创建传感器对象

// 用于控制打印频率
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // 每100ms打印一次

// =============================================================================
//                              初始化
// =============================================================================

void setup()
{
    // ─────────────────────────────────────────────────────
    // 1. 初始化调试串口
    // ─────────────────────────────────────────────────────
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    } // 等待串口就绪 (仅USB串口需要)

    Serial.println();
    Serial.println("========================================");
    Serial.println("  LQ-S48 光流测距模块 测试程序");
    Serial.println("  武汉凌启科技有限公司");
    Serial.println("========================================");
    Serial.println();

    // ─────────────────────────────────────────────────────
    // 2. 初始化传感器串口
    // ─────────────────────────────────────────────────────
    // 根据实际硬件修改串口号和引脚
    // Arduino Mega: Serial1 (TX1=18, RX1=19)
    // ESP32:        Serial1 或 Serial2

    Serial1.begin(115200); // 默认波特率 115200

    // ESP32 可指定引脚:
    // Serial1.begin(115200, SERIAL_8N1, 16, 17);  // RX=16, TX=17

    // ─────────────────────────────────────────────────────
    // 3. 初始化传感器驱动
    // ─────────────────────────────────────────────────────
    sensor.begin(Serial1);

    // ─────────────────────────────────────────────────────
    // 4. 重置统计和累加器
    // ─────────────────────────────────────────────────────
    sensor.resetStatistics();
    sensor.resetAccumulators();

    Serial.println("[INFO] 初始化完成，等待数据...");
    Serial.println();
}

// =============================================================================
//                              主循环
// =============================================================================

void loop()
{
    // ─────────────────────────────────────────────────────
    // 1. 更新传感器数据 (必须频繁调用)
    // ─────────────────────────────────────────────────────
    bool newData = sensor.update();

    // ─────────────────────────────────────────────────────
    // 2. 有新数据时处理
    // ─────────────────────────────────────────────────────
    if (newData)
    {
        // 控制打印频率，避免刷屏
        unsigned long now = millis();
        if (now - lastPrintTime >= PRINT_INTERVAL)
        {
            lastPrintTime = now;
            printSensorData();
        }
    }

    // ─────────────────────────────────────────────────────
    // 3. 处理串口命令 (用于调试)
    // ─────────────────────────────────────────────────────
    handleSerialCommand();
}

// =============================================================================
//                              打印传感器数据
// =============================================================================

void printSensorData()
{
    Serial.println("────────────────────────────────────────");

    // ─────────────────────────────────────────────────────
    // 1. 数据质量
    // ─────────────────────────────────────────────────────
    uint8_t quality = sensor.getQuality();
    Serial.print("[质量] ");
    switch (quality)
    {
    case LQS48_QUALITY_STRONG:
        Serial.print("强可信(0)");
        break;
    case LQS48_QUALITY_WEAK:
        Serial.print("弱可信(1)");
        break;
    case LQS48_QUALITY_BAD_XYR:
        Serial.print("光流不可信(2)");
        break;
    case LQS48_QUALITY_BAD_H:
        Serial.print("高度不可信(3)");
        break;
    case LQS48_QUALITY_BAD_ALL:
        Serial.print("全部不可信(23)");
        break;
    default:
        Serial.print("未知");
        break;
    }
    Serial.print("  光照: ");
    Serial.println(sensor.getLux());

    // ─────────────────────────────────────────────────────
    // 2. 单帧增量数据
    // ─────────────────────────────────────────────────────
    Serial.println();
    Serial.println("[单帧增量]");

    if (sensor.isFlowValid())
    {
        Serial.print("  光流 X: ");
        Serial.print(sensor.getFlowX_rad(), 6);
        Serial.print(" rad  (");
        Serial.print(sensor.getDisplacementX_mm(), 2);
        Serial.println(" mm)");

        Serial.print("  光流 Y: ");
        Serial.print(sensor.getFlowY_rad(), 6);
        Serial.print(" rad  (");
        Serial.print(sensor.getDisplacementY_mm(), 2);
        Serial.println(" mm)");

        Serial.print("  旋转角: ");
        Serial.print(sensor.getRotation_deg(), 3);
        Serial.println(" °");
    }
    else
    {
        Serial.println("  光流数据: [无效，已丢弃]");
    }

    if (sensor.isHeightValid())
    {
        Serial.print("  高度: ");
        Serial.print(sensor.getHeight_mm());
        Serial.println(" mm");
    }
    else
    {
        Serial.println("  高度数据: [无效，已丢弃]");
    }

    // ─────────────────────────────────────────────────────
    // 3. 速度数据
    // ─────────────────────────────────────────────────────
    Serial.println();
    Serial.println("[速度]");

    if (sensor.isFlowValid() && sensor.isHeightValid())
    {
        Serial.print("  线速度 X: ");
        Serial.print(sensor.getVelocityX_mm_s(), 1);
        Serial.print(" mm/s (");
        Serial.print(sensor.getVelocityX_m_s(), 3);
        Serial.println(" m/s)");

        Serial.print("  线速度 Y: ");
        Serial.print(sensor.getVelocityY_mm_s(), 1);
        Serial.print(" mm/s (");
        Serial.print(sensor.getVelocityY_m_s(), 3);
        Serial.println(" m/s)");

        Serial.print("  合成速度: ");
        Serial.print(sensor.getVelocity_mm_s(), 1);
        Serial.print(" mm/s (");
        Serial.print(sensor.getVelocity_m_s(), 3);
        Serial.println(" m/s)");

        Serial.print("  旋转速度: ");
        Serial.print(sensor.getRotationVel_deg_s(), 2);
        Serial.println(" °/s");
    }
    else
    {
        Serial.println("  速度数据: [无效]");
    }

    // ─────────────────────────────────────────────────────
    // 4. 累加数据 (从起点开始的总位移)
    // ─────────────────────────────────────────────────────
    Serial.println();
    Serial.println("[累计位移]");
    Serial.print("  X轴: ");
    Serial.print(sensor.getAccumX_mm(), 1);
    Serial.print(" mm (");
    Serial.print(sensor.getAccumX_rad(), 4);
    Serial.println(" rad)");

    Serial.print("  Y轴: ");
    Serial.print(sensor.getAccumY_mm(), 1);
    Serial.print(" mm (");
    Serial.print(sensor.getAccumY_rad(), 4);
    Serial.println(" rad)");

    Serial.print("  旋转: ");
    Serial.print(sensor.getAccumRot_deg(), 2);
    Serial.println(" °");

    // ─────────────────────────────────────────────────────
    // 5. 通信统计
    // ─────────────────────────────────────────────────────
    Serial.println();
    Serial.print("[统计] 有效帧: ");
    Serial.print(sensor.getValidFrames());
    Serial.print("  错误帧: ");
    Serial.print(sensor.getErrorFrames());
    Serial.print("  丢帧: ");
    Serial.print(sensor.getLostFrames());
    Serial.print(" (");
    Serial.print(sensor.getLostRate(), 1);
    Serial.println("%)");

    Serial.println();
}

// =============================================================================
//                              串口命令处理
// =============================================================================

void handleSerialCommand()
{
    if (Serial.available() > 0)
    {
        char cmd = Serial.read();

        switch (cmd)
        {
        // ─────────────────────────────────────────
        // 'r' - 重置累加器
        // ─────────────────────────────────────────
        case 'r':
        case 'R':
            sensor.resetAccumulators();
            Serial.println("[CMD] 累加器已重置");
            break;

        // ─────────────────────────────────────────
        // 's' - 重置统计
        // ─────────────────────────────────────────
        case 's':
        case 'S':
            sensor.resetStatistics();
            Serial.println("[CMD] 统计已重置");
            break;

        // ─────────────────────────────────────────
        // 'c' - 高度校准 (示例: 1000mm)
        // ─────────────────────────────────────────
        case 'c':
        case 'C':
            Serial.println("[CMD] 发送高度校准指令 (1000mm)...");
            sensor.calibrateHeight(1000);
            Serial.println("[CMD] 请等待约5秒，观察模块LED状态");
            break;

        // ─────────────────────────────────────────
        // 'v' - 切换到 VOFA+ 协议
        // ─────────────────────────────────────────
        case 'v':
        case 'V':
            Serial.println("[CMD] 切换到 VOFA+ 协议...");
            sensor.setProtocol(LQS48_PROTOCOL_VOFA);
            Serial.println("[CMD] 已切换，本程序将无法继续解析数据");
            break;

        // ─────────────────────────────────────────
        // '0' - 切换回自定义协议
        // ─────────────────────────────────────────
        case '0':
            Serial.println("[CMD] 切换到自定义协议...");
            sensor.setProtocol(LQS48_PROTOCOL_CUSTOM);
            break;

        // ─────────────────────────────────────────
        // 'h' - 帮助
        // ─────────────────────────────────────────
        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        default:
            break;
        }

        // 清空串口缓冲区
        while (Serial.available())
            Serial.read();
    }
}

// =============================================================================
//                              打印帮助信息
// =============================================================================

void printHelp()
{
    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║          可用命令                      ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║  r - 重置累加器 (归零位移)             ║");
    Serial.println("║  s - 重置统计计数                      ║");
    Serial.println("║  c - 高度校准 (1000mm)                 ║");
    Serial.println("║  v - 切换到 VOFA+ 协议                 ║");
    Serial.println("║  0 - 切换回自定义协议                  ║");
    Serial.println("║  h - 显示帮助                          ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
}

// =============================================================================
//                        高级应用示例 (供参考)
// =============================================================================

/**
 * @brief 示例: 速度控制回调
 *
 * 获取当前线速度，可用于速度环控制
 */
void getVelocity(float &vx_mm_s, float &vy_mm_s)
{
    if (sensor.isFlowValid() && sensor.isHeightValid())
    {
        // 直接使用库函数获取线速度
        vx_mm_s = sensor.getVelocityX_mm_s();
        vy_mm_s = sensor.getVelocityY_mm_s();
    }
    else
    {
        // 数据无效时返回0
        vx_mm_s = 0;
        vy_mm_s = 0;
    }
}

/**
 * @brief 示例: 获取速度向量 (用于飞控)
 */
void getVelocityVector(float &vx_m_s, float &vy_m_s, float &speed_m_s)
{
    if (sensor.isFlowValid() && sensor.isHeightValid())
    {
        vx_m_s = sensor.getVelocityX_m_s();   // X轴速度 (m/s)
        vy_m_s = sensor.getVelocityY_m_s();   // Y轴速度 (m/s)
        speed_m_s = sensor.getVelocity_m_s(); // 合成速度 (m/s)
    }
    else
    {
        vx_m_s = 0;
        vy_m_s = 0;
        speed_m_s = 0;
    }
}

/**
 * @brief 示例: 检查是否需要开启补光灯
 */
bool needBottomLight()
{
    // 手册建议环境照度 > 20Lux
    // lux 值范围 0~100，越大越亮
    // 这里设定阈值为 30，可根据实际情况调整
    return (sensor.getLux() < 30);
}

/**
 * @brief 示例: 自动高度校准流程 (仅APM飞控)
 *
 * 手册第8页描述的自动校准流程:
 * 1. 上电后5秒内，将两个遥控器摇杆都拨到右上方顶端
 * 2. 光流质量变为0，再变为12，表示进入校准模式
 * 3. 飞到指定高度 (LQ-S48: 13米)
 * 4. 悬停约8秒，光流质量 > 100 时校准完成
 *
 * @note 此功能需要配合飞控实现，这里仅作说明
 */
