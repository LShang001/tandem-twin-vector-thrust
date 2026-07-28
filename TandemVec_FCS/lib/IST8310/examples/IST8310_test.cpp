#include <Arduino.h>
#include <Wire.h>
#include "IST8310.h"

// 实例化驱动对象
IST8310 compass;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // 等待串口监视器连接

    Serial.println("Initializing IST8310 Magnetometer...");

    // 启动 I2C 总线
    Wire.begin();
    Wire.setClock(400000); // 设置 I2C 速度为 400kHz (Fast Mode)

    // 初始化传感器
    if (!compass.begin())
    {
        Serial.println("Error: IST8310 not found or init failed!");
        Serial.println("Please check wiring (SDA, SCL) and Power.");
        while (1)
            ; // 初始化失败，死循环
    }

    Serial.println("IST8310 Initialized Successfully!");

    // 设置本地磁偏角 (可选)
    // 例如：北京的磁偏角约为 -6度 (-0.104 弧度)
    // 请查询当地磁偏角：https://www.magnetic-declination.com/
    // compass.set_declination(-0.104);
    // 长沙的磁偏角: -4° 8' = -4.1333° ≈ -0.07214 弧度
    // 来源：magnetic-declination.com
    compass.set_declination(-0.07214);
}

void loop()
{
    // 触发读取
    if (compass.read())
    {
        // 获取处理后的微特斯拉数据
        float x = compass.get_x_uT();
        float y = compass.get_y_uT();
        float z = compass.get_z_uT();
        float heading = compass.get_heading_degrees();

        // 串口打印格式化输出
        Serial.print("Mag (uT): [");
        Serial.print(x, 2);
        Serial.print(", ");
        Serial.print(y, 2);
        Serial.print(", ");
        Serial.print(z, 2);
        Serial.print("]  Heading: ");
        Serial.print(heading, 1);
        Serial.println(" deg");
    }
    else
    {
        Serial.println("Warning: Sensor read timeout.");
    }

    // 控制采样率
    // 由于设置了 16次平均，传感器物理耗时约 6-10ms
    // 这里延时 100ms 意味着约 10Hz 输出率
    delay(100);
}