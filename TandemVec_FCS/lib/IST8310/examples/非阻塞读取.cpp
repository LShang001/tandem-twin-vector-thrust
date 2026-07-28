#include "IST8310.h"

IST8310 mag;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); // 强烈建议使用 400kHz I2C 速率

    Serial.println("Initializing IST8310...");

    // 初始化，使用推荐的 16 次平均 (低噪声)
    if (!mag.begin(&Wire, IST8310_I2C_ADDR_DEFAULT, IST_AVG_16))
    {
        Serial.println("IST8310 Init Failed!");
        while (1)
            ;
    }
    Serial.println("IST8310 Ready.");
}

void loop()
{
    // ---------------------------------------------------------
    // 1. 传感器更新 (非阻塞)
    // ---------------------------------------------------------
    // 每次循环都调用，它会自动处理状态
    if (mag.update())
    {
        // 只有当返回 true 时，才说明有新数据产生
        IST8310_Vector data = mag.get_data();

        Serial.print("Mag(uT): ");
        Serial.print(data.x, 2);
        Serial.print(", ");
        Serial.print(data.y, 2);
        Serial.print(", ");
        Serial.println(data.z, 2);
    }

    // ---------------------------------------------------------
    // 2. 飞控核心任务 (PID, 姿态解算)
    // ---------------------------------------------------------
    // 由于 mag.update() 没有 delay，这里可以全速运行
    // flight_control_pid_loop();

    // 模拟高频任务
    delayMicroseconds(100);
}
