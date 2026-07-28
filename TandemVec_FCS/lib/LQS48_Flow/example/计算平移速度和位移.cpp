#include "LQS48_Flow.h"

// 全局变量
float velocity_x_m_s = 0; // X轴对地速度 (m/s)
float velocity_y_m_s = 0; // Y轴对地速度 (m/s)
float position_x_m = 0;   // X轴累计位移 (m)
float position_y_m = 0;   // Y轴累计位移 (m)

// 实例化传感器
LQS48_Flow flow;

void calculateGroundMotion()
{
    // 1. 获取光流传感器数据
    LQS48_Data_t data = flow.getData();

    // 安全检查：如果高度不可信或光流质量太差，不计算（防止数据飞掉）
    if (!flow.isFlowValid() || !flow.isHeightValid())
    {
        velocity_x_m_s = 0;
        velocity_y_m_s = 0;
        return;
    }

    // 2. 提取基础物理量
    // 换算高度单位：mm -> m
    float height_m = data.height_mm / 1000.0f;
    // 限制最小高度：防止除以0或贴地时噪声过大（例如限制在 0.1m 以上）
    if (height_m < 0.1f)
        height_m = 0.1f;

    // 提取时间间隔 dt (ms -> s)
    float dt_flow_sec = data.dt_flow_ms / 1000.0f;
    if (dt_flow_sec <= 0.001f)
        dt_flow_sec = 0.025f; // 防止除0，默认25ms

    // 3. 获取光流角速度 (Rad/s)
    // 注意坐标系：通常模块 Y轴 指向 机头 X轴
    float flow_rate_x_body = data.flow_y_rad / dt_flow_sec;
    float flow_rate_y_body = data.flow_x_rad / dt_flow_sec;

    // 4. 获取陀螺仪角速度 (需要你自己的IMU库)
    // 假设单位是 rad/s。注意轴向对应！
    // 绕Y轴旋转是俯仰(Pitch)，影响机头前后运动
    // 绕X轴旋转是横滚(Roll)，影响左右运动
    float gyro_rate_pitch = getGyroPitchRate(); // 需用户实现
    float gyro_rate_roll = getGyroRollRate();   // 需用户实现

    // 5. 核心融合算法：补偿旋转 (互补)
    // 公式：平移角速度 = 总光流角速度 - 旋转角速度
    // 注意：正负号极度重要！建议先通过 log 观察数据来确定符号方向
    float linear_rate_x = flow_rate_x_body - gyro_rate_pitch;
    float linear_rate_y = flow_rate_y_body - gyro_rate_roll;
    // *调试技巧：手持飞机仅原地旋转，linear_rate 应该接近 0*

    // 6. 计算线速度 (v = omega * r)
    velocity_x_m_s = linear_rate_x * height_m;
    velocity_y_m_s = linear_rate_y * height_m;

    // 7. 计算位移 (积分)
    // 这里的 dt 应该是主循环的 dt，或者简单使用 dt_flow_sec
    position_x_m += velocity_x_m_s * dt_flow_sec;
    position_y_m += velocity_y_m_s * dt_flow_sec;
}

// ------------------------------------------------
// 模拟获取陀螺仪数据的函数 (请替换为真实的 MPU6050/ICM20602 读取代码)
float getGyroPitchRate()
{
    // return mpu.getGyroY();
    return 0.0f;
}
float getGyroRollRate()
{
    // return mpu.getGyroX();
    return 0.0f;
}