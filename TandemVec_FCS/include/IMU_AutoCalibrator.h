/*
 * IMU_AutoCalibrator.h - IMU 自动静止检测与校准模块
 *
 * 版本：v3.0 (Pro)
 * 核心改进：
 * 1. 彻底重构静止检测算法：由“绝对模值检测”改为“动态方差检测”。
 *    解决了因传感器初始零偏导致无法进入校准状态的死锁问题。
 * 2. 引入递归统计滤波器，无需大内存 buffer 即可计算信号波动。
 *
 * 算法原理：
 * 静止 = (0.5g < Acc < 1.5g) && (Var(Acc) < Threshold)
 */

#ifndef IMU_AUTO_CALIBRATOR_H
#define IMU_AUTO_CALIBRATOR_H

#include <Arduino.h>
#include <math.h>

class IMU_AutoCalibrator
{
private:
    // --- 配置参数 ---

    // 1. 波动容差 (Variance Threshold)
    // 这是判断静止的核心参数。
    // 值越小，要求越稳。方差阈值：2e-5 (对应约 4.5mg 的抖动幅度)
    // 计算公式大致对应：(噪声峰峰值 / 2)^2
    const float STABILITY_VARIANCE_THRESH = 0.00002f;

    // 2. 宽松的重力范围检查
    // 只要读数在这个范围内，都认为是合理的“地球表面重力环境”，防止在自由落体时误判为静止
    const float GRAVITY_MIN_G = 0.5f;
    const float GRAVITY_MAX_G = 1.5f;

    // 3. 陀螺仪阈值：0.02 rad/s (约 1.1 deg/s)
    // ICM-42688 的陀螺仪也非常稳，可以设得更严
    const float STABILITY_GYRO_THRESH = 0.02f;

    // 4. 需保持静止的持续时间 (ms)
    const int STABILITY_DURATION_MS = 3000;

    // 5. 校准采集样本数
    const int CALIBRATION_SAMPLES = 4000;

    // --- 滤波器参数 ---
    // 递归滤波器的系数 (0.0 ~ 1.0)。值越小，历史权重越大，平滑效果越强。
    // 用于估算当前的平均值，以便计算方差。
    const float FILTER_ALPHA = 0.02f;

    // --- 内部状态 ---
    enum State
    {
        WAITING_FOR_STABILITY, // 等待机体静止
        CALIBRATING,           // 正在采集数据
        FINISHED               // 校准完成
    } current_state;

    unsigned long stable_start_time; // 记录开始静止的时间戳
    int sample_count;                // 当前采集样本计数
    bool _last_armed_state;          // 边缘检测

    // --- 动态统计变量 ---
    float acc_norm_mean;     // 加速度模值的“移动平均”
    float acc_norm_variance; // 加速度模值的“移动方差”

    // --- 累加器 (用于计算最终校准值) ---
    double sum_gyro[3];
    double sum_acc_norm;

    // --- 校准结果 ---
    float gyro_bias[3];  // [x, y, z] rad/s
    float accel_scale;   // 无量纲缩放因子
    bool _is_calibrated; // 标志位

public:
    IMU_AutoCalibrator()
    {
        reset();
    }

    void reset()
    {
        restart_process();
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
        accel_scale = 1.0f;
        _is_calibrated = false;
        _last_armed_state = false;

        // 初始化滤波器状态
        acc_norm_mean = 1.0f;
        acc_norm_variance = 1.0f; // 初始设大一点，防止上电瞬间误判
    }

    void restart_process()
    {
        current_state = WAITING_FOR_STABILITY;
        sample_count = 0;
        sum_gyro[0] = sum_gyro[1] = sum_gyro[2] = 0.0;
        sum_acc_norm = 0.0;
        stable_start_time = 0;

        // 重置滤波器，使其快速收敛到当前状态
        acc_norm_variance = 1.0f;
    }

    bool isCalibrated() const { return _is_calibrated; }
    bool isCalibrating() const { return current_state == CALIBRATING; }

    /**
     * @brief 核心更新函数
     * @param gx, gy, gz : rad/s
     * @param ax, ay, az : g
     * @param is_armed   : 解锁状态
     */
    bool update(float gx, float gy, float gz, float ax, float ay, float az, bool is_armed)
    {
        // 1. 状态切换逻辑 (同前版)
        if (_last_armed_state && !is_armed)
            restart_process();
        _last_armed_state = is_armed;

        if (is_armed)
        {
            if (current_state == CALIBRATING)
                restart_process();
            return false;
        }

        if (current_state == FINISHED)
            return false;

        // 2. 计算当前模值
        float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
        float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

        // 3. 更新递归统计滤波器 (计算方差)
        // 步骤 A: 更新均值 (低通滤波)
        // Mean_new = Mean_old + alpha * (Current - Mean_old)
        acc_norm_mean += FILTER_ALPHA * (acc_norm - acc_norm_mean);

        // 步骤 B: 计算当前偏差
        float diff = acc_norm - acc_norm_mean;

        // 步骤 C: 更新方差 (偏差平方的低通滤波)
        // Var_new = Var_old + alpha * (diff^2 - Var_old)
        acc_norm_variance += FILTER_ALPHA * ((diff * diff) - acc_norm_variance);

        // 4. 状态机逻辑
        switch (current_state)
        {
        case WAITING_FOR_STABILITY:
            // 判定静止的核心逻辑 v3.0:
            // 条件1: 模值在合理重力范围内 (0.5g ~ 1.5g)，排除自由落体。
            // 条件2: 方差极小 (STABILITY_VARIANCE_THRESH)，代表数值稳定不波动。
            // 条件3: 角速度小。
            if ((acc_norm > GRAVITY_MIN_G && acc_norm < GRAVITY_MAX_G) &&
                (acc_norm_variance < STABILITY_VARIANCE_THRESH) &&
                (gyro_norm < STABILITY_GYRO_THRESH))
            {
                if (stable_start_time == 0)
                {
                    stable_start_time = millis();
                }
                else if ((int32_t)(millis() - stable_start_time) > STABILITY_DURATION_MS)
                {
                    current_state = CALIBRATING;
                    sample_count = 0;
                    sum_gyro[0] = sum_gyro[1] = sum_gyro[2] = 0.0;
                    sum_acc_norm = 0.0;
                }
            }
            else
            {
                stable_start_time = 0;
            }
            break;

        case CALIBRATING:
            // 监控干扰：如果方差突然变大，说明受扰动
            if (acc_norm_variance > STABILITY_VARIANCE_THRESH * 2.0f ||
                gyro_norm > STABILITY_GYRO_THRESH * 2.0f)
            {
                restart_process();
                break;
            }

            sum_gyro[0] += gx;
            sum_gyro[1] += gy;
            sum_gyro[2] += gz;
            sum_acc_norm += acc_norm;
            sample_count++;

            if (sample_count >= CALIBRATION_SAMPLES)
            {
                // 计算结果
                gyro_bias[0] = (float)(sum_gyro[0] / sample_count);
                gyro_bias[1] = (float)(sum_gyro[1] / sample_count);
                gyro_bias[2] = (float)(sum_gyro[2] / sample_count);

                float avg_acc_norm = (float)(sum_acc_norm / sample_count);

                // 这里的 avg_acc_norm 可能等于 1.05g，这正是我们要修正的！
                if (avg_acc_norm > 0.1f)
                    accel_scale = 1.0f / avg_acc_norm;
                else
                    accel_scale = 1.0f;

                _is_calibrated = true;
                current_state = FINISHED;
                return true;
            }
            break;

        case FINISHED:
            break;
        }
        return false;
    }

    // 应用校准 (同前版)
    void apply(float &gx, float &gy, float &gz, float &ax, float &ay, float &az)
    {
        if (!_is_calibrated)
            return;

        gx -= gyro_bias[0];
        gy -= gyro_bias[1];
        gz -= gyro_bias[2];

        ax *= accel_scale;
        ay *= accel_scale;
        az *= accel_scale;
    }

    // 调试接口：增加打印方差信息，方便调试阈值
    void printDebug(Stream &serial)
    {
        serial.print("[IMU_Stat] Var: ");
        serial.print(acc_norm_variance, 6); // 打印6位小数
        serial.print(" | Mean: ");
        serial.print(acc_norm_mean, 3);

        if (_is_calibrated)
        {
            serial.print(" | Scale: ");
            serial.print(accel_scale, 4);
        }
        serial.println();
    }
};

#endif
