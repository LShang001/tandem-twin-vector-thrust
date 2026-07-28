#pragma once

#include <Arduino.h>
#include <math.h>
#include <string.h>

/**
 * @class HorizontalKF_1Axis_VelMeas
 * @brief 水平单轴卡尔曼滤波器 (针对速度观测优化)
 *
 * @details
 * 该滤波器用于融合 "加速度计(IMU)" 和 "光流速度(Optical Flow)"。
 * 它可以输出平滑后的【速度】以及通过积分修正得到的【相对位移】。
 *
 * 适用于: 北向(North) 和 东向(East) 通道，需实例化两个对象。
 *
 * 核心模型差异 (与垂直KF相比):
 * - 垂直KF: 观测是位置 (H=[1,0])，修正位置并微分得到速度。
 * - 本KF:   观测是速度 (H=[0,1])，修正速度并积分得到位置。
 *
 * 状态向量 x = [position, velocity]^T
 *
 * 注意:
 * 由于没有绝对位置观测(如GPS)，位置状态 x[0] 会随着时间产生缓慢的漂移(Random Walk)，
 * 但这对于短时间的 "定点悬停(Loiter)" 或 "光流定位" 已经足够精确且鲁棒。
 */
class HorizontalKF_1Axis_VelMeas
{
public:
    HorizontalKF_1Axis_VelMeas()
    {
        reset();
    }

    /**
     * @brief 重置滤波器状态
     */
    void reset()
    {
        x[0] = 0.0f; // 位移 (m)
        x[1] = 0.0f; // 速度 (m/s)

        // 初始化协方差 P
        P[0][0] = 0.0f; // 位置初始并不确定，但在相对模式下设为0起点
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 1.0f; // 速度初始有一定不确定性

        // 默认参数
        q_pos_std_ = 0.01f; // 位置过程噪声 (模型误差)
        q_vel_std_ = 0.1f;  // 速度过程噪声 (加速度计噪声积分)

        measurement_outlier_gate_sq_ = 16.0f; // 4-sigma
        max_input_accel_ = 10.0f;             // 1g左右

        S_min_ = 1e-6f;

        // 协方差限制
        P_min_[0] = 1e-6f;
        P_max_[0] = 1000.0f;
        P_min_[1] = 1e-6f;
        P_max_[1] = 100.0f;
    }

    /**
     * @brief 配置噪声参数
     * @param process_noise_accel 加速度计的过程噪声标准差 (影响速度预测的不确定性)
     * @param process_noise_pos   位置积分的随机游走噪声 (通常设很小)
     */
    void begin(float process_noise_accel, float process_noise_pos)
    {
        q_vel_std_ = process_noise_accel;
        q_pos_std_ = process_noise_pos;
    }

    /**
     * @brief 预测步骤 (Predict)
     * @param accel_input 对应轴向的线性加速度 (m/s^2, 需去除重力且旋转到NED系)
     * @param dt 时间间隔 (s)
     */
    void predict(float accel_input, float dt)
    {
        // 1. 输入限幅
        if (accel_input > max_input_accel_)
            accel_input = max_input_accel_;
        if (accel_input < -max_input_accel_)
            accel_input = -max_input_accel_;

        // 2. 状态预测 (与垂直KF相同)
        // p = p + v*dt + 0.5*a*dt^2
        // v = v + a*dt
        x[0] += x[1] * dt + 0.5f * accel_input * dt * dt;
        x[1] += accel_input * dt;

        // 3. 协方差预测
        // F = [[1, dt], [0, 1]]
        float dt2 = dt * dt;

        // P_pred = F * P * F' + Q
        // 手动展开矩阵乘法以优化性能
        float p00 = P[0][0];
        float p01 = P[0][1];
        float p10 = P[1][0];
        float p11 = P[1][1];

        // F * P * F' 展开结果
        float np00 = p00 + dt * (p10 + p01) + dt2 * p11;
        float np01 = p01 + dt * p11;
        float np10 = p10 + dt * p11; // 理论上应等于 np01
        float np11 = p11;

        P[0][0] = np00 + (q_pos_std_ * q_pos_std_) * dt;
        P[0][1] = np01;
        P[1][0] = np10;
        P[1][1] = np11 + (q_vel_std_ * q_vel_std_) * dt;

        force_symmetry();
        apply_covariance_limits();
    }

    /**
     * @brief 更新步骤 (Update) - 针对【速度】观测
     * @param measurement_vel 光流测得的速度 (m/s)
     * @param measurement_noise_std 光流速度的噪声标准差 (m/s)
     */
    void update(float measurement_vel, float measurement_noise_std)
    {
        // 1. 观测矩阵 H = [0, 1] (因为观测的是速度 x[1])
        // y = z - Hx
        float y = measurement_vel - x[1];

        // 2. 计算新息协方差 S = H P H' + R
        // H = [0, 1], 所以 H P H' 就是 P[1][1]
        float R = measurement_noise_std * measurement_noise_std;
        float S = P[1][1] + R;

        S = fmaxf(S, S_min_); // 保护

        // 3. 异常值剔除
        if ((y * y) > (measurement_outlier_gate_sq_ * S))
        {
            return; // 拒绝该次光流数据
        }

        // 4. 计算卡尔曼增益 K = P H' S^-1
        // K = [ P[0][1], P[1][1] ]' / S
        float S_inv = 1.0f / S;
        float K[2];
        K[0] = P[0][1] * S_inv; // 位置增益 (通过速度误差修正位置，这是关键！)
        K[1] = P[1][1] * S_inv; // 速度增益

        // 5. 更新状态
        x[0] += K[0] * y;
        x[1] += K[1] * y;

        // 6. 更新协方差 (Joseph form)
        // I - KH
        // KH = [[0, K0], [0, K1]] (因为H=[0,1])
        // I - KH = [[1, -K0], [0, 1-K1]]

        float P_old[2][2];
        memcpy(P_old, P, sizeof(P));

        // 定义 A = I - KH
        float A[2][2] = {
            {1.0f, -K[0]},
            {0.0f, 1.0f - K[1]}};

        // 计算 P_new = A * P_old * A' + K * R * K'

        // Step A: temp = A * P_old
        float temp[2][2];
        matrix_mult_2x2(A, P_old, temp);

        // Step B: term1 = temp * A'
        float At[2][2];
        matrix_transpose_2x2(A, At);
        float term1[2][2];
        matrix_mult_2x2(temp, At, term1);

        // Step C: term2 = K * R * K'
        float term2[2][2];
        term2[0][0] = K[0] * K[0] * R;
        term2[0][1] = K[0] * K[1] * R;
        term2[1][0] = K[1] * K[0] * R;
        term2[1][1] = K[1] * K[1] * R;

        // Sum
        P[0][0] = term1[0][0] + term2[0][0];
        P[0][1] = term1[0][1] + term2[0][1];
        P[1][0] = term1[1][0] + term2[1][0];
        P[1][1] = term1[1][1] + term2[1][1];

        force_symmetry();
        apply_covariance_limits();
    }

    // --- Getters ---
    float getPosition() const { return x[0]; }
    float getVelocity() const { return x[1]; }

private:
    float x[2]; // x[0]: Position, x[1]: Velocity
    float P[2][2];

    float q_pos_std_;
    float q_vel_std_;
    float measurement_outlier_gate_sq_;
    float max_input_accel_;
    float S_min_;
    float P_min_[2];
    float P_max_[2];

    void matrix_mult_2x2(const float a[2][2], const float b[2][2], float out[2][2])
    {
        out[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
        out[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
        out[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
        out[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];
    }

    void matrix_transpose_2x2(const float in[2][2], float out[2][2])
    {
        out[0][0] = in[0][0];
        out[0][1] = in[1][0];
        out[1][0] = in[0][1];
        out[1][1] = in[1][1];
    }

    void force_symmetry()
    {
        P[1][0] = P[0][1] = 0.5f * (P[0][1] + P[1][0]);
    }

    void apply_covariance_limits()
    {
        if (P[0][0] < P_min_[0])
            P[0][0] = P_min_[0];
        if (P[0][0] > P_max_[0])
            P[0][0] = P_max_[0];
        if (P[1][1] < P_min_[1])
            P[1][1] = P_min_[1];
        if (P[1][1] > P_max_[1])
            P[1][1] = P_max_[1];
    }
};