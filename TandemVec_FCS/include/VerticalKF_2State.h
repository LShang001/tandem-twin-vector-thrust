#pragma once

#include <Arduino.h>
#include <math.h>   // 引入数学库以使用 isnan(), fmaxf()
#include <string.h> // 引入字符串库以使用 memcpy()

/**
 * @class VerticalKF_2State
 * @brief 鲁棒型二状态垂直卡尔曼滤波器 (高度、速度)
 *
 * @details
 * 这是针对特定应用场景优化的卡尔曼滤波器版本，将状态估计简化为最核心的两个变量：
 * 高度和速度。它移除了加速度计零偏的估计，以换取在某些条件下的更高稳定性和
 * 更简化的调试过程。
 *
 * 当满足以下条件之一时，此二状态模型是比三状态模型更好的选择：
 * 1.  **IMU质量高:** 加速度计的零偏非常小、稳定，且已在系统启动前进行了充分校准。
 * 2.  **动态激励不足:** 系统长时间处于静止或匀速运动状态，缺乏足够的加速度变化来
 *     有效地区分真实加速度和传感器零偏。
 *
 * 本实现完整保留了前一版本的全部鲁棒性特性：
 * - **异常值抑制:** 对输入加速度和测量高度进行严格的异常值剔除。
 * - **Joseph形式协方差更新:** 保证协方差矩阵的对称性和正定性，杜绝滤波器发散。
 * - **协方差限制:** 防止滤波器“傲慢”或“遗忘”，维持合理的学习能力。
 * - **安全计算:** 避免计算过程中的除零等数值问题。
 *
 * @section model 核心模型
 *
 * 状态向量 `x` 定义为:
 * $x = [h, v]^T$ (高度, 速度)
 *
 * 状态预测模型:
 * $h_k = h_{k-1} + v_{k-1} \cdot dt + 0.5 \cdot a_{meas} \cdot dt^2$
 * $v_k = v_{k-1} + a_{meas} \cdot dt$
 *
 * 过程噪声 Q 直接作为调整参数，反映了模型预测的不确定性。
 *
 * 观测模型:
 * $z_h = h_k + \text{noise}$
 */
class VerticalKF_2State
{
public:
    /**
     * @brief 构造函数。
     * @details 初始化状态、协方差和默认的鲁棒性参数。
     */
    VerticalKF_2State()
    {
        // 初始化状态向量 x
        x[0] = 0.0f; // 高度 (m)
        x[1] = 0.0f; // 速度 (m/s)

        // 初始化状态协方差矩阵 P
        P[0][0] = 1.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 1.0f;

        // 初始化过程噪声参数
        q_pos_variance_ = 0.0f;
        q_vel_variance_ = 0.0f;

        // 初始化鲁棒性增强参数的默认值
        measurement_outlier_gate_sq_ = 25.0f; // 默认3-sigma门限 (3*3=9)
        max_input_accel_ = 20.0f;             // 默认最大输入加速度 ±20 m/s^2 (约 ±2g)
        S_min_ = 1e-9f;                       // 增益计算中S的最小值，防除零

        // 默认协方差限制
        P_min_[0] = 1e-6f;
        P_max_[0] = 100.0f; // 高度方差限制
        P_min_[1] = 1e-4f;
        P_max_[1] = 25.0f; // 速度方差限制
    }

    /**
     * @brief 配置并初始化卡尔曼滤波器。
     * @details 在系统启动后，传感器数据稳定时调用。
     *
     * @param initial_height 系统的初始高度 (米)。
     * @param process_noise_pos_variance 过程噪声中由模型引入的位置方差。
     * @param process_noise_vel_variance 过程噪声中由模型引入的速度方差。
     */
    void begin(float initial_height, float process_noise_pos_variance, float process_noise_vel_variance)
    {
        x[0] = initial_height;
        q_pos_variance_ = process_noise_pos_variance;
        q_vel_variance_ = process_noise_vel_variance;
    }

    /**
     * @brief (可选) 配置鲁棒性参数。
     * @details 可在 begin() 后调用以覆盖默认的鲁棒性参数。
     *
     * @param outlier_gate N-sigma门限，用于测量值异常抑制。例如，传入3.0代表3-sigma。
     * @param max_accel 允许的最大输入加速度 (m/s^2)，用于输入值异常抑制。
     * @param p_min_h 高度方差下限。
     * @param p_max_h 高度方差上限。
     * @param p_min_v 速度方差下限。
     * @param p_max_v 速度方差上限。
     */
    void configureRobustness(float outlier_gate, float max_accel,
                             float p_min_h, float p_max_h,
                             float p_min_v, float p_max_v)
    {
        measurement_outlier_gate_sq_ = outlier_gate * outlier_gate;
        max_input_accel_ = max_accel;
        P_min_[0] = p_min_h;
        P_max_[0] = p_max_h;
        P_min_[1] = p_min_v;
        P_max_[1] = p_max_v;
    }

    /**
     * @brief 执行卡尔曼滤波器的预测步骤。
     *
     * @param accel_z 经过坐标变换和重力补偿后的垂直线性加速度 (m/s^2)。
     * @param dt 时间间隔 (秒)。
     */
    void predict(float accel_z, float dt)
    {
        // --- 0. 输入异常值抑制 ---
        // 对输入加速度进行饱和限制，防止异常尖峰污染状态。
        if (accel_z > max_input_accel_)
            accel_z = max_input_accel_;
        if (accel_z < -max_input_accel_)
            accel_z = -max_input_accel_;

        // --- 1. 状态预测: x_k = F * x_{k-1} + G * u_k ---
        // 在此模型中，不再估计和减去零偏
        x[0] += x[1] * dt + 0.5f * accel_z * dt * dt;
        x[1] += accel_z * dt;

        // --- 2. 协方差预测: P_k = F * P_{k-1} * F' + Q ---
        // 状态转移矩阵 F
        float F[2][2] = {
            {1.0f, dt},
            {0.0f, 1.0f}};

        // 计算 F * P
        float FP[2][2];
        matrix_mult_2x2(F, P, FP);

        // 计算 (F * P) * F'
        float Ft[2][2];
        float FPFt[2][2];
        matrix_transpose_2x2(F, Ft);
        matrix_mult_2x2(FP, Ft, FPFt);

        // 更新协方差矩阵 P
        memcpy(P, FPFt, sizeof(P));

        // 累加过程噪声协方差 Q
        // 这是一个简化的Q模型，直接对位置和速度的不确定性进行建模
        // Q = diag(q_pos, q_vel)
        P[0][0] += q_pos_variance_;
        P[1][1] += q_vel_variance_;

        // --- 3. 强制对称与协方差限制 ---
        force_symmetry();
        apply_covariance_limits();
    }

    /**
     * @brief 执行卡尔曼滤波器的更新步骤。
     *
     * @param measurement_height 测量到的高度值 (米)。
     * @param measurement_noise_std 测量传感器噪声的标准差 (米)。
     */
    void update(float measurement_height, float measurement_noise_std)
    {
        // 安全检查：跳过无效测量
        if (isnan(measurement_height))
            return;

        // 观测矩阵 H = [1, 0]

        // --- 1. 计算残差 y 和残差协方差 S ---
        float y = measurement_height - x[0]; // y = z - H*x_pred
        float R = measurement_noise_std * measurement_noise_std;
        float S = P[0][0] + R; // S = H*P_pred*H' + R

        // --- 2. 测量值异常抑制 ---
        // 使用N-sigma门限检查残差是否过大。
        if (y * y > measurement_outlier_gate_sq_ * S)
        {
            return; // 拒绝异常值
        }

        // --- 3. 改进的卡尔曼增益计算 ---
        // 增加对S的下限保护，防止除零。
        S = fmaxf(S, S_min_);
        float S_inv = 1.0f / S;
        float K[2]; // 卡尔曼增益 K = P_pred*H'*S^-1
        K[0] = P[0][0] * S_inv;
        K[1] = P[1][0] * S_inv;

        // --- 4. 更新状态估计 x_new = x_pred + K * y ---
        x[0] += K[0] * y;
        x[1] += K[1] * y;

        // --- 5. 使用Joseph形式更新协方差矩阵 P_new = (I - KH)P(I-KH)' + KRK' ---
        // 这是数值稳定性的关键！
        float P_old[2][2];
        memcpy(P_old, P, sizeof(P_old));

        // 计算 (I - KH)
        // H = [1, 0], K = [K0, K1]' -> KH = [K0, 0; K1, 0]
        // I - KH = [1-K0, 0; -K1, 1]
        float I_m_KH[2][2] = {
            {1.0f - K[0], 0.0f},
            {-K[1], 1.0f}};

        // 计算 (I - KH) * P_old
        float IKH_P[2][2];
        matrix_mult_2x2(I_m_KH, P_old, IKH_P);

        // 计算 (I - KH)'
        float I_m_KH_T[2][2];
        matrix_transpose_2x2(I_m_KH, I_m_KH_T);

        // 计算第一项: (I - KH) * P_old * (I - KH)'
        float term1[2][2];
        matrix_mult_2x2(IKH_P, I_m_KH_T, term1);

        // 计算第二项: K * R * K'
        float term2[2][2];
        // K * K' (外积)
        term2[0][0] = K[0] * K[0] * R;
        term2[0][1] = K[0] * K[1] * R;
        term2[1][0] = K[1] * K[0] * R;
        term2[1][1] = K[1] * K[1] * R;

        // P_new = term1 + term2
        P[0][0] = term1[0][0] + term2[0][0];
        P[0][1] = term1[0][1] + term2[0][1];
        P[1][0] = term1[1][0] + term2[1][0];
        P[1][1] = term1[1][1] + term2[1][1];

        // --- 6. 强制对称与协方差限制 ---
        force_symmetry();
        apply_covariance_limits();
    }

    /** @brief 获取当前估计的高度。 @return 滤波后的垂直高度 (米)。 */
    float getHeight() const { return x[0]; }
    /** @brief 获取当前估计的速度。 @return 滤波后的垂直速度 (米/秒)。 */
    float getVelocity() const { return x[1]; }

private:
    // --- 核心变量 ---
    float x[2];    ///< 状态向量: x = [height, velocity]'
    float P[2][2]; ///< 状态协方差矩阵

    // --- 噪声参数 ---
    float q_pos_variance_; ///< 位置过程噪声方差
    float q_vel_variance_; ///< 速度过程噪声方差

    // --- 鲁棒性增强参数 ---
    float measurement_outlier_gate_sq_; ///< 测量异常值门限的平方 (N*N)
    float max_input_accel_;             ///< 最大允许输入加速度
    float S_min_;                       ///< 增益计算中S的最小值
    float P_min_[2];                    ///< 协方差对角线元素下限
    float P_max_[2];                    ///< 协方差对角线元素上限

    // --- 私有辅助函数 (2x2) ---

    /** @brief 2x2矩阵乘法: out = a * b */
    void matrix_mult_2x2(const float a[2][2], const float b[2][2], float out[2][2])
    {
        out[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
        out[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
        out[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
        out[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];
    }

    /** @brief 2x2矩阵转置: out = in' */
    void matrix_transpose_2x2(const float in[2][2], float out[2][2])
    {
        out[0][0] = in[0][0];
        out[0][1] = in[1][0];
        out[1][0] = in[0][1];
        out[1][1] = in[1][1];
    }

    /**
     * @brief 强制协方差矩阵P对称。
     * @details 通过取对称元素的平均值来强制对称，提高数值稳定性。
     */
    void force_symmetry()
    {
        P[1][0] = P[0][1] = 0.5f * (P[0][1] + P[1][0]);
    }

    /**
     * @brief 对协方差矩阵的对角线元素施加限制。
     * @details 防止协方差过大或过小，维持滤波器的响应性和稳定性。
     */
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
