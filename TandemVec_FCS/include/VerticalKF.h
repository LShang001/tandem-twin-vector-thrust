#pragma once

#include <Arduino.h>
#include <math.h>   // 引入数学库以使用 isnan(), fmaxf()
#include <string.h> // 引入字符串库以使用 memcpy()

/**
 * @class VerticalKF
 * @brief 鲁棒型垂直状态估计卡尔曼滤波器 (高度、速度、加速度计零偏)
 *
 * @details
 * 本类是标准三状态垂直卡尔曼滤波器的增强版本，专为在复杂和不可靠的现实环境中
 * 长期稳定运行而设计。它不仅融合了高频IMU和低频高度测量，还集成了多项
 * 关键的鲁棒性与稳定性增强技术：
 *
 * 1.  **异常值抑制 (Outlier Rejection):**
 *     -   **测量异常值抑制:** 通过3-sigma门限检验，自动拒绝由传感器故障或外部干扰
 *         导致的无效高度测量值，防止状态估计被污染。
 *     -   **输入异常值抑制:** 对输入的加速度值进行饱和限制，防止因IMU数据尖峰
 *         导致的速度和位置估计剧烈跳变。
 *
 * 2.  **Joseph形式协方差更新:**
 *     -   采用数值上更稳定的Joseph形式更新协方差矩阵，从根本上保证了协方差矩阵的
 *         对称性和正定性，有效防止了因浮点累积误差导致的滤波器发散。
 *
 * 3.  **协方差限制 (Covariance Limiting):**
 *     -   对协方差矩阵的对角线元素设置上、下限。防止滤波器因过度依赖精确测量而
 *         变得“傲慢”（协方差过小），或因长期无有效测量而“遗忘”（协方差过大）。
 *         确保滤波器始终保持合理的学习能力和响应速度。
 *
 * 4.  **安全的增益计算:**
 *     -   在计算卡尔曼增益时，对除数进行保护，防止出现除零错误，增强了算法的健壮性。
 *
 * @section model 核心模型
 *
 * 状态向量 `x` 定义为:
 * $x = [h, v, b_a]^T$ (高度, 速度, 加速度计零偏)
 *
 * 状态预测模型:
 * $h_k = h_{k-1} + v_{k-1} \cdot dt + 0.5 \cdot (a_{meas} - b_a) \cdot dt^2$
 * $v_k = v_{k-1} + (a_{meas} - b_a) \cdot dt$
 * $b_{a,k} = b_{a,k-1}$
 *
 * 观测模型:
 * $z_h = h_k + \text{noise}$
 *
 * 这个经过强化的版本，是部署于实际无人机或机器人产品的理想选择。
 */
class VerticalKF
{
public:
    /**
     * @brief 构造函数。
     * @details 初始化状态、协方差和默认的鲁棒性参数。
     */
    VerticalKF()
    {
        // 初始化状态向量 x
        x[0] = 0.0f; // 高度 (m)
        x[1] = 0.0f; // 速度 (m/s)
        x[2] = 0.0f; // 加速度计零偏 (m/s^2)

        // 初始化状态协方差矩阵 P
        P[0][0] = 1.0f;
        P[0][1] = 0.0f;
        P[0][2] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 1.0f;
        P[1][2] = 0.0f;
        P[2][0] = 0.0f;
        P[2][1] = 0.0f;
        P[2][2] = 1.0f;

        // 初始化噪声参数
        sigma_a = 0.0f;
        sigma_b = 0.0f;

        // 初始化鲁棒性增强参数的默认值
        measurement_outlier_gate_sq_ = 9.0f; // 默认3-sigma门限 (3*3=9)
        max_input_accel_ = 10.0f;            // 默认最大输入加速度 ±10 m/s^2 (约 ±1g)
        S_min_ = 1e-9f;                      // 增益计算中S的最小值，防除零

        // 默认协方差限制
        P_min_[0] = 1e-6f;
        P_max_[0] = 100.0f; // 高度方差限制
        P_min_[1] = 1e-4f;
        P_max_[1] = 25.0f; // 速度方差限制
        P_min_[2] = 1e-8f;
        P_max_[2] = 1.0f; // 零偏方差限制
    }

    /**
     * @brief 配置并初始化卡尔曼滤波器。
     * @details 在系统启动后，传感器数据稳定时调用。
     *
     * @param initial_height 系统的初始高度 (米)。
     * @param process_noise_accel 过程噪声中加速度项的标准差 (m/s^2)。
     * @param process_noise_bias 加速度计零偏的噪声谱密度 (m/s^2/sqrt(Hz))。
     */
    void begin(float initial_height, float process_noise_accel, float process_noise_bias)
    {
        x[0] = initial_height;
        sigma_a = process_noise_accel;
        sigma_b = process_noise_bias;
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
     * @param p_min_b 零偏方差下限。
     * @param p_max_b 零偏方差上限。
     */
    void configureRobustness(float outlier_gate, float max_accel,
                             float p_min_h, float p_max_h,
                             float p_min_v, float p_max_v,
                             float p_min_b, float p_max_b)
    {
        measurement_outlier_gate_sq_ = outlier_gate * outlier_gate;
        max_input_accel_ = max_accel;
        P_min_[0] = p_min_h;
        P_max_[0] = p_max_h;
        P_min_[1] = p_min_v;
        P_max_[1] = p_max_v;
        P_min_[2] = p_min_b;
        P_max_[2] = p_max_b;
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
        float acc_corrected = accel_z - x[2];
        x[0] += x[1] * dt + 0.5f * acc_corrected * dt * dt;
        x[1] += acc_corrected * dt;
        // x[2] (零偏) 保持不变

        // --- 2. 协方差预测: P_k = F * P_{k-1} * F' + Q ---
        float dt2 = dt * dt;
        float F[3][3] = {
            {1.0f, dt, -0.5f * dt2},
            {0.0f, 1.0f, -dt},
            {0.0f, 0.0f, 1.0f}};

        float FP[3][3];
        float FPFt[3][3];
        float Ft[3][3];
        matrix_mult_3x3(F, P, FP);
        matrix_transpose_3x3(F, Ft);
        matrix_mult_3x3(FP, Ft, FPFt);

        memcpy(P, FPFt, sizeof(P));

        // 累加过程噪声协方差 Q
        float sigma_a2 = sigma_a * sigma_a;
        float sigma_b2 = sigma_b * sigma_b;
        float dt3 = dt * dt2;
        float dt4 = dt2 * dt2;

        P[0][0] += 0.25f * dt4 * sigma_a2;
        P[0][1] += 0.5f * dt3 * sigma_a2;
        P[1][0] += 0.5f * dt3 * sigma_a2;
        P[1][1] += dt2 * sigma_a2;
        P[2][2] += dt * sigma_b2;

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

        // 观测矩阵 H = [1, 0, 0]

        // --- 1. 计算残差 y 和残差协方差 S ---
        float y = measurement_height - x[0];
        float R = measurement_noise_std * measurement_noise_std;
        float S = P[0][0] + R;

        // --- 2. 测量值异常抑制 ---
        // 使用N-sigma门限检查残差是否过大。
        // 如果 y^2 > gate^2 * S，则认为该测量是异常值，跳过本次更新。
        if (y * y > measurement_outlier_gate_sq_ * S)
        {
            return; // 拒绝异常值
        }

        // --- 3. 改进的卡尔曼增益计算 ---
        // 增加对S的下限保护，防止除零。
        S = fmaxf(S, S_min_);
        float S_inv = 1.0f / S;
        float K[3];
        K[0] = P[0][0] * S_inv;
        K[1] = P[1][0] * S_inv;
        K[2] = P[2][0] * S_inv;

        // --- 4. 更新状态估计 x_new = x_pred + K * y ---
        x[0] += K[0] * y;
        x[1] += K[1] * y;
        x[2] += K[2] * y;

        // --- 5. 使用Joseph形式更新协方差矩阵 P_new = (I - KH)P(I-KH)' + KRK' ---
        // 这是数值稳定性的关键！
        float P_old[3][3];
        memcpy(P_old, P, sizeof(P_old));

        // 计算 (I - KH)
        float I_m_KH[3][3] = {
            {1.0f - K[0], 0.0f, 0.0f},
            {-K[1], 1.0f, 0.0f},
            {-K[2], 0.0f, 1.0f}};

        // 计算 (I - KH) * P_old
        float IKH_P[3][3];
        matrix_mult_3x3(I_m_KH, P_old, IKH_P);

        // 计算 (I - KH)'
        float I_m_KH_T[3][3];
        matrix_transpose_3x3(I_m_KH, I_m_KH_T);

        // 计算第一项: (I - KH) * P_old * (I - KH)'
        float term1[3][3];
        matrix_mult_3x3(IKH_P, I_m_KH_T, term1);

        // 计算第二项: K * R * K'
        float term2[3][3];
        float K_KT[3][3];
        // K * K' (外积)
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                K_KT[i][j] = K[i] * K[j];
            }
        }
        // R * (K * K')
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                term2[i][j] = R * K_KT[i][j];
            }
        }

        // P_new = term1 + term2
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                P[i][j] = term1[i][j] + term2[i][j];
            }
        }

        // --- 6. 强制对称与协方差限制 ---
        force_symmetry();
        apply_covariance_limits();
    }

    /** @brief 获取当前估计的高度。 @return 滤波后的垂直高度 (米)。 */
    float getHeight() const { return x[0]; }
    /** @brief 获取当前估计的速度。 @return 滤波后的垂直速度 (米/秒)。 */
    float getVelocity() const { return x[1]; }
    /** @brief 获取当前估计的加速度计零偏。 @return 滤波后的加速度计零偏 (m/s^2)。 */
    float getAccelBias() const { return x[2]; }

private:
    // --- 核心变量 ---
    float x[3];    ///< 状态向量: x = [height, velocity, accel_bias]'
    float P[3][3]; ///< 状态协方差矩阵

    // --- 噪声参数 ---
    float sigma_a; ///< 加速度过程噪声标准差
    float sigma_b; ///< 零偏过程噪声谱密度

    // --- 鲁棒性增强参数 ---
    float measurement_outlier_gate_sq_; ///< 测量异常值门限的平方 (N*N)
    float max_input_accel_;             ///< 最大允许输入加速度
    float S_min_;                       ///< 增益计算中S的最小值
    float P_min_[3];                    ///< 协方差对角线元素下限
    float P_max_[3];                    ///< 协方差对角线元素上限

    // --- 私有辅助函数 ---

    /** @brief 3x3矩阵乘法: out = a * b */
    void matrix_mult_3x3(const float a[3][3], const float b[3][3], float out[3][3])
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
    }

    /** @brief 3x3矩阵转置: out = in' */
    void matrix_transpose_3x3(const float in[3][3], float out[3][3])
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                out[j][i] = in[i][j];
            }
        }
    }

    /**
     * @brief 强制协方差矩阵P对称。
     * @details 通过取对称元素的平均值来强制对称，提高数值稳定性。
     */
    void force_symmetry()
    {
        P[1][0] = P[0][1] = 0.5f * (P[0][1] + P[1][0]);
        P[2][0] = P[0][2] = 0.5f * (P[0][2] + P[2][0]);
        P[2][1] = P[1][2] = 0.5f * (P[1][2] + P[2][1]);
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

        if (P[2][2] < P_min_[2])
            P[2][2] = P_min_[2];
        if (P[2][2] > P_max_[2])
            P[2][2] = P_max_[2];
    }
};
