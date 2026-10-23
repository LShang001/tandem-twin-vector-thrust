/**
 * @file    TVC_Control_3rdOrder_Poly.h
 * @module  固件/控制（历史 TVC 舵机标定模型，已弃用）
 * @brief   用 3 阶多项式回归把两个摆角映射为两路舵机控制量（0–100）
 * @details 输入：S1/S2 目标摆角 [deg]；处理：9 项多项式特征（含交叉项）
 *          分别对 S1/S2 控制量做线性组合；输出：两路控制量（无量纲 0–100）。
 *          ⚠️ 本模型已弃用：现行控制分配走 TandemVec_ControlAllocation.h，
 *          舵机映射走 flight_control.cpp 的层3（ServoConfig 齿轮比 + 标定）。
 *          保留文件仅为对照历史标定数据，**不得接入控制链**。
 * @theory  台架/试飞数据拟合的经验多项式（无物理推导）
 * @ref     CTL-001 §2（本构型不用该模型；分配链理论见 THY-003 §3）
 * @frame   N/A（摆角→控制量的标量映射，坐标系已在标定数据中固化）
 * @units   输入 deg；输出控制量 0–100（≈PWM 百分比）
 * @note    ★ 9 个系数来自旧共轴构型的回归，系数含义与当前纵列布局无关；
 *          重新标定请改 ServoConfig（half_travel_deg / zero_*_pct / dir_*），
 *          不要复活本多项式。
 */
inline void calculateTVCControl(float s1_angle, float s2_angle, float &s1_control, float &s2_control)
{
    // S1控制量模型参数(3阶多项式回归结果)
    const float s1_intercept = 36.825818; // 截距项
    const float s1_coeff[9] = {
        // 9个系数对应3阶多项式特征
        5.07938776,      // s1
        -0.0889794419,   // s2
        -0.0233712646,  // s1^2
        -0.00507479873,  // s1*s2
        -0.0226695026,  // s2^2
        -0.000575150424, // s1^3
        0.000469695869,  // s1^2*s2
        -0.000133166588, // s1*s2^2
        -0.000707062210  // s2^3
    };

    // S2控制量模型参数(3阶多项式回归结果)
    const float s2_intercept = 57.086849; // 截距项
    const float s2_coeff[9] = {
        // 9个系数对应3阶多项式特征
        0.0431704266,    // s1
        4.94859777,      // s2
        -0.0242492579,   // s1^2
        0.00813404648,   // s1*s2 
        -0.0287087867,   // s2^2
        0.000631635730,  // s1^3
        -0.000202820188, // s1^2*s2
        -0.000381995524, // s1*s2^2
        -0.000262486810  // s2^3
    };

    // 计算多项式特征项(3阶)
    float features[9] = {
        s1_angle,                       // s1
        s2_angle,                       // s2
        s1_angle * s1_angle,            // s1^2
        s1_angle * s2_angle,            // s1*s2
        s2_angle * s2_angle,            // s2^2
        s1_angle * s1_angle * s1_angle, // s1^3
        s1_angle * s1_angle * s2_angle, // s1^2*s2
        s1_angle * s2_angle * s2_angle, // s1*s2^2
        s2_angle * s2_angle * s2_angle  // s2^3
    };

    // 计算S1控制量(多项式回归)
    s1_control = s1_intercept;
    for (int i = 0; i < 9; i++)
    {
        s1_control += s1_coeff[i] * features[i];
    }

    // 计算S2控制量(多项式回归)
    s2_control = s2_intercept;
    for (int i = 0; i < 9; i++)
    {
        s2_control += s2_coeff[i] * features[i];
    }
}