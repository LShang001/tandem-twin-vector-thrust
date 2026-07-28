/**
 * TVC控制量计算函数(3阶多项式回归版本)
 * 根据两个摆角输入，计算对应的两个控制量输出
 *
 * @param s1_angle S1摆角输入(度)
 * @param s2_angle S2摆角输入(度)
 * @param s1_control 输出参数: S1控制量
 * @param s2_control 输出参数: S2控制量
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