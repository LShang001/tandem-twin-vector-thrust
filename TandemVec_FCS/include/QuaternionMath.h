/**
 * @file QuaternionMath.h
 * @brief 四元数和三维向量数学运算库
 * @author LShang
 * @date 2025/05/31
 *
 * 本文件包含了用于VTVL火箭飞控系统的四元数和向量运算函数。
 * 主要用于姿态控制、坐标变换和旋转计算。
 *
 * 坐标系约定：
 * - 机体坐标系 (B系, FRD): +X前方, +Y右方, +Z下方
 * - 导航坐标系 (N系, NED): +X北方, +Y东方, +Z下方
 * - 四元数表示从源坐标系到目标坐标系的旋转
 */

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

// #include <Arduino.h> // 已在主文件顶部包含
#include <math.h> // 已在主文件顶部包含

// ========================================================================
// 常量定义 (QuaternionMath.h)
// ========================================================================

// DEG_TO_RAD 和 RAD_TO_DEG 已在 Arduino.h (或其依赖) 中通过 PI 定义
// const double DEG_TO_RAD = PI / 180.0f;
// const double RAD_TO_DEG = 180.0f / PI;

// ========================================================================
// 数据结构定义 (QuaternionMath.h)
// ========================================================================

/**
 * @brief 四元数结构体
 *
 * 四元数用于表示三维空间中的旋转，避免万向节死锁问题。
 * 四元数 q = w + xi + yj + zk，其中：
 * - w: 标量部分（实部）
 * - x, y, z: 向量部分（虚部）
 *
 * 单位四元数表示旋转：
 * - |q| = sqrt(w² + x² + y² + z²) = 1
 * - 旋转角度 θ = 2 * arccos(w)
 * - 旋转轴 = (x, y, z) / sin(θ/2)
 */
struct Quaternion
{
  double w, x, y, z; ///< 四元数分量：w(实部), x, y, z(虚部)

  /**
   * @brief 构造函数
   * @param w_val 实部，默认为1.0（单位四元数，无旋转）
   * @param x_val X分量，默认为0.0
   * @param y_val Y分量，默认为0.0
   * @param z_val Z分量，默认为0.0
   */
  Quaternion(double w_val = 1.0f, double x_val = 0.0f, double y_val = 0.0f, double z_val = 0.0f)
      : w(w_val), x(x_val), y(y_val), z(z_val) {}
};

/**
 * @brief 三维向量结构体
 *
 * 用于表示三维空间中的位置、速度、加速度、角速度等物理量。
 * 坐标系的定义取决于具体应用场景。
 */
#ifndef VECTOR3_TYPE_GUARD
#define VECTOR3_TYPE_GUARD
struct Vector3
{
  double x, y, z; ///< 向量分量：x, y, z

  /**
   * @brief 构造函数
   * @param x_val X分量，默认为0.0
   * @param y_val Y分量，默认为0.0
   * @param z_val Z分量，默认为0.0
   */
  Vector3(double x_val = 0.0f, double y_val = 0.0f, double z_val = 0.0f)
      : x(x_val), y(y_val), z(z_val) {}
};
#endif // VECTOR3_TYPE_GUARD

// ========================================================================
// 四元数基本运算函数 (QuaternionMath.h)
// ========================================================================

/**
 * @brief 计算四元数的共轭
 *
 * 四元数共轭定义为：q* = w - xi - yj - zk
 * 几何意义：表示相反方向的旋转
 * 对于单位四元数，共轭等于其逆。
 *
 * @param q 输入四元数
 * @return 四元数的共轭
 */
inline Quaternion quaternionConjugate(const Quaternion &q)
{
  return {q.w, -q.x, -q.y, -q.z};
}

/**
 * @brief 四元数乘法运算 (Hamilton乘积)
 *
 * 几何意义：复合旋转，结果表示先应用q2旋转，再应用q1旋转。
 * 注意：四元数乘法不满足交换律 (q1*q2 != q2*q1)。
 *
 * @param q1 第一个四元数（左乘）
 * @param q2 第二个四元数（右乘）
 * @return 四元数乘积 q1 * q2
 */
inline Quaternion quaternionMultiply(const Quaternion &q1, const Quaternion &q2)
{
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}

/**
 * @brief 四元数归一化
 * 确保四元数模长为1，用于数值稳定。
 * @param q 输入四元数
 * @return 归一化后的四元数
 */
inline Quaternion normalizeQuaternion(const Quaternion &q)
{
  double mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (mag_sq < 1e-9f)
  {                                  // 模长过小，避免除零
    return {1.0f, 0.0f, 0.0f, 0.0f}; // 返回单位四元数
  }
  double mag_inv = 1.0f / sqrtf(mag_sq);
  return {q.w * mag_inv, q.x * mag_inv, q.y * mag_inv, q.z * mag_inv};
}

// ========================================================================
// 向量基本运算函数 (QuaternionMath.h)
// ========================================================================

/**
 * @brief 向量归一化
 *
 * 将向量转换为同方向的单位向量（模长为1）。
 * 若输入向量为零向量或模长极小，则返回一个默认方向的单位向量
 * (机体Z轴向下：{0,0,1})，以避免除零错误。
 *
 * @param v 输入向量
 * @return 归一化后的单位向量
 */
inline Vector3 normalizeVector(const Vector3 &v)
{
  double mag_sq = v.x * v.x + v.y * v.y + v.z * v.z;
  if (mag_sq < 1e-9f)
  { // 模长过小，避免除零
    // 对于推力方向，如果计算出的推力为0，则返回一个中性方向，例如向上 (0,0,-1 in NED) 或 火箭的-Z_B
    // 这里根据原注释，返回{0,0,1}代表机体Z轴正向（向下），可能需要根据上下文调整
    // 对于一般的向量归一化，如果源自叉乘等可能产生零向量的操作，需要小心处理
    return {0.0f, 0.0f, 1.0f}; // 默认指向机体Z轴正方向（向下）
  }
  double mag = sqrtf(mag_sq);
  return {v.x / mag, v.y / mag, v.z / mag};
}

/**
 * @brief 向量叉积运算 (a x b)
 *
 * 计算两个三维向量的叉积。结果向量垂直于输入的两个向量，
 * 方向遵循右手定则。
 *
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 叉积结果向量 a × b
 */
inline Vector3 crossProduct(const Vector3 &a, const Vector3 &b)
{
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
}

/**
 * @brief 向量点积运算 (a · b)
 *
 * 计算两个三维向量的点积（标量积）。
 * 可用于计算向量间的夹角或投影。
 *
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 点积结果（标量）
 */
inline double dotProduct(const Vector3 &a, const Vector3 &b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// ========================================================================
// 高级几何运算函数 (QuaternionMath.h)
// ========================================================================

/**
 * @brief 使用四元数旋转向量
 *
 * 将坐标系F1中的向量v_F1，通过旋转四元数q_F2_from_F1（表示从F1到F2的旋转），
 * 转换为在坐标系F2中表示的向量。
 * 旋转公式：v_F2 = q_F2_from_F1 * v_F1_pure_quat * conjugate(q_F2_from_F1)
 *
 * @param q_F2_from_F1 从坐标系F1到F2的旋转四元数
 * @param v_F1 在坐标系F1中表示的向量
 * @return 在坐标系F2中表示的向量
 */
inline Vector3 rotateVector(const Quaternion &q_F2_from_F1, const Vector3 &v_F1)
{
  // 将向量v_F1表示为纯四元数 (w=0)
  Quaternion v_F1_pure_quat = {0.0f, v_F1.x, v_F1.y, v_F1.z};

  // 计算旋转后的四元数: q * v * q_conjugate
  Quaternion temp_quat = quaternionMultiply(q_F2_from_F1, v_F1_pure_quat);
  Quaternion rotated_pure_quat = quaternionMultiply(temp_quat, quaternionConjugate(q_F2_from_F1));

  // 提取旋转后四元数的向量部分
  return {rotated_pure_quat.x, rotated_pure_quat.y, rotated_pure_quat.z};
}

/**
 * @brief 计算将一个单位向量旋转到另一个单位向量的最小旋转四元数
 *
 * 给定两个单位向量v_from_unit和v_to_unit，此函数计算一个四元数q_align，
 * 使得 rotateVector(q_align, v_from_unit) 约等于 v_to_unit。
 * 该旋转是沿最短路径（最小角度）的旋转。
 *
 * @param v_from_unit 起始单位向量 (必须归一化)
 * @param v_to_unit 目标单位向量 (必须归一化)
 * @return 最小旋转四元数 (已归一化)
 */
inline Quaternion computeAlignQuaternion(const Vector3 &v_from_unit, const Vector3 &v_to_unit)
{
  Quaternion q_align;

  // 计算旋转轴：叉积 v_from x v_to
  Vector3 rotation_axis = crossProduct(v_from_unit, v_to_unit);

  // 计算两向量夹角的余弦值：点积 v_from · v_to
  double cos_angle = dotProduct(v_from_unit, v_to_unit);

  // 情况1：向量已几乎对齐 (夹角接近0度)
  if (cos_angle > 0.999999f)
  {
    return {1.0f, 0.0f, 0.0f, 0.0f}; // 单位四元数，无旋转
  }
  // 情况2：向量几乎完全相反 (夹角接近180度)
  else if (cos_angle < -0.999999f)
  {
    // 需找一个与v_from_unit垂直的轴进行180度旋转
    // 尝试一个通用方法构造垂直轴：如果v_from不是(1,0,0)或(-1,0,0)，则与X轴叉乘；否则与Y轴叉乘。
    Vector3 perp_axis;
    if (fabsf(v_from_unit.x) < 0.9f)
    { // 如果X分量不是主导（避免与X轴平行/反平行）
      perp_axis = crossProduct(v_from_unit, {1.0f, 0.0f, 0.0f});
    }
    else
    { // 如果X分量主导，则与Y轴叉乘
      perp_axis = crossProduct(v_from_unit, {0.0f, 1.0f, 0.0f});
    }
    // 检查叉乘结果是否为零向量 (理论上如果v_from_unit非零，这里不会为零，除非选择的轴平行)
    // 如果上面逻辑选的轴平行导致零向量，则用备用
    if (dotProduct(perp_axis, perp_axis) < 1e-6f)
    {
      // 尝试原始代码中的方法
      perp_axis = {v_from_unit.y, -v_from_unit.x, 0.0f};
      if (dotProduct(perp_axis, perp_axis) < 1e-6f)
      {
        perp_axis = {0.0f, v_from_unit.z, -v_from_unit.y};
        if (dotProduct(perp_axis, perp_axis) < 1e-6f)
        {
          perp_axis = {1.0f, 0.0f, 0.0f};
        }
      }
    }

    perp_axis = normalizeVector(perp_axis); // 归一化垂直轴
    // 180度旋转四元数: w=0, (x,y,z) = unit_rotation_axis
    q_align = {0.0f, perp_axis.x, perp_axis.y, perp_axis.z};
    // 对于180度旋转，四元数本身已经是单位的，因为 w=0, x^2+y^2+z^2 = 1 (perp_axis是单位向量)
    return q_align;
  }

  // 情况3：一般情况 (0 < 夹角 < 180度)
  // 使用半角公式构造四元数:
  // q.w = cos(angle/2) = sqrt((1 + cos_angle) / 2)
  // q.vector_part = sin(angle/2) * unit_rotation_axis
  // sin(angle/2) = sqrt((1 - cos_angle) / 2)
  // rotation_axis的模长是 |v_from|*|v_to|*sin(angle) = sin(angle) (因为是单位向量)
  // sin(angle) = 2 * sin(angle/2) * cos(angle/2)
  // unit_rotation_axis * sin(angle/2) = (rotation_axis / sin(angle)) * sin(angle/2)
  //                                   = rotation_axis / (2 * cos(angle/2))

  q_align.w = sqrtf((1.0f + cos_angle) * 0.5f);
  // rotation_axis 已经包含了 sin(angle) 的因子。我们需要 sin(angle/2) * unit_rotation_axis
  // unit_rotation_axis = rotation_axis / sqrt(1 - cos_angle^2) IF sin(angle) != 0
  // vector_part = rotation_axis * (sin(angle/2) / sin(angle))
  //             = rotation_axis * (sin(angle/2) / (2 * sin(angle/2) * cos(angle/2)))
  //             = rotation_axis / (2 * cos(angle/2))
  //             = rotation_axis / (2 * q_align.w)
  // 这要求 q_align.w (即 cos(angle/2)) 不为零。在180度旋转时为零，已单独处理。

  double sin_half_angle_val_times_2_qw = sqrtf((1.0f - cos_angle) * 0.5f); // This is sin(angle/2)
  // So, vector_part = unit_rotation_axis * sin_half_angle_val
  Vector3 unit_rotation_axis = normalizeVector(rotation_axis); // 归一化旋转轴 (叉积结果)

  q_align.x = unit_rotation_axis.x * sin_half_angle_val_times_2_qw;
  q_align.y = unit_rotation_axis.y * sin_half_angle_val_times_2_qw;
  q_align.z = unit_rotation_axis.z * sin_half_angle_val_times_2_qw;

  // 最终归一化四元数，确保数值精度 (理论上已是单位，但浮点运算可能引入微小误差)
  // double q_mag_sq = q_align.w * q_align.w + q_align.x * q_align.x + q_align.y * q_align.y + q_align.z * q_align.z;
  // if (q_mag_sq > 1e-9f)
  // {
  //   double q_mag_inv = 1.0f / sqrtf(q_mag_sq);
  //   q_align.w *= q_mag_inv;
  //   q_align.x *= q_mag_inv;
  //   q_align.y *= q_mag_inv;
  //   q_align.z *= q_mag_inv;
  // }
  // else
  // { // 理论上不应发生，除非v_from和v_to已对齐 (已在开头处理)
  //   return {1.0f, 0.0f, 0.0f, 0.0f};
  // }
  return normalizeQuaternion(q_align); // 使用独立的归一化函数
}

/**
 * @brief 将欧拉角 (Roll, Pitch, Yaw) 转换为四元数
 *
 * 此函数根据输入的滚转(Roll, φ), 俯仰(Pitch, θ), 偏航(Yaw, ψ)角，
 * 计算出对应的姿态四元数。该四元数表示从导航坐标系(N系)到机体坐标系(B系)的旋转。
 *
 * 旋转顺序为 Z-Y'-X'' (偏航 -> 俯仰 -> 滚转)，这是航空航天领域最常用的约定。
 * 1. 绕N系的Z轴旋转ψ (Yaw)
 * 2. 绕中间坐标系的Y轴旋转θ (Pitch)
 * 3. 绕最终机体坐标系的X轴旋转φ (Roll)
 *
 * @param roll_rad  滚转角 (φ)，单位：弧度
 * @param pitch_rad 俯仰角 (θ)，单位：弧度
 * @param yaw_rad   偏航角 (ψ)，单位：弧度
 * @return Quaternion 表示该姿态的四元数 (从N系到B系的旋转)
 */
inline Quaternion eulerToQuaternion(double roll_rad, double pitch_rad, double yaw_rad)
{
  // 方法注释：
  // 该函数实现了从欧拉角到四元数的标准转换。
  // 欧拉角描述了一系列围绕特定轴的旋转，而四元数则提供了一种更稳健的姿态表示方法，
  // 可以避免万向节死锁问题。转换公式基于将三次单独的轴旋转（偏航、俯仰、滚转）
  // 合成为一个等效的单一旋转。

  // 行间注释：
  // 计算每个角度一半的正弦值和余弦值。
  // 这是因为四元数的构造公式直接使用了半角三角函数。
  // 'c' 前缀代表 cos, 's' 前缀代表 sin。
  double c_roll_half = cosf(roll_rad * 0.5f);   // cos(φ/2)
  double s_roll_half = sinf(roll_rad * 0.5f);   // sin(φ/2)
  double c_pitch_half = cosf(pitch_rad * 0.5f); // cos(θ/2)
  double s_pitch_half = sinf(pitch_rad * 0.5f); // sin(θ/2)
  double c_yaw_half = cosf(yaw_rad * 0.5f);     // cos(ψ/2)
  double s_yaw_half = sinf(yaw_rad * 0.5f);     // sin(ψ/2)

  // 行间注释：
  // 应用 Z-Y'-X'' 旋转顺序的欧拉角到四元数转换公式。
  // 这个公式是通过将代表三次旋转的三个四元数相乘推导出来的：
  // q = q_yaw * q_pitch * q_roll
  Quaternion result;
  result.w = c_roll_half * c_pitch_half * c_yaw_half + s_roll_half * s_pitch_half * s_yaw_half;
  result.x = s_roll_half * c_pitch_half * c_yaw_half - c_roll_half * s_pitch_half * s_yaw_half;
  result.y = c_roll_half * s_pitch_half * c_yaw_half + s_roll_half * c_pitch_half * s_yaw_half;
  result.z = c_roll_half * c_pitch_half * s_yaw_half - s_roll_half * s_pitch_half * c_yaw_half;

  // 方法注释：
  // 虽然理论上从精确的欧拉角转换过来的四元数应该是单位四元数，
  // 但由于浮点运算的累积误差，最好还是进行一次归一化，以确保其模长严格为1。
  // 这对于后续的四元数运算（如旋转向量）的数值稳定性至关重要。
  return normalizeQuaternion(result);
}

/**
 * @brief 将四元数转换为欧拉角 (Roll, Pitch, Yaw)
 *
 * 此函数根据输入的姿态四元数（表示从导航坐标系N系到机体坐标系B系的旋转），
 * 计算出对应的滚转(Roll, φ), 俯仰(Pitch, θ), 偏航(Yaw, ψ)角。
 *
 * 转换遵循 Z-Y'-X'' (偏航 -> 俯仰 -> 滚转) 旋转顺序，与 eulerToQuaternion 函数一致。
 * 欧拉角的范围：
 * - Roll (φ): [-π, π]
 * - Pitch (θ): [-π/2, π/2]
 * - Yaw (ψ): [-π, π]
 *
 * 注意：当俯仰角接近 ±90度时，会出现万向节死锁 (Gimbal Lock) 问题。
 * 在这种情况下，偏航角和滚转角会耦合，本函数会通过将偏航角设为0来处理，
 * 并将所有剩余的水平旋转归因于滚转角。
 *
 * @param q_N_to_B 输入的姿态四元数 (从N系到B系的旋转)
 * @param[out] roll_rad 计算出的滚转角 (φ)，单位：弧度
 * @param[out] pitch_rad 计算出的俯仰角 (θ)，单位：弧度
 * @param[out] yaw_rad 计算出的偏航角 (ψ)，单位：弧度
 */
inline void quaternionToEuler(const Quaternion &q_N_to_B, float &roll_rad, float &pitch_rad, float &yaw_rad)
{
  // 方法注释：
  // 该函数实现了从四元数到欧拉角的标准转换。
  // 它将四元数表示的旋转分解为 Z-Y'-X'' 顺序的三个独立旋转角：偏航、俯仰和滚转。
  // 转换公式基于四元数到旋转矩阵的转换，并从中提取欧拉角。
  // 特别注意处理万向节死锁情况，以确保数值稳定性。

  // 提取四元数分量
  double w = q_N_to_B.w;
  double x = q_N_to_B.x;
  double y = q_N_to_B.y;
  double z = q_N_to_B.z;

  // 计算俯仰角 (Pitch, θ)
  // sin(pitch) = 2 * (w*y - x*z)
  // 俯仰角的范围是 [-π/2, π/2]，使用 asin 即可。
  // 但为了避免数值误差导致参数超出 [-1, 1] 范围，需要进行钳位。
  double sin_pitch = 2.0 * (w * y - x * z);
  sin_pitch = fmax(-1.0, fmin(1.0, sin_pitch)); // 钳位到 [-1, 1] 范围
  pitch_rad = asinf(sin_pitch);

  // 检查万向节死锁 (Gimbal Lock)
  // 当俯仰角接近 ±90度时，cos(pitch) 接近 0，此时偏航和滚转会耦合。
  // 我们使用一个小的阈值来检测这种情况。
  const double GIMBAL_LOCK_THRESHOLD = 0.99999; // 接近 1.0
  if (fabs(sin_pitch) >= GIMBAL_LOCK_THRESHOLD)
  {
    // 发生万向节死锁，偏航角和滚转角耦合。
    // 在这种情况下，通常将偏航角设为0，并将所有剩余的旋转归因于滚转角。
    // Roll (φ) = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    roll_rad = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (y * y + z * z));
    yaw_rad = 0.0; // 偏航角设为0
  }
  else
  {
    // 未发生万向节死锁，正常计算滚转角和偏航角。

    // 计算滚转角 (Roll, φ)
    // tan(roll) = (2*(w*x + y*z)) / (1 - 2*(x*x + y*y))
    roll_rad = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

    // 计算偏航角 (Yaw, ψ)
    // tan(yaw) = (2*(w*z + x*y)) / (1 - 2*(y*y + z*z))
    yaw_rad = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }
}

/**
 * @brief 根据期望的单位推力矢量在NED系下的分量，构造一个纯倾斜的目标姿态四元数。
 *
 * 这个函数计算一个从水平姿态到目标姿态的最短路径旋转。
 * 输出的四元数 q_tilt_target 代表了一个姿态，其特点是：
 * 1. 它的机体-Z轴与期望的推力方向完全对齐。
 * 2. 它不包含任何绕机体-Z轴的多余旋转（即从水平姿态倾斜过来的“最经济”的动作）。
 *
 * @param thrust_comp_N 期望的单位推力矢量在NED系的北向分量。
 * @param thrust_comp_E 期望的单位推力矢量在NED系的东向分量。
 * @param[out] q_tilt_target 计算出的纯倾斜目标姿态四元数。
 * @return bool 如果输入有效且计算成功，返回true；否则返回false。
 */
inline bool constructTiltTargetQuaternion(double thrust_comp_N, double thrust_comp_E, Quaternion &q_tilt_target)
{
  // ========================================================================
  // 步骤 1: 输入验证与目标推力矢量构造
  // ========================================================================

  // 检查输入的水平分量是否构成了有效的推力请求。
  // 水平分量的模长平方不能大于1，否则无法构成单位矢量。
  double horizontal_mag_sq = thrust_comp_N * thrust_comp_N + thrust_comp_E * thrust_comp_E;
  if (horizontal_mag_sq > 1.0f)
  {
    // 输入无效，无法构造单位推力矢量。
    // 可以选择返回false，或者将水平分量归一化以处理超限指令。
    // 这里我们选择归一化，以增强鲁棒性。
    double horizontal_mag = sqrtf(horizontal_mag_sq);
    thrust_comp_N /= horizontal_mag;
    thrust_comp_E /= horizontal_mag;
    horizontal_mag_sq = 1.0f;
  }

  // 计算地向分量 d_D。
  // 推力是向上的，在NED坐标系中是-Z方向，所以d_D总是负值或零。
  double d_D = -sqrtf(1.0f - horizontal_mag_sq);

  // ========================================================================
  // 步骤 2: 构造原始四元数 q_raw
  // q_raw = (1 + dot_product, cross_product)
  // dot_product = (0,0,-1) . (d_N, d_E, d_D) = -d_D
  // cross_product = (0,0,-1) x (d_N, d_E, d_D) = (d_E, -d_N, 0)
  // ========================================================================

  Quaternion q_raw;
  q_raw.w = 1.0f - d_D; // w分量，总是非负
  q_raw.x = thrust_comp_E;
  q_raw.y = -thrust_comp_N;
  q_raw.z = 0.0f; // z分量恒为0，这保证了旋转轴在XY平面上，即无绕Z轴旋转

  // ========================================================================
  // 步骤 3: 归一化与特殊情况处理
  // ========================================================================

  // 计算q_raw的模长平方。
  // mag_sq = (1-d_D)^2 + d_E^2 + d_N^2
  //        = 1 - 2*d_D + d_D^2 + horizontal_mag_sq
  //        = 1 - 2*d_D + d_D^2 + (1 - d_D^2)
  //        = 2 - 2*d_D = 2 * (1 - d_D)
  // 这个结果与q_raw.w直接相关，可以简化计算。
  double mag_sq = 2.0f * q_raw.w;

  // 检查特殊情况：如果目标推力方向与参考方向(0,0,-1)非常接近。
  // 此时 d_D 接近 -1, q_raw.w 接近 2, mag_sq 接近 4。
  // 如果目标推力方向与参考方向完全相反(0,0,1)（即垂直向下），
  // 此时 d_D = 1, q_raw.w = 0, mag_sq = 0。归一化会失败。
  // 这种情况在无人机中几乎不可能发生，但为了代码的数学完备性，我们处理它。
  if (mag_sq < 1e-9f)
  {
    // 目标是垂直向下推，这是一个180度的旋转。
    // 旋转轴不唯一，我们可以任选一个水平轴，例如绕世界Y轴(East)旋转180度。
    q_tilt_target = {0.0f, 0.0f, 1.0f, 0.0f}; // q = (cos(180/2), sin(180/2)*axis)
    return true;
  }

  // 归一化q_raw得到最终的纯倾斜目标四元数。
  double inv_mag = 1.0f / sqrtf(mag_sq);
  q_tilt_target.w = q_raw.w * inv_mag;
  q_tilt_target.x = q_raw.x * inv_mag;
  q_tilt_target.y = q_raw.y * inv_mag;
  q_tilt_target.z = q_raw.z * inv_mag; // 结果恒为0

  return true;
}

#endif // QUATERNION_MATH_H