#include "sensor_imu.h"
#include "math_utils.h"
#include "QuaternionMath.h"

namespace
{
  constexpr uint32_t ICM42688_INIT_SAMPLE_TIMEOUT_MS = 20; // 单个初始化样本等待超时，避免IMU异常时卡死setup
  constexpr int ICM42688_INIT_DATA_TIMEOUT_ERROR = -1001;  // 自定义错误码：初始化阶段等待数据超时
}

bool initMagnetometer()
{
  Serial8.println("----------------------------------------");
  Serial8.println("[Init] Starting Magnetometer Setup...");

  // 1. 启动 I2C 总线硬件
  //    在 STM32Duino 中，这一步会配置 GPIO 复用功能
  Wire1.begin();
  Wire1.setClock(400000); // 400kHz Fast Mode
  Serial8.println("[Init] I2C Bus (Wire1) Started at 400kHz");

  // 2. 初始化传感器驱动
  //    传入 &Wire1 指针，指定使用 PB7/PB6 端口
  if (!compass.begin(&Wire1))
  {
    Serial8.println("[Error] IST8310 Not Found!");
    Serial8.println("[Error] Please check wiring (SDA/SCL) and Power.");
    return false;
  }

  Serial8.println("[Init] IST8310 Setup Finished Successfully.");
  Serial8.println("----------------------------------------");

  return true;
}

/**
 * @brief 初始化 ICM42688 六轴惯性测量单元
 *
 * 完整初始化流程：
 * 1. 配置 SPI 总线 (PA5-SCLK, PA6-MISO, PA7-MOSI, PA4-CS, 10MHz, Mode3)
 * 2. 初始化 ICM42688 驱动
 * 3. 配置数字低通滤波器 (陀螺仪100Hz, 加速度计50Hz)
 * 4. 配置量程和采样率 (加速度±8G/2kHz, 陀螺仪±2000dps/2kHz)
 * 5. 采集 100 个样本计算加速度平均值
 * 6. 初始化互补滤波器 (使用原始 RUB 坐标系数据)
 * 7. 坐标系适配 (RUB -> FLU) 后初始化 Madgwick 姿态滤波器
 *
 * @return 0 初始化成功
 * @return 负值 错误码 (-1001: 数据读取超时, 其他: IMU.begin() 错误)
 */
int initICM42688()
{
  // === 步骤 1: 配置 SPI 总线 ===
  // ICM42688 使用 SPI Mode3 (CPOL=1, CPHA=1): 时钟空闲高电平，第二个边沿采样
  SPI.setMISO(SPI_MISO);                                            // PA6: 主入从出
  SPI.setMOSI(SPI_MOSI);                                            // PA7: 主出从入
  SPI.setSCLK(SPI_SCLK);                                            // PA5: 时钟线
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // 10MHz, 高位先发, Mode3

  // === 步骤 2: 初始化 IMU 驱动 ===
  int status = IMU.begin(); // 通过 SPI 初始化 ICM42688，返回 0 表示成功
  if (status < 0)
  {
    return status; // 初始化失败，返回错误码
  }

  // === 步骤 3: 配置数字低通滤波器 (DLPF) ===
  // 陀螺仪: 100Hz 截止频率 (true 表示陀螺仪通道)
  // 目的: 滤除高频机械振动噪声，保留有用的角速度信号
  IMU.setLowPassFilter(ICM42688::FREQ_100HZ, true);
  // 加速度计: 50Hz 截止频率 (false 表示加速度计通道)
  // 加速度计噪声通常比陀螺仪大，使用更激进的滤波
  IMU.setLowPassFilter(ICM42688::FREQ_50HZ, false);

  // === 步骤 4: 配置量程和采样率 ===
  // 加速度计: ±8G 量程, 2kHz 输出数据率 (ODR)
  // ±8G 量程对应灵敏度: 4096 LSB/g，足够覆盖正常飞行加速度范围
  IMU.setAccelFS(ICM42688::AccelFS::gpm8);
  IMU.setAccelODR(ICM42688::ODR::odr2k);

  // 陀螺仪: ±2000°/s 量程, 2kHz 输出数据率 (ODR)
  // ±2000°/s 量程对应灵敏度: 164 LSB/(°/s)，覆盖快速旋转场景
  IMU.setGyroFS(ICM42688::GyroFS::dps2000);
  IMU.setGyroODR(ICM42688::ODR::odr2k);

  delay(500); // 等待 500ms，让传感器稳定并开始输出数据

  // === 步骤 5: 采集初始样本并计算平均值 ===
  // 目的: 为滤波器提供合理的初始值，避免启动时的阶跃响应
  float ax = 0, ay = 0, az = 0; // 加速度累加器 (单位: g)
  for (int i = 0; i < 100; i++) // 采集 100 个样本
  {
    // 带超时的数据等待，防止 SPI 总线异常时永久阻塞
    const uint32_t sample_wait_start_ms = millis();
    while (IMU.getAGT() != 1) // 轮询数据就绪标志 (getAGT 返回 1 表示新数据可用)
    {
      // 超时保护: 单个样本等待超过 20ms 则认为传感器异常
      // 这避免了系统卡死在 setup() 中无法进入故障处理分支
      if ((millis() - sample_wait_start_ms) > ICM42688_INIT_SAMPLE_TIMEOUT_MS)
      {
        Serial8.println("[Error] ICM42688 init data timeout.");
        return ICM42688_INIT_DATA_TIMEOUT_ERROR; // 返回自定义错误码 -1001
      }
    }
    // 读取原始加速度数据 (单位: g，传感器 RUB 坐标系)
    float axRaw = IMU.accX(); // X轴: 传感器板右方向
    float ayRaw = IMU.accY(); // Y轴: 传感器板上方向
    float azRaw = IMU.accZ(); // Z轴: 传感器板后方向
    ax += axRaw;              // 累加 X 轴
    ay += ayRaw;              // 累加 Y 轴
    az += azRaw;              // 累加 Z 轴

    digitalToggle(LED_green); // 翻转绿灯指示采样进度
  }
  ax /= 100.0; // 计算 X 轴加速度平均值
  ay /= 100.0; // 计算 Y 轴加速度平均值
  az /= 100.0; // 计算 Z 轴加速度平均值 (静止时应接近 1g)

  // === 步骤 6: 初始化互补滤波器 ===
  // 使用原始 RUB 坐标系数据初始化，避免滤波器启动时的阶跃响应
  accelXFilter.initialize(ax);        // X轴加速度滤波器初值
  accelYFilter.initialize(ay);        // Y轴加速度滤波器初值
  accelZFilter.initialize(az);        // Z轴加速度滤波器初值
  gyroXFilter.initialize(IMU.gyrX()); // X轴陀螺仪滤波器初值 (单位: dps)
  gyroYFilter.initialize(IMU.gyrY()); // Y轴陀螺仪滤波器初值
  gyroZFilter.initialize(IMU.gyrZ()); // Z轴陀螺仪滤波器初值

  // === 步骤 7: 坐标系适配后初始化 Madgwick 滤波器 ===
  // 传感器物理安装坐标系为 RUB (右-上-后)
  // Madgwick 算法期望的输入坐标系为 FLU (前-左-上)
  // 坐标变换关系:
  //   FLU_X (前) = -RUB_Z (后取反)
  //   FLU_Y (左) = -RUB_X (右取反)
  //   FLU_Z (上) = +RUB_Y (上保持)
  float axForMadgwick = -az; // FLU_X = -RUB_Z
  float ayForMadgwick = -ax; // FLU_Y = -RUB_X
  float azForMadgwick = ay;  // FLU_Z = +RUB_Y

  // 使用经过坐标适配的加速度数据初始化 Madgwick 滤波器
  // 该函数会根据重力方向计算初始姿态四元数
  madgwick.initializeFromAccelerometer(axForMadgwick, ayForMadgwick, azForMadgwick);

  return 0; // 初始化成功
}

bool readIMUData()
{
  // 检查IMU数据是否就绪 (数据准备好中断标志)
  if (IMU.getAGT() == 1)
  {
    // ========================================================================
    // 步骤 1: 计算时间差 (Delta Time)
    // ========================================================================
    static unsigned long lastTimeMicros = 0;    // 静态变量，记录上次调用的时间
    unsigned long currentTimeMicros = micros(); // 获取当前时间（微秒）

    // 如果是首次运行，则使用一个小的默认时间差；否则，计算实际时间差
    double deltaTimeSecs = (lastTimeMicros == 0) ? 0.0001 : (currentTimeMicros - lastTimeMicros) / 1000000.0;
    lastTimeMicros = currentTimeMicros; // 更新上次时间戳

    // 防御性编程：防止因系统计时问题导致时间差为零或负数，这会破坏姿态解算
    if (deltaTimeSecs <= 0)
    {
      deltaTimeSecs = 0.0005; // 分配一个合理的最小时间差
    }

    // ========================================================================
    // 步骤 2: 读取原始数据 (Raw Data Acquisition)
    // ========================================================================
    // 读取 ICM42688 的原始数据
    // 加速度计单位: g (重力加速度倍数)
    float raw_ax_g = IMU.accX();
    float raw_ay_g = IMU.accY();
    float raw_az_g = IMU.accZ();
    // 陀螺仪单位: dps (度/秒)
    float raw_gx_dps = IMU.gyrX();
    float raw_gy_dps = IMU.gyrY();
    float raw_gz_dps = IMU.gyrZ();

    // ========================================================================
    // 步骤 3: 自动校准与修正 (Auto-Calibration & Correction)
    // ========================================================================

    // 转换角速度为 rad/s (校准模块和后续解算都需要 rad/s)
    float gx_rads = raw_gx_dps * DEG_TO_RAD;
    float gy_rads = raw_gy_dps * DEG_TO_RAD;
    float gz_rads = raw_gz_dps * DEG_TO_RAD;

    // 3.2 获取当前解锁状态
    // raw_rc_values[4] 是通道5，>1500 表示解锁。
    // 只有在未解锁（Locked/Disarmed）状态下，才允许进入校准模式。
    bool is_armed = (raw_rc_values[4] > 1500);

    // 3.3 更新校准器状态
    // 该函数会自动检测静止状态，并在满足条件后采集数据、计算零偏。
    // 如果刚刚完成了一次校准，返回 true，我们可以打印调试信息。
    if (imuCalibrator.update(gx_rads, gy_rads, gz_rads, raw_ax_g, raw_ay_g, raw_az_g, is_armed))
    {
      Serial8.println(">> IMU Re-Calibration Completed!");
      imuCalibrator.printDebug(Serial8);
    }

    // 3.4 应用校准参数
    // apply() 函数会修改传入的变量：
    imuCalibrator.apply(gx_rads, gy_rads, gz_rads, raw_ax_g, raw_ay_g, raw_az_g);

    // 3.5 将校准后的数据转换回原流程需要的单位 (g 和 dps)
    float gx_cal_dps = gx_rads * RAD_TO_DEG;
    float gy_cal_dps = gy_rads * RAD_TO_DEG;
    float gz_cal_dps = gz_rads * RAD_TO_DEG;

    // ========================================================================
    // 步骤 4: 低通滤波 (Low-Pass Filtering)
    // ========================================================================
    // 使用校准后的数据进行滤波。
    // 传感器物理安装定义的坐标系为：右(X+), 上(Y+), 后(Z+)
    float axFilteredRaw = accelXFilter.filter(raw_ax_g);  // 滤波加速度X (单位: g)
    float ayFilteredRaw = accelYFilter.filter(raw_ay_g);  // 滤波加速度Y (单位: g)
    float azFilteredRaw = accelZFilter.filter(raw_az_g);  // 滤波加速度Z (单位: g)
    float gxFilteredRaw = gyroXFilter.filter(gx_cal_dps); // 滤波陀螺仪X (单位: dps)
    float gyFilteredRaw = gyroYFilter.filter(gy_cal_dps); // 滤波陀螺仪Y (单位: dps)
    float gzFilteredRaw = gyroZFilter.filter(gz_cal_dps); // 滤波陀螺仪Z (单位: dps)

    // ========================================================================
    // 步骤 5: 第一次坐标变换 (Sensor Frame -> Body Frame)
    // ========================================================================
    // 传感器系 (右-上-后) -> 机体系
    // ★ 2026-08-07 实机轴重映射（构型迁移修正）：
    //   旧 VTVL 项目板子安装 = NED 标准（x 水平前 / z 竖直下）；
    //   纵列双发 VTOL 悬停构型约定 x_b=机头(竖直朝天) / z_b=水平后。
    // ★ 与原始 VTVL 实飞存档版完全一致（板子安装未变，响应逻辑必须一致）：
    //   机体X (前) = -传感器Z (后)
    //   机体Y (右) = +传感器X (右)
    //   机体Z (下) = -传感器Y (上)   ← z_b = 推力轴（机头朝天时指向地面）
    //   手性：bX×bY = (-sZ)×(sX) = -sY = bZ ✓ 右手系
    //   ⚠️ 关键：此约定下"机头竖直朝天静止"解算为 roll=pitch=0（悬停基态），
    //      天然避开欧拉奇异点。2026-08-07 曾误改为"x_b 竖直"，导致
    //      pitch≈+89° 落进万向锁、roll 与 Heading 退化耦合（实测
    //      roll=-110/Heading=-121，二者和≈-231），打杆与目标姿态关系错乱。
    float axBody = -azFilteredRaw;
    float ayBody = axFilteredRaw;
    float azBody = -ayFilteredRaw;
    float gxBody = -gzFilteredRaw; // 陀螺仪应用相同的旋转关系
    float gyBody = gxFilteredRaw;
    float gzBody = -gyFilteredRaw;

    // EKF使用与IMU_Packet一致的轻微低通后FRD机体系数据，避免平均值和增量路径数据源不一致。
    const float accel_body_x_mps2 = axBody * G_TO_MS2;
    const float accel_body_y_mps2 = ayBody * G_TO_MS2;
    const float accel_body_z_mps2 = azBody * G_TO_MS2;
    const float gyro_body_x_radps = gxBody * DEG_TO_RAD;
    const float gyro_body_y_radps = gyBody * DEG_TO_RAD;
    const float gyro_body_z_radps = gzBody * DEG_TO_RAD;

    noInterrupts();
    acc_sum_x += accel_body_x_mps2;
    acc_sum_y += accel_body_y_mps2;
    acc_sum_z += accel_body_z_mps2;

    gyro_sum_x += gyro_body_x_radps;
    gyro_sum_y += gyro_body_y_radps;
    gyro_sum_z += gyro_body_z_radps;

    if (imu_delta_sample_count < NAV_IMU_DELTA_BUFFER_SIZE)
    {
      const int imu_delta_idx = imu_delta_sample_count;
      imu_delta_theta_x[imu_delta_idx] = gyro_body_x_radps * deltaTimeSecs;
      imu_delta_theta_y[imu_delta_idx] = gyro_body_y_radps * deltaTimeSecs;
      imu_delta_theta_z[imu_delta_idx] = gyro_body_z_radps * deltaTimeSecs;
      imu_delta_v_x[imu_delta_idx] = accel_body_x_mps2 * deltaTimeSecs;
      imu_delta_v_y[imu_delta_idx] = accel_body_y_mps2 * deltaTimeSecs;
      imu_delta_v_z[imu_delta_idx] = accel_body_z_mps2 * deltaTimeSecs;
      imu_delta_dt_s[imu_delta_idx] = deltaTimeSecs;
      imu_delta_time_sum_s += deltaTimeSecs;
      imu_delta_sample_count++;
    }
    else
    {
      imu_delta_overflow = true;
    }

    imu_sample_count++;
    interrupts();

    // ========================================================================
    // 步骤 6: 第二次坐标变换 (Body Frame -> Madgwick Frame)
    // ========================================================================
    // 机体FRD系 -> Madgwick算法输入系 (伪NWU/FLU)
    // 这一步是为了适配 Madgwick 算法的内部坐标系定义。
    // ★ 与原始 VTVL 实飞存档版完全一致：
    //   算法输入X = +机体X
    //   算法输入Y = -机体Y
    //   算法输入Z = -机体Z

    float axForMadgwick = axBody;
    float ayForMadgwick = -ayBody;
    float azForMadgwick = -azBody;
    float gxForMadgwick_dps = gxBody;
    float gyForMadgwick_dps = -gyBody;
    float gzForMadgwick_dps = -gzBody;

    // ========================================================================
    // 步骤 7: 使用Madgwick算法更新姿态估计
    // ========================================================================
    // if (compass.update())
    if (false) // 临时禁用磁力计
    {
      // 1. 读取 IST8310 原始数据 (单位: uT)
      IST8310_Vector mag_raw = compass.get_data();

      // 2. 执行坐标系映射 (关键步骤！！！)
      // 目标：将数据对齐到 IMU 的 "前-左-上" 坐标系
      double mx_flu = -mag_raw.y; // 芯片Y指向后，取反变成前
      double my_flu = -mag_raw.x; // 芯片X指向右，取反变成左
      double mz_flu = mag_raw.z;  // 芯片Z指向上，保持不变

      // 3. 传入 Madgwick 算法
      // 注意：ax, ay, az, gx, gy, gz 已经是 "前-左-上" 了
      madgwick.update(
          gxForMadgwick_dps,
          gyForMadgwick_dps,
          gzForMadgwick_dps,
          axForMadgwick,
          ayForMadgwick,
          azForMadgwick,
          mx_flu,
          my_flu,
          mz_flu,
          deltaTimeSecs);
    }
    else
    {
      // 输入参数：陀螺仪(dps), 加速度计(g), 时间差(s)
      madgwick.updateIMU(
          gxForMadgwick_dps,
          gyForMadgwick_dps,
          gzForMadgwick_dps,
          axForMadgwick,
          ayForMadgwick,
          azForMadgwick,
          deltaTimeSecs);
    }

    // ========================================================================
    // 步骤 8: 获取并打包姿态数据 (Data Packing)
    // ========================================================================
    // 从Madgwick获取原始的 q_FLU_from_NWU 四元数
    double q_flu_nwu[4];
    madgwick.getQuaternion(q_flu_nwu);
    Quaternion q_FLU_from_NWU = {q_flu_nwu[0], q_flu_nwu[1], q_flu_nwu[2], q_flu_nwu[3]};

    // 定义修正四元数，将算法输出转换为 NED->机体 标准
    // 新机体系（x=竖直机头）：静止机头朝天 = 相对 NED 绕 y 转 -90°
    //   （x_b = -z_NED：绕 NED y 轴 -90° 把 x 转到 -z ✓）
    // ★ 与原始 VTVL 实飞存档版完全一致：绕 X 轴旋转 180°
    // （FLU→FRD：y/z 同时取反，即 q = {w=0, x=1, y=0, z=0}）
    Quaternion q_FRD_from_FLU = {0.0, 1.0, 0.0, 0.0};
    Quaternion q_NWU_from_NED = {0.0, -1.0, 0.0, 0.0}; // 绕X轴旋转-180度

    // 执行转换: q_FRD_from_NED = q_FRD_from_FLU * q_FLU_from_NWU * q_NWU_from_NED
    Quaternion temp_q = quaternionMultiply(q_FRD_from_FLU, q_FLU_from_NWU);
    Quaternion q_FRD_from_NED = quaternionMultiply(temp_q, q_NWU_from_NED);
    q_FRD_from_NED = normalizeQuaternion(q_FRD_from_NED);

    // 更新用于显示的辅助变量
    icm_Qw = q_FRD_from_NED.w;
    icm_Qx = q_FRD_from_NED.x;
    icm_Qy = q_FRD_from_NED.y;
    icm_Qz = q_FRD_from_NED.z;

    /* =========================================================
     *  姿态解算：获取当前机体欧拉角（Roll / Pitch / Yaw）
     * ---------------------------------------------------------
     *  1. 从 Madgwick 滤波器读取 NED 系下的姿态角（弧度）
     *  2. 叠加长沙地区磁偏角，得到真航向
     *  3. 将结果同步至 AHRS 数据包与全局调试变量
     * ========================================================= */

    // const double MAGNETIC_DECLINATION_DEG = -4.133; // 长沙磁偏角：-4°8′ (西偏为负)
    double roll, pitch, yaw; // 机体欧拉角（弧度）

    // 1. 获取磁航向 (NED系, -PI ~ +PI)
    madgwick.getEulerAngles_NED_FRD(roll, pitch, yaw);

    // 2. 叠加磁偏角 (Yaw_True = Yaw_Mag + Declination)
    //    注意：这里必须使用 +=，因为西偏是负值，相当于减去偏差
    // double declination_rad = MAGNETIC_DECLINATION_DEG * DEG_TO_RAD;
    // yaw += declination_rad;

    // 3. 【关键步骤】归一化到 [-PI, +PI] 范围
    //    防止越界导致控制律失效
    // if (yaw > M_PI)
    // {
    //   yaw -= 2.0 * M_PI;
    // }
    // else if (yaw < -M_PI)
    // {
    //   yaw += 2.0 * M_PI;
    // }

    // 后台 AHRS 航向叠加 EKF 校正偏置，保持与 EKF 航向基准对齐。
    backup_ahrs_roll = roll;
    backup_ahrs_pitch = pitch;
    // 内部计算统一使用 [-pi, pi]，避免0/2pi边界导致EKF/AHRS差值出现跳变。
    backup_ahrs_yaw = wrapAnglePi(static_cast<float>(yaw) + ahrs_yaw_correction_rad);
    Quaternion corrected_backup_q = eulerToQuaternion(backup_ahrs_roll,
                                                      backup_ahrs_pitch,
                                                      backup_ahrs_yaw);
    backup_ahrs_Qw = corrected_backup_q.w;
    backup_ahrs_Qx = corrected_backup_q.x;
    backup_ahrs_Qy = corrected_backup_q.y;
    backup_ahrs_Qz = corrected_backup_q.z;

    // AHRS_Packet 由 navigation_task.cpp 的 EKF 输出桥统一写入，
    // IMU 任务只维护 backup_ahrs_* 后台变量供 EKF 输出桥在 GNSS 失效时回退使用。

    icm_Roll = backup_ahrs_roll; // 同步至后台 AHRS 调试变量
    icm_Pitch = backup_ahrs_pitch;
    icm_Yaw = wrapAngleTwoPi(backup_ahrs_yaw);

    // ========================================================================
    // 步骤 9: 打包处理后的传感器数据 (IMU Packet)
    // ========================================================================
    // 将机体坐标系下的数据存入IMU数据包，并转换为标准单位 (rad/s, m/s^2)
    // 注意：这里使用的是 axBody, gxBody，它们源自校准并滤波后的数据

    IMU_Packet.gyroscope_x = gxBody * DEG_TO_RAD; // 角速度 (rad/s)
    IMU_Packet.gyroscope_y = gyBody * DEG_TO_RAD;
    IMU_Packet.gyroscope_z = gzBody * DEG_TO_RAD;
    icm_gyro_x = gxBody * DEG_TO_RAD; // 更新全局辅助变量
    icm_gyro_y = gyBody * DEG_TO_RAD;
    icm_gyro_z = gzBody * DEG_TO_RAD;

    IMU_Packet.accelerometer_x = axBody * G_TO_MS2; // 加速度 (m/s^2)
    IMU_Packet.accelerometer_y = ayBody * G_TO_MS2;
    IMU_Packet.accelerometer_z = azBody * G_TO_MS2;
    icm_accel_x = axBody * G_TO_MS2; // 更新全局辅助变量
    icm_accel_y = ayBody * G_TO_MS2;
    icm_accel_z = azBody * G_TO_MS2;

    return true; // 返回成功标志，表示姿态已更新
  }
  return false;
}

void handleICM42688()
{
  imu_read_success = readIMUData(); // 读取并处理ICM42688数据

  // 如果IMU读取失败，点亮黄灯作为指示；成功则熄灭 (或根据其他传感器状态决定)
  if (!imu_read_success)
  {
    digitalWrite(LED_yellow, HIGH); // 指示IMU读取问题
  }
  else
  {
    // 黄灯也可能由DPS310控制，这里简化为仅IMU成功时熄灭，实际应用中可能需要更复杂的逻辑
    // if (dps310_read_success) digitalWrite(LED_yellow, LOW);
  }
}
