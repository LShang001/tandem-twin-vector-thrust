/**
 * @file    sensor_imu.cpp
 * @module  固件/传感器（ICM42688 与磁力计采集）
 * @brief   初始化 IMU/磁力计，逐帧完成校准、滤波、轴变换、Madgwick 姿态与数据打包
 * @details 输入：SPI 原始加速度(g)/角速度(dps)/温度、I2C 磁力计样本；
 *          处理：`initICM42688()` 配滤波与量程并用 100 样本静止均值初始化滤波器；
 *          `readIMUData()` 每帧：自动校准 → 一阶互补滤波 → RUB→FRD 轴映射 →
 *          累加双子样增量（关中断）→ FRD→FLU 适配 → Madgwick → 四元数修正 →
 *          欧拉角 → 写 `IMU_Packet` 与后台 AHRS 变量；
 *          输出：`IMU_Packet`、`icm_*`、`backup_ahrs_*`、`imu_delta_*`、`imu_die_temp_c`。
 *          不变量：所有对外物理量在写包前统一换算为 rad/s 与 m/s²。
 * @theory  四元数姿态修正 q_FRD_from_NED = q_FRD_from_FLU⊗q_FLU_from_NWU⊗q_NWU_from_NED
 *          @ref REF-120 §9.2.2 p.256、§9.2.5 p.264（规范化）；
 *          轴映射与悬停基态（机头朝天 roll=pitch=0）@ref MOD-002 §1.2；
 *          欧拉角提取 theta=-asin(R13) @ref MOD-002 §3
 * @frame   传感器 RUB（右-上-后）→ 机体系 FRD（前右下）→ Madgwick 输入 FLU/NWU；
 *          输出四元数为 FRD←NED，欧拉角为 NED 系 3-2-1
 * @units   原始 g/dps，输出 m/s²/rad/s，时间 µs/s，温度 °C，磁场 uT
 * @redline 传感器轴映射（bX=-sZ、bY=+sX、bZ=-sY）与四元数分量顺序 [w,x,y,z]；
 *          改动会导致 roll/Heading 退化耦合（2026-08-07 实机事故），
 *          必须与实飞存档版逐字一致（见 TandemVec_FCS/AGENTS.md）
 * @note    磁力计馈入 Madgwick 的开关 `kMagFeedMadgwick` 默认关闭（未完成
 *          轴映射上板标定与椭球标定）；磁力计轮询不受开关影响，样本始终新鲜。
 *          `AHRS_Packet` 由 `navigation_task.cpp` 输出桥写，本文件只写 backup_ahrs_*。
 * @see     `sensor_imu.h`、`navigation_task.cpp`、`MagCalStore.h`
 */

#include "sensor_imu.h"
#include "math_utils.h"
#include "QuaternionMath.h"
#include "MagCalStore.h" // 标定参数内部 Flash 持久化（上电加载）
#include "imu_factory.h" // IMU 型号无关实例创建（编译期选定后端，见 imu_factory.cpp）
#include "imu_range.h"   // 量程常量：换 FS 时必须同步改这里
#include "ins_vibration_hf.h" // 高频振动通道：用低通前/后之差测 80–120 Hz 机械振动
#include "NavImuIncrementSelect.h" // 导航增量源选择（默认宏关闭时不参与累加）

namespace
{
  constexpr uint32_t IMU_INIT_SAMPLE_TIMEOUT_MS = 20; // 单个初始化样本等待超时，避免IMU异常时卡死setup
  constexpr int IMU_INIT_DATA_TIMEOUT_ERROR = -1001;  // 自定义错误码：初始化阶段等待数据超时

  // ★ 磁力计馈入 Madgwick 的总开关（当前关闭，与既有"临时禁用磁力计"行为一致）。
  // 启用前必须完成：① 对应芯片轴映射上板标定（MagMath.h 常量表）；
  //                ② 椭球标定（DBG 命令 `magcal start/fit/save`，见 communication.cpp）。
  // 磁力计轮询不受此开关影响（见步骤 7），样本始终保持新鲜，
  // 供遥测 / EKF 初始化与磁矢量融合使用；本开关只决定是否馈入 Madgwick。
  constexpr bool kMagFeedMadgwick = false;
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

  // 2. 自动探测磁力计（IST8310@0x0E 优先，其次 QMC5883L@0x0D）
  //    轴对齐与硬铁偏置统一在 MagSensor 层完成，上层只看到机体 FRD 数据。
  if (!magSensor.begin(&Wire1))
  {
    Serial8.println("[Error] Magnetometer Not Found!");
    Serial8.println("[Error] Probed: IST8310 (0x0E), QMC5883L (0x0D).");
    Serial8.println("[Error] Please check wiring (SDA/SCL) and Power.");
    return false;
  }

  Serial8.print("[Init] Magnetometer: ");
  Serial8.print(magSensor.chipName());
  Serial8.println(" Setup Finished Successfully.");

  // 3. 从内部 Flash 加载上次椭球标定（magic/版本/CRC 校验通过才应用；
  //    失败静默保持未标定直通，不影响飞行）
  magcal::MagCalParams storedCal;
  storedCal.identity();
  if (magcal::storeLoad(storedCal) && magSensor.setCal(storedCal))
  {
    Serial8.print("[Init] MagCal loaded from flash: bias=(");
    Serial8.print(storedCal.hardIron[0], 1);
    Serial8.print(", ");
    Serial8.print(storedCal.hardIron[1], 1);
    Serial8.print(", ");
    Serial8.print(storedCal.hardIron[2], 1);
    Serial8.println(") uT");
  }
  else
  {
    Serial8.println("[Init] MagCal: no valid stored calibration (uncalibrated pass-through).");
  }
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

  // === 步骤 2: 创建型号无关实例（编译期选定，见 imu_factory.cpp）===
  IMU = createImuDevice(SPI, SPI_CS, 10000000);
  if (IMU == nullptr)
  {
    Serial8.println("[Error] IMU device creation failed.");
    return -1002;
  }

  // === 步骤 3: 用统一配置初始化（ODR/FSR/LPF 一次配齐）===
  // 默认配置 = 现役硬件参数（accel ±8g / gyro ±2000dps / ODR 目标 2000Hz /
  // LPF 陀螺 100Hz + 加速度 50Hz），与改造前逐项等价。
  // ★ 换 ICM-45686 后 ODR 会落到 3200Hz（该芯片无 2000Hz 档），
  //   actualAccelOdrHz() 回报真实生效值；调度任务的 IMU 周期与 Madgwick
  //   更新率都由它反推（见 main.cpp），不再写死 2kHz。
  ImuConfig imu_cfg;
  const int status = IMU->begin(imu_cfg);
  if (status != 0)
  {
    Serial8.print("[Error] IMU init failed: ");
    Serial8.print(IMU->modelName());
    Serial8.print(" code=");
    Serial8.print(status);
    Serial8.print(" (");
    Serial8.print(IMU->lastError());
    Serial8.println(")");
    return status;
  }

  Serial8.print("[Init] IMU: ");
  Serial8.print(IMU->modelName());
  Serial8.print(" accelODR=");
  Serial8.print(IMU->actualAccelOdrHz());
  Serial8.print("Hz gyroODR=");
  Serial8.print(IMU->actualGyroOdrHz());
  Serial8.println("Hz");

  delay(500); // 等待 500ms，让传感器稳定并开始输出数据

  // === 步骤 5: 采集初始样本并计算平均值 ===
  // 目的: 为滤波器提供合理的初始值，避免启动时的阶跃响应
  float ax = 0, ay = 0, az = 0; // 加速度累加器 (单位: g)
  for (int i = 0; i < 100; i++) // 采集 100 个样本
  {
    // 带超时的数据等待，防止 SPI 总线异常时永久阻塞
    const uint32_t sample_wait_start_ms = millis();
    while (IMU->dataReady() != 1) // 轮询数据就绪（型号无关接口；返回 1 表示有新样本）
    {
      // 超时保护: 单个样本等待超过 20ms 则认为传感器异常
      // 这避免了系统卡死在 setup() 中无法进入故障处理分支
      if ((millis() - sample_wait_start_ms) > IMU_INIT_SAMPLE_TIMEOUT_MS)
      {
        Serial8.println("[Error] IMU init data timeout.");
        return IMU_INIT_DATA_TIMEOUT_ERROR; // 返回自定义错误码 -1001
      }
    }
    // 读取原始加速度数据 (单位: g，传感器 RUB 坐标系)
    float axRaw = IMU->accX(); // X轴: 传感器板右方向
    float ayRaw = IMU->accY(); // Y轴: 传感器板上方向
    float azRaw = IMU->accZ(); // Z轴: 传感器板后方向
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
  accelXFilter.initialize(ax);         // X轴加速度滤波器初值
  accelYFilter.initialize(ay);         // Y轴加速度滤波器初值
  accelZFilter.initialize(az);         // Z轴加速度滤波器初值
  gyroXFilter.initialize(IMU->gyrX()); // X轴陀螺仪滤波器初值 (单位: dps)
  gyroYFilter.initialize(IMU->gyrY()); // Y轴陀螺仪滤波器初值
  gyroZFilter.initialize(IMU->gyrZ()); // Z轴陀螺仪滤波器初值

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
  // ★2026-11-01 防御：IMU 已由全局对象改为 setup() 中创建的指针，
  //   极端时序（初始化失败后仍有任务调用）下判空，按“无数据”处理。
  if (IMU == nullptr)
  {
    return false;
  }

  // 检查 IMU 数据是否就绪（型号无关接口；1 = 有新样本）
  if (IMU->dataReady() == 1)
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
    // 读取 IMU 原始数据（型号无关接口，输出单位 g / dps）
    // 加速度计单位: g (重力加速度倍数)
    float raw_ax_g = IMU->accX();
    float raw_ay_g = IMU->accY();
    float raw_az_g = IMU->accZ();
    // 陀螺仪单位: dps (度/秒)
    float raw_gx_dps = IMU->gyrX();
    float raw_gy_dps = IMU->gyrY();
    float raw_gz_dps = IMU->gyrZ();

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
    // 步骤 4.5: 高频振动监测（不改变任何控制/导航数据通路）
    // ========================================================================
    // hp = raw − lp 是软件低通的补滤波器（高通），保留低通截止（fc≈6/16 Hz）以上
    // 的带内能量。`ins_vibration_adaptive.h` 的 0.5 Hz 高通吃在低通**之后**，
    // 对弹载 80–120 Hz 主谱只有 2–8% 灵敏度，而本通道在 2 kHz 历元上直接测到。
    {
      const float hf_raw_g[3] = {raw_ax_g, raw_ay_g, raw_az_g};
      const float hf_lp_g[3] = {axFilteredRaw, ayFilteredRaw, azFilteredRaw};
      g_ins_hf_vibration.Update(hf_raw_g, hf_lp_g, static_cast<float>(deltaTimeSecs));
    }

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
#ifdef BFS_NAVIGATION_RAW_IMU_INCREMENT
    // 导航累加（均值 + 双子样增量）改用**未过软件低通**的同轴映射值，绕开低通的
    // 时变群延迟；惯导健康 Failed 时由选择器回退到滤波值（含 5 s 后持）。
    // ★ 打开本宏即刻意打破上行"与 IMU_Packet 同源"的口径：导航侧吃未滤波增量，
    //   IMU_Packet/Madgwick/控制侧仍吃滤波值——§4.24 的提带宽正是靠这个不一致。
    // 低通在传感器系做、轴映射是定常带符号置换，两者可交换 ⇒ 这里的 raw 分支
    // 与"SIL 里先映射后低通"逐轴等价。Madgwick 与控制侧不受影响（仍用 axBody 等）。
    // ★ 自参考回路：`acc_sum_*` 同时喂 IMU 健康监测（navigation_task 的 Failed 判据），
    //   而 Failed 又决定本拍用哪一路 ⇒ 提带宽后监测吃的是 raw 样本（方差更大，
    //   但饱和/振动按模块不变量只进 Warning、不进 Failed）。该回路只由 47 格矩阵
    //   （41/47 逐 seed 不变、无格变差 >1%）与 K2 四风险格 nrfb=0 覆盖，实机需盯
    //   navi_raw/navi_fb 占空比。
    constexpr uint8_t kImuHealthFailed = 2U; // 与 ins_imu_health.h::ImuHealthState 一致

    // 两条路径共用同一份 RUB→FRD 置换（宿主 T10–T13 锁死）；选源必须六轴一次性
    // 取齐（`NavImuSelectIncrement` 返回整结构），不允许逐轴混选。
    const float accel_rub_filtered_g[3] = {axFilteredRaw, ayFilteredRaw, azFilteredRaw};
    const float gyro_rub_filtered_dps[3] = {gxFilteredRaw, gyFilteredRaw, gzFilteredRaw};
    const bfs::NavImuBodySample nav_body_filtered =
        bfs::NavImuSensorRubToBodyFrd(accel_rub_filtered_g, gyro_rub_filtered_dps);

    static bfs::NavImuIncrementSelector s_nav_increment_sel(
        true, bfs::NavImuIncrementSelector::kDefaultHoldTicks);
    const bfs::NavImuSource nav_source = s_nav_increment_sel.select(
        g_imu_health_state == kImuHealthFailed);
    g_nav_increment_raw_ticks = s_nav_increment_sel.rawTicks();
    g_nav_increment_fallback_ticks = s_nav_increment_sel.fallbackTicks();

    const float accel_rub_raw_g[3] = {raw_ax_g, raw_ay_g, raw_az_g};
    const float gyro_rub_raw_dps[3] = {gx_cal_dps, gy_cal_dps, gz_cal_dps};
    const bfs::NavImuBodySample nav_body_raw =
        bfs::NavImuSensorRubToBodyFrd(accel_rub_raw_g, gyro_rub_raw_dps);
    const bfs::NavImuBodySample nav_body =
        bfs::NavImuSelectIncrement(nav_body_filtered, nav_body_raw, nav_source);

    const float accel_body_x_mps2 = nav_body.accel_g[0] * G_TO_MS2;
    const float accel_body_y_mps2 = nav_body.accel_g[1] * G_TO_MS2;
    const float accel_body_z_mps2 = nav_body.accel_g[2] * G_TO_MS2;
    const float gyro_body_x_radps = nav_body.gyro_dps[0] * DEG_TO_RAD;
    const float gyro_body_y_radps = nav_body.gyro_dps[1] * DEG_TO_RAD;
    const float gyro_body_z_radps = nav_body.gyro_dps[2] * DEG_TO_RAD;
#else
    const float accel_rub_filtered_g[3] = {axFilteredRaw, ayFilteredRaw, azFilteredRaw};
    const float gyro_rub_filtered_dps[3] = {gxFilteredRaw, gyFilteredRaw, gzFilteredRaw};
    const bfs::NavImuBodySample nav_body =
        bfs::NavImuSensorRubToBodyFrd(accel_rub_filtered_g, gyro_rub_filtered_dps);

    const float accel_body_x_mps2 = nav_body.accel_g[0] * G_TO_MS2;
    const float accel_body_y_mps2 = nav_body.accel_g[1] * G_TO_MS2;
    const float accel_body_z_mps2 = nav_body.accel_g[2] * G_TO_MS2;
    const float gyro_body_x_radps = nav_body.gyro_dps[0] * DEG_TO_RAD;
    const float gyro_body_y_radps = nav_body.gyro_dps[1] * DEG_TO_RAD;
    const float gyro_body_z_radps = nav_body.gyro_dps[2] * DEG_TO_RAD;
#endif

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
    // ★ 与原始 VTVL 实飞存档版完全一致：算法输入 X 取 +机体 X、
    //   算法输入 Y 取 −机体 Y、算法输入 Z 取 −机体 Z。

    float axForMadgwick = axBody;
    float ayForMadgwick = -ayBody;
    float azForMadgwick = -azBody;
    float gxForMadgwick_dps = gxBody;
    float gyForMadgwick_dps = -gyBody;
    float gzForMadgwick_dps = -gzBody;

    // ========================================================================
    // 步骤 7: 使用Madgwick算法更新姿态估计
    // ========================================================================
    // 7.1 磁力计非阻塞轮询（无论是否馈入姿态解算都保持采样新鲜）：
    //     驱动内部 FSM 限速，绝大多数调用只做一次 micros() 比较（~微秒级）；
    //     I2C 事务仅在采样点发生（IST8310 约每 7ms、QMC5883L 约每 10ms 一次突发读）。
    //     输出已由 MagSensor 层完成椭球标定（若有效）并对齐到机体 FRD。
    if (magSensor.isPresent())
    {
      magSensor.update();
    }

    // 7.1b 大疆式两圈引导校准状态机（DBG `magcal guided`；判定帧率 = IMU 帧率）
    if (magSensor.guideActive())
    {
      // SI 单位转换：g → m/s²、dps → rad/s（axBody/gxBody 为机体系 FRD）
      const float accFrd[3] = {axBody * G_TO_MS2, ayBody * G_TO_MS2, azBody * G_TO_MS2};
      const float gyrFrd[3] = {gxBody * DEG_TO_RAD, gyBody * DEG_TO_RAD, gzBody * DEG_TO_RAD};
      magSensor.guideUpdate(accFrd, gyrFrd, micros());
    }

    if (kMagFeedMadgwick)
    {
      // 7.2 磁力计馈入路径（开关 kMagFeedMadgwick，见文件头）
      if (magSensor.fresh())
      {
        // 1. 读取统一 FRD 样本 (单位: uT)
        MagSample mag_frd = magSensor.sampleFRD();

        // 2. 执行坐标系映射：机体 FRD(前右下) -> Madgwick 输入系 (前左上)
        //    即 (x, -y, -z)，与下方加速度计/陀螺仪的 FLU 转换约定一致。
        double mx_flu = mag_frd.x;
        double my_flu = -mag_frd.y;
        double mz_flu = -mag_frd.z;

        // 3. 传入 Madgwick 算法
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
        // 尚无有效磁样本：回退纯 IMU
        madgwick.updateIMU(
            gxForMadgwick_dps,
            gyForMadgwick_dps,
            gzForMadgwick_dps,
            axForMadgwick,
            ayForMadgwick,
            azForMadgwick,
            deltaTimeSecs);
      }
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

    // 磁偏角补偿当前关闭：Madgwick 由伪磁/未标定磁力计驱动时，叠加长沙磁偏角
    // （约 -4.13°）只会把未标定的航向基准再平移，故暂不引入（见文件头 note）。
    double roll, pitch, yaw; // 机体欧拉角（弧度）

    // 1. 获取磁航向 (NED系, -PI ~ +PI)
    madgwick.getEulerAngles_NED_FRD(roll, pitch, yaw);

    // 2. 真航向 = 磁航向 + 磁偏角（西偏为负）；当前关闭，见上方说明。

    // 3. 归一化到 [-PI, +PI]：由下方 wrapAnglePi() 统一完成（避免与 0/2π
    //    边界处的 EKF/AHRS 差值跳变冲突），此处不再手写分支。

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

    // IMU 芯片温度：1 Hz 更新，用于后续温漂建模/诊断（型号无关接口）。
    static uint32_t last_temp_ms = 0;
    if (millis() - last_temp_ms >= 1000U)
    {
      last_temp_ms = millis();
      const float t_c = IMU->dieTempC();
      if (std::isfinite(t_c)) imu_die_temp_c = t_c;
    }

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
