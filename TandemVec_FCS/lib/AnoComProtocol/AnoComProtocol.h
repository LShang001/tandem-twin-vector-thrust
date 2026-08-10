#ifndef ANOCOM_PROTOCOL_H
#define ANOCOM_PROTOCOL_H

#include <Arduino.h>

// 定义帧头
#define ANO_FRAME_HEAD 0xAB

// 定义最大数据长度 (根据需要调整)
#define ANO_MAX_DATA_LEN 256

// 定义硬件地址 (根据需要修改)
#define ANO_LOCAL_ADDR 0x05       // 本机地址 (飞控)
#define ANO_GND_STATION_ADDR 0xFF // 地面站地址

// 定义功能码 (根据协议文档定义所有功能码)

// 安全通信协议
#define ANO_FUNC_DATA_CHECK 0x00 // 数据校验帧

// 灵活格式帧 (用户自定义数据帧)
#define ANO_FUNC_FLEX_DATA_START 0xF1 // 灵活格式帧起始 ID
#define ANO_FUNC_FLEX_DATA_END 0xFA   // 灵活格式帧结束 ID

// 飞控相关信息类
#define ANO_FUNC_IMU_DATA 0x01            // 惯性传感器数据 (加速度、陀螺仪、震动状态)
#define ANO_FUNC_MAG_PRESS_TEMP_DATA 0x02 // 磁力计、气压计、温度数据
#define ANO_FUNC_ATTITUDE_EULER 0x03      // 姿态数据 (欧拉角)
#define ANO_FUNC_ATTITUDE_QUAT 0x04       // 姿态数据 (四元数)
#define ANO_FUNC_ALTITUDE_DATA 0x05       // 高度数据 (气压高度、附加高度、融合高度、测距状态)
#define ANO_FUNC_FLIGHT_MODE 0x06         // 飞控运行模式 (模式、功能标志、当前指令)
#define ANO_FUNC_FLIGHT_SPEED 0x07        // 飞行速度数据 (X、Y、Z 轴速度)
#define ANO_FUNC_POS_OFFSET 0x08          // 位置偏移数据 (X、Y、Z 轴位置偏移)
#define ANO_FUNC_WIND_EST 0x09            // 风速估计 (X、Y 轴风速)
#define ANO_FUNC_TARGET_ATTITUDE 0x0A     // 目标姿态数据 (横滚、俯仰、航向)
#define ANO_FUNC_TARGET_SPEED 0x0B        // 目标速度数据 (X、Y、Z 轴目标速度)
#define ANO_FUNC_RETURN_INFO 0x0C         // 回航信息 (回航角度、回航距离)
#define ANO_FUNC_VOLT_CURR 0x0D           // 电压电流数据
#define ANO_FUNC_EXT_MODULE_STATUS 0x0E   // 外接模块工作状态 (通用速度传感器、通用位置传感器、GPS、附加测高)
#define ANO_FUNC_RGB_OUTPUT 0x0F          // RGB 亮度信息输出

// 飞控控制量输出类
#define ANO_FUNC_PWM_OUTPUT 0x20       // PWM 控制量输出 (8 个 PWM 通道)
#define ANO_FUNC_ATTITUDE_CONTROL 0x21 // 姿态控制量输出 (横滚、俯仰、油门、航向)
#define ANO_FUNC_RC_DATA 0x40          // 遥控器数据 (10 通道, 1000-2000 us, 手册定义)

// 飞控接收信息类
#define ANO_FUNC_GPS_INFO1 0x30          // GPS 传感器信息 1 (定位状态、星数、经纬度、高度、速度、精度)
#define ANO_FUNC_RAW_OPTICAL_FLOW 0x31   // 原始光流信息 (预留)
#define ANO_FUNC_POS_SENSOR_DATA 0x32    // 通用位置型传感器数据 (非捷联载体测量型)(预留)
#define ANO_FUNC_VEL_SENSOR_DATA 0x33    // 通用速度型传感器数据 (捷联载体测量型)(预留)
#define ANO_FUNC_DIS_SENSOR_DATA 0x34    // 通用测距传感器数据 (捷联载体测量型)(预留)
#define ANO_FUNC_IMG_FEATURE_POINTS 0x35 // 通用图像特征点信息帧 (预留)

// 其他功能码 (已定义但当前代码中未使用)
// 命令帧 (用于触发特定功能)
#define ANO_FUNC_CMD_COMMAND 0xC0  // CMD 命令帧,用于读取命令列表数量和请求具体的命令帧内容
#define ANO_FUNC_CMD_FUNCTION 0xC1 // CMD 功能帧,用于返回具体的命令帧内容
#define ANO_FUNC_CMD_INFO 0xC2     // CMD 命令信息帧,用于返回命令帧的具体信息，如包含的参数，参数的数据类型

// 参数读写帧 (用于读写设备参数)
#define ANO_FUNC_PARAM_CMD 0xE0        // 参数命令帧 (读取设备信息、参数数量、参数值、参数信息、恢复默认值、保存参数)
#define ANO_FUNC_PARAM_WRITE_READ 0xE1 // 参数值写入、参数值读取返回
#define ANO_FUNC_PARAM_INFO 0xE2       // 参数信息返回 (参数 ID、参数类型、参数名称、参数介绍)
#define ANO_FUNC_DEVICE_INFO 0xE3      // 设备信息返回 (设备 ID、硬件版本、软件版本、Bootloader 版本、通信协议版本、设备名称)

// 固件升级
#define ANO_FUNC_FIRMWARE_UPGRADE 0xF0 // 固件升级

// 其他帧
#define ANO_FUNC_LOG_STRING 0xA0       // LOG 信息输出 (字符串)
#define ANO_FUNC_LOG_STRING_NUM 0xA1   // LOG 信息输出 (字符串+数字)
#define ANO_FUNC_IMAGE_DATA 0xB0       // 图像数据
#define ANO_FUNC_IP_NETWORK_DATA1 0xB1 // 基于 IP 组网的数据 (格式 1)
#define ANO_FUNC_IP_NETWORK_DATA2 0xB2 // 基于 IP 组网的数据 (格式 2)
#define ANO_FUNC_SPECIAL_DATA 0xFB     // 特殊数据 (用户自定义的特殊数据)

// 数据类型枚举
enum AnoDataType
{
    ANO_UINT8 = 0,
    ANO_INT8,
    ANO_UINT16,
    ANO_INT16,
    ANO_UINT32,
    ANO_INT32,
    ANO_UINT64,
    ANO_INT64,
    ANO_FLOAT,
    ANO_DOUBLE,
    ANO_STRING
};

// 灵活格式帧数据信息
struct AnoFlexDataInfo
{
    uint8_t id;       // 数据 ID
    const char *name; // 数据名称
    AnoDataType type; // 数据类型
};

// ---------------------------------------------------------
// POWER_STATE 位掩码定义 (Bitmask)
// ---------------------------------------------------------
// Bit0: PMU 发生其他报警信息
#define ANO_PWR_FLAG_PMU_ERR (1 << 0)

// Bit1: 某个电芯达到报警电压 (警告)
#define ANO_PWR_FLAG_CELL_ALARM (1 << 1)

// Bit2: 某个电芯达到停机电压 (严重低压)
#define ANO_PWR_FLAG_CELL_LOW (1 << 2)

// Bit3: 电池总电压异常
#define ANO_PWR_FLAG_VOLT_ALL_ERR (1 << 3)

// Bit4: PMU 串口输出电流超过报警电流
#define ANO_PWR_FLAG_CURR_OVER (1 << 4)

// Bit5: PMU 飞控串口看门狗动作
#define ANO_PWR_FLAG_DOG_ACT (1 << 5)

class AnoComProtocol
{
public:
    AnoComProtocol(Stream *serial);

    // 初始化
    void begin(long baudRate);

    // 发送数据帧
    void sendData(uint8_t destAddr, uint8_t funcCode, uint8_t *data, uint16_t len);

    // ★ 2026-08-10 组帧模式（COMM-001 A3）：beginGroup() 后所有 sendXxx 帧
    //   拼入内部缓冲不写串口，endGroup() 一次 write——组内多帧合并单次写，
    //   减少 write 调用/写竞争窗口。组缓冲满时自动先 flush 再入组（防御）。
    //   用法：beginGroup() → sendXxx()×N → endGroup()。
    void beginGroup();
    void endGroup();

    // ★ 2026-08-10 组帧（COMM-001 A2/A3）：把一帧组装进外部缓冲并返回帧长
    //   （含 8B 开销），不写串口。len 超过 ANO_MAX_DATA_LEN 时截断。
    //   校验算法与 sendData 逐字节一致（单循环边拷边算）。
    uint16_t buildFrame(uint8_t *buf, uint16_t off, uint8_t destAddr, uint8_t funcCode,
                        const uint8_t *data, uint16_t len);

    // 接收数据帧
    void receiveData();

    /**
     * @brief 发送惯性传感器数据 (ID: 0x01)
     *
     * @param accX 加速度 X轴, 单位: m/s²
     * @param accY 加速度 Y轴, 单位: m/s²
     * @param accZ 加速度 Z轴, 单位: m/s²
     * @param gyrX 陀螺仪 X轴, 单位: deg/s (度/秒)
     * @param gyrY 陀螺仪 Y轴, 单位: deg/s (度/秒)
     * @param gyrZ 陀螺仪 Z轴, 单位: deg/s (度/秒)
     * @param shockSta 震动状态 (0:无震动, 1-255:震动强度等级)
     */
    void sendIMUData(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, uint8_t shockSta);

    /**
     * @brief 发送罗盘、气压、温度数据 (ID: 0x02)
     * @param magX 磁力计 X 轴数据
     * @param magY 磁力计 Y 轴数据
     * @param magZ 磁力计 Z 轴数据
     * @param temp 温度数据, 放大 10 倍传输, 0.1 摄氏度
     * @param magSta 磁力计状态
     */
    void sendMagPressTempData(int16_t magX, int16_t magY, int16_t magZ, float temp, uint8_t magSta);

    /**
     * @brief 发送飞控姿态：欧拉角格式 (ID: 0x03)
     * @param rol 横滚角, 单位 0.01 度
     * @param pit 俯仰角, 单位 0.01 度
     * @param yaw 航向角, 单位 0.01 度
     * @param fusionSta 融合状态
     */
    void sendAttitudeEuler(float rol, float pit, float yaw, uint8_t fusionSta);

    /**
     * @brief 发送飞控姿态：四元数格式 (ID: 0x04)
     * @param v0 四元数, 传输时扩大 10000 倍
     * @param v1 四元数, 传输时扩大 10000 倍
     * @param v2 四元数, 传输时扩大 10000 倍
     * @param v3 四元数, 传输时扩大 10000 倍
     * @param fusionSta 融合状态
     */
    void sendAttitudeQuat(float v0, float v1, float v2, float v3, uint8_t fusionSta);

    /**
     * @brief 发送高度数据 (ID: 0x05)
     * @param altBar 气压计高度, 单位厘米
     * @param altAdd 附加高度, 如超声波、激光测距, 单位厘米
     * @param altFu 融合后对地高度, 单位厘米
     * @param altSta 测距状态
     */
    void sendAltitudeData(float altBar, float altAdd, float altFu, uint8_t altSta);

    /**
     * @brief 发送飞控运行模式 (ID: 0x06)
     * @param mode 飞控模式
     * @param sFlag 功能标志, 0 锁定, 1 解锁, 2 已起飞
     * @param cId 当前飞控执行的指令功能 (指示最近的一次, 完成后复位为“悬停功能”)
     * @param cmd0 当前飞控执行的指令功能 (指示最近的一次, 完成后复位为“悬停功能”)
     * @param cmd1 当前飞控执行的指令功能 (指示最近的一次, 完成后复位为“悬停功能”)
     */
    void sendFlightMode(uint8_t mode, uint8_t sFlag, uint8_t cId, uint8_t cmd0, uint8_t cmd1);

    /**
     * @brief 发送飞行速度数据 (ID: 0x07)
     * @param speedX X 轴速度, 单位 cm/s
     * @param speedY Y 轴速度, 单位 cm/s
     * @param speedZ Z 轴速度, 单位 cm/s
     */
    void sendFlightSpeed(int16_t speedX, int16_t speedY, int16_t speedZ);

    /**
     * @brief 发送位置偏移数据 (ID: 0x08)
     * @param posX X 轴位置偏移, 单位 cm
     * @param posY Y 轴位置偏移, 单位 cm
     * @param posZ Z 轴位置偏移, 单位 cm
     */
    void sendPosOffset(int32_t posX, int32_t posY, int32_t posZ);

    /**
     * @brief 发送风速估计 (ID: 0x09)
     * @param windX X 轴风速估计, 单位 cm/s
     * @param windY Y 轴风速估计, 单位 cm/s
     */
    void sendWindEstimate(int16_t windX, int16_t windY);

    /**
     * @brief 发送目标姿态数据 (ID: 0x0A)
     * @param tarRol 目标横滚角, 单位 0.01 度
     * @param tarPit 目标俯仰角, 单位 0.01 度
     * @param tarYaw 目标航向角, 单位 0.01 度
     */
    void sendTargetAttitude(int16_t tarRol, int16_t tarPit, int16_t tarYaw);

    /**
     * @brief 发送目标速度数据 (ID: 0x0B)
     * @param tarSpeedX 目标 X 轴速度, 单位 cm/s
     * @param tarSpeedY 目标 Y 轴速度, 单位 cm/s
     * @param tarSpeedZ 目标 Z 轴速度, 单位 cm/s
     */
    void sendTargetSpeed(int16_t tarSpeedX, int16_t tarSpeedY, int16_t tarSpeedZ);

    /**
     * @brief 发送回航信息 (ID: 0x0C)
     * @param ra 回航角度, 传输时扩大 10 倍, 单位 0.1 度
     * @param rd 回航距离, 单位米
     */
    void sendReturnInfo(int16_t ra, uint16_t rd);

    /**
     * @brief 发送电压电流及电源状态数据 (ID: 0x0D)
     * @param bat_voltage 电池电压 (V)
     * @param bat_current 电池电流 (A)
     * @param fc_voltage  飞控电压 (V)
     * @param fc_current  飞控电流 (A)
     * @param power_state_flags 电源状态标志位组合 (使用 ANO_PWR_FLAG_xxx 按位或组合)
     */
    void sendVoltCurr(float bat_voltage, float bat_current, float fc_voltage, float fc_current, uint16_t power_state_flags);

    /**
     * @brief 发送外接模块工作状态 (ID: 0x0E)
     * @param staGVel 通用速度传感器状态
     * @param staGPos 通用位置传感器状态
     * @param staGps GPS 传感器状态
     * @param staAltAdd 附加测高传感器状态
     */
    void sendExtModuleStatus(uint8_t staGVel, uint8_t staGPos, uint8_t staGps, uint8_t staAltAdd);

    /**
     * @brief 发送 RGB 亮度信息输出 (ID: 0x0F)
     * @param briR 红色亮度, 0-20
     * @param briG 绿色亮度, 0-20
     * @param briB 蓝色亮度, 0-20
     * @param briA 单独 LED 亮度, 0-20
     */
    void sendRGBOutput(uint8_t briR, uint8_t briG, uint8_t briB, uint8_t briA);

    /**
     * @brief 发送 PWM 控制量 (ID: 0x20)
     * @param pwm1 PWM 通道 1, 范围 0-10000, 单位 0.01%
     * @param pwm2 PWM 通道 2, 范围 0-10000, 单位 0.01%
     * @param pwm3 PWM 通道 3, 范围 0-10000, 单位 0.01%
     * @param pwm4 PWM 通道 4, 范围 0-10000, 单位 0.01%
     * @param pwm5 PWM 通道 5, 范围 0-10000, 单位 0.01%
     * @param pwm6 PWM 通道 6, 范围 0-10000, 单位 0.01%
     * @param pwm7 PWM 通道 7, 范围 0-10000, 单位 0.01%
     * @param pwm8 PWM 通道 8, 范围 0-10000, 单位 0.01%
     */
    void sendPWMOutput(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5, uint16_t pwm6, uint16_t pwm7, uint16_t pwm8);

    /**
     * @brief 发送遥控器数据 (ID: 0x40, 手册定义)
     *
     * 手册 0x40：THR/YAW/ROL/PIT/AUX1-6 共 10 通道，数据范围 1000-2000 us，
     * 0 表示无通信/失控。本工程遥控走 ELRS/CRSF，raw_rc_values 即真实输入。
     * @param rc 10 通道原始值 (us)
     */
    void sendRCData(const uint16_t *rc);

    /**
     * @brief 发送姿态控制量 (ID: 0x21)
     * @param ctrlRol 横滚控制量, 范围 +-5000
     * @param ctrlPit 俯仰控制量, 范围 +-5000
     * @param ctrlThr 油门控制量, 范围 0-10000
     * @param ctrlYaw 航向控制量, 范围 +-5000
     */
    void sendAttitudeControl(int16_t ctrlRol, int16_t ctrlPit, int16_t ctrlThr, int16_t ctrlYaw);

    /**
     * @brief 发送 GPS 传感器信息 1 (ID: 0x30)
     * @param fixSta 定位状态, UBX 协议的 FIX_STA
     * @param sNum 卫星数量
     * @param lng 经度, 传输时扩大 10000000 倍
     * @param lat 纬度, 传输时扩大 10000000 倍
     * @param altGps GPS 高度, 单位 米
     * @param nSpe 北向速度, 单位 m/s
     * @param eSpe 东向速度, 单位 m/s
     * @param dSpe 地向速度, 单位 m/s
     * @param pdop 定位精度, 传输时放大 10 倍
     * @param vacc 速度精度, 传输时放大 10 倍
     * @param sacc 高度精度, 传输时放大 10 倍
     */
    void sendGPSInfo1(uint8_t fixSta, uint8_t sNum, float lng, float lat, float altGps, float nSpe, float eSpe, float dSpe, float pdop, float vacc, float sacc);

    // 添加灵活格式帧数据
    void addFlexData(uint8_t frameId, AnoFlexDataInfo dataInfo);

    // 发送灵活格式帧数据
    void sendFlexData(uint8_t frameId);

    // 注册数据接收回调函数
    void onDataCheck(void (*callback)(uint8_t idGet, uint8_t scGet, uint8_t acGet));

    /**
     * @brief 注册通用上行帧回调
     * @param cb 回调函数, 参数为 (功能码, DATA指针, DATA长度, 和校验SC, 附加校验AC)
     * @note parseData 在校验通过后调用此回调, 通信模块可在此处理任意功能码。
     *       SC/AC 为源帧的校验值, 供安全协议（0xE0/0xE1 参数帧）回传 0x00 校验帧使用。
     */
    void setRxCallback(void (*cb)(uint8_t, uint8_t *, uint16_t, uint8_t, uint8_t));
    // ... 注册其他数据接收回调函数

    // ---- 上行通信：发送校验帧/设备信息/参数信息 ----

    /**
     * @brief 发送数据校验帧 (ID: 0x00) — 安全协议回传
     * @param idGet  需校验的帧的功能码 ID
     * @param scGet  需校验的帧的和校验值
     * @param acGet  需校验的帧的附加校验值
     */
    void sendDataCheck(uint8_t idGet, uint8_t scGet, uint8_t acGet);

    /**
     * @brief 发送设备信息返回 (ID: 0xE3)
     * @param devId   设备 ID (与设备地址相同)
     * @param hwVer   硬件版本
     * @param swVer   软件版本
     * @param blVer   Bootloader 版本 (无则填 0)
     * @param ptVer   通信协议版本
     * @param devName 设备名称字符串 (最长 20 字节)
     */
    void sendDeviceInfo(uint8_t devId, int16_t hwVer, int16_t swVer,
                        int16_t blVer, int16_t ptVer, const char *devName);

private:
    Stream *_serial;                         // 串口对象
    uint8_t _rxBuffer[ANO_MAX_DATA_LEN + 8]; // 接收缓冲区
    uint8_t _rxIndex;                        // 接收缓冲区索引
    // ★ 2026-08-10 组帧模式（COMM-001 A3）：组缓冲 + 状态（beginGroup/endGroup 管理）
    uint8_t _groupBuf[256];                  // 组缓冲（快环 ~72B / 慢环 ~114B，256 足够）
    uint16_t _groupLen;                      // 组缓冲已用字节
    bool _groupActive;                       // 组模式激活标志
    bool _dataReceived;                      // 数据接收标志

public:
    // ★ 2026-08-10 上行自愈：半帧冻结检测/复位。
    //   receiveData 在 available()=0 时退出，_rxIndex 若卡在 1-7（收到过 0xAB
    //   但帧未凑齐），后续新帧的帧头被当普通字节错位累积、长度字段读错、
    //   永远凑不齐 → 上行永久哑火（下行 0xF2 照发，实测 DBG 进出/噪声可触发）。
    //   调用方（handleAnoCom）周期检查，stall >100ms 强制复位。
    bool rxStalled() const { return _rxIndex != 0; }
    void rxReset() { _rxIndex = 0; }

private:
    uint8_t calculateSumCheck(uint8_t *data, uint16_t len);
    uint8_t calculateAddCheck(uint8_t *data, uint16_t len);

    // 解析数据帧
    void parseData(uint8_t *data, uint16_t len);

    // 灵活格式帧数据存储
    AnoFlexDataInfo _flexData[10][5]; // 假设每个灵活格式帧最多5个数据
    uint8_t _flexDataCount[10];

    // 回调函数指针
    void (*_onDataCheckCallback)(uint8_t idGet, uint8_t scGet, uint8_t acGet);
    // ... 其他回调函数指针
};

#endif