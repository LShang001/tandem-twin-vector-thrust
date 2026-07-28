/**
 * @file deta100_types.h
 * @brief DETA100 惯性导航模块的纯类型定义（枚举、结构体、类）
 *
 * 本文件从 DETA100_module.h 中提取了所有纯类型定义，不包含任何变量定义或函数实现，
 * 因此可以被多个 .cpp 文件安全包含而不会产生链接重复定义错误。
 *
 * 包含内容：
 *   - 协议常量（帧头/帧尾、数据类型标识、数据长度）
 *   - GPS 定位状态枚举 (GPSFixType)
 *   - 数据包结构体（IMU/AHRS/INS_GNSS/GeodeticPos/Status）
 *   - 系统状态和滤波器状态位域结构体
 *
 * 协议格式：[帧头(1B)] [类型(1B)] [长度(1B)] [数据(N B)] [CRC8(1B)] [CRC16(2B)] [帧尾(1B)]
 */
#pragma once

#include <cstdint>

// ================== 协议常量定义 ==================
// 以下常量定义了 DETA100 模块串口通信协议的帧结构和数据长度
// --- 帧定界符 ---
#define FRAME_HEAD 0xFC // 帧头标识字节
#define FRAME_END 0xFD  // 帧尾标识字节

// --- 数据类型标识 (Type Field) ---
#define TYPE_IMU 0x40          // IMU 原始数据包类型标识
#define TYPE_AHRS 0x41         // AHRS 姿态解算数据包类型标识
#define TYPE_INS_GNSS 0x42     // INS/GNSS 组合导航数据包类型标识
#define TYPE_GEODETIC_POS 0x5C // 大地坐标位置数据包类型标识
#define TYPE_STATUS 0x53       // 系统状态数据包类型标识

// --- 有效载荷长度 (Payload Length, 不含帧头/帧尾/CRC) ---
#define IMU_LEN 0x38          // IMU 数据包有效载荷长度 (56字节)
#define AHRS_LEN 0x30         // AHRS 数据包有效载荷长度 (48字节)
#define INS_GNSS_LEN 0x48     // INS/GNSS 数据包有效载荷长度 (72字节)
#define GEODETIC_POS_LEN 0x20 // 大地坐标数据包有效载荷长度 (32字节)
#define STATUS_LEN 0x04       // 系统状态数据包有效载荷长度 (4字节)

// --- 完整帧长度 (含帧头/类型/长度/CRC/帧尾) ---
#define IMU_TYPE_LEN 64          // IMU 完整帧长度
#define AHRS_TYPE_LEN 56         // AHRS 完整帧长度
#define INS_GNSS_TYPE_LEN 80     // INS/GNSS 完整帧长度
#define GEODETIC_POS_TYPE_LEN 40 // 大地坐标完整帧长度
#define STATUS_TYPE_LEN 12       // 系统状态完整帧长度

// ================== 枚举类型定义 ==================
/**
 * @brief GNSS 定位状态枚举
 *
 * 定义了 DETA100 模块报告的 GNSS 定位模式，对应 StatusPacket 中
 * filter_status 的 bit[7:4] 字段。数值越大表示定位精度越高。
 * 该枚举用于导航系统判断当前 GNSS 数据的可用性和精度等级。
 */
typedef enum
{
    DETA100_GPS_FIX_TYPE_NO_GPS = 0,    // 无 GNSS 模块
    DETA100_GPS_FIX_TYPE_NO_FIX = 1,    // 有 GNSS 模块但无定位
    DETA100_GPS_FIX_TYPE_2D_FIX = 2,    // 2D 定位（仅有水平位置）
    DETA100_GPS_FIX_TYPE_3D_FIX = 3,    // 3D 定位（水平+垂直位置）
    DETA100_GPS_FIX_TYPE_DGPS = 4,      // 差分 GNSS 定位
    DETA100_GPS_FIX_TYPE_RTK_FLOAT = 5, // RTK 浮点解（亚米级精度）
    DETA100_GPS_FIX_TYPE_RTK_FIXED = 6, // RTK 固定解（厘米级精度）
    DETA100_GPS_FIX_TYPE_STATIC = 7,    // 静态定位模式
    DETA100_GPS_FIX_TYPE_PPP = 8,       // 精密单点定位
    DETA100_GPS_FIX_TYPE_RTK_DUAL = 9   // 双频 RTK 定位
} GPSFixType;

// ================== 数据结构定义 ==================
/**
 * @brief IMU 原始数据包结构体
 *
 * 存储 DETA100 模块输出的惯性测量单元原始数据，包括：
 * - 三轴陀螺仪角速度 (rad/s)
 * - 三轴加速度计比力 (m/s^2)
 * - 三轴磁力计磁场强度 (uT)
 * - 传感器温度和气压
 * - 64位微秒级时间戳
 */
class IMUPacket
{
public:
    float gyroscope_x, gyroscope_y, gyroscope_z;             // 三轴陀螺仪角速度 (rad/s, 机体系FRD)
    float accelerometer_x, accelerometer_y, accelerometer_z; // 三轴加速度计比力 (m/s^2, 机体系FRD)
    float magnetometer_x, magnetometer_y, magnetometer_z;    // 三轴磁力计磁场强度 (uT)
    float imu_temperature;                                   // IMU 芯片温度 (°C)
    float Pressure;                                          // 板载气压计压力 (Pa)
    float pressure_temperature;                              // 气压计温度 (°C)
    uint64_t Timestamp;                                      // 数据时间戳 (微秒, 64位)
};

/**
 * @brief AHRS 姿态航向参考系统数据包结构体
 *
 * 存储 DETA100 模块解算出的姿态信息，包括：
 * - 三轴角速度 (rad/s)
 * - 欧拉角: 滚转角Roll、俯仰角Pitch、航向角Heading (rad)
 * - 姿态四元数 (w, x, y, z)
 * - 64位微秒级时间戳
 *
 * 坐标系约定: NED (北-东-地)
 * 欧拉角旋转顺序: Z-Y'-X'' (偏航-俯仰-滚转)
 */
class AHRSPacket
{
public:
    float RollSpeed, PitchSpeed, HeadingSpeed; // 三轴角速度 (rad/s)
    float Roll, Pitch, Heading;                // 欧拉角 (rad): 滚转、俯仰、航向
    float Qw, Qx, Qy, Qz;                      // 姿态四元数 (NED->FRD)
    uint64_t Timestamp;                        // 数据时间戳 (微秒, 64位)
};

/**
 * @brief INS/GNSS 组合导航数据包结构体
 *
 * 存储 DETA100 模块的惯性/卫星组合导航解算结果，包括：
 * - 机体系速度和加速度
 * - NED 导航系下的位置、速度、加速度
 * - 气压高度
 * - 64位微秒级时间戳
 */
class INS_GNSSPacket
{
public:
    float body_velocity_x, body_velocity_y, body_velocity_z;             // 机体系速度 (m/s, FRD)
    float body_acceleration_x, body_acceleration_y, body_acceleration_z; // 机体系加速度 (m/s^2, FRD)
    float location_north, location_east, location_down;                  // NED 相对位置 (m)
    float velocity_north, velocity_east, velocity_down;                  // NED 速度 (m/s)
    float acceleration_north, acceleration_east, acceleration_down;      // NED 加速度 (m/s^2)
    float pressure_altitude;                                             // 气压高度 (m)
    uint64_t timestamp;                                                  // 数据时间戳 (微秒, 64位)
};

/**
 * @brief 大地坐标位置数据包结构体
 *
 * 存储 DETA100 模块输出的 WGS84 大地坐标和定位精度信息。
 * 经纬度使用弧度表示，高度为 WGS84 椭球高。
 */
class GeodeticPosPacket
{
public:
    double latitude, longitude; // 纬度、经度 (弧度, WGS84)
    double height;              // 大地高 (米, WGS84 椭球高)
    float hAcc;                 // 水平定位精度 (米, 1-sigma)
    float vAcc;                 // 垂直定位精度 (米, 1-sigma)
};

/**
 * @brief 系统状态位域结构体
 *
 * 对应 DETA100 StatusPacket 中的系统状态字 (2字节, 16位)。
 * 每一位表示一个独立的硬件/传感器故障或报警状态。
 * true 表示故障/报警发生，false 表示正常。
 */
struct SystemStatus
{
    bool system_failure;               // bit 0: 系统级故障
    bool accelerometer_sensor_failure; // bit 1: 加速度计传感器故障
    bool gyroscope_sensor_failure;     // bit 2: 陀螺仪传感器故障
    bool magnetometer_sensor_failure;  // bit 3: 磁力计传感器故障
    bool pressure_sensor_failure;      // bit 4: 气压传感器故障
    bool gnss_failure;                 // bit 5: GNSS 接收机故障
    bool accelerometer_over_range;     // bit 6: 加速度计量程溢出
    bool gyroscope_over_range;         // bit 7: 陀螺仪量程溢出
    bool magnetometer_over_range;      // bit 8: 磁力计量程溢出
    bool pressure_over_range;          // bit 9: 气压计量程溢出
    bool minimum_temperature_alarm;    // bit 10: 温度过低报警
    bool maximum_temperature_alarm;    // bit 11: 温度过高报警
    bool low_voltage_alarm;            // bit 12: 供电电压过低报警
    bool high_voltage_alarm;           // bit 13: 供电电压过高报警
    bool gnss_antenna_disconnected;    // bit 14: GNSS 天线断开
    bool data_output_overflow_alarm;   // bit 15: 数据输出溢出报警
};

/**
 * @brief 滤波器状态位域结构体
 *
 * 对应 DETA100 StatusPacket 中的滤波器状态字 (2字节, 16位)。
 * 描述了 DETA100 内部各滤波器和子系统的初始化及工作状态。
 */
struct FilterStatus
{
    bool orientation_filter_initialised; // bit 0: 姿态滤波器已初始化
    bool navigation_filter_initialised;  // bit 1: 导航滤波器已初始化
    bool heading_initialised;            // bit 2: 航向已初始化
    bool utc_time_initialised;           // bit 3: UTC 时间已同步
    GPSFixType gnss_fix_status;          // bit[7:4]: GNSS 定位类型 (见 GPSFixType 枚举)
    bool event_occurred;                 // bit 8: 系统事件发生
    bool internal_gnss_enabled;          // bit 9: 内部 GNSS 已使能
    bool magnetic_heading_active;        // bit 10: 磁航向已激活
    bool velocity_heading_enabled;       // bit 11: 速度航向已使能
    bool atmospheric_altitude_enabled;   // bit 12: 大气高度已使能
    bool external_position_active;       // bit 13: 外部位置源已激活
    bool external_velocity_active;       // bit 14: 外部速度源已激活
    bool external_heading_active;        // bit 15: 外部航向源已激活
};

/**
 * @brief 系统状态数据包结构体
 *
 * 封装了 DETA100 模块的完整系统状态信息，包括硬件故障状态和滤波器工作状态。
 * 通过解析 Status_Data 原始字节数组填充各字段。
 */
class StatusPacket
{
public:
    SystemStatus system_status; // 硬件/传感器故障状态
    FilterStatus filter_status; // 滤波器/子系统工作状态
};
