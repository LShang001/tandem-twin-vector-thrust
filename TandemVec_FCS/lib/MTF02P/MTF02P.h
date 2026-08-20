/**
 * @file MTF02P.h
 * @author LShang
 * @brief MTF-02P光流测距一体传感器Arduino库的头文件。
 * @version 1.1
 * @date 2025-7-12
 *
 * @copyright Copyright (c) 2025
 *
 * @details
 *  此文件定义了 MTF02P 类的公共接口、内部使用的数据结构以及
 *  与Micolink通信协议相关的常量。用户通过包含此头文件并实例化
 *  MTF02P 类来与传感器进行交互。
 *  该库设计为非阻塞式，并与Arduino的Stream类兼容，
 *  使其可以轻松与硬件串口或软件串口配合使用。
 */

#ifndef MTF02P_h
#define MTF02P_h

// 包含Arduino核心库，以使用如Stream, uint8_t等基本类型和函数
#include <Arduino.h>
// 包含Stream类定义，使本库能够兼容所有继承自Stream的串口类
#include <Stream.h>

// -----------------------------------------------------------------------------
// 协议常量定义 (源自 Micolink 协议规范)
// -----------------------------------------------------------------------------
#define MICOLINK_MSG_HEAD 0xEF                          // Micolink协议的固定帧头，用于同步数据帧
#define MICOLINK_MAX_PAYLOAD_LEN 64                     // 协议定义的最大负载长度，用于防止缓冲区溢出
#define MICOLINK_MAX_LEN (MICOLINK_MAX_PAYLOAD_LEN + 7) // 整帧最大长度 (payload + 7字节固定开销)

/**
 * @brief Micolink协议中定义的消息ID枚举。
 * @details 用于区分不同类型的传感器数据包。
 */
enum MicolinkMsgID
{
    /**
     * @brief 测距与光流传感器数据包的消息ID。
     *        当接收到的数据帧msg_id为此值时，我们知道其负载是测距和光流数据。
     */
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,
};

// -----------------------------------------------------------------------------
// 协议数据结构定义 (源自 Micolink 协议规范)
// -----------------------------------------------------------------------------

/**
 * @brief Micolink协议消息帧的内部表示。
 * @details 此结构体用于在状态机解析过程中暂存一帧数据。
 */
typedef struct
{
    uint8_t head;                              // 帧头 (固定为 0xEF)
    uint8_t dev_id;                            // 设备ID
    uint8_t sys_id;                            // 系统ID
    uint8_t msg_id;                            // 消息ID
    uint8_t seq;                               // 包序列号 (0-255循环)
    uint8_t len;                               // 数据负载的长度
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN]; // 数据负载字节数组
    uint8_t checksum;                          // 接收到的原始校验和

    // --- 状态机内部变量 ---
    uint8_t status;           // 有限状态机的当前状态
    uint8_t payload_cnt;      // 已接收的payload字节计数器
    uint8_t running_checksum; // 在接收过程中动态计算的校验和，用于高效校验
} MicolinkMsg_t;

/**
 * @brief 数据负载定义: 测距与光流传感器 (对应消息ID: 0x51)
 * @details 这个结构体精确地映射了传感器数据负载的二进制布局。
 *          #pragma pack(1) 指令至关重要，它告诉编译器取消内存对齐，
 *          确保结构体在内存中的大小和布局与协议在字节流中的定义完全一致。
 *          这避免了因编译器自动插入填充字节而导致的解析错误。
 */
#pragma pack(1)
typedef struct
{
    uint32_t time_ms;     // 传感器内部系统时间戳 (单位: ms)
    uint32_t distance;    // 测量的距离 (单位: mm)。0 表示数据不可用
    uint8_t strength;     // ToF测距信号强度
    uint8_t precision;    // ToF测距精度或置信度
    uint8_t tof_status;   // ToF测距状态 (1: 数据可用, 其他值: 不可用或错误)
    uint8_t reserved1;    // 预留字节，用于未来扩展
    int16_t flow_vel_x;   // X轴光流速度 (单位: cm/s @ 1m 高度)。这是一个归一化值
    int16_t flow_vel_y;   // Y轴光流速度 (单位: cm/s @ 1m 高度)。这是一个归一化值
    uint8_t flow_quality; // 光流质量，值越大表示数据越可信
    uint8_t flow_status;  // 光流状态 (1: 数据可用, 其他值: 不可用或错误)
    uint16_t reserved2;   // 预留字节，用于未来扩展
} MicolinkPayloadRangeSensor_t;
#pragma pack()

// 协议格式锁定：0x51 payload 按手册固定为 20 字节，布局必须有 pack(1) 保证。
static_assert(sizeof(MicolinkPayloadRangeSensor_t) == 20,
              "Micolink 0x51 payload size must be 20 bytes");

// -----------------------------------------------------------------------------
// MTF-02P 传感器库主类
// -----------------------------------------------------------------------------
/**
 * @brief MTF02P 链路统计结构体
 *
 * 用于监控串口链路质量和协议解析状态。
 */
typedef struct
{
    uint32_t valid_frames;        // 校验通过的有效帧数
    uint32_t checksum_errors;     // 校验失败的帧数
    uint32_t payload_len_errors;  // 0x51 消息 payload 长度异常的帧数
    uint32_t lost_frames;         // 由 seq 不连续检测到的丢帧数量
} MTF02P_Stats_t;

/**
 * @brief 用于与MTF-02P传感器通信的主类。
 * @details 封装了Micolink协议的解析逻辑，为用户提供简洁的API来获取传感器数据。
 */
class MTF02P
{
public:
    /**
     * @brief 构造函数。
     * @details 初始化类的内部变量。
     */
    MTF02P();

    /**
     * @brief 初始化库并指定通信串口。
     * @param port 一个Stream对象的引用。这可以是硬件串口(如 Serial1)或软件串口对象。
     */
    void begin(Stream &port);

    /**
     * @brief 更新函数，从串口读取并解析数据。
     * @details 此函数应在Arduino的 `loop()` 中被尽可能频繁地调用。
     *          它是非阻塞的，会处理串口缓冲区中的所有可用数据。
     * @return bool 如果在本次调用中成功解析到一帧全新的、校验通过的数据，则返回 true；否则返回 false。
     */
    bool update();

    // --- 数据获取API ---

    /** @brief 获取最新的距离测量值。 @return uint32_t 距离，单位为毫米(mm)。 */
    uint32_t getDistance_mm();

    /** @brief 获取最新的测距信号强度。 @return uint8_t 信号强度值。 */
    uint8_t getSignalStrength();

    /** @brief 获取最新的X轴光流速度 (归一化值)。 @note 实际速度(cm/s) = 返回值 * 高度(m)。 @return int16_t X轴光流速度 (cm/s @ 1m)。 */
    int16_t getFlowVelX_cms();

    /** @brief 获取最新的Y轴光流速度 (归一化值)。 @note 实际速度(cm/s) = 返回值 * 高度(m)。 @return int16_t Y轴光流速度 (cm/s @ 1m)。 */
    int16_t getFlowVelY_cms();

    /** @brief 获取最新的光流质量。 @return uint8_t 光流质量值，越大越可信。 */
    uint8_t getFlowQuality();

    /** @brief 获取 ToF 测距精度或置信度。 @return uint8_t 精度值。 */
    uint8_t getPrecision();

    /** @brief 获取 ToF 测距状态原始值。 @return uint8_t 1 表示数据可用。 */
    uint8_t getTofStatus();

    /** @brief 获取光流状态原始值。 @return uint8_t 1 表示数据可用。 */
    uint8_t getFlowStatus();

    /** @brief 获取当前帧的包序列号。 @return uint8_t 0~255 循环。 */
    uint8_t getSequence();

    /** @brief 获取当前帧的设备 ID。 @return uint8_t dev_id。 */
    uint8_t getDeviceId();

    /** @brief 获取当前帧的系统 ID。 @return uint8_t sys_id。 */
    uint8_t getSystemId();

    /** @brief 检查最新的测距数据是否有效。 @return bool 如果 tof_status==1 且 distance>=10mm，返回 true。 */
    bool isRangeDataValid();

    /** @brief 检查最新的光流数据是否有效。 @return bool 如果flow_status为1，返回true。 */
    bool isFlowDataValid();

    /** @brief 获取传感器发送的内部时间戳。 @return uint32_t 时间戳，单位为毫秒(ms)。 */
    uint32_t getTimestamp_ms();

    // --- 链路统计API ---

    /** @brief 获取链路统计信息。 @return MTF02P_Stats_t 统计结构体副本。 */
    MTF02P_Stats_t getStatistics();

    /** @brief 重置链路统计信息。 */
    void resetStatistics();

private:
    /**
     * @brief 内部核心解析函数，实现状态机逻辑。
     * @param data 从串口读取的单个字节。
     * @return bool 如果接收完一帧并且校验和正确，返回 true。
     */
    bool _parse_char(uint8_t data);

    Stream *_port;                            // 指向用户传入的串口对象的指针
    MicolinkMsg_t _msg;                       // 用于状态机解析的内部消息帧结构体
    MicolinkPayloadRangeSensor_t _sensorData; // 存储最新解析出的、有效传感器数据的结构体
    MTF02P_Stats_t _stats;                    // 链路统计信息
    uint8_t _last_seq;                        // 上一帧序列号，用于丢帧检测
    bool _has_last_seq;                       // 是否已收到过有效序列号
};

#endif // MTF02P_h
