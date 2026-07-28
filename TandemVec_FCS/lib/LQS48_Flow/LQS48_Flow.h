/**
 * =============================================================================
 * @file    LQS48_Flow.h
 * @brief   LQ-S48 光流测距一体模块 Arduino 驱动库 (完整修正版)
 * @author  基于武汉凌启科技官方协议实现
 * @version 2.0.0
 * @date    2025
 * =============================================================================
 *
 * 【产品信息】
 *   - 产品型号: LQ-S48
 *   - 产品名称: 光流测距一体定位模块
 *   - 制造商:   武汉凌启科技有限公司
 *
 * 【模块参数】(摘自手册第3页)
 *   - 尺寸:       60x20x10mm
 *   - 重量:       7g
 *   - 工作电压:   4.0V ~ 5.5V
 *   - 通信接口:   UART 串口 (TTL 3.3V)
 *   - 默认波特率: 115200
 *   - 数据帧率:   40~50Hz
 *   - 视场角:     20° (水平/垂直)
 *   - 环境照度:   >20Lux
 *   - 测距量程:   0.25m ~ 16m (精度2%~3%)
 *   - 光流精度:   1% (@1m, 0.5m/s)
 *   - 最大测速:   8m/s (@2m高度)
 *
 * 【协议说明】
 *   本库实现自定义协议(协议代码0x00)的解析。
 *   数据帧固定17字节，采用 CRC8-DVB-S2 校验。
 *   输出数据为帧间增量，使用时需累加。
 *
 * =============================================================================
 */

#ifndef LQS48_FLOW_H
#define LQS48_FLOW_H

#include <Arduino.h>

// =============================================================================
//                              常量与枚举定义
// =============================================================================

/**
 * @brief 数据质量/可信度标志 (手册第5页)
 *
 * 模块会自动评估数据可信度，用户应根据此标志决定是否使用数据。
 * 注意：可信标志码有很高准确率，但某些场景下仍有漏报，接收端需做额外拦截。
 */
enum LQS48_Quality
{
    LQS48_QUALITY_STRONG = 0,  ///< 强可信 - 数据可直接使用
    LQS48_QUALITY_WEAK = 1,    ///< 弱可信 - 数据可用但精度略低
    LQS48_QUALITY_BAD_XYR = 2, ///< X/Y/R 不可信 - 应丢弃光流数据，高度仍有效
    LQS48_QUALITY_BAD_H = 3,   ///< H 不可信 - 应丢弃高度数据，光流仍有效
    LQS48_QUALITY_BAD_ALL = 23 ///< 全部不可信 - 应丢弃所有数据
};

/**
 * @brief 通信协议类型 (手册第4页、第7页)
 *
 * 通过 setProtocol() 切换，切换后立即生效。
 */
enum LQS48_Protocol
{
    LQS48_PROTOCOL_CUSTOM = 0x00,      ///< 自定义协议 (本库使用)
    LQS48_PROTOCOL_MSP_V2 = 0x01,      ///< MSP_V2 协议 (INAV飞控)
    LQS48_PROTOCOL_MAVLINK_PX4 = 0x02, ///< MAVLINK 协议 (PX4飞控)
    LQS48_PROTOCOL_MAVLINK_APM = 0x03, ///< MAVLINK 协议 (APM飞控)
    LQS48_PROTOCOL_VOFA = 0x04         ///< VOFA+ 协议 (上位机调试)
};

/**
 * @brief 串口波特率选项 (手册第10页)
 *
 * 通过 setBaudRate() 设置，设置后需重启模块生效。
 */
enum LQS48_BaudRate
{
    LQS48_BAUD_19200 = 0x01,  ///< 19200 bps
    LQS48_BAUD_38400 = 0x02,  ///< 38400 bps
    LQS48_BAUD_57600 = 0x03,  ///< 57600 bps
    LQS48_BAUD_115200 = 0x04, ///< 115200 bps (默认)
    LQS48_BAUD_460800 = 0x05, ///< 460800 bps
    LQS48_BAUD_921600 = 0x06  ///< 921600 bps
};

// =============================================================================
//                              数据结构定义
// =============================================================================

/**
 * @brief 单帧解析数据结构体
 *
 * 存储从模块接收并解析后的一帧数据。
 * 注意：flow_x_rad、flow_y_rad、flow_rot_deg 都是【帧间增量】，不是累计值。
 */
typedef struct
{
    // -------- 光流数据 (帧间增量) --------
    float flow_x_rad;   ///< X轴角位移增量 (单位: 弧度, rad)
    float flow_y_rad;   ///< Y轴角位移增量 (单位: 弧度, rad)
    float flow_rot_deg; ///< 旋转角增量 (单位: 度, °)

    // -------- 测距数据 --------
    uint32_t height_mm; ///< 对地高度 (单位: 毫米, mm)

    // -------- 状态信息 --------
    uint8_t quality;      ///< 数据可信标志 (见 LQS48_Quality 枚举)
    uint8_t lux;          ///< 环境光照强度 (范围: 0~100, 越大光照越强)
    uint8_t dt_flow_ms;   ///< 光流数据时间间隔 (单位: ms, 用于与IMU对齐时间戳)
    uint8_t dt_height_ms; ///< 高度数据时间间隔 (单位: ms, 用于与IMU对齐时间戳)
    uint8_t sequence;     ///< 数据包序列号 (范围: 0~255, 用于检测丢帧)

} LQS48_Data_t;

/**
 * @brief 累加数据结构体
 *
 * 存储从起始点开始累加的总位移。
 * 使用 double 类型以保证长时间累加的精度。
 */
typedef struct
{
    double x_rad;   ///< X轴累计角位移 (弧度)
    double y_rad;   ///< Y轴累计角位移 (弧度)
    double rot_deg; ///< 累计旋转角 (度)
    double x_mm;    ///< X轴累计线位移 (毫米) - 已乘以高度换算
    double y_mm;    ///< Y轴累计线位移 (毫米) - 已乘以高度换算

} LQS48_Accum_t;

/**
 * @brief 统计信息结构体
 *
 * 用于监控通信质量和数据可靠性。
 */
typedef struct
{
    uint32_t valid_frames;   ///< 校验通过的有效帧数
    uint32_t error_frames;   ///< 校验失败的错误帧数
    uint32_t lost_frames;    ///< 检测到的丢帧数量
    uint32_t invalid_flow;   ///< 光流数据不可信的帧数 (quality=2或23)
    uint32_t invalid_height; ///< 高度数据不可信的帧数 (quality=3或23)

} LQS48_Stats_t;

// =============================================================================
//                              主类定义
// =============================================================================

/**
 * @class LQS48_Flow
 * @brief LQ-S48 光流测距模块驱动类
 *
 * 【使用步骤】
 * 1. 创建实例: LQS48_Flow sensor;
 * 2. 初始化:   sensor.begin(Serial1);
 * 3. 主循环:   if(sensor.update()) { 处理数据 }
 *
 * 【注意事项】(摘自手册第23页)
 * - 地面纹理弱、光照过暗会影响精度
 * - 快速倾斜/晃动时数据会短暂波动
 * - 建议在中断中接收数据，避免丢帧
 * - 累加数据时注意精度问题
 */
class LQS48_Flow
{
public:
    // =========================================================================
    //                          构造与初始化
    // =========================================================================

    /**
     * @brief 构造函数
     */
    LQS48_Flow();

    /**
     * @brief 初始化模块
     * @param serial 串口对象的引用 (如 Serial1, Serial2)
     *
     * 【示例】
     * @code
     * Serial1.begin(115200);  // 先初始化串口
     * sensor.begin(Serial1);  // 再绑定到驱动
     * @endcode
     */
    void begin(Stream &serial);

    // =========================================================================
    //                          数据更新 (核心函数)
    // =========================================================================

    /**
     * @brief 更新数据 - 必须在主循环中频繁调用
     * @return true  成功解析出一帧新数据
     * @return false 数据未就绪或校验失败
     *
     * 【工作原理】
     * 采用非阻塞状态机解析，每次调用处理当前串口缓冲区中的数据，
     * 不会阻塞主循环。建议调用频率 >= 100Hz。
     *
     * 【示例】
     * @code
     * void loop() {
     *     if (sensor.update()) {
     *         // 有新数据，进行处理
     *     }
     *     // 其他任务...
     * }
     * @endcode
     */
    bool update();

    // =========================================================================
    //                          单帧数据获取
    // =========================================================================

    /**
     * @brief 获取最新一帧的完整数据结构
     * @return LQS48_Data_t 数据结构体副本
     */
    LQS48_Data_t getData() const;

    /**
     * @brief 获取 X 轴光流角位移增量 (弧度)
     * @note 这是单帧增量，需要累加才能得到总位移
     */
    float getFlowX_rad() const { return _data.flow_x_rad; }

    /**
     * @brief 获取 Y 轴光流角位移增量 (弧度)
     */
    float getFlowY_rad() const { return _data.flow_y_rad; }

    /**
     * @brief 获取旋转角增量 (度)
     */
    float getRotation_deg() const { return _data.flow_rot_deg; }

    /**
     * @brief 获取对地高度 (毫米)
     * @note 量程 250mm ~ 16000mm，超出量程输出不大于30000mm
     */
    uint32_t getHeight_mm() const { return _data.height_mm; }

    /**
     * @brief 获取数据质量标志
     * @return 见 LQS48_Quality 枚举
     */
    uint8_t getQuality() const { return _data.quality; }

    /**
     * @brief 获取环境光照强度
     * @return 0~100，数值越大光照越强
     * @note 可用于判断是否需要开启底部补光灯
     */
    uint8_t getLux() const { return _data.lux; }

    /**
     * @brief 获取光流数据的时间间隔 (毫秒)
     * @note 用于与 IMU 数据对齐时间戳，或计算角速度
     */
    uint8_t getDtFlow_ms() const { return _data.dt_flow_ms; }

    /**
     * @brief 获取高度数据的时间间隔 (毫秒)
     */
    uint8_t getDtHeight_ms() const { return _data.dt_height_ms; }

    /**
     * @brief 获取数据包序列号
     * @return 0~255，每帧递增1，溢出后归零
     */
    uint8_t getSequence() const { return _data.sequence; }

    // =========================================================================
    //                          单帧换算数据
    // =========================================================================

    /**
     * @brief 获取 X 轴线位移增量 (毫米)
     * @return 位移 = flow_x_rad × height_mm
     * @note 仅当高度数据有效时结果才准确
     */
    float getDisplacementX_mm() const;

    /**
     * @brief 获取 Y 轴线位移增量 (毫米)
     */
    float getDisplacementY_mm() const;

    /**
     * @brief 获取 X 轴角速度 (弧度/秒)
     * @return 角速度 = flow_x_rad / (dt_flow_ms / 1000)
     */
    float getAngularVelX_rad_s() const;

    /**
     * @brief 获取 Y 轴角速度 (弧度/秒)
     */
    float getAngularVelY_rad_s() const;

    /**
     * @brief 获取 X 轴线速度 (毫米/秒)
     * @return 线速度 = (flow_x_rad × height_mm) / (dt_flow_ms / 1000)
     * @note 仅当光流和高度都有效时结果才准确
     */
    float getVelocityX_mm_s() const;

    /**
     * @brief 获取 Y 轴线速度 (毫米/秒)
     */
    float getVelocityY_mm_s() const;

    /**
     * @brief 获取 X 轴线速度 (米/秒)
     */
    float getVelocityX_m_s() const;

    /**
     * @brief 获取 Y 轴线速度 (米/秒)
     */
    float getVelocityY_m_s() const;

    /**
     * @brief 获取合成线速度 (毫米/秒)
     * @return sqrt(vx² + vy²)
     */
    float getVelocity_mm_s() const;

    /**
     * @brief 获取合成线速度 (米/秒)
     */
    float getVelocity_m_s() const;

    /**
     * @brief 获取旋转角速度 (度/秒)
     */
    float getRotationVel_deg_s() const;

    // =========================================================================
    //                          累加数据获取
    // =========================================================================

    /**
     * @brief 获取累加数据结构
     * @return LQS48_Accum_t 累加数据结构体副本
     */
    LQS48_Accum_t getAccumData() const;

    /**
     * @brief 获取 X 轴累计角位移 (弧度)
     * @note 仅累加 quality=0或1 的有效数据
     */
    double getAccumX_rad() const { return _accum.x_rad; }

    /**
     * @brief 获取 Y 轴累计角位移 (弧度)
     */
    double getAccumY_rad() const { return _accum.y_rad; }

    /**
     * @brief 获取累计旋转角 (度)
     */
    double getAccumRot_deg() const { return _accum.rot_deg; }

    /**
     * @brief 获取 X 轴累计线位移 (毫米)
     * @note 每帧使用当时的高度换算后累加
     */
    double getAccumX_mm() const { return _accum.x_mm; }

    /**
     * @brief 获取 Y 轴累计线位移 (毫米)
     */
    double getAccumY_mm() const { return _accum.y_mm; }

    /**
     * @brief 重置所有累加值为零
     * @note 通常在起飞前或返回起点时调用
     */
    void resetAccumulators();

    // =========================================================================
    //                          数据有效性判断
    // =========================================================================

    /**
     * @brief 判断当前光流数据是否有效
     * @return true  quality=0或1，数据可用
     * @return false quality=2或23，数据不可信应丢弃
     */
    bool isFlowValid() const;

    /**
     * @brief 判断当前高度数据是否有效
     * @return true  quality=0、1或2，数据可用
     * @return false quality=3或23，数据不可信应丢弃
     */
    bool isHeightValid() const;

    /**
     * @brief 判断所有数据是否都有效
     * @return true  quality=0或1
     */
    bool isAllValid() const;

    // =========================================================================
    //                          统计信息
    // =========================================================================

    /**
     * @brief 获取统计信息结构
     */
    LQS48_Stats_t getStatistics() const;

    /**
     * @brief 获取有效帧计数
     */
    uint32_t getValidFrames() const { return _stats.valid_frames; }

    /**
     * @brief 获取错误帧计数 (校验失败)
     */
    uint32_t getErrorFrames() const { return _stats.error_frames; }

    /**
     * @brief 获取丢帧计数
     */
    uint32_t getLostFrames() const { return _stats.lost_frames; }

    /**
     * @brief 计算丢帧率 (百分比)
     * @return 丢帧数 / (有效帧数 + 丢帧数) × 100
     */
    float getLostRate() const;

    /**
     * @brief 重置所有统计计数
     */
    void resetStatistics();

    // =========================================================================
    //                          模块配置指令
    // =========================================================================

    /**
     * @brief 执行高度校准 (手册第7-9页)
     * @param actual_height_mm 当前模块到地面的实际距离 (毫米)
     *
     * 【使用场景】
     * - 安装后首次使用
     * - 发现高度数据偏差较大时
     *
     * 【校准步骤】
     * 1. 将模块正对有纹理的平面
     * 2. 测量模块到平面的实际距离
     * 3. 调用此函数发送校准指令
     * 4. 等待约5秒，收到 AB BC EE 表示成功
     *
     * 【示例】
     * @code
     * sensor.calibrateHeight(5500);  // 校准距离5.5米
     * @endcode
     */
    void calibrateHeight(uint32_t actual_height_mm);

    /**
     * @brief 设置串口波特率 (手册第10页)
     * @param baud 波特率选项 (见 LQS48_BaudRate 枚举)
     *
     * @warning 设置后需要重启模块才能生效！
     *          同时需要修改 MCU 端的串口波特率。
     */
    void setBaudRate(LQS48_BaudRate baud);

    /**
     * @brief 切换通信协议 (手册第7页)
     * @param protocol 协议类型 (见 LQS48_Protocol 枚举)
     *
     * @note 切换后立即生效。
     *       切换到非自定义协议后，本库将无法解析数据。
     */
    void setProtocol(LQS48_Protocol protocol);

    // =========================================================================
    //                          调试辅助
    // =========================================================================

    /**
     * @brief 获取原始数据缓冲区 (用于调试)
     * @param buf 输出缓冲区，至少17字节
     */
    void getRawBuffer(uint8_t *buf) const;

private:
    // =========================================================================
    //                          私有成员变量
    // =========================================================================

    Stream *_serial; ///< 串口指针

    LQS48_Data_t _data;   ///< 当前帧数据
    LQS48_Accum_t _accum; ///< 累加数据
    LQS48_Stats_t _stats; ///< 统计信息

    uint8_t _buffer[20];     ///< 接收缓冲区
    uint8_t _state;          ///< 状态机当前状态
    uint8_t _bytes_received; ///< 已接收字节数
    uint8_t _last_sequence;  ///< 上一帧序列号 (用于丢帧检测)

    // =========================================================================
    //                          协议常量
    // =========================================================================

    static const uint8_t HEADER_1 = 0xFA; ///< 帧头第1字节
    static const uint8_t HEADER_2 = 0xAA; ///< 帧头第2字节
    static const uint8_t FRAME_LEN = 17;  ///< 帧总长度

    // 状态机状态定义
    static const uint8_t STATE_WAIT_HEADER_1 = 0; ///< 等待帧头1
    static const uint8_t STATE_WAIT_HEADER_2 = 1; ///< 等待帧头2
    static const uint8_t STATE_RECV_PAYLOAD = 2;  ///< 接收数据体

    // =========================================================================
    //                          私有成员函数
    // =========================================================================

    /**
     * @brief CRC8-DVB-S2 校验 (官方算法)
     * @param buf 数据缓冲区
     * @param len 帧长度 (必须为17)
     * @return true 校验通过
     */
    bool verifyChecksum(const uint8_t *buf, uint8_t len);

    /**
     * @brief 解析缓冲区数据
     */
    void parseBuffer();

    /**
     * @brief 更新累加数据
     */
    void updateAccumulators();

    /**
     * @brief 检测丢帧
     */
    void checkLostFrames();
};

#endif // LQS48_FLOW_H