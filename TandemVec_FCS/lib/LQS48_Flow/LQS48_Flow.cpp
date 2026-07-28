/**
 * =============================================================================
 * @file    LQS48_Flow.cpp
 * @brief   LQ-S48 光流测距一体模块 Arduino 驱动库 - 实现文件
 * @version 2.0.0
 * =============================================================================
 *
 * 【自定义协议帧格式】(手册第4-5页)
 *
 *   字节序号    内容          说明
 *   ────────────────────────────────────────────────────
 *   Byte 0     0xFA          帧头1
 *   Byte 1     0xAA          帧头2
 *   Byte 2     X_H           X轴位移 高8位 ┐
 *   Byte 3     X_L           X轴位移 低8位 ┘ 有符号16位, ÷200000 得弧度
 *   Byte 4     Y_H           Y轴位移 高8位 ┐
 *   Byte 5     Y_L           Y轴位移 低8位 ┘ 有符号16位, ÷200000 得弧度
 *   Byte 6     R_H           旋转角 高8位  ┐
 *   Byte 7     R_L           旋转角 低8位  ┘ 有符号16位, ÷1000 得度
 *   Byte 8     H_H           高度 高8位   ┐
 *   Byte 9     H_M           高度 中8位   │ 无符号24位, 单位mm
 *   Byte 10    H_L           高度 低8位   ┘
 *   Byte 11    Quality       数据可信标志 (0/1/2/3/23)
 *   Byte 12    Lux           光照信息 (0~100)
 *   Byte 13    dt_flow       光流时间间隔 (ms)
 *   Byte 14    dt_height     高度时间间隔 (ms)
 *   Byte 15    Sequence      包序列号 (0~255)
 *   Byte 16    CRC           校验位 (CRC8-DVB-S2, 计算Byte2~15)
 *
 * =============================================================================
 */

#include "LQS48_Flow.h"

// =============================================================================
//                      CRC8-DVB-S2 校验算法 (官方提供)
// =============================================================================

/**
 * @brief CRC8-DVB-S2 单字节计算
 * @param crc 当前CRC值
 * @param a   输入字节
 * @return    更新后的CRC值
 *
 * 算法参数:
 *   - 多项式: 0xD5
 *   - 初始值: 0x00
 *   - 无反转
 */
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc = crc << 1;
        }
    }
    return crc;
}

// =============================================================================
//                              构造函数
// =============================================================================

LQS48_Flow::LQS48_Flow()
{
    _serial = nullptr;

    // 状态机初始化
    _state = STATE_WAIT_HEADER_1;
    _bytes_received = 0;
    _last_sequence = 0xFF; // 0xFF 表示尚未接收到任何数据

    // 清零数据结构
    memset(&_data, 0, sizeof(_data));
    memset(&_accum, 0, sizeof(_accum));
    memset(&_stats, 0, sizeof(_stats));
    memset(_buffer, 0, sizeof(_buffer));

    // 默认数据质量设为不可信
    _data.quality = LQS48_QUALITY_BAD_ALL;
}

// =============================================================================
//                              初始化
// =============================================================================

void LQS48_Flow::begin(Stream &serial)
{
    _serial = &serial;
}

// =============================================================================
//                          核心更新函数 (状态机)
// =============================================================================

/**
 * @brief 非阻塞数据更新
 *
 * 【状态机流程】
 *
 *   ┌─────────────────┐
 *   │ WAIT_HEADER_1   │◄────── 初始状态 / 帧结束
 *   └────────┬────────┘
 *            │ 收到 0xFA
 *            ▼
 *   ┌─────────────────┐
 *   │ WAIT_HEADER_2   │
 *   └────────┬────────┘
 *            │ 收到 0xAA        │ 收到其他
 *            ▼                  └──► 回到 WAIT_HEADER_1
 *   ┌─────────────────┐
 *   │ RECV_PAYLOAD    │
 *   └────────┬────────┘
 *            │ 接收满15字节 (Byte2~16)
 *            ▼
 *        校验 CRC
 *            │
 *      ┌─────┴─────┐
 *      │           │
 *    通过        失败
 *      │           │
 *      ▼           ▼
 *   解析数据    统计错误
 *      │           │
 *      └─────┬─────┘
 *            ▼
 *     回到 WAIT_HEADER_1
 */
bool LQS48_Flow::update()
{
    // 检查串口是否已初始化
    if (_serial == nullptr)
    {
        return false;
    }

    // 处理所有可用的串口数据
    while (_serial->available() > 0)
    {
        uint8_t byte = _serial->read();

        switch (_state)
        {
        // ─────────────────────────────────────────────────
        // 状态0: 等待帧头第1字节 (0xFA)
        // ─────────────────────────────────────────────────
        case STATE_WAIT_HEADER_1:
            if (byte == HEADER_1)
            {
                _buffer[0] = byte;
                _state = STATE_WAIT_HEADER_2;
            }
            // 非帧头字节直接丢弃，继续等待
            break;

        // ─────────────────────────────────────────────────
        // 状态1: 等待帧头第2字节 (0xAA)
        // ─────────────────────────────────────────────────
        case STATE_WAIT_HEADER_2:
            if (byte == HEADER_2)
            {
                _buffer[1] = byte;
                _bytes_received = 2; // 已有2字节
                _state = STATE_RECV_PAYLOAD;
            }
            else
            {
                // 帧头不匹配，可能是噪声或上一帧的数据
                // 检查是否为新帧头的开始
                if (byte == HEADER_1)
                {
                    _buffer[0] = byte;
                    // 保持在 STATE_WAIT_HEADER_2
                }
                else
                {
                    _state = STATE_WAIT_HEADER_1; // 重新开始
                }
            }
            break;

        // ─────────────────────────────────────────────────
        // 状态2: 接收数据体 (Byte2 ~ Byte16)
        // ─────────────────────────────────────────────────
        case STATE_RECV_PAYLOAD:
            _buffer[_bytes_received] = byte;
            _bytes_received++;

            // 检查是否接收完整帧 (17字节)
            if (_bytes_received >= FRAME_LEN)
            {
                // 重置状态，准备接收下一帧
                _state = STATE_WAIT_HEADER_1;

                // 校验数据
                if (verifyChecksum(_buffer, FRAME_LEN))
                {
                    // 校验通过，解析数据
                    parseBuffer();
                    checkLostFrames();
                    updateAccumulators();

                    _stats.valid_frames++;
                    return true; // 成功解析一帧
                }
                else
                {
                    // 校验失败
                    _stats.error_frames++;
                    return false;
                }
            }
            break;

        // ─────────────────────────────────────────────────
        // 异常状态处理
        // ─────────────────────────────────────────────────
        default:
            _state = STATE_WAIT_HEADER_1;
            _bytes_received = 0;
            break;
        }
    }

    // 本次调用未能完成一帧的解析
    return false;
}

// =============================================================================
//                              校验函数
// =============================================================================

bool LQS48_Flow::verifyChecksum(const uint8_t *buf, uint8_t len)
{
    /*
     * 根据官方源码 (附件1):
     *   - 对 Byte2 ~ Byte15 (共14字节) 计算 CRC8-DVB-S2
     *   - 结果与 Byte16 比较
     */
    uint8_t crc = 0;
    for (uint8_t i = 2; i < 16; ++i)
    {
        crc = crc8_dvb_s2(crc, buf[i]);
    }
    return (crc == buf[16]);
}

// =============================================================================
//                              数据解析
// =============================================================================

void LQS48_Flow::parseBuffer()
{
    /*
     * 【数据转换公式】(手册第4-5页)
     *
     * X轴位移: (int16_t)(Byte2<<8 | Byte3) / 200000.0  → 弧度
     * Y轴位移: (int16_t)(Byte4<<8 | Byte5) / 200000.0  → 弧度
     * 旋转角:  (int16_t)(Byte6<<8 | Byte7) / 1000.0    → 度
     * 高度:    (uint32_t)(Byte8<<16 | Byte9<<8 | Byte10) → 毫米
     */

    // ─────────────────────────────────────────────────────
    // 1. 解析 X 轴光流 (Byte 2-3)
    // ─────────────────────────────────────────────────────
    int16_t raw_x = (int16_t)(((uint16_t)_buffer[2] << 8) | (uint16_t)_buffer[3]);
    _data.flow_x_rad = (float)raw_x / 200000.0f;

    // ─────────────────────────────────────────────────────
    // 2. 解析 Y 轴光流 (Byte 4-5)
    // ─────────────────────────────────────────────────────
    int16_t raw_y = (int16_t)(((uint16_t)_buffer[4] << 8) | (uint16_t)_buffer[5]);
    _data.flow_y_rad = (float)raw_y / 200000.0f;

    // ─────────────────────────────────────────────────────
    // 3. 解析旋转角 R (Byte 6-7)
    // ─────────────────────────────────────────────────────
    int16_t raw_r = (int16_t)(((uint16_t)_buffer[6] << 8) | (uint16_t)_buffer[7]);
    _data.flow_rot_deg = (float)raw_r / 1000.0f;

    // ─────────────────────────────────────────────────────
    // 4. 解析高度 H (Byte 8-10, 24位无符号)
    // ─────────────────────────────────────────────────────
    _data.height_mm = ((uint32_t)_buffer[8] << 16) |
                      ((uint32_t)_buffer[9] << 8) |
                      ((uint32_t)_buffer[10]);

    // ─────────────────────────────────────────────────────
    // 5. 解析状态信息 (Byte 11-15)
    // ─────────────────────────────────────────────────────
    _data.quality = _buffer[11];
    _data.lux = _buffer[12];
    _data.dt_flow_ms = _buffer[13];
    _data.dt_height_ms = _buffer[14];
    _data.sequence = _buffer[15];

    // ─────────────────────────────────────────────────────
    // 6. 更新无效数据统计
    // ─────────────────────────────────────────────────────
    if (!isFlowValid())
    {
        _stats.invalid_flow++;
    }
    if (!isHeightValid())
    {
        _stats.invalid_height++;
    }
}

// =============================================================================
//                              丢帧检测
// =============================================================================

void LQS48_Flow::checkLostFrames()
{
    /*
     * 序列号为 0~255 循环递增。
     * 如果当前序列号不等于 (上一序列号+1)%256，则发生丢帧。
     */
    if (_last_sequence != 0xFF) // 非首次接收
    {
        uint8_t expected = (_last_sequence + 1) & 0xFF;
        if (_data.sequence != expected)
        {
            // 计算丢失的帧数
            uint8_t lost_count;
            if (_data.sequence > expected)
            {
                lost_count = _data.sequence - expected;
            }
            else
            {
                // 序列号回绕的情况
                lost_count = (256 - expected) + _data.sequence;
            }
            _stats.lost_frames += lost_count;
        }
    }
    _last_sequence = _data.sequence;
}

// =============================================================================
//                              累加数据更新
// =============================================================================

void LQS48_Flow::updateAccumulators()
{
    /*
     * 【累加规则】(手册第5页备注)
     *
     * - X、Y、R 三个数据需要累加
     * - 由于每次累加的数据很小，要注意数据精度问题
     * - 本库使用 double 类型累加以保证精度
     * - 仅累加 quality=0或1 的有效数据
     */

    // 只累加有效的光流数据
    if (isFlowValid())
    {
        // 累加角位移 (弧度)
        _accum.x_rad += (double)_data.flow_x_rad;
        _accum.y_rad += (double)_data.flow_y_rad;
        _accum.rot_deg += (double)_data.flow_rot_deg;

        // 累加线位移 (毫米) - 每帧使用当时的高度换算
        // 公式: 位移(mm) = 弧度 × 高度(mm)
        if (isHeightValid())
        {
            _accum.x_mm += (double)_data.flow_x_rad * (double)_data.height_mm;
            _accum.y_mm += (double)_data.flow_y_rad * (double)_data.height_mm;
        }
    }
}

// =============================================================================
//                              数据获取接口
// =============================================================================

LQS48_Data_t LQS48_Flow::getData() const
{
    return _data;
}

LQS48_Accum_t LQS48_Flow::getAccumData() const
{
    return _accum;
}

LQS48_Stats_t LQS48_Flow::getStatistics() const
{
    return _stats;
}

// =============================================================================
//                              换算数据接口
// =============================================================================

float LQS48_Flow::getDisplacementX_mm() const
{
    // 位移 = 弧度 × 高度
    return _data.flow_x_rad * (float)_data.height_mm;
}

float LQS48_Flow::getDisplacementY_mm() const
{
    return _data.flow_y_rad * (float)_data.height_mm;
}

float LQS48_Flow::getAngularVelX_rad_s() const
{
    // 角速度 = 角位移 / 时间
    if (_data.dt_flow_ms == 0)
    {
        return 0.0f;
    }
    return _data.flow_x_rad / ((float)_data.dt_flow_ms / 1000.0f);
}

float LQS48_Flow::getAngularVelY_rad_s() const
{
    if (_data.dt_flow_ms == 0)
    {
        return 0.0f;
    }
    return _data.flow_y_rad / ((float)_data.dt_flow_ms / 1000.0f);
}

// =============================================================================
//                              线速度计算
// =============================================================================

float LQS48_Flow::getVelocityX_mm_s() const
{
    /*
     * 【线速度计算公式】
     *
     * 线速度 = 线位移 / 时间
     *        = (角位移 × 高度) / 时间
     *        = 角速度 × 高度
     *
     * 单位换算:
     *   角速度: rad/s
     *   高度:   mm
     *   线速度: mm/s
     */
    if (_data.dt_flow_ms == 0)
    {
        return 0.0f;
    }

    // 方法: 线位移 / 时间
    float displacement_mm = _data.flow_x_rad * (float)_data.height_mm;
    float dt_s = (float)_data.dt_flow_ms / 1000.0f;

    return displacement_mm / dt_s;
}

float LQS48_Flow::getVelocityY_mm_s() const
{
    if (_data.dt_flow_ms == 0)
    {
        return 0.0f;
    }

    float displacement_mm = _data.flow_y_rad * (float)_data.height_mm;
    float dt_s = (float)_data.dt_flow_ms / 1000.0f;

    return displacement_mm / dt_s;
}

float LQS48_Flow::getVelocityX_m_s() const
{
    return getVelocityX_mm_s() / 1000.0f;
}

float LQS48_Flow::getVelocityY_m_s() const
{
    return getVelocityY_mm_s() / 1000.0f;
}

float LQS48_Flow::getVelocity_mm_s() const
{
    float vx = getVelocityX_mm_s();
    float vy = getVelocityY_mm_s();
    return sqrtf(vx * vx + vy * vy);
}

float LQS48_Flow::getVelocity_m_s() const
{
    return getVelocity_mm_s() / 1000.0f;
}

float LQS48_Flow::getRotationVel_deg_s() const
{
    if (_data.dt_flow_ms == 0)
    {
        return 0.0f;
    }
    return _data.flow_rot_deg / ((float)_data.dt_flow_ms / 1000.0f);
}

// =============================================================================
//                              有效性判断
// =============================================================================

bool LQS48_Flow::isFlowValid() const
{
    /*
     * 光流数据有效条件:
     *   - quality = 0 (强可信)
     *   - quality = 1 (弱可信)
     *
     * 光流数据无效:
     *   - quality = 2  (X/Y/R 不可信)
     *   - quality = 23 (全部不可信)
     */
    return (_data.quality == LQS48_QUALITY_STRONG ||
            _data.quality == LQS48_QUALITY_WEAK);
}

bool LQS48_Flow::isHeightValid() const
{
    /*
     * 高度数据有效条件:
     *   - quality = 0 (强可信)
     *   - quality = 1 (弱可信)
     *   - quality = 2 (光流不可信，但高度有效)
     *
     * 高度数据无效:
     *   - quality = 3  (H 不可信)
     *   - quality = 23 (全部不可信)
     */
    return (_data.quality != LQS48_QUALITY_BAD_H &&
            _data.quality != LQS48_QUALITY_BAD_ALL);
}

bool LQS48_Flow::isAllValid() const
{
    return (_data.quality == LQS48_QUALITY_STRONG ||
            _data.quality == LQS48_QUALITY_WEAK);
}

// =============================================================================
//                              统计信息
// =============================================================================

float LQS48_Flow::getLostRate() const
{
    uint32_t total = _stats.valid_frames + _stats.lost_frames;
    if (total == 0)
    {
        return 0.0f;
    }
    return (float)_stats.lost_frames / (float)total * 100.0f;
}

void LQS48_Flow::resetStatistics()
{
    memset(&_stats, 0, sizeof(_stats));
    _last_sequence = 0xFF;
}

void LQS48_Flow::resetAccumulators()
{
    memset(&_accum, 0, sizeof(_accum));
}

// =============================================================================
//                              模块配置指令
// =============================================================================

void LQS48_Flow::calibrateHeight(uint32_t actual_height_mm)
{
    /*
     * 【高度校准指令格式】(手册第9页)
     *
     *   Byte 0-1: 0xAA 0xAA   指令头
     *   Byte 2-3: 0xAB 0xBC   指令标识
     *   Byte 4:   高度高8位
     *   Byte 5:   高度中8位
     *   Byte 6:   高度低8位
     *
     * 示例: 校准距离 5500mm (0x00157C)
     *       发送: AA AA AB BC 00 15 7C
     *
     * 应答:
     *   AB BC EE → 校准成功
     *   AB BC 11 → 心跳包 (校准中)
     *   AB BC 02 → 校准失败
     */

    if (_serial == nullptr)
    {
        return;
    }

    uint8_t cmd[7];
    cmd[0] = 0xAA;
    cmd[1] = 0xAA;
    cmd[2] = 0xAB;
    cmd[3] = 0xBC;
    cmd[4] = (actual_height_mm >> 16) & 0xFF; // 高8位
    cmd[5] = (actual_height_mm >> 8) & 0xFF;  // 中8位
    cmd[6] = actual_height_mm & 0xFF;         // 低8位

    _serial->write(cmd, 7);
}

void LQS48_Flow::setBaudRate(LQS48_BaudRate baud)
{
    /*
     * 【波特率设置指令格式】(手册第10页)
     *
     *   Byte 0-1: 0xAA 0xAA   指令头
     *   Byte 2-3: 0x9A 0xAB   指令标识
     *   Byte 4:   波特率索引
     *
     * 应答:
     *   9A AB EE XX → 设置成功
     *   9A AB 00    → 输入不合法
     *   9A AB 01    → 写入失败
     *
     * @warning 设置后需重启模块生效！
     */

    if (_serial == nullptr)
    {
        return;
    }

    uint8_t cmd[5];
    cmd[0] = 0xAA;
    cmd[1] = 0xAA;
    cmd[2] = 0x9A;
    cmd[3] = 0xAB;
    cmd[4] = (uint8_t)baud;

    _serial->write(cmd, 5);
}

void LQS48_Flow::setProtocol(LQS48_Protocol protocol)
{
    /*
     * 【协议切换指令格式】(手册第7页)
     *
     *   Byte 0-1: 0xAA 0xAA   指令头
     *   Byte 2-3: 0xCD 0xDE   指令标识
     *   Byte 4:   协议代码
     *             0x00 = 自定义协议
     *             0x01 = MSP_V2 (INAV)
     *             0x02 = MAVLINK_PX4
     *             0x03 = MAVLINK_APM
     *             0x04 = VOFA+
     *
     * 应答:
     *   CD DE EE XX → 设置成功
     *   CD DE 00    → 输入不合法
     *   CD DE 01    → 写入失败
     */

    if (_serial == nullptr)
    {
        return;
    }

    uint8_t cmd[5];
    cmd[0] = 0xAA;
    cmd[1] = 0xAA;
    cmd[2] = 0xCD;
    cmd[3] = 0xDE;
    cmd[4] = (uint8_t)protocol;

    _serial->write(cmd, 5);
}

// =============================================================================
//                              调试辅助
// =============================================================================

void LQS48_Flow::getRawBuffer(uint8_t *buf) const
{
    if (buf != nullptr)
    {
        memcpy(buf, _buffer, FRAME_LEN);
    }
}