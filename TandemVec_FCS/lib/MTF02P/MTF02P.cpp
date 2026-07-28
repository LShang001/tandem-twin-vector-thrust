/**
 * @file MTF02P.cpp
 * @author LShang
 * @brief MTF-02P光流测距一体传感器Arduino库的实现文件。
 * @version 1.1
 * @date 2025-7-12
 *
 * @copyright Copyright (c) 2025
 *
 * @details
 *  此文件包含了 MTF02P 类中所有方法的具体实现，
 *  核心是 `_parse_char` 函数中实现的有限状态机（FSM），
 *  它负责高效地解析来自传感器的Micolink协议字节流。
 */

#include "MTF02P.h"

/**
 * @brief MTF02P类构造函数。
 * @details 在对象创建时被调用，负责初始化内部成员变量，
 *          特别是将状态机置于初始状态。
 */
MTF02P::MTF02P()
{
    // 将串口指针初始化为空，表示尚未通过begin()方法进行配置
    _port = nullptr;
    // 将状态机状态置为0 (等待帧头)
    _msg.status = 0;
    // 将负载计数器清零
    _msg.payload_cnt = 0;
}

/**
 * @brief 初始化MTF02P库。
 * @param port 传入一个Stream对象的引用，例如 Serial1。
 * @details 保存用户传入的串口对象的指针，以便后续进行读写操作。
 */
void MTF02P::begin(Stream &port)
{
    // 保存传入的Stream对象的地址
    _port = &port;
}

/**
 * @brief 更新函数，从串口读取并解析数据。
 * @details 此函数应在Arduino的 `loop()` 中被持续调用。
 *          它会读取串口缓冲区中的所有可用字节，并驱动状态机进行解析。
 * @return bool 当且仅当接收并校验通过一帧完整的新数据时，返回true。
 */
bool MTF02P::update()
{
    // 安全检查：如果begin()未被调用，则不执行任何操作
    if (_port == nullptr)
    {
        return false;
    }

    // 循环处理串口接收缓冲区中的所有字节，确保不丢失数据
    while (_port->available())
    {
        // 从串口读取一个字节
        uint8_t data = _port->read();

        // 将读取到的字节喂给状态机进行解析
        if (_parse_char(data))
        {
            // 如果_parse_char返回true，表示一帧数据接收完毕且校验正确
            // 检查消息ID是否是我们关心的传感器数据
            if (_msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR)
            {
                // 使用memcpy将payload字节流安全地转换为结构体
                // 这是安全且正确的做法，因为我们在头文件中使用了 #pragma pack(1)
                // 来保证_sensorData结构体的内存布局与payload字节流完全一致。
                memcpy(&_sensorData, _msg.payload, _msg.len);
                return true; // 成功接收到新数据，通知主程序
            }
        }
    }

    // 如果循环结束仍未解析出完整数据帧，则返回false
    return false;
}

/**
 * @brief 核心状态机解析函数 (私有)。
 * @details 此函数实现了Micolink协议的字节流解析逻辑。
 *          每接收一个字节，状态就可能发生一次转换。
 *          优化点：采用动态计算校验和的方式，在接收每个字节的同时进行累加，
 *          避免了在帧末尾进行内存拷贝和循环计算，提高了效率。
 * @param data 从串口读取的单个字节。
 * @return bool 当接收完一帧并且校验和正确时，返回 true。
 */
bool MTF02P::_parse_char(uint8_t data)
{
    // 使用switch语句实现有限状态机 (FSM)
    switch (_msg.status)
    {
    case 0: // 状态0: 等待帧头 (0xEF)
        if (data == MICOLINK_MSG_HEAD)
        {
            _msg.head = data;
            _msg.running_checksum = data; // 初始化动态校验和，帧头也参与计算
            _msg.status++;                // 状态转移到下一状态
        }
        break;

    case 1: // 状态1: 接收设备ID
        _msg.dev_id = data;
        _msg.running_checksum += data; // 更新动态校验和
        _msg.status++;
        break;

    case 2: // 状态2: 接收系统ID
        _msg.sys_id = data;
        _msg.running_checksum += data;
        _msg.status++;
        break;

    case 3: // 状态3: 接收消息ID
        _msg.msg_id = data;
        _msg.running_checksum += data;
        _msg.status++;
        break;

    case 4: // 状态4: 接收包序列
        _msg.seq = data;
        _msg.running_checksum += data;
        _msg.status++;
        break;

    case 5: // 状态5: 接收负载长度
        _msg.len = data;
        _msg.running_checksum += data;
        if (_msg.len > MICOLINK_MAX_PAYLOAD_LEN)
        {
            // 负载长度超出定义的最大值，判定为错误帧，重置状态机
            _msg.status = 0;
        }
        else if (_msg.len == 0)
        {
            // 负载长度为0，直接跳到等待校验和的状态
            _msg.status = 7;
        }
        else
        {
            // 准备接收负载数据
            _msg.payload_cnt = 0; // 重置负载计数器
            _msg.status++;
        }
        break;

    case 6: // 状态6: 接收数据负载
        _msg.payload[_msg.payload_cnt++] = data;
        _msg.running_checksum += data;
        if (_msg.payload_cnt == _msg.len)
        {
            // 负载数据接收完毕，转移到下一状态
            _msg.status++;
        }
        break;

    case 7:                   // 状态7: 接收并校验
        _msg.checksum = data; // 这是接收到的原始校验和
        _msg.status = 0;      // 无论校验是否成功，都重置状态机以准备接收下一帧

        // 比较动态计算的校验和与接收到的校验和
        if (_msg.running_checksum == _msg.checksum)
        {
            // 校验成功！返回true，通知上层函数
            return true;
        }
        // 如果校验失败，函数将继续执行到末尾并返回false
        break;

    default: // 异常状态处理
        // 如果因未知原因进入了未定义的状态，强制重置状态机
        _msg.status = 0;
        break;
    }

    // 如果未形成完整的一帧，或校验失败，则返回false
    return false;
}

// --- 公共API实现 ---
// 这些getter方法非常简单，直接返回内部_sensorData结构体中对应的成员变量值。
// 它们的作用是提供一个干净、稳定的公共接口，将用户代码与库的内部数据结构解耦。

uint32_t MTF02P::getDistance_mm() { return _sensorData.distance; }
uint8_t MTF02P::getSignalStrength() { return _sensorData.strength; }
int16_t MTF02P::getFlowVelX_cms() { return _sensorData.flow_vel_x; }
int16_t MTF02P::getFlowVelY_cms() { return _sensorData.flow_vel_y; }
uint8_t MTF02P::getFlowQuality() { return _sensorData.flow_quality; }
uint32_t MTF02P::getTimestamp_ms() { return _sensorData.time_ms; }

/**
 * @brief 检查最新的测距数据是否有效。
 * @details 根据协议文档，tof_status为1表示数据可用。
 *          同时，距离值为0也表示数据不可用。因此我们检查两个条件。
 * @return bool 如果数据有效，返回true。
 */
bool MTF02P::isRangeDataValid() { return (_sensorData.tof_status == 1 && _sensorData.distance > 0); }

/**
 * @brief 检查最新的光流数据是否有效。
 * @details 根据协议文档，flow_status为1表示数据可用。
 * @return bool 如果数据有效，返回true。
 */
bool MTF02P::isFlowDataValid() { return (_sensorData.flow_status == 1); }
