/**
 * @file IST8310.h
 * @brief iSentek IST8310 高性能非阻塞驱动
 * @details 专为飞控 (Flight Controller) 和实时机器人系统设计。
 *          采用有限状态机 (FSM) 架构，消除所有阻塞延时。
 * @author AI Assistant
 * @version 3.0 (Async FSM Optimized)
 */

#ifndef _IST_8310_H
#define _IST_8310_H

#include <Arduino.h>
#include <Wire.h>

/* ============================================================================
   寄存器定义 (参考 Datasheet v1.5)
   ============================================================================ */
#define IST8310_I2C_ADDR_DEFAULT 0x0E //< 默认 I2C 地址
#define IST8310_CHIP_ID_VAL 0x10      //< 芯片 ID

// 核心寄存器
#define IST8310_REG_WHO_AM_I 0x00
#define IST8310_REG_STAT1 0x02   //< 状态寄存器 (DRDY)
#define IST8310_REG_DATA_XL 0x03 //< 数据起始地址
#define IST8310_REG_CNTL1 0x0A   //< 模式控制
#define IST8310_REG_CNTL2 0x0B   //< 复位控制
#define IST8310_REG_AVGCNTL 0x41 //< 平均采样控制
#define IST8310_REG_PDCNTL 0x42  //< 脉冲持续时间控制

/* ============================================================================
   物理量转换参数
   ============================================================================ */
// 灵敏度: 330 LSB/Gauss, 1 Gauss = 100 uT
// Factor = 100 / 330 = 0.3030303...
#define IST8310_SENSITIVITY_UT 0.3030303f

/* ============================================================================
   数据类型定义
   ============================================================================ */

/**
 * @brief 驱动内部状态机状态
 */
enum IST8310_State
{
    IST_STATE_IDLE,    //< 空闲：准备发送测量指令
    IST_STATE_WAITING, //< 等待：测量进行中 (不占用 I2C)
    IST_STATE_READING  //< 读取：数据就绪，执行 I2C 读取
};

/**
 * @brief 平均采样配置
 * @note 采样数越高，噪声越低，但最大采样率(ODR)越低
 */
enum IST8310_Avg
{
    IST_AVG_1 = 0x00, //< 转换时间 ~3ms (Max ODR ~200Hz)
    IST_AVG_2 = 0x01,
    IST_AVG_4 = 0x02,
    IST_AVG_8 = 0x03,
    IST_AVG_16 = 0x04 //< 转换时间 ~6ms (Max ODR ~100Hz) [推荐]
};

struct IST8310_Vector
{
    float x;
    float y;
    float z;
};

/* ============================================================================
   类定义
   ============================================================================ */
class IST8310
{
public:
    IST8310();

    /**
     * @brief 初始化传感器
     * @param wire I2C总线对象
     * @param addr I2C地址
     * @param avg_setting 平均采样配置 (默认 16 次平均以获得最佳低噪性能)
     * @return true 初始化成功
     */
    bool begin(TwoWire *wire = &Wire, uint8_t addr = IST8310_I2C_ADDR_DEFAULT, IST8310_Avg avg_setting = IST_AVG_16);

    /**
     * @brief 状态机更新函数 (核心)
     * @details 必须在主循环中高频调用。此函数非阻塞。
     *          流程：触发 -> 计时(非阻塞) -> 读取 -> 返回True
     *
     * @return true  当且仅当完成一次完整的采样并更新了数据时
     * @return false 正在等待或通信中
     */
    bool update();

    /**
     * @brief 获取当前的磁场数据 (uT)
     * @return 包含 x,y,z 的结构体
     */
    IST8310_Vector get_data() const { return _data; }

    // 辅助读取函数
    float get_x() const { return _data.x; }
    float get_y() const { return _data.y; }
    float get_z() const { return _data.z; }

private:
    TwoWire *_wire;
    uint8_t _addr;

    // 数据缓存
    IST8310_Vector _data;

    // 状态机变量
    IST8310_State _state;
    uint32_t _start_time_us;    //< 测量开始的时间戳
    uint32_t _wait_duration_us; //< 需要等待的时长

    // 内部方法
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_reg(uint8_t reg, uint8_t &val);
    bool read_bytes(uint8_t start_reg, uint8_t *buf, uint8_t len);
};

#endif // _IST_8310_H
