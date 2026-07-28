/**
 * @file IST8310.cpp
 * @brief IST8310 非阻塞驱动实现
 */

#include "IST8310.h"

IST8310::IST8310()
{
    _wire = nullptr;
    _addr = IST8310_I2C_ADDR_DEFAULT;
    _state = IST_STATE_IDLE;
    _data = {0.0f, 0.0f, 0.0f};
    _start_time_us = 0;
    _wait_duration_us = 6000; // 默认安全值
}

bool IST8310::begin(TwoWire *wire, uint8_t addr, IST8310_Avg avg_setting)
{
    _wire = wire;
    _addr = addr;
    _state = IST_STATE_IDLE;

    // --------------------------------------------------------
    // 1. 软件复位 (Soft Reset)
    // --------------------------------------------------------
    // 写入 CNTL2 (0x0B) 的 Bit 0
    if (!write_reg(IST8310_REG_CNTL2, 0x01))
        return false;

    // 复位需要约 10ms，这里是初始化阶段，可以使用 delay
    delay(20);

    // --------------------------------------------------------
    // 2. 检查 Chip ID
    // --------------------------------------------------------
    uint8_t chip_id;
    if (!read_reg(IST8310_REG_WHO_AM_I, chip_id))
        return false;
    if (chip_id != IST8310_CHIP_ID_VAL)
        return false;

    // --------------------------------------------------------
    // 3. 性能优化配置 (关键!)
    // --------------------------------------------------------
    // 配置脉冲持续时间 (PDCNTL) 为 0xC0，降低噪声
    if (!write_reg(IST8310_REG_PDCNTL, 0xC0))
        return false;

    // --------------------------------------------------------
    // 4. 平均采样配置 & 时间计算
    // --------------------------------------------------------
    // AVGCNTL (0x41): [5:3] Y轴平均, [2:0] XZ轴平均
    // 通常所有轴保持一致
    uint8_t avg_reg = (avg_setting << 3) | avg_setting;
    if (!write_reg(IST8310_REG_AVGCNTL, avg_reg))
        return false;

    // 根据采样次数设置非阻塞等待时间
    // 数据来源：Datasheet Table 3 (ODR vs AVG)
    switch (avg_setting)
    {
    case IST_AVG_1:
        _wait_duration_us = 3000;
        break; // 预留 3ms
    case IST_AVG_2:
        _wait_duration_us = 3500;
        break;
    case IST_AVG_4:
        _wait_duration_us = 4000;
        break;
    case IST_AVG_8:
        _wait_duration_us = 5000;
        break;
    case IST_AVG_16:
        _wait_duration_us = 7000;
        break; // 预留 7ms (转换约6ms)
    default:
        _wait_duration_us = 7000;
        break;
    }

    return true;
}

bool IST8310::update()
{
    uint32_t now = micros();

    switch (_state)
    {
    // ====================================================================
    // 阶段 1: 触发测量 (Trigger)
    // ====================================================================
    case IST_STATE_IDLE:
    {
        // 发送 "Single Measurement Mode" (0x01) 到 CNTL1
        // IST8310 完成一次测量后会自动回到 Standby，所以每次都要发
        if (write_reg(IST8310_REG_CNTL1, 0x01))
        {
            _start_time_us = now;       // 记录开始时间
            _state = IST_STATE_WAITING; // 进入等待状态
        }
        // 此时还没有新数据
        return false;
    }

    // ====================================================================
    // 阶段 2: 异步等待 (Async Wait)
    // ====================================================================
    case IST_STATE_WAITING:
    {
        // 计算时间差，处理 micros() 溢出情况
        // (unsigned long 减法自动处理溢出环绕)
        if ((now - _start_time_us) >= _wait_duration_us)
        {
            // 时间到了，准备读取
            _state = IST_STATE_READING;

            // 立即进入下一个 case 执行，减少一帧的延迟
            // (fallthrough)
        }
        else
        {
            // 时间没到，直接返回，不占用任何 I2C 资源
            return false;
        }
    }

    // ====================================================================
    // 阶段 3: 读取数据 (Readout)
    // ====================================================================
    case IST_STATE_READING:
    {
        // 1. (可选) 检查 DRDY
        // 虽然时间到了，但为了保险，检查 STAT1 寄存器的 Bit 0
        uint8_t status;
        if (!read_reg(IST8310_REG_STAT1, status))
        {
            // I2C 错误，重置状态机
            _state = IST_STATE_IDLE;
            return false;
        }

        if ((status & 0x01) == 0)
        {
            // 极少数情况：时间到了但数据还没好，继续等
            // 或者设置一个超时强制重置
            return false;
        }

        // 2. 突发读取 6 字节 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
        uint8_t buf[6];
        if (read_bytes(IST8310_REG_DATA_XL, buf, 6))
        {
            // 3. 组装数据 (Little Endian)
            int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
            int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
            int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

            // 4. 转换为物理单位 (uT)
            _data.x = raw_x * IST8310_SENSITIVITY_UT;
            _data.y = raw_y * IST8310_SENSITIVITY_UT;
            _data.z = raw_z * IST8310_SENSITIVITY_UT;

            // 5. 任务完成，重置状态到 IDLE，准备下一次触发
            _state = IST_STATE_IDLE;
            return true; // 只有此时返回 true
        }
        else
        {
            // 读取失败，重置
            _state = IST_STATE_IDLE;
            return false;
        }
    }
    }

    return false;
}

/* ============================================================================
   底层 I2C 封装
   ============================================================================ */

bool IST8310::write_reg(uint8_t reg, uint8_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

bool IST8310::read_reg(uint8_t reg, uint8_t &val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission() != 0)
        return false;

    if (_wire->requestFrom(_addr, (uint8_t)1) != 1)
        return false;
    val = _wire->read();
    return true;
}

bool IST8310::read_bytes(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    _wire->beginTransmission(_addr);
    _wire->write(start_reg);
    if (_wire->endTransmission() != 0)
        return false;

    if (_wire->requestFrom(_addr, len) != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = _wire->read();
    }
    return true;
}
