/**
 * @file QMC5883L.cpp
 * @brief QMC5883L 非阻塞驱动实现
 */

#include "QMC5883L.h"

QMC5883L::QMC5883L()
{
    _wire = nullptr;
    _addr = QMC5883L_I2C_ADDR_DEFAULT;
    _data = {0.0f, 0.0f, 0.0f};
    _last_poll_us = 0;
    _last_data_us = 0;
    _reinit_pending_us = 0;
    _ovl_count = 0;
    _dor_count = 0;
    _reinit_count = 0;
}

bool QMC5883L::begin(TwoWire *wire, uint8_t addr, const QMC5883L_Config &cfg)
{
    _wire = wire;
    _addr = addr;
    _cfg = cfg;
    _ovl_count = 0;
    _dor_count = 0;
    _reinit_count = 0;
    _reinit_pending_us = 0;
    _last_data_us = 0;

    // 1. 软复位（初始化阶段允许阻塞延时：复位后芯片需要约 10-20ms 稳定）
    if (!softReset())
        return false;
    delay(20);

    // 2. 芯片 ID 校验：0x0D 必须为 0xFF（真 QMC5883L）
    if (!probeChipId())
        return false;

    // 3. 写入 SET/RESET 周期 (0x0B=0x01) 与 CTRL1，并回读 CTRL1 校验。
    //    回读校验可防"假阳性"（HMC5883L 的 0x09 是只读状态寄存器，
    //    写读不一致会在此处被拦截）。
    if (!applyConfig())
        return false;

    // 4. 进入连续测量模式后首个样本约需 1/ODR + 转换时间，轮询起点置为当前时刻
    _last_poll_us = micros();
    return true;
}

bool QMC5883L::update()
{
    const uint32_t now = micros();

    // ====================================================================
    // 看门狗：长时间无新数据（芯片锁死 / OVL 卡死 / 总线故障）-> 软复位
    // 参考 Betaflight PR #13467 "Fix qmc5883l lockup"。
    // 复位发出后 10ms 内不接受轮询，10ms 后再补写配置（全程非阻塞）。
    // ====================================================================
    if (_last_data_us != 0 && (now - _last_data_us) > _cfg.watchdog_us)
    {
        armWatchdogReinit();
    }
    if (_reinit_pending_us != 0)
    {
        if ((now - _reinit_pending_us) >= 10000UL)
        {
            applyConfig();                 // 补写 SET/RESET + CTRL1
            _reinit_pending_us = 0;
            _last_data_us = 0;             // 重新计时，等待第一个新样本
        }
        return false;
    }

    // ====================================================================
    // DRDY 轮询（限速，空转成本仅一次 micros() 比较）
    // ====================================================================
    if ((now - _last_poll_us) < _cfg.poll_interval_us)
    {
        return false;
    }
    _last_poll_us = now;

    uint8_t status = 0;
    if (!read_reg(QMC5883L_REG_STATUS, status))
    {
        return false; // I2C 错误：下一轮继续
    }

    if ((status & QMC5883L_STATUS_DRDY) == 0)
    {
        return false; // 数据未就绪
    }

    // ====================================================================
    // 数据就绪：突发读 6 字节（读数据会自动清 DRDY/OVL/DOR）
    // ====================================================================
    uint8_t buf[6];
    if (!read_bytes(QMC5883L_REG_X_LSB, buf, 6))
    {
        return false; // 读失败：DRDY 已清，等下一次数据就绪
    }

    if (status & QMC5883L_STATUS_OVL)
        _ovl_count++; // 量程溢出（数据仍有效，标记诊断）
    if (status & QMC5883L_STATUS_DOR)
        _dor_count++; // 上次数据未被读取即被覆盖

    float f[3];
    magmath::qmc5883lDecode(buf, _cfg.range_index, f);
    _data = {f[0], f[1], f[2]};
    _last_data_us = now;
    return true;
}

/* ============================================================================
   初始化辅助（阻塞路径，仅在 begin() 调用）
   ============================================================================ */

bool QMC5883L::softReset()
{
    return write_reg(QMC5883L_REG_CTRL2, QMC5883L_SOFT_RST);
}

bool QMC5883L::probeChipId()
{
    uint8_t id = 0;
    if (!read_reg(QMC5883L_REG_CHIP_ID, id))
        return false;
    return (id == QMC5883L_CHIP_ID_VAL);
}

bool QMC5883L::applyConfig()
{
    // SET/RESET 周期置 0x01：数据手册与工程实现共同要求，
    // 缺省 0x00 时部分批次/克隆芯片会输出异常（全零或噪声大）。
    if (!write_reg(QMC5883L_REG_SET_RESET, 0x01))
        return false;
    if (!write_reg(QMC5883L_REG_CTRL1, _cfg.ctrl1))
        return false;
    // 注意：此处不放 delay —— applyConfig 也会被 update() 的看门狗路径调用
    //（运行在 IMU 任务上下文，严禁 10ms 级阻塞）。看门狗路径已通过
    // _reinit_pending 的 10ms 非阻塞等待保证复位稳定期；begin() 路径则
    // 有 softReset 后的 delay(20) 兜底。

    // 回读校验：防地址冲突芯片（如 HMC5883L）假阳性
    uint8_t readback = 0;
    if (!read_reg(QMC5883L_REG_CTRL1, readback))
        return false;
    return (readback == _cfg.ctrl1);
}

void QMC5883L::armWatchdogReinit()
{
    softReset();
    _reinit_pending_us = micros();
    _reinit_count++;
}

/* ============================================================================
   底层 I2C 封装（与 IST8310 驱动同风格）
   ============================================================================ */

bool QMC5883L::write_reg(uint8_t reg, uint8_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return (_wire->endTransmission() == 0);
}

bool QMC5883L::read_reg(uint8_t reg, uint8_t &val)
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

bool QMC5883L::read_bytes(uint8_t start_reg, uint8_t *buf, uint8_t len)
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
