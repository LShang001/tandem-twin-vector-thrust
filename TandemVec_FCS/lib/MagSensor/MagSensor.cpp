/**
 * @file MagSensor.cpp
 * @brief 磁力计统一管理层实现
 */

#include "MagSensor.h"

MagSensor::MagSensor()
{
    _wire = nullptr;
    _chip = MagChip::NONE;
    _last = {0.0f, 0.0f, 0.0f, 0};
    _seq = 0;
    _calCollecting = false;
    _collector.reset();
    _cal.identity();
    _guideActive = false;
    _guideFitReady = false;
    _guideFitOk = false;
    _lastChip[0] = _lastChip[1] = _lastChip[2] = 0.0f;
    _lastChipValid = false;
}

bool MagSensor::begin(TwoWire *wire)
{
    _wire = wire;
    _chip = MagChip::NONE;
    _last = {0.0f, 0.0f, 0.0f, 0};
    _seq = 0;
    _calCollecting = false;
    _collector.reset();
    _cal.identity();
    _guideActive = false;
    _guideFitReady = false;
    _guideFitOk = false;
    _lastChipValid = false;
    _guide.reset();

    // 探测顺序：本机默认硬件 IST8310 优先；未命中再试 QMC5883L。
    // 两个芯片地址不同（0x0E / 0x0D），同总线共存也不会冲突；
    // 但同一时刻只取用一个（避免两个地磁测量源打架）。
    if (_ist.begin(wire))
    {
        _chip = MagChip::IST8310;
        return true;
    }
    if (_qmc.begin(wire))
    {
        _chip = MagChip::QMC5883L;
        return true;
    }
    return false;
}

bool MagSensor::update()
{
    if (_chip == MagChip::NONE)
        return false;

    const bool got = (_chip == MagChip::IST8310) ? _ist.update() : _qmc.update();
    if (!got)
        return false;

    // 取芯片坐标系 uT 原始数据
    float chip[3];
    if (_chip == MagChip::IST8310)
    {
        const IST8310_Vector v = _ist.get_data();
        chip[0] = v.x;
        chip[1] = v.y;
        chip[2] = v.z;
    }
    else
    {
        const QMC5883L_Vector v = _qmc.get_data();
        chip[0] = v.x;
        chip[1] = v.y;
        chip[2] = v.z;
    }

    // 保存最近芯片系原始样本（引导校准采集用）
    _lastChip[0] = chip[0];
    _lastChip[1] = chip[1];
    _lastChip[2] = chip[2];
    _lastChipValid = true;

    // 标定采集：用未标定的芯片系原始数据（含抽稀），与标定是否生效无关
    if (_calCollecting)
    {
        _collector.addSample(chip[0], chip[1], chip[2], micros());
    }

    // 标定应用（芯片系）：c = scale·M·(raw − b)
    float corrected[3];
    if (_cal.valid)
    {
        magcal::applyCal(_cal, chip, corrected);
    }
    else
    {
        corrected[0] = chip[0];
        corrected[1] = chip[1];
        corrected[2] = chip[2];
    }

    // 芯片系 -> 机体 FRD 轴对齐（矩阵见 MagMath.h；硬铁偏置由标定层负责）
    const int8_t(*rot)[3] = (_chip == MagChip::IST8310)
                                ? magmath::kAxisIst8310ToFrd
                                : magmath::kAxisQmc5883lToFrd;
    const float zeroOff[3] = {0.0f, 0.0f, 0.0f};
    float frd[3];
    magmath::magAlignToBody(corrected, rot, zeroOff, frd);

    _last = {frd[0], frd[1], frd[2], ++_seq};
    return true;
}

const char *MagSensor::chipName() const
{
    switch (_chip)
    {
    case MagChip::IST8310:
        return "IST8310";
    case MagChip::QMC5883L:
        return "QMC5883L";
    case MagChip::NONE:
    default:
        return "NONE";
    }
}

bool MagSensor::calStart()
{
    if (_chip == MagChip::NONE)
        return false;
    _collector.reset();
    _calCollecting = true;
    return true;
}

bool MagSensor::setCal(const magcal::MagCalParams &cal)
{
    if (!magcal::paramsSanity(cal))
        return false;
    _cal = cal;
    return true;
}

void MagSensor::calClear()
{
    _cal.identity();
}

bool MagSensor::guideStart()
{
    if (_chip == MagChip::NONE)
        return false;
    calStop(); // 与自由采集互斥
    _collector.reset();
    _guide.start();
    _guideActive = true;
    _guideFitReady = false;
    _guideFitOk = false;
    return true;
}

void MagSensor::guideStop()
{
    _guideActive = false;
    _guide.reset();
}

void MagSensor::guideUpdate(const float accFrd[3], const float gyroFrd[3], uint32_t nowUs)
{
    if (!_guideActive)
        return;

    // 判定输入：磁方位用 FRD 输出（方向参考，含已生效标定）
    const float magFrd[3] = {_last.x, _last.y, _last.z};
    if (_guide.update(accFrd, gyroFrd, magFrd, _last.seq != 0))
    {
        // 本帧判定合格：把对应芯片系原始样本送入采集器（内部自带抽稀）
        if (_lastChipValid)
        {
            _collector.addSample(_lastChip[0], _lastChip[1], _lastChip[2], nowUs);
        }
    }

    // 两圈完成：一次性椭球拟合 + 应用（仅完成瞬间执行，~ms 级；
    // 此时为地面校准场景，IMU 任务短暂加时无安全影响）
    if (_guide.stage() == magcal::GuideStage::DONE)
    {
        _guideActive = false; // 结束引导（灯效随之退出校准态）
        magcal::MagCalParams p;
        p.identity();
        _guideFitOk = _collector.fit(p, _guideFitStats);
        if (_guideFitOk)
        {
            setCal(p);
        }
        else
        {
            _guide.setFailed();
        }
        _guideFitReady = true;
    }
}

uint32_t MagSensor::overflowCount() const
{
    return (_chip == MagChip::QMC5883L) ? _qmc.overflow_count() : 0;
}

uint32_t MagSensor::dorCount() const
{
    return (_chip == MagChip::QMC5883L) ? _qmc.dor_count() : 0;
}

uint32_t MagSensor::reinitCount() const
{
    return (_chip == MagChip::QMC5883L) ? _qmc.reinit_count() : 0;
}
