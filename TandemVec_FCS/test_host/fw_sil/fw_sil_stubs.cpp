/**
 * @file    fw_sil_stubs.cpp
 * @module  固件/宿主 SIL 桩（fw-tests：真实固件源码的链接替身）
 * @brief   为链接真实 src/flight_control.cpp + src/state_data.cpp 提供最小硬件/胶水符号
 * @details 被测：无（本文件是替身层，不是被测对象）。
 *          输入：无；处理：定义串口/SPI/I2C 全局对象、各传感器与日志类构造函数、
 *          以及导航/MAVLink 空实现；输出：可链接的目标文件。
 *          与真实硬件差异：所有外设方法为空实现或返回常量，不产生时序、不读写寄存器、
 *          不做 CRC 校验；SIL 主程序把状态直接注入 `state_data` 全局量。
 * @theory  N/A（替身代码，无控制/导航算法）
 * @frame   N/A（不解释姿态/位置；坐标系由注入方 fw_sil_main/fw_sil_env_main 负责）
 * @units   N/A（无物理量计算）
 * @note    替身只补齐链接所需的符号，任何“行为”断言都不能落在本文件上；
 *          若新增固件全局对象，需要在此补桩，否则 SIL 链接失败。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_main.cpp、
 *          TandemVec_FCS/test_host/fw_sil/fw_sil_env_main.cpp
 */
#include "state_data.h"
#include "flight_control.h"
#include <cstdio>
#include <ctime>

#ifdef _WIN32
// MinGW 的 libminmea 使用 timegm；MSVC 运行时对应 _mkgmtime。
extern "C" time_t timegm(struct tm *tm) { return _mkgmtime(tm); }
#endif
uint32_t g_sil_time_us = 0;  // 仿真虚拟时钟（由 SIL 主循环推进）
HardwareSerial Serial;
SPIClass SPI;
TwoWire Wire;
Vector3 current_omega_dps_body_filtered(0.0, 0.0, 0.0);

#ifndef FW_SIL_REAL_NAV
// 真实导航任务参与链接时，该函数由 navigation_task.cpp 提供，避免重复定义。
void setNavigationOriginFromLlaRad(double, double, double) {}
#endif
void resetBaroAltitudeReference() {}

// SIL 不链接 communication.cpp（其遥测/CRSF 依赖过重），这里按真实
// linkUp/linkDown 策略提供行为替身：飞行中断链保持解锁、摇杆归中、
// 模式落 AUTO_ALTITUDE；地面断链全部回锁定值。策略变更时必须同步
// src/communication.cpp:linkDownCallback()。
void linkUpCallback()
{
  isLinkUp = true;
  failsafe_in_flight = false;
}

void linkDownCallback()
{
  isLinkUp = false;
  const bool was_in_flight =
      (raw_rc_values[4] > 1500) && (raw_rc_values[2] > 1100);
  failsafe_in_flight = was_in_flight;
  for (int i = 0; i < 9; ++i)
  {
    if (i == 0 || i == 1 || i == 3)
      raw_rc_values[i] = 1500;
    else if (i == 4)
      raw_rc_values[i] = was_in_flight ? 1500 : 988;
    else if (i == 6)
      raw_rc_values[i] = was_in_flight ? 1500 : 988;
    else
      raw_rc_values[i] = 988;
  }
}
void mavlinkSendStatustext(unsigned char, const char*) {}

MagSensor::MagSensor() {}
ICM42688::ICM42688(SPIClass&, uint8_t, uint32_t) {}
MTF02P::MTF02P() {}
LQS48_Flow::LQS48_Flow() {}
// Madgwick 由真实 lib/MadgwickAHRS-master 提供（SIL 需复现生产 AHRS，
// 不能再用空构造桩；否则 backup_ahrs_* 恒为 0，会伪造“姿态被拉平”）。
W25N01GV::W25N01GV(SPIClass& bus, uint8_t cs)
    : _bus(bus), _cs(cs), _manId(0), _devId(0), _devId2(0), _present(false),
      _lastEcc(0), _lastProgErr(0) {}
W25N01GVLog::W25N01GVLog(W25N01GV& f) : _flash(f) {}
AnoComProtocol::AnoComProtocol(Stream*) {}
// 参数注册表（ano_params.cpp）引用 sendData 回传 0xE1 应答；宿主直调库只读写
// 参数值、不跑协议收发，因此这里给空实现满足链接。SIL/固件侧的协议行为
// 由 communication.cpp + 真实 AnoComProtocol 覆盖，不落在这个桩上。
void AnoComProtocol::sendData(uint8_t, uint8_t, uint8_t *, uint16_t) {}
CrsfSerial::CrsfSerial(HardwareSerial& port, uint32_t baud)
    : _port(port), _crc(0xd5), _baud(baud), _lastReceive(0),
      _lastChannelsPacket(0), _linkIsUp(false), _passthroughBaud(0) {}
Crc8::Crc8(uint8_t) {}
IST8310::IST8310() {}
QMC5883L::QMC5883L() {}
uint8_t Crc8::calc(uint8_t*, uint8_t) { return 0; }
