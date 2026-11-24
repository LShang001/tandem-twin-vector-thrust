#pragma once

#include <Arduino.h>
#include <crc8.h>
#include "crsf_protocol.h"

enum eFailsafeAction { fsaNoPulses, fsaHold };

class CrsfSerial
{
public:
    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    CrsfSerial(HardwareSerial &port, uint32_t baud = CRSF_BAUDRATE);
    void begin(uint32_t baud = 0);
    void loop();
    void write(uint8_t b);
    void write(const uint8_t *buf, size_t len);
    void queuePacket(uint8_t type, const void *payload, uint8_t len);
    void queuePacketChannels();

    uint32_t getBaud() const { return _baud; };
    // Return current channel value (1-based) in us
    int getChannel(unsigned int ch) const { return _channels[ch - 1]; }
    void setChannel(unsigned int ch, unsigned int value_us) { _channels[ch - 1] = value_us; }
    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    const crsf_sensor_gps_t *getGpsSensor() const { return &_gpsSensor; }
    bool isLinkUp() const { return _linkIsUp; }

    // ---- 上行字节层统计（诊断用，DBG `health` 的 rc_* 字段）----
    // 只加计数、不改解析行为：链路「能飞」与「字节全到了」是两件事，
    // 420k 下若有串口溢出或无线丢包，只有这些计数看得出来。
    uint32_t packetsIn() const { return _packetsIn; }         ///< 校验通过的帧总数
    uint32_t channelPackets() const { return _channelPackets; } ///< 其中通道帧（0x16）数
    uint32_t crcErrors() const { return _crcErrors; }         ///< CRC 失败次数
    uint32_t lenErrors() const { return _lenErrors; }         ///< 声明长度非法的次数
    uint32_t bufferResyncs() const { return _bufferResyncs; } ///< 缓冲填满强制的重同步次数
    bool getPassthroughMode() const { return _passthroughBaud != 0; }
    void setPassthroughMode(bool val, uint32_t passthroughBaud = 0);

    // Event Handlers
    void (*onLinkUp)();
    void (*onLinkDown)();
    // OobData is any byte which is not CRSF, including passthrough
    void (*onOobData)(uint8_t b);
    // CRSF Packet Callbacks
    void (*onPacketChannels)();
    void (*onPacketLinkStatistics)(crsfLinkStatistics_t *ls);
    void (*onPacketGps)(crsf_sensor_gps_t *gpsSensor);

private:
    HardwareSerial &_port;
    uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
    uint8_t _rxBufPos;
    Crc8 _crc;
    crsfLinkStatistics_t _linkStatistics;
    crsf_sensor_gps_t _gpsSensor;
    uint32_t _baud;
    uint32_t _lastReceive;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;
    uint32_t _passthroughBaud;
    int _channels[CRSF_NUM_CHANNELS];
    // 诊断计数：默认成员初始化，构造函数（含 SIL 桩）不写也不会留垃圾值
    uint32_t _packetsIn = 0;
    uint32_t _channelPackets = 0;
    uint32_t _crcErrors = 0;
    uint32_t _lenErrors = 0;
    uint32_t _bufferResyncs = 0;

    void handleSerialIn();
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);
    void checkPacketTimeout();
    void checkLinkDown();

    // Packet Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetGps(const crsf_header_t *p);
};
