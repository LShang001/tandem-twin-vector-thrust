/**
 * @file    test_crsf_parse.cpp
 * @module  固件/通信（CRSF 上行星历解析回归）
 * @brief   覆盖 CrsfSerial 的四条关键路径：正常通道帧解包、声明长度不足的
 *          "伪合法短帧"拒收、CRC 错帧后的重同步、半帧超时清理
 * @details 方法：用 fw_sil/stub 的 HardwareSerial 注入合成字节流，驱动真实的
 *          CrsfSerial::loop()，只通过公开接口断言（通道值、计数器、链路状态）。
 *          时钟用桩里的 g_sil_time_us 虚拟推进，不依赖真实等待。
 * @theory  N/A（协议解析器回归，不含算法）
 * @frame   通道值按 CRSF 11bit → µs 的 crsf*5/8+880 换算
 * @units   通道 µs；计数器为次数
 * @note    用例 2 是 2026-11-26 长度校验修复的回归锁：修复前短 RC 帧会让
 *          packetChannelsPacked 越界读 _rxBuf 之后的相邻成员。
 * @see     lib/CrsfSerial/CrsfSerial.cpp（processPacketIn）、test_host/run_all.sh
 */
#include <cstdio>
#include <cstring>
#include <vector>

#include "CrsfSerial.h"
#include "crc8.h"

// 桩的虚拟时钟（test_host/fw_sil/stub/Arduino.h 声明为 extern）
uint32_t g_sil_time_us = 0;

static int g_fail = 0;

static void check(bool ok, const char *what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
    {
        ++g_fail;
    }
}

/// 16 通道 × 11 bit 的 LSB-first 打包，与 CrsfSerial 的解包顺序对称
static void packChannels(const uint16_t ch[16], uint8_t out[22])
{
    std::memset(out, 0, 22);
    unsigned bit = 0;
    for (int i = 0; i < 16; ++i)
    {
        const unsigned v = ch[i] & 0x7FFu;
        for (int b = 0; b < 11; ++b)
        {
            if ((v >> b) & 1u)
            {
                const unsigned pos = bit + static_cast<unsigned>(b);
                out[pos >> 3] |= static_cast<uint8_t>(1u << (pos & 7u));
            }
        }
        bit += 11;
    }
}

/// 按 CRSF 帧格式组装：[0xC8][len][type][payload][crc8]，len = payload + 2
static std::vector<uint8_t> makeFrame(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    std::vector<uint8_t> f;
    f.push_back(0xC8);
    const uint8_t len = static_cast<uint8_t>(payload_len + 2);
    f.push_back(len);
    f.push_back(type);
    for (uint8_t i = 0; i < payload_len; ++i)
    {
        f.push_back(payload[i]);
    }
    Crc8 crc(0xd5);
    f.push_back(crc.calc(&f[2], static_cast<uint8_t>(len - 1)));
    return f;
}

static void feed(CrsfSerial &crsf, HardwareSerial &port, const std::vector<uint8_t> &f)
{
    port.feed(f.data(), f.size());
    crsf.loop();
}

int main()
{
    HardwareSerial port;
    CrsfSerial crsf(port, 420000);
    crsf.begin(420000);

    // 16 通道原始值：172 起每通道 +100，覆盖低位到高位
    uint16_t raw[16];
    for (int i = 0; i < 16; ++i)
    {
        raw[i] = static_cast<uint16_t>(172 + i * 100);
    }
    uint8_t payload[22];
    packChannels(raw, payload);

    // ---- 1) 正常通道帧 ----
    feed(crsf, port, makeFrame(0x16, payload, 22));
    check(crsf.channelPackets() == 1, "正常通道帧解包一次");
    check(crsf.crcErrors() == 0 && crsf.lenErrors() == 0, "正常帧不产生坏帧计数");
    check(crsf.isLinkUp(), "收到通道帧后链路置 up");

    bool unpack_ok = true;
    for (int i = 0; i < 16; ++i)
    {
        const int expect = raw[i] * 5 / 8 + 880;
        if (crsf.getChannel(static_cast<unsigned>(i + 1)) != expect)
        {
            unpack_ok = false;
        }
    }
    check(unpack_ok, "16 通道 11bit 解包与 crsf*5/8+880 换算一致");

    // ---- 2) 声明长度不足的通道帧：CRC 合法也必须拒收 ----
    //    len = 3 + 2 = 5，payload 只有 3 字节，远不够 16 通道所需的 22 字节。
    const uint32_t ch_before = crsf.channelPackets();
    const uint32_t len_before = crsf.lenErrors();
    const int ch2_before = crsf.getChannel(2);
    const uint8_t short_payload[3] = {0xAA, 0x55, 0x33};
    feed(crsf, port, makeFrame(0x16, short_payload, 3));
    check(crsf.lenErrors() == len_before + 1, "短通道帧计长度错");
    check(crsf.channelPackets() == ch_before, "短通道帧不进解包");
    check(crsf.getChannel(2) == ch2_before, "短通道帧不污染既有通道值");

    // ---- 3) CRC 错帧后仍能重同步 ----
    //    注：CRSF 解析不校验帧首地址字节，坏帧后只能靠"长度合理 + CRC 通过"
    //    逐字节滑窗重新对齐，实测要滑过约一整帧，紧随其后的第一帧好帧会被
    //    残留错位吞掉；连续流里由后续帧补齐。所以这里喂 3 帧、断言至少解出 1 帧，
    //    而不是断言"下一帧必然解出"。
    std::vector<uint8_t> bad = makeFrame(0x16, payload, 22);
    bad[bad.size() - 1] = static_cast<uint8_t>(bad[bad.size() - 1] ^ 0xFFu);
    const uint32_t crc_before = crsf.crcErrors();
    port.feed(bad.data(), bad.size());
    crsf.loop();
    check(crsf.crcErrors() > crc_before, "CRC 错帧被计错");

    const uint32_t ch_after_bad = crsf.channelPackets();
    for (int k = 0; k < 3; ++k)
    {
        feed(crsf, port, makeFrame(0x16, payload, 22));
    }
    check(crsf.channelPackets() > ch_after_bad, "CRC 错后连续帧仍能重同步");

    // ---- 4) 半帧超时清理 ----
    std::vector<uint8_t> half = makeFrame(0x16, payload, 22);
    half.resize(10);
    port.feed(half.data(), half.size());
    crsf.loop();
    g_sil_time_us += 200000u; // 200 ms，超过 CRSF_PACKET_TIMEOUT_MS(100)
    crsf.loop();
    const uint32_t ch_after_half = crsf.channelPackets();
    feed(crsf, port, makeFrame(0x16, payload, 22));
    check(crsf.channelPackets() == ch_after_half + 1, "半帧超时被清、后续帧可解");

    std::printf("\nCRSF 解析回归：%s\n", g_fail == 0 ? "全部通过" : "存在失败");
    return g_fail == 0 ? 0 : 1;
}