// -*- coding: utf-8 -*-
// test_anocom_packing.cpp — AnoComProtocol 打包回归测试（★2026-08-10 COMM-001）
//
// 覆盖：
//   A1 长度保护：buildFrame len>256 截断（防栈/外部缓冲越界写）
//   A2 校验单循环边拷边算：sum/add 与独立参考实现逐字节一致
//   A3 组模式：beginGroup+多帧 sendData+endGroup = 单次 write，
//      字节流 = 逐帧 sendData 的拼接（协议兼容：官方上位机逐帧解析无感）
//   帧结构：AB 05 <dest> <func> <lenL> <lenH> <data...> <sum> <add>
//
// 编译（run_all.sh 注册为 "ap" 套件，多文件）：
//   g++ -std=c++17 -Iinclude -Itest_host/stub test_anocom_packing.cpp \
//       lib/AnoComProtocol/AnoComProtocol.cpp -o bin/ap
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

#include "AnoComProtocol.h"

// ---------------- Mock 串口：记录全部 write 字节 ----------------
class MockSerial : public Stream
{
public:
    std::vector<uint8_t> tx;   // 所有 write 的拼接字节
    std::vector<size_t> writes; // 每次 write 的长度（断言"单次 write"用）

    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual size_t write(const uint8_t *buf, size_t len)
    {
        tx.insert(tx.end(), buf, buf + len);
        writes.push_back(len);
        return len;
    }
};

// ---------------- 参考校验实现（独立重算，非被测代码） ----------------
static void refChecksum(const uint8_t *buf, size_t len6, uint8_t &sum, uint8_t &add)
{
    sum = 0; add = 0;
    for (size_t i = 0; i < len6; i++)
    {
        sum += buf[i];
        add += sum;
    }
}

static void expectFrame(const std::vector<uint8_t> &frame, uint8_t func,
                        const std::vector<uint8_t> &payload)
{
    assert(frame.size() == payload.size() + 8);
    assert(frame[0] == 0xAB);                 // 帧头
    assert(frame[1] == 0x05);                 // 源地址 = 本机（飞控）
    assert(frame[2] == 0xFF);                 // 目标 = 地面站
    assert(frame[3] == func);                 // 功能码
    assert(frame[4] == (payload.size() & 0xFF));
    assert(frame[5] == ((payload.size() >> 8) & 0xFF));
    assert(std::memcmp(frame.data() + 6, payload.data(), payload.size()) == 0);
    uint8_t sum, add;
    refChecksum(frame.data(), 6 + payload.size(), sum, add);
    assert(frame[6 + payload.size()] == sum);
    assert(frame[7 + payload.size()] == add);
    printf("  ✓ 帧校验 func=0x%02X len=%zu\n", func, payload.size());
}

int main()
{
    int passed = 0;

    // ---- A2：buildFrame 校验与帧结构 ----
    {
        MockSerial s;
        AnoComProtocol ano(&s);
        uint8_t buf[300];
        uint8_t payload[5] = {1, 2, 3, 4, 5};
        uint16_t flen = ano.buildFrame(buf, 0, 0xFF, 0x06, payload, 5);
        assert(flen == 13);
        expectFrame(std::vector<uint8_t>(buf, buf + flen), 0x06,
                    std::vector<uint8_t>(payload, payload + 5));
        passed++;
        printf("  ✓ A2 buildFrame 单帧结构+校验\n");
    }

    // ---- A1：长度保护（len=300 → 截断 256） ----
    {
        MockSerial s;
        AnoComProtocol ano(&s);
        uint8_t buf[300];
        uint8_t big[300];
        std::memset(big, 0x5A, sizeof(big));
        uint16_t flen = ano.buildFrame(buf, 0, 0xFF, 0x7F, big, 300);
        assert(flen == 256 + 8);             // 截断为 ANO_MAX_DATA_LEN=256
        assert(buf[4] == 0x00 && buf[5] == 0x01); // 长度字段 = 256 LE
        // 数据区全 0x5A，校验按截断后长度计算
        uint8_t sum, add;
        refChecksum(buf, 6 + 256, sum, add);
        assert(buf[6 + 256] == sum && buf[7 + 256] == add);
        passed++;
        printf("  ✓ A1 长度保护截断 256（原 300B 请求）\n");
    }

    // ---- A3 组模式：多帧合并单次 write = 逐帧 sendData 拼接 ----
    {
        MockSerial g;
        AnoComProtocol anoGroup(&g);
        // 参考：逐帧独立 sendData（非组模式）
        MockSerial r;
        AnoComProtocol anoRef(&r);
        uint8_t p1[3] = {0xAA, 0xBB, 0xCC};
        uint8_t p2[5] = {1, 2, 3, 4, 5};
        uint8_t p3[12] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0x7F, 0x01};
        // 组模式：3 帧 → 应 1 次 write
        anoGroup.beginGroup();
        anoGroup.sendData(0xFF, 0x01, p1, 3);
        anoGroup.sendData(0xFF, 0x21, p2, 5);
        anoGroup.sendData(0xFF, 0xF1, p3, 12);
        anoGroup.endGroup();
        // 参考：逐帧
        anoRef.sendData(0xFF, 0x01, p1, 3);
        anoRef.sendData(0xFF, 0x21, p2, 5);
        anoRef.sendData(0xFF, 0xF1, p3, 12);
        assert(g.writes.size() == 1);                  // 单次 write
        assert(g.tx == r.tx);                          // 字节流完全一致
        passed++;
        printf("  ✓ A3 组模式 3 帧合并 1 次 write，字节流与逐帧一致（%zuB）\n", g.tx.size());
    }

    // ---- A3 组模式：组缓冲满自动 flush（>256B 场景防御） ----
    {
        MockSerial g;
        AnoComProtocol anoGroup(&g);
        uint8_t big[200];
        std::memset(big, 0x11, sizeof(big));
        anoGroup.beginGroup();
        anoGroup.sendData(0xFF, 0x01, big, 200);   // 208B → 组内
        anoGroup.sendData(0xFF, 0x02, big, 200);   // 208B 超 256 上限 → flush 前帧 + 入组
        anoGroup.endGroup();
        assert(g.writes.size() == 2);              // 一次自动 flush + 一次 endGroup
        assert(g.tx.size() == (200 + 8) * 2);      // 字节不丢
        passed++;
        printf("  ✓ A3 组缓冲满自动 flush（%zu 次 write，%zuB 完整）\n",
               g.writes.size(), g.tx.size());
    }

    // ---- 高层 sendXxx 在组模式下同样走组缓冲（协议兼容锚点） ----
    {
        MockSerial g;
        AnoComProtocol anoGroup(&g);
        anoGroup.beginGroup();
        anoGroup.sendAttitudeEuler(10.0f, 20.0f, 30.0f, 0x80);
        anoGroup.sendFlightMode(2, 1, 0, 0, 0);
        anoGroup.endGroup();
        assert(g.writes.size() == 1);
        assert(g.tx.size() == (7 + 8) + (5 + 8));  // 欧拉 7B + 模式 5B + 2×8B 开销
        expectFrame(std::vector<uint8_t>(g.tx.begin(), g.tx.begin() + 15), 0x03,
                    std::vector<uint8_t>({0xE8, 0x03, 0xD0, 0x07, 0xB8, 0x0B, 0x80}));
        passed++;
        printf("  ✓ 高层 sendXxx 组模式兼容（欧拉+模式合并 1 次 write）\n");
    }

    printf("\n==== test_anocom_packing: %d 组断言全部通过 ====\n", passed);
    return 0;
}
