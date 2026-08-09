# nmea-host-test — UBX+NMEA 双协议库宿主机回归测试

验证 `lib/ublox-main` 的库级双协议能力（`bfs::Ubx::GpsProtocol`）：

- **kNmea**：GGA/RMC 句子切分、minmea 解析、快照→`UbxEpoch` 换算、
  fix 映射（GGA quality→Fix 枚举，含 DGPS/RTK）、伪 tow、秒键去重 +
  800ms 合成节流（GGA+RMC 双句 1Hz 流 → 1Hz epoch，位置最新/速度滞后 ≤1s）、
  无效句保留快照、坏校验和计数、负 geoid 分离
- **kUbx**：UBX 帧（NAV-PVT+NAV-EOE）照常解析入队，NMEA 句被忽略
- **kAuto**：UBX 优先（300ms backoff 窗口内不合成 NMEA），UBX 失效后 NMEA 兜底
- **SetProtocol / SwitchProtocol**（含波特率重设）

非 Arduino 编译路径：`ubx.h` 的平台分支走 `core/core.h` 兼容层
（库原设计；shim 提供 HardwareSerial + 可控时钟，测试手动推进时间）。

## 运行

```bash
cd TandemVec_FCS/tools/nmea-host-test
gcc -c -I. -I../../lib/ublox-main/src ../../lib/ublox-main/src/minmea.c -o minmea.o
g++ -std=gnu++17 -I. -I../../lib/ublox-main/src nmea_host_test.cpp \
    ../../lib/ublox-main/src/ubx.cpp minmea.o -o nmea_test
./nmea_test       # 期望"全部通过 (失败 0)"
```

改 `lib/ublox-main/src/ubx.{h,cpp}` 的 NMEA 解析/合成/节流逻辑后必须重跑本测试
（2026-08-09 库级双协议集成，43 项断言通过）。
