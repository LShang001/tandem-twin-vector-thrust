/**
 * ESP32 UBX GPS 深度解析示例 (PVT + DOP + EOE 专用)
 *
 * 硬件连接:
 * - ESP32 GPIO 13 (RX) <--> GPS TX
 * - ESP32 GPIO 14 (TX) <--> GPS RX
 * - 波特率: 921600 (请确保 GPS 模块已配置为此速率)
 *
 * 功能:
 * 1. 仅解析 NAV-PVT 和 NAV-DOP 包，剔除无效的 0 值数据。
 * 2. 提供详细的中文注释，解释每个导航参数的物理意义。
 * 3. 包含 Hz 刷新率计算和断连报警。
 */

#include <Arduino.h>
#include "ubx.h"

// ================= 用户配置区域 =================
#define GPS_RX_PIN 13       // ESP32 接收引脚
#define GPS_TX_PIN 14       // ESP32 发送引脚
#define GPS_BAUD 921600     // GPS 串口波特率 (必须与模块一致)
#define CONSOLE_BAUD 921600 // 串口监视器波特率 (必须足够快以防阻塞)
#define TIMEOUT_MS 1500     // 断连超时时间 (毫秒)

// ================= 对象实例化 =================
HardwareSerial gpsSerial(1); // 使用 UART1
bfs::Ubx ubx(&gpsSerial);    // 绑定 UBX 解析器

// ================= 全局变量 =================
uint32_t last_packet_ms = 0;  // 上次收到数据的时间戳
uint32_t msg_count = 0;       // 消息计数器
uint32_t last_hz_calc_ms = 0; // Hz 计算计时器
float update_rate_hz = 0.0;   // 当前刷新率
bool is_connected = false;    // 连接状态标志

// 辅助函数：将定位状态枚举转换为中文/英文描述
const char *getFixStr(bfs::Ubx::Fix fix)
{
  switch (fix)
  {
  case bfs::Ubx::FIX_NONE:
    return "无定位 (No Fix)";
  case bfs::Ubx::FIX_2D:
    return "2D定位 (2D)";
  case bfs::Ubx::FIX_3D:
    return "3D定位 (3D)";
  case bfs::Ubx::FIX_DGNSS:
    return "差分增强 (DGNSS/SBAS)";
  case bfs::Ubx::FIX_RTK_FLOAT:
    return "RTK 浮点解 (Float)"; // 精度分米级
  case bfs::Ubx::FIX_RTK_FIXED:
    return "RTK 固定解 (Fixed)"; // 精度厘米级
  default:
    return "未知 (Unknown)";
  }
}

void setup()
{
  // 1. 初始化调试串口
  Serial.begin(CONSOLE_BAUD);
  while (!Serial)
    delay(10);

  Serial.println("\n\n=== ESP32 GPS 深度解析系统启动 ===");
  Serial.println("等待 GPS 数据流 (NAV-PVT / NAV-DOP)...");

  // 2. 初始化 GPS 串口
  // 注意：我们手动调用 begin，避免库内部重置引脚映射
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // 3. 将串口指针传递给库
  ubx.Config(&gpsSerial);

  last_packet_ms = millis();
}

void loop()
{
  // ubx.Read() 会不断读取缓冲区，直到解析完一个完整的导航周期 (EOE)
  if (ubx.Read())
  {
    uint32_t now = millis();
    last_packet_ms = now;
    is_connected = true;

    // --- 计算实际数据刷新率 (Hz) ---
    msg_count++;
    if (now - last_hz_calc_ms >= 1000)
    {
      update_rate_hz = (float)msg_count * 1000.0f / (now - last_hz_calc_ms);
      msg_count = 0;
      last_hz_calc_ms = now;
    }

    bfs::Ubx::Fix fixType = ubx.fix();

    // 只有当定位状态有效（非 None）时，打印详细数据
    if (fixType != bfs::Ubx::FIX_NONE)
    {

      Serial.println("\n====================== [ 导航数据详解 ] ======================");

      // ------------------------------------------------------------------
      // 1. 系统状态与时间 (System Status & Time)
      // ------------------------------------------------------------------
      // Rate: 实际接收到的包频率，用于检查是否丢包。
      // UTC: 协调世界时，由原子钟维持，非常精准。
      Serial.printf("[状态] 模式: %-15s | 卫星数: %-2d | 刷新率: %4.1f Hz\n",
                    getFixStr(fixType), ubx.num_sv(), update_rate_hz);

      Serial.printf("[时间] UTC:  %04d年%02d月%02d日 %02d:%02d:%02d.%03d (纳秒偏移: %d ns)\n",
                    ubx.utc_year(), ubx.utc_month(), ubx.utc_day(),
                    ubx.utc_hour(), ubx.utc_min(), ubx.utc_sec(),
                    ubx.utc_nano() / 1000000, ubx.utc_nano());

      // ------------------------------------------------------------------
      // 2. 地理定位信息 (Geodetic Position)
      // ------------------------------------------------------------------
      // Lat/Lon: 经纬度，保留 8 位小数可精确到毫米级。
      // MSL (Mean Sea Level): 海拔高度，即相对于“大地水准面”的高度（我们日常理解的海拔）。
      // WGS84 (Ellipsoid): 椭球高度，相对于数学定义的椭球体的高度。
      // 注意：MSL = WGS84 - 地球重力差距 (Geoid Separation)
      Serial.println("--- 位置信息 (Position) ---");
      Serial.printf("  纬度 (Lat):   %13.8f 度\n", ubx.lat_deg());
      Serial.printf("  经度 (Lon):   %13.8f 度\n", ubx.lon_deg());
      Serial.printf("  海拔 (MSL):   %10.3f 米 (常用高度)\n", ubx.alt_msl_m());
      Serial.printf("  椭球高(WGS84):%10.3f 米 (数学高度)\n", ubx.alt_wgs84_m());

      // ------------------------------------------------------------------
      // 3. NED 速度矢量 (NED Velocity) - 来自 NAV-PVT
      // ------------------------------------------------------------------
      // 这是无人机/机器人控制最关键的数据！
      // 北向速度 (N): 正=向北，负=向南
      // 东向速度 (E): 正=向东，负=向西
      // 地向速度 (D): 正=向下(下降)，负=向上(上升) <--- 注意这里！
      Serial.println("--- 速度矢量 (NED Velocity) ---");
      Serial.printf("  北向速度 (N): %10.3f m/s\n", ubx.north_vel_mps());
      Serial.printf("  东向速度 (E): %10.3f m/s\n", ubx.east_vel_mps());
      Serial.printf("  地向速度 (D): %10.3f m/s (正值代表下降!)\n", ubx.down_vel_mps());

      // ------------------------------------------------------------------
      // 4. 航迹与合成速度 (Track & Ground Speed)
      // ------------------------------------------------------------------
      // 地速: 水平方向的合成速度 (不包含垂直速度)。
      // 航向 (Heading): 运动方向的角度 (0=北, 90=东)。注意：这是“运动方向”，不是“机头朝向”。
      // 如果设备静止，航向数据通常会乱跳，这是正常的。
      Serial.println("--- 航迹信息 (Track) ---");
      Serial.printf("  地面速度:     %10.3f m/s\n", ubx.gnd_spd_mps());
      Serial.printf("  运动航向:     %10.3f 度\n", ubx.track_deg());

      // ------------------------------------------------------------------
      // 5. 精度评估 (Accuracy Estimates)
      // ------------------------------------------------------------------
      // 这里的数值代表 1-sigma 或 CEP 误差范围。
      // H_Acc: 水平位置误差 (例如 0.02m 表示你有 68% 的概率在 2cm 范围内)。
      // V_Acc: 垂直误差 (通常比水平误差大 1.5~2 倍)。
      Serial.println("--- 精度评估 (Accuracy) ---");
      Serial.printf("  水平误差:     %10.3f 米\n", ubx.horz_acc_m());
      Serial.printf("  垂直误差:     %10.3f 米\n", ubx.vert_acc_m());
      Serial.printf("  速度误差:     %10.3f m/s\n", ubx.spd_acc_mps());
      Serial.printf("  航向误差:     %10.3f 度\n", ubx.track_acc_deg());

      // ------------------------------------------------------------------
      // 6. 卫星分布精度因子 (DOP - Dilution of Precision) - 来自 NAV-DOP
      // ------------------------------------------------------------------
      // PDOP (位置精度因子): 综合评价 3D 位置的好坏。
      // 值越小越好：
      // < 1.0: 极佳 | 1.0-2.0: 优秀 | > 5.0: 较差 (不可信)
      Serial.println("--- 卫星几何因子 (DOP) ---");
      Serial.printf("  PDOP (综合):  %5.2f (越小越好)\n", ubx.pdop());
      Serial.printf("  HDOP (水平):  %5.2f\n", ubx.hdop());
      Serial.printf("  VDOP (垂直):  %5.2f\n", ubx.vdop());
    }
    else
    {
      // 正在搜星时的简略显示
      Serial.printf("[等待] 正在搜索卫星... 可见卫星数: %d\n", ubx.num_sv());
    }
  }

  // --- 断连/超时检测 ---
  // 如果超过 TIMEOUT_MS 毫秒没有收到任何数据包
  if (millis() - last_packet_ms > TIMEOUT_MS)
  {
    if (is_connected)
    {
      Serial.println("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.printf("!!! 警告: GPS 连接断开 (已超时 %d ms) !!!\n", TIMEOUT_MS);
      Serial.println("请检查: 1.接线是否松动  2.波特率是否匹配  3.GPS是否掉电");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      is_connected = false;
    }
  }
}
