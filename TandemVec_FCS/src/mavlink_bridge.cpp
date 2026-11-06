/**
 * @file    mavlink_bridge.cpp
 * @module  固件/通信（MAVLink 双向桥：QGC / Mission Planner）
 * @brief   实现 MAVLink 参数服务、命令应答与 STATUSTEXT 队列
 *
 * @details 上行：
 *          PARAM_REQUEST_LIST → PARAM_VALUE 逐条广播（条数 = ANO_PARAMS_COUNT，
 *          200Hz 每拍 1 条，约 0.6s 收全）；
 *          PARAM_REQUEST_READ → 按截断名/索引读单条；
 *          PARAM_SET → 写入 + `applyFlightCtrlParams()` + 回读确认（写失败也回当前值）；
 *          COMMAND_LONG → 246 重启（回 ACK 后 300ms 软复位）；400 ARM / 176 SET_MODE
 *          安全拒绝（解锁/模式由 RC CH5 硬判定，地面站远程控制是红线）；其余 UNSUPPORTED；
 *          所有命令必回 COMMAND_ACK。
 *          下行：PARAM_VALUE 广播状态机、COMMAND_ACK、STATUSTEXT 队列（1Hz 轮发）。
 * @theory  N/A（协议桥接实现）
 * @ref     docs/mavlink-ref/MAVLink-参数服务.md、docs/mavlink-ref/MAVLink-命令与状态.md
 * @frame   N/A（消息字段语义由 MAVLink 定义）
 * @units   参数一律 float；STATUSTEXT 文本 ≤50B；param_id ≤15B 可见字符
 * @note    参数名 char[16] 截断规则：线上名 >15B 时 QGC 侧按截断名匹配（唯一性已核实，
 *          见 docs/mavlink-ref/MAVLink-参数服务.md）。参数表 = ano_params 注册表，
 *          数量以 ANO_PARAMS_COUNT 为准（追加 rc_* 后短名仍 ≤15B，无需额外截断）。
 */
#include "mavlink_bridge.h"
#include "ano_params.h"
#include "state_data.h"   // g_is_unlocked / failsafe_in_flight（重启互锁）
#include <cstring>
#include <cmath>

// ★ 2026-11-06 补齐下行消息所需的导航/控制量（定义在 communication.cpp / navigation_task.cpp）
extern float relative_north;
extern float relative_east;
extern float relative_down;
extern float rollTarget;
extern float pitchTarget;
extern float ch3_output;
extern float ch4_output;
extern float baro_altitude;
extern float temperature;
extern float g_tvc_upper_deg;
extern float g_tvc_lower_deg;
extern double origin_lat_deg, origin_lon_deg, origin_lat_rad, origin_lon_rad, origin_alt_m;

// ---- 参数广播状态机 ----
static bool  s_param_broadcast_active = false;
static uint16_t s_param_broadcast_idx = 0;

// ---- STATUSTEXT 环形队列 ----
#define MAV_STATUS_Q_LEN 4
struct StatusEntry { uint8_t severity; char text[51]; };
static StatusEntry s_status_q[MAV_STATUS_Q_LEN];
static uint8_t s_status_q_head = 0;   // 下一个写入位置
static uint8_t s_status_q_count = 0;  // 待发条数
static uint8_t s_status_q_send = 0;   // 下一个发送位置

// ---- 重启挂起 ----
static bool s_reboot_pending = false;

// ==================================================================
// ★ 2026-11-06 消息间隔控制（MAV_CMD_SET_MESSAGE_INTERVAL / 511）
// ------------------------------------------------------------------
// 固件原本按 s_tick 硬分频，地面站无法干预。实现这条命令后，QGC 的
// 「消息间隔」面板可直接调每条消息频率（-1 停发），带宽按需分配。
// 只拦截可调遥测；ACK / PARAM_VALUE / STATUSTEXT 等协议必需帧一律不拦。
// ==================================================================
static bool mavIsTunable(uint32_t id)
{
    switch (id)
    {
    case 27:  case 30:  case 31:  case 32:  case 33:
    case 35:  case 36:  case 74:  case 83:  case 105:
    case 141: case 230: case 250: case 251:
        return true;
    default:
        return false;
    }
}

static uint32_t s_msg_interval_us[256];
static uint32_t s_msg_last_us[256];

/** 该消息本拍是否允许发送。未设置间隔时返回 true，沿用原有 tick 分频。 */
bool mavIntervalOk(uint32_t id)
{
    if (id >= 256U || !mavIsTunable(id))
    {
        return true;
    }
    const uint32_t iv = s_msg_interval_us[id];
    if (iv == 0U)
    {
        return true;                      // 未设置 → 默认分频
    }
    if (iv == 0xFFFFFFFFu)
    {
        return false;                     // -1 → 停发
    }
    const uint32_t now = (uint32_t)micros();
    if ((uint32_t)(now - s_msg_last_us[id]) < iv)
    {
        return false;                     // 还没到点
    }
    s_msg_last_us[id] = now;
    return true;
}

// ------------------------------------------------------------------
//  发送工具
// ------------------------------------------------------------------
static void mavSend(const mavlink_message_t &msg)
{
    if (!mavIntervalOk(msg.msgid))
    {
        return;                            // 地面站把这条调停了 / 还没到间隔
    }
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    extern Uart Serial6;
    if (Serial6.availableForWrite() >= len + 4)
        Serial6.write(buf, len);
}

// 参数线上名 → MAVLink param_id（≤15B 唯一短名）
// ★ 2026-08-10 短名算法（GCS/server/params.py 侧已按名称做唯一性校验，测试双保险）：
//   field 缩写 out_min→omin / out_max→omax / int_limit→ilim / threshold→thr /
//   enabled→en；inertia_comp_mask→inertia_mask；rc_rate/rc_expo[i]/rc_super[i] 原样（≤10B）；
//   其余截 15B。
//   注意：直接截断有冲突（att_pitch.out_min/out_max 都变 out_m，实测）！
static void mavParamIdOfName(const char *name, char out[16])
{
    memset(out, 0, 16);
    if (!name)
        return;
    // 特殊映射（无 loop 前缀的全局参数）
    if (strcmp(name, "inertia_comp_mask") == 0)
    {
        memcpy(out, "inertia_mask", 12);
        return;
    }
    // 拆分 loop.field
    const char *dot = strchr(name, '.');
    const char *field = dot ? dot + 1 : nullptr;
    const char *shortField = nullptr;
    if (field)
    {
        if (strcmp(field, "out_min") == 0) shortField = "omin";
        else if (strcmp(field, "out_max") == 0) shortField = "omax";
        else if (strcmp(field, "int_limit") == 0) shortField = "ilim";
        else if (strcmp(field, "threshold") == 0) shortField = "thr";
        else if (strcmp(field, "enabled") == 0) shortField = "en";
    }
    // 拼 "loop.field"（field 用缩写），超 15B 截 loop 段
    char cand[24];
    if (field)
    {
        uint16_t loopLen = (uint16_t)(dot - name);
        if (loopLen > 12) loopLen = 12;  // 截 loop 到 12（总长 ≤15）
        memcpy(cand, name, loopLen);
        cand[loopLen] = '.';
        const char *f = shortField ? shortField : field;
        strcpy(cand + loopLen + 1, f);
    }
    else
    {
        strcpy(cand, name);
    }
    uint8_t n = 0;
    while (cand[n] && n < 15)
        n++;
    memcpy(out, cand, n);
}

// 参数 ID 版（mavParamIdOfName 封装）
static void mavParamIdOf(uint16_t id, char out[16])
{
    mavParamIdOfName(anoParamNameAt(id), out);
}

// ★ 短名 → 参数 ID（QGC 回传的 param_id 是短名，须反向遍历生成比对——
//   原始名前缀匹配会失败："att_roll.omin" vs "att_roll.out_min"）
static int16_t mavParamIdFromMavName(const char *mavName)
{
    if (!mavName)
        return -1;
    for (uint16_t i = 0; i < anoParamCount(); i++)
    {
        char cand[16];
        mavParamIdOf(i, cand);
        if (strcmp(cand, mavName) == 0)
            return (int16_t)i;
    }
    return -1;
}

// 发单条 PARAM_VALUE（param_index 语义：-1 = 独立查询无索引）
static void mavSendParamValue(uint16_t id, int16_t param_index)
{
    float v = 0.0f;
    anoParamReadFloat(id, &v);
    char pid[16];
    mavParamIdOf(id, pid);
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                 pid, v, MAV_PARAM_TYPE_REAL32,
                                 anoParamCount(), param_index);
    mavSend(msg);
}

// 回 COMMAND_ACK（所有命令必回，文档硬性要求）
static void mavSendCommandAck(uint16_t command, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                 command, result, 0, 0, 0, 0);
    mavSend(msg);
}

// AUTOPILOT_VERSION（QGC/MP 连接能力协商；不宣称任务/遥控接管）
static void mavSendAutopilotVersion()
{
    static const uint8_t custom[8] = {'T','V','E','C',0,0,0,0};
    static const uint8_t uid2[18] = {0};
    mavlink_message_t msg;
    mavlink_msg_autopilot_version_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg,
        0,                    // capabilities：无 mission/command-int 之外的承诺
        0x00010000U,          // flight_sw_version 1.0
        0, 0, 0,
        custom, custom, custom,
        0, 0, 0, uid2);
    mavSend(msg);
}

// ------------------------------------------------------------------
//  COMMAND_LONG 处理
// ------------------------------------------------------------------
static void mavHandleCommand(const mavlink_command_long_t &cmd)
{
    switch (cmd.command)
    {
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:  // 246
        // ★2026-10-31 审计 P1-6：补解锁/飞行互锁。ARM 与 SET_MODE 都按“地面站远程
        //   控制是红线”安全拒绝，唯独重启命令没有判断 —— 飞行中一条 GCS/脚本命令
        //   即可软复位飞控，电机停转、飞机坠落。失控保护期间 g_is_unlocked 被
        //   failsafe 旁路为真，因此显式再排除 failsafe_in_flight。
        if (g_is_unlocked || failsafe_in_flight)
        {
            mavlinkSendStatustext(MAV_SEVERITY_WARNING,
                                  "Reboot denied: armed/in-flight");
            mavSendCommandAck(cmd.command, MAV_RESULT_DENIED);
            break;
        }
        if ((int32_t)cmd.param1 == 1)  // 1 = 重启飞控
        {
            mavSendCommandAck(cmd.command, MAV_RESULT_ACCEPTED);
            s_reboot_pending = true;  // mavlinkBridgeTick 末尾执行软复位
        }
        else
        {
            mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        }
        break;

    case MAV_CMD_COMPONENT_ARM_DISARM:  // 400 —— 安全红线：拒绝
        mavlinkSendStatustext(MAV_SEVERITY_WARNING,
                          "Arm via RC CH5 only (MAVLink arm denied)");
        mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        break;

    case MAV_CMD_DO_SET_MODE:  // 176 —— 模式由 RC 每帧判定，拒绝
        mavlinkSendStatustext(MAV_SEVERITY_WARNING,
                          "Mode via RC CH6 only (MAVLink mode denied)");
        mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        break;

    case MAV_CMD_SET_MESSAGE_INTERVAL:         // 511：地面站设置消息频率
    {
        const uint32_t mid = (uint32_t)cmd.param1;
        const int32_t iv = (int32_t)cmd.param2;   // 微秒；-1 停发；0 恢复默认
        if (mid < 256U && mavIsTunable(mid))
        {
            s_msg_interval_us[mid] = (iv < 0) ? 0xFFFFFFFFu : (uint32_t)iv;
            s_msg_last_us[mid] = 0;               // 重置计时，立即生效
            mavSendCommandAck(cmd.command, MAV_RESULT_ACCEPTED);
        }
        else
        {
            // 不在可调列表（协议必需帧或未知 ID）：明确拒绝，别让地面站空等
            mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        }
        break;
    }

    case MAV_CMD_REQUEST_MESSAGE:              // 512：请求指定消息
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: // 520
        mavSendAutopilotVersion();
        mavSendCommandAck(cmd.command, MAV_RESULT_ACCEPTED);
        break;

    default:
        mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        break;
    }
}

static void mavHandleCommandLong(const mavlink_message_t &msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    mavHandleCommand(cmd);
}

static void mavHandleCommandInt(const mavlink_message_t &msg)
{
    mavlink_command_int_t ci;
    mavlink_msg_command_int_decode(&msg, &ci);
    // 现有命令只使用 param1/param2/command；转换为统一结构处理。
    mavlink_command_long_t cmd{};
    cmd.command = ci.command;
    cmd.param1 = ci.param1;
    cmd.param2 = ci.param2;
    cmd.param3 = ci.param3;
    cmd.param4 = ci.param4;
    cmd.param5 = static_cast<float>(ci.x);
    cmd.param6 = static_cast<float>(ci.y);
    cmd.param7 = ci.z;
    mavHandleCommand(cmd);
}

// ------------------------------------------------------------------
//  上行分发（handleAnoCom mavlink 分支调用）
// ------------------------------------------------------------------
static void mavSendMissionAck(const mavlink_message_t &src, uint8_t type)
{
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                 src.sysid, src.compid, type, 0, 0);
    mavSend(msg);
}

static void mavSendMissionCountZero(const mavlink_message_t &src)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                   src.sysid, src.compid, 0, 0, 0);
    mavSend(msg);
}

static void mavSendPingReply(const mavlink_message_t &src)
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&src, &ping);
    mavlink_message_t msg;
    mavlink_msg_ping_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                          ping.time_usec, ping.seq, src.sysid, src.compid);
    mavSend(msg);
}

static void mavSendTimesyncReply(const mavlink_message_t &src)
{
    mavlink_timesync_t ts;
    mavlink_msg_timesync_decode(&src, &ts);
    mavlink_message_t msg;
    mavlink_msg_timesync_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                              ts.tc1, 0, src.sysid, src.compid);
    mavSend(msg);
}

void mavlinkHandleRx(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  // 21：广播全部参数
        s_param_broadcast_active = true;
        s_param_broadcast_idx = 0;
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  // 20：读单条（按名或按索引）
    {
        mavlink_param_request_read_t req;
        mavlink_msg_param_request_read_decode(&msg, &req);
        int16_t id = -1;
        if (req.param_index >= 0 && req.param_index < (int16_t)anoParamCount())
        {
            id = req.param_index;
        }
        else
        {
            id = mavParamIdFromMavName(req.param_id);  // 短名反向映射
        }
        if (id >= 0)
            mavSendParamValue((uint16_t)id, id);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:  // 23：写入 + 回读确认
    {
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(&msg, &set);
        int16_t id = mavParamIdFromMavName(set.param_id);  // 短名反向映射
        if (id >= 0)
        {
            // 统一 REAL32 来源；u8 参数按 0.0→false 其余→true 钳制（anoParamWriteFloat）
            anoParamWriteFloat((uint16_t)id, set.param_value);
        }
        // ★ 文档要求：无论成败都回当前值（QGC 比对期望值判断写入成功）
        if (id >= 0)
            mavSendParamValue((uint16_t)id, id);
        break;
    }

    case MAVLINK_MSG_ID_PING:  // 4：时间同步探测
        mavSendPingReply(msg);
        break;

    case MAVLINK_MSG_ID_TIMESYNC:  // 111：时间同步
        mavSendTimesyncReply(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:  // 76
        mavHandleCommandLong(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_INT:  // 75
        mavHandleCommandInt(msg);
        break;

    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  // 43：无任务，回0条
        mavSendMissionCountZero(msg);
        break;

    case MAVLINK_MSG_ID_MISSION_COUNT:          // 44
    case MAVLINK_MSG_ID_MISSION_REQUEST:        // 40
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:    // 51
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:       // 73
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // 45
        mavSendMissionAck(msg, MAV_MISSION_UNSUPPORTED);
        break;

    default:
        break;  // HEARTBEAT 等忽略（遥测下行持续应答 QGC 的存在性）
    }
}

// ------------------------------------------------------------------
//  下行周期发送（mavlinkSendTelemetry 内每拍调用，200Hz 时基）
// ------------------------------------------------------------------
void mavlinkBridgeTick()
{
    static uint32_t s_tick = 0;
    s_tick++;

    // ================================================================
    // ★ 2026-11-06 补齐的下行消息（QGC/MP 面板需要，此前缺失）
    // ================================================================

    // LOCAL_POSITION_NED 50Hz —— QGC「本地坐标」面板只读这条
    if ((s_tick % 4U) == 1U)
    {
        mavlink_message_t m;
        mavlink_msg_local_position_ned_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m, (uint32_t)millis(),
            relative_north, relative_east, -relative_down,
            (float)INS_GNSS_Packet.velocity_north,
            (float)INS_GNSS_Packet.velocity_east,
            (float)(-INS_GNSS_Packet.velocity_down));
        mavSend(m);
    }

    // ATTITUDE_TARGET 20Hz —— QGC「目标姿态」面板
    // type_mask=0xE7：只认 q，忽略体轴角速率与推力（本机是角速率指令）
    if ((s_tick % 10U) == 6U)
    {
        mavlink_message_t m;
        const float h = 0.00872664626f;   // deg → rad 再乘 0.5
        const float cr = cosf(rollTarget * h), sr = sinf(rollTarget * h);
        const float cp = cosf(pitchTarget * h), sp = sinf(pitchTarget * h);
        // 航向无绝对目标（本机 yaw 是速率指令），目标四元数取航向 0
        const float cy = 1.0f, sy = 0.0f;
        float q[4];
        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;
        mavlink_msg_attitude_target_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m, (uint32_t)millis(),
            0xE7, q, 0.0f, 0.0f, 0.0f, 0.0f);
        mavSend(m);
    }

    // SERVO_OUTPUT_RAW 10Hz —— QGC 舵机输出面板
    // 摆角 ±25° → 1000..2000us；电机 0..100% → 1000..2000us
    if ((s_tick % 20U) == 8U)
    {
        mavlink_message_t m;
        auto deg2pwm = [](float d) -> uint16_t {
            return (uint16_t)constrain(1500.0f + constrain(d, -25.0f, 25.0f) * 20.0f,
                                       1000.0f, 2000.0f);
        };
        auto pct2pwm = [](float p) -> uint16_t {
            return (uint16_t)constrain(1000.0f + constrain(p, 0.0f, 100.0f) * 10.0f,
                                       1000.0f, 2000.0f);
        };
        mavlink_msg_servo_output_raw_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m, (uint32_t)micros(), 0,
            deg2pwm(g_tvc_upper_deg), deg2pwm(g_tvc_lower_deg),
            pct2pwm(ch3_output), pct2pwm(ch4_output),
            0, 0, 0, 0,      // servo5..8
            0, 0, 0, 0,      // servo9..12
            0, 0, 0, 0);     // servo13..16
        mavSend(m);
    }

    // SCALED_PRESSURE 10Hz —— 气压高度/温度（由气压高度按 ISA 反算绝对压强）
    if ((s_tick % 20U) == 10U)
    {
        mavlink_message_t m;
        const float ratio = 1.0f - baro_altitude / 44330.7f;
        const float press = (ratio > 0.05f) ? 1013.25f * powf(ratio, 5.255f) : 1013.25f;
        mavlink_msg_scaled_pressure_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m, (uint32_t)millis(),
            press, 0.0f, (int16_t)(temperature * 100.0f), 0);
        mavSend(m);
    }

    // ================================================================
    // ★ 2026-11-06 第三批补齐：HIGHres IMU / EKF 健康 / 电源 / VTOL 状态 / 高度
    // ================================================================

    // HIGHRES_IMU 10Hz —— QGC「传感器 → 高频 IMU」图表（带温度与气压）
    if ((s_tick % 20U) == 12U)
    {
        mavlink_message_t m;
        const float ratio = 1.0f - baro_altitude / 44330.7f;
        const float press = (ratio > 0.05f) ? 1013.25f * powf(ratio, 5.255f) : 1013.25f;
        mavlink_msg_highres_imu_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m,
            (uint64_t)micros() * 1000ULL,
            (float)IMU_Packet.accelerometer_x, (float)IMU_Packet.accelerometer_y,
            (float)IMU_Packet.accelerometer_z,
            (float)IMU_Packet.gyroscope_x, (float)IMU_Packet.gyroscope_y,
            (float)IMU_Packet.gyroscope_z,
            0.0f, 0.0f, 0.0f,           // 无磁力计
            press, 0.0f,
            (float)baro_altitude, (float)temperature,
            0x07,                        // fields_updated: accel|gyro|mag 有效
            0);
        mavSend(m);
    }

    // ESTIMATOR_STATUS 2Hz —— QGC「EKF 状态」面板
    // flags: bit0 attitude / bit1 vel_horiz / bit2 vel_vert / bit3 pos_horiz / bit4 pos_vert
    if ((s_tick % 100U) == 14U)
    {
        mavlink_message_t m;
        const uint16_t flags = g_ekf_converged ? 0x001Fu : 0x0000u;
        mavlink_msg_estimator_status_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m,
            (uint64_t)micros() * 1000ULL, flags,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            g_ekf_pos_sigma_m,          // pos_horiz_accuracy（m）
            g_ekf_pos_sigma_m);         // pos_vert_accuracy（m）
        mavSend(m);
    }

    // POWER_STATUS 1Hz —— QGC 电源面板（5V 轨按 5.0V 估算）
    if ((s_tick % 200U) == 16U)
    {
        mavlink_message_t m;
        mavlink_msg_power_status_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m,
            5000,                                            // Vcc 5.00V
            (uint16_t)constrain(bat_voltage_mv, 0.0f, 65535.0f), // Vservo = 电池
            0);
        mavSend(m);
    }

    // EXTENDED_SYS_STATE 1Hz —— 告诉地面站这是 VTOL 且当前在地面/空中
    // vtol_state: 1=transition_to_fw 2=transition_to_mc 3=MC 4=FW
    // landed_state: 1=on_ground 2=in_air
    if ((s_tick % 200U) == 18U)
    {
        mavlink_message_t m;
        mavlink_msg_extended_sys_state_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m,
            3,                                    // 多旋翼/VTVL 模式
            g_is_unlocked ? 2 : 1);               // 解锁视为在空中
        mavSend(m);
    }

    // ALTITUDE 20Hz —— QGC 高度显示（相对/绝对/本地三套口径）
    if ((s_tick % 10U) == 4U)
    {
        mavlink_message_t m;
        const float alt_msl = (float)(estimated_height + (float)origin_alt_m);
        mavlink_msg_altitude_pack(
            1, MAV_COMP_ID_AUTOPILOT1, &m,
            (uint32_t)millis(),
            (float)baro_altitude, alt_msl, (float)baro_altitude,
            (float)estimated_height, 0.0f, 0.0f);
        mavSend(m);
    }

    // 0) AUTOPILOT_VERSION 1Hz：QGC/MP 连接时可靠获取能力，不依赖请求帧丢失
    if ((s_tick % 200U) == 50U)
    {
        mavSendAutopilotVersion();
    }

    // 1) PARAM_VALUE 广播状态机：每拍 1 条（200Hz → 全部参数约 0.6s 收全）
    if (s_param_broadcast_active)
    {
        mavSendParamValue(s_param_broadcast_idx, (int16_t)s_param_broadcast_idx);
        s_param_broadcast_idx++;
        if (s_param_broadcast_idx >= anoParamCount())
            s_param_broadcast_active = false;
    }

    // 2) STATUSTEXT 队列轮发（1Hz 节流）
    if (s_tick % 200 == 0 && s_status_q_count > 0)
    {
        const StatusEntry &e = s_status_q[s_status_q_send];
        mavlink_message_t msg;
        // 新格式（LEN=54）：severity + text[50] + id u16 + chunk_seq；id=0/chunk_seq=0 单条
        mavlink_msg_statustext_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                    e.severity, e.text, 0, 0);
        mavSend(msg);
        s_status_q_send = (s_status_q_send + 1) % MAV_STATUS_Q_LEN;
        s_status_q_count--;
    }

    // 3) 重启挂起（COMMAND_LONG 246 已回 ACK）——等 ACK 发出后执行
    if (s_reboot_pending)
    {
        s_reboot_pending = false;
        delay(300);
        NVIC_SystemReset();
    }
}

// ------------------------------------------------------------------
//  STATUSTEXT 入队
// ------------------------------------------------------------------
void mavlinkSendStatustext(uint8_t severity, const char *text)
{
    StatusEntry &e = s_status_q[s_status_q_head];
    e.severity = severity;
    memset(e.text, 0, sizeof(e.text));
    uint8_t n = 0;
    while (text && text[n] && n < 50)
        n++;
    if (n) memcpy(e.text, text, n);
    if (s_status_q_count < MAV_STATUS_Q_LEN)
        s_status_q_count++;
    s_status_q_head = (s_status_q_head + 1) % MAV_STATUS_Q_LEN;
}
