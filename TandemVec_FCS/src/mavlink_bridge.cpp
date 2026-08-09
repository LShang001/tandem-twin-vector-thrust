// ============================================================
//  mavlink_bridge.cpp — MAVLink 双向功能实现（QGC 桥接）
//
//  2026-08-10 双向扩展：
//    PARAM_REQUEST_LIST → PARAM_VALUE 逐条广播（121 条，200Hz 每拍 1 条 ≈0.6s 收全）
//    PARAM_REQUEST_READ → 按截断名/索引读单条
//    PARAM_SET          → 写入 + applyFlightCtrlParams + 回读确认（写失败也回当前值）
//    COMMAND_LONG       → 246 重启（回 ACK 后 300ms 软复位）；400 ARM / 176 SET_MODE
//                         安全拒绝（解锁/模式由 RC CH5 硬判定，地面站远程控制是红线）；
//                         其余 UNSUPPORTED；所有命令必回 COMMAND_ACK
//    STATUSTEXT         → 关键事件文本队列（1Hz 轮发）
//
//  参数名 char[16] 截断规则：线上名 >15B 时 QGC 侧按截断名匹配（唯一性已核实，
//  见 docs/mavlink-ref/MAVLink-参数服务.md）。参数表 = ano_params 注册表。
// ============================================================
#include "mavlink_bridge.h"
#include "ano_params.h"
#include <cstring>

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

// ------------------------------------------------------------------
//  发送工具
// ------------------------------------------------------------------
static void mavSend(const mavlink_message_t &msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    extern HardwareSerial Serial6;
    if (Serial6.availableForWrite() >= len + 4)
        Serial6.write(buf, len);
}

// 参数线上名 → MAVLink param_id（≤15B 唯一短名）
// ★ 2026-08-10 短名算法（Python 侧 121 参数唯一性已验证，测试双保险）：
//   field 缩写 out_min→omin / out_max→omax / int_limit→ilim / threshold→thr /
//   enabled→en；inertia_comp_mask→inertia_mask；其余截 15B。
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

// ------------------------------------------------------------------
//  COMMAND_LONG 处理
// ------------------------------------------------------------------
static void mavHandleCommandLong(const mavlink_message_t &msg)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    switch (cmd.command)
    {
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:  // 246
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

    default:
        mavSendCommandAck(cmd.command, MAV_RESULT_UNSUPPORTED);
        break;
    }
}

// ------------------------------------------------------------------
//  上行分发（handleAnoCom mavlink 分支调用）
// ------------------------------------------------------------------
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

    case MAVLINK_MSG_ID_COMMAND_LONG:  // 76
        mavHandleCommandLong(msg);
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

    // 1) PARAM_VALUE 广播状态机：每拍 1 条（200Hz → 121 条约 0.6s 收全）
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
