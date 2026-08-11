// ============================================================
//  ano_vars.cpp — 通用变量注册表 + 多协议上报（见 ano_vars.h）
//
//  预注册变量 = 控制/状态链路高价值中间量。加变量 = kAnoVars 加一行宏
//  （宏展开表顺序即变量 ID，与 ano_params 同模式）。
//
//  上报链路：
//    0xF2 值帧（AnoCom 灵活帧）：[id u16 LE] + [float LE] = 6B，watch 集合轮发
//    0xF3 清单帧：GCS 发 [0x01]→回 count；发 [0x02,id]→回 [id,type,name 16B]
//    DBG vars 命令：list/add/remove/clear/rate/status
//    MAVLink NAMED_VALUE_FLOAT/DEBUG_VECT：见 communication.cpp mavlinkSendTelemetry
// ============================================================
#include "ano_vars.h"
#include "AnoComProtocol.h"
#include "state_data.h"

// 注册表（宏展开，顺序即变量 ID）
// ★ 名称 ≤16 字符（0xF3 帧 PAR_NAME 定长 16B，超长截断导致 GCS 按名匹配失败）
// extern：const 定义默认 internal linkage，须显式 extern 与头声明一致；
// ★ 数组不带大小（[] 由初始化列表定长）——带 ANO_VARS_MAX_COUNT 会让 sizeof
//   按 64 计、ANO_VARS_COUNT 虚高，遍历到空项 name=nullptr 崩溃（host 实测 segfault）
extern const AnoVarEntry kAnoVars[] = {
    // ---- 控制链路中间量（gnc_tel，flight_control.cpp 层2 写入）----
    ANO_VAR_FLOAT("error_roll_deg",  gnc_tel.error_deg[0]),
    ANO_VAR_FLOAT("error_pitch_deg", gnc_tel.error_deg[1]),
    ANO_VAR_FLOAT("error_yaw_deg",   gnc_tel.error_deg[2]),
    ANO_VAR_FLOAT("omega_ref_roll",  gnc_tel.omega_ref_dps[0]),
    ANO_VAR_FLOAT("omega_ref_pitch", gnc_tel.omega_ref_dps[1]),
    ANO_VAR_FLOAT("omega_ref_yaw",   gnc_tel.omega_ref_dps[2]),
    // 兼容既有变量名：alpha_ref_* 始终为物理域 rad/s²。
    ANO_VAR_FLOAT("alpha_ref_x",     gnc_tel.alpha_ref_radps2[0]),
    ANO_VAR_FLOAT("alpha_ref_y",     gnc_tel.alpha_ref_radps2[1]),
    ANO_VAR_FLOAT("alpha_ref_z",     gnc_tel.alpha_ref_radps2[2]),
    ANO_VAR_FLOAT("m_cmd_x",         gnc_tel.M_cmd[0]),
    ANO_VAR_FLOAT("m_cmd_y",         gnc_tel.M_cmd[1]),
    ANO_VAR_FLOAT("m_cmd_z",         gnc_tel.M_cmd[2]),
    ANO_VAR_FLOAT("m_ff_x",          gnc_tel.M_ff[0]),
    ANO_VAR_FLOAT("m_ff_y",          gnc_tel.M_ff[1]),
    ANO_VAR_FLOAT("m_ff_z",          gnc_tel.M_ff[2]),
    ANO_VAR_FLOAT("w0_eff",          gnc_tel.w0_eff),
    ANO_VAR_FLOAT("yaw_gain_sched",  gnc_tel.yaw_gain_sched),
    ANO_VAR_FLOAT("delta_f_deg",     gnc_tel.delta_f_deg),
    ANO_VAR_FLOAT("delta_t_deg",     gnc_tel.delta_t_deg),
    ANO_VAR_FLOAT("dw",              gnc_tel.dw),
    ANO_VAR_BOOL("alloc_sat_df",     gnc_tel.alloc_sat[0]),
    ANO_VAR_BOOL("alloc_sat_dt",     gnc_tel.alloc_sat[1]),
    ANO_VAR_BOOL("alloc_sat_dw",     gnc_tel.alloc_sat[2]),
    // ---- τm 观测器（flight_control.cpp 暴露为全局，用户点名观测量）----
    ANO_VAR_FLOAT("wf_est",          g_wf_est),
    ANO_VAR_FLOAT("wt_est",          g_wt_est),
    // ---- 状态/高度/电压 ----
    ANO_VAR_FLOAT("est_height",      estimated_height),
    ANO_VAR_FLOAT("est_vel",         estimated_velocity),
    ANO_VAR_FLOAT("vfk_height",      vfk_height),
    ANO_VAR_FLOAT("baro_alt",        baro_altitude),
    ANO_VAR_FLOAT("bat_mv",          bat_voltage_mv),
    ANO_VAR_FLOAT("throttle_pct",    throttlePercent),
    ANO_VAR_FLOAT("ch3_out_pct",     ch3_output),
    ANO_VAR_FLOAT("ch4_out_pct",     ch4_output),
    ANO_VAR_FLOAT("tvc_upper_deg",   g_tvc_upper_deg),
    ANO_VAR_FLOAT("tvc_lower_deg",   g_tvc_lower_deg),
    ANO_VAR_BOOL("unlocked",         g_is_unlocked),
    // ---- 在线辨识（角速度模式 DBG id 激励后更新）----
    ANO_VAR_FLOAT("id_b_roll",       id_b_est[0]),
    ANO_VAR_FLOAT("id_b_pitch",      id_b_est[1]),
    ANO_VAR_FLOAT("id_b_yaw",        id_b_est[2]),
    ANO_VAR_FLOAT("id_d_yaw",        id_d_est[2]),
    ANO_VAR_FLOAT("id_kp_roll",      id_kp_suggest[0]),
    ANO_VAR_FLOAT("id_kp_pitch",     id_kp_suggest[1]),
    ANO_VAR_FLOAT("id_kp_yaw",       id_kp_suggest[2]),
    // ---- EKF 速度（NED，m/s）----
    ANO_VAR_FLOAT("ekf_vel_n",       INS_GNSS_Packet.velocity_north),
    ANO_VAR_FLOAT("ekf_vel_e",       INS_GNSS_Packet.velocity_east),
    ANO_VAR_FLOAT("ekf_vel_d",       INS_GNSS_Packet.velocity_down),
    // 角度域调参观测量追加在表尾，保持既有变量 ID 不变。
    // 名称同时满足 AnoCom 16B 与 MAVLink NAMED_VALUE_FLOAT 9 字符上限；
    // 三轴在 MAVLink 中必须保持唯一，不能用会同截断为 alpha_dps 的长前缀。
    ANO_VAR_FLOAT("adps2_x",          gnc_tel.alpha_ref_dps2[0]),
    ANO_VAR_FLOAT("adps2_y",          gnc_tel.alpha_ref_dps2[1]),
    ANO_VAR_FLOAT("adps2_z",          gnc_tel.alpha_ref_dps2[2]),
};
static_assert(sizeof(kAnoVars) / sizeof(kAnoVars[0]) <= ANO_VARS_MAX_COUNT,
              "ano_vars: 注册表超上限");

// extern：const 定义默认 internal linkage，须显式 extern 与头声明一致
extern const uint16_t ANO_VARS_COUNT = sizeof(kAnoVars) / sizeof(kAnoVars[0]);

// 上报集合状态（默认 50Hz）
AnoVarWatchState g_ano_var_watch = { {0}, 0, 50 };

// 变量 ID → 表项（越界返回 nullptr；MAVLink 调试通道共用）
const AnoVarEntry *anoVarAt(uint16_t id)
{
    if (id >= ANO_VARS_COUNT)
        return nullptr;
    return &kAnoVars[id];
}

// ------------------------------------------------------------------
//  0xF3 清单请求应答（仿 0xE0/0xE2 模式，只读不回 0x00 校验帧）
// ------------------------------------------------------------------
static void anoVarsHandleList(AnoComProtocol &ano, const uint8_t *data, uint16_t len)
{
    if (len < 1)
        return;
    // ★ 2026-08-10 越界修复：变量信息帧 = CMD(1) + ID(2) + TYPE(1) + NAME(16) = 20B，
    //   原数组 1+2+16=19B 致 memset(&tx[4],0,16) 越界写 1 字节（编译 -Warray-bounds 坐实）
    uint8_t tx[1 + 2 + 1 + 16];
    if (data[0] == 0x01) // 读变量个数
    {
        tx[0] = 0x01;
        tx[1] = ANO_VARS_COUNT & 0xFF;
        tx[2] = (ANO_VARS_COUNT >> 8) & 0xFF;
        ano.sendData(ANO_GND_STATION_ADDR, 0xF3, tx, 3);
        return;
    }
    if (data[0] == 0x02 && len >= 3) // 读变量信息 [id u16]
    {
        uint16_t id = (uint16_t)(data[1] | (data[2] << 8));
        const AnoVarEntry *e = anoVarAt(id);
        if (!e)
            return;
        tx[0] = 0x02;
        tx[1] = id & 0xFF;
        tx[2] = (id >> 8) & 0xFF;
        tx[3] = e->type;
        memset(&tx[4], 0, 16); // PAR_NAME 定长 16B
        uint16_t n = 0;
        while (e->name[n] && n < 15)
            n++;
        memcpy(&tx[4], e->name, n);
        ano.sendData(ANO_GND_STATION_ADDR, 0xF3, tx, 4 + 16);
        return;
    }
    // 未实现命令：静默（仿 0xE2 无校验回执）
}

// ------------------------------------------------------------------
//  0xF2 值帧发送（handleAnoCom 200Hz 调用，内部节流 + watch 轮转）
//  每 (200/rate) tick 发一帧（当前轮转变量）——watch 1 个变量即达
//  rate Hz；N 个变量时每个 = rate/N Hz。live-read，不走组缓存。
// ------------------------------------------------------------------
static uint32_t s_vars_tick = 0;
static uint8_t s_vars_idx = 0;

void anoVarsSendTick(AnoComProtocol &ano)
{
    if (anoVarWatchEmpty())
        return;
    const uint32_t period = 200u / (uint32_t)g_ano_var_watch.rate_hz; // 1-200Hz
    if (s_vars_tick++ % period != 0)
        return;

    uint8_t tx[6];
    uint16_t id = g_ano_var_watch.ids[s_vars_idx];
    const AnoVarEntry *e = anoVarAt(id);
    if (!e)
    {
        s_vars_idx = 0;
        return;
    }
    float v = anoVarGetValue(*e);
    tx[0] = id & 0xFF;
    tx[1] = (id >> 8) & 0xFF;
    memcpy(&tx[2], &v, 4);
    ano.sendData(ANO_GND_STATION_ADDR, 0xF2, tx, 6);

    s_vars_idx = (s_vars_idx + 1) % g_ano_var_watch.count;
}

// ------------------------------------------------------------------
//  上行分派（communication.cpp onAnoRxFrame 调用）
// ------------------------------------------------------------------
void anoVarsHandleRx(AnoComProtocol &ano, uint8_t funcCode,
                     const uint8_t *data, uint16_t len)
{
    if (funcCode == 0xF3)
        anoVarsHandleList(ano, data, len);
    // 0xF2 是下行值帧，无上行
}

// ------------------------------------------------------------------
//  DBG vars 命令（handleDebugConsole 分派）
// ------------------------------------------------------------------
void anoVarsDebugCommand(HardwareSerial &serial, char *args)
{
    // 跳前导空格（分派传入 line+4，命令与参数间有空格）
    while (args && *args == ' ')
        args++;
    // 无参数：状态
    if (!args || args[0] == '\0')
    {
        serial.print(F("[DBG] vars: watch "));
        serial.print(g_ano_var_watch.count);
        serial.print(F("/"));
        serial.print(ANO_VARS_MAX_WATCH);
        serial.print(F(" @"));
        serial.print(g_ano_var_watch.rate_hz);
        serial.println(F("Hz  (list/add <name>/remove <name>/clear/rate <hz>)"));
        for (uint8_t i = 0; i < g_ano_var_watch.count; i++)
        {
            const AnoVarEntry *e = anoVarAt(g_ano_var_watch.ids[i]);
            serial.print(F("  ["));
            serial.print(i);
            serial.print(F("] "));
            serial.println(e ? e->name : "?");
        }
        return;
    }
    if (strncmp(args, "list", 4) == 0 && (args[4] == '\0' || args[4] == ' '))
    {
        serial.print(F("[DBG] vars list: "));
        serial.print(ANO_VARS_COUNT);
        serial.println(F(" registered"));
        for (uint16_t i = 0; i < ANO_VARS_COUNT; i++)
        {
            serial.print(F("  "));
            serial.print(i);
            serial.print(F(" "));
            serial.print(kAnoVars[i].name);
            serial.print(F(" type="));
            serial.println((int)kAnoVars[i].type);
        }
        return;
    }
    if (strncmp(args, "add ", 4) == 0)
    {
        char *name = args + 4;
        // 去尾空白
        char *end = name + strlen(name);
        while (end > name && (end[-1] == ' ' || end[-1] == '\r' || end[-1] == '\n'))
            *--end = '\0';
        if (anoVarWatchAddName(name))
        {
            serial.print(F("[DBG] vars: added '"));
            serial.print(name);
            serial.print(F("' ("));
            serial.print(g_ano_var_watch.count);
            serial.println(F("/16)"));
        }
        else
        {
            serial.print(F("[DBG] vars: add failed — 未找到 '"));
            serial.print(name);
            serial.println(F("' 或已存在/满 16"));
        }
        return;
    }
    if (strncmp(args, "remove ", 7) == 0)
    {
        if (anoVarWatchRemoveName(args + 7))
        {
            serial.print(F("[DBG] vars: removed '"));
            serial.print(args + 7);
            serial.println(F("'"));
        }
        else
        {
            serial.print(F("[DBG] vars: remove failed — 未找到 '"));
            serial.print(args + 7);
            serial.println(F("'"));
        }
        return;
    }
    if (strncmp(args, "clear", 5) == 0)
    {
        anoVarWatchClear();
        serial.println(F("[DBG] vars: watch cleared"));
        return;
    }
    if (strncmp(args, "rate ", 5) == 0)
    {
        int hz = atoi(args + 5);
        if (hz < 1 || hz > 200)
        {
            serial.println(F("[DBG] vars: rate 范围 1-200Hz"));
            return;
        }
        anoVarWatchSetRate((uint8_t)hz);
        serial.print(F("[DBG] vars: rate="));
        serial.print(g_ano_var_watch.rate_hz);
        serial.println(F("Hz"));
        return;
    }
    serial.println(F("[DBG] vars: 用法 list / add <name> / remove <name> / clear / rate <hz>"));
}
