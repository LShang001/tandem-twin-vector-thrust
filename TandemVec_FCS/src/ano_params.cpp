// ============================================================
//  ano_params.cpp — AnoCom 参数在线读写实现（见 ano_params.h）
//
//  参数表 = kFlightCtrlParams 全部字段，宏展开生成 117 项：
//    PID_FLOAT(loop, field)  → { "loop.field", ANO_FLOAT,  &kFlightCtrlParams.loop.field }
//    PID_ENABLED(loop)       → { "loop.enabled", ANO_UINT8, (uint8_t*)&kFlightCtrlParams.loop.enabled }
//    ALPHA_ENTRY(name, arr,i)→ { "name[i]", ANO_FLOAT, &kFlightCtrlParams.name[i] }
//  名称/类型单一事实源：上位机经 0xE0 CMD 0x03 拉取 E2 信息帧获取。
// ============================================================
#include "ano_params.h"
#include "FlightCtrlParams.h"
#include "state_data.h"   // applyFlightCtrlParams()

// 设备地址常量（协议固定）
static const uint8_t ANO_GND_ADDR = ANO_GND_STATION_ADDR; // 0xFF

// ---- 参数表项 ----
struct AnoParamEntry
{
    const char *name;   // 参数名（≤20 字符，E2 帧按 20B 定长填充）
    uint8_t type;       // AnoDataType（ANO_FLOAT=8 / ANO_UINT8=0）
    float *fptr;        // float 字段指针（type==ANO_FLOAT）
    uint8_t *bptr;      // bool 字段指针（type==ANO_UINT8，存 0/1）
    uint8_t *u8ptr;     // ★ 2026-08-10 全面审查修复：uint8 值字段指针（0-255 语义，
                        //   区别于 bptr 的 bool 语义）——inertia_comp_mask 双位掩码
                        //   此前被 bool 化（0xE1 写 2/3 被拒、MAVLink 钳 0/1），
                        //   A/B 测试无法恢复默认 0x03
};

#define PID_FLOAT(loop, field) \
    { #loop "." #field, ANO_FLOAT, &kFlightCtrlParams.loop.field, nullptr, nullptr }
#define PID_ENABLED(loop) \
    { #loop ".enabled", ANO_UINT8, nullptr, (uint8_t *)&kFlightCtrlParams.loop.enabled, nullptr }
// ★ 参数名 ≤20B（协议 PAR_NAME 定长 20B，超限被截断导致上位机按名匹配失败）：
//   filter_alpha → falpha；滤波数组用短名
#define PID_ALPHA(loop) \
    { #loop ".falpha", ANO_FLOAT, &kFlightCtrlParams.loop.filter_alpha, nullptr, nullptr }
#define ALPHA_ENTRY(name, field, idx) \
    { name "[" #idx "]", ANO_FLOAT, &kFlightCtrlParams.field[idx], nullptr, nullptr }
#define PID_U8(name, var) \
    { name, ANO_UINT8, nullptr, nullptr, &(var) }

// 参数注册表（顺序即参数 ID，0..ANO_PARAMS_COUNT-1）
// ★ 与上位机 GCS/server/params.py 的分组/单位元数据按名称匹配，勿改命名
static const AnoParamEntry kAnoParams[ANO_PARAMS_COUNT] = {
    // ---- 姿态外环 ----
    PID_FLOAT(att_roll, kp), PID_FLOAT(att_roll, ki), PID_FLOAT(att_roll, kd),
    PID_FLOAT(att_roll, out_min), PID_FLOAT(att_roll, out_max),
    PID_FLOAT(att_roll, int_limit), PID_FLOAT(att_roll, threshold),
    PID_ALPHA(att_roll), PID_ENABLED(att_roll),
    PID_FLOAT(att_pitch, kp), PID_FLOAT(att_pitch, ki), PID_FLOAT(att_pitch, kd),
    PID_FLOAT(att_pitch, out_min), PID_FLOAT(att_pitch, out_max),
    PID_FLOAT(att_pitch, int_limit), PID_FLOAT(att_pitch, threshold),
    PID_ALPHA(att_pitch), PID_ENABLED(att_pitch),
    PID_FLOAT(att_yaw, kp), PID_FLOAT(att_yaw, ki), PID_FLOAT(att_yaw, kd),
    PID_FLOAT(att_yaw, out_min), PID_FLOAT(att_yaw, out_max),
    PID_FLOAT(att_yaw, int_limit), PID_FLOAT(att_yaw, threshold),
    PID_ALPHA(att_yaw), PID_ENABLED(att_yaw),
    // ---- 角速率内环 ----
    PID_FLOAT(rate_roll, kp), PID_FLOAT(rate_roll, ki), PID_FLOAT(rate_roll, kd),
    PID_FLOAT(rate_roll, out_min), PID_FLOAT(rate_roll, out_max),
    PID_FLOAT(rate_roll, int_limit), PID_FLOAT(rate_roll, threshold),
    PID_ALPHA(rate_roll), PID_ENABLED(rate_roll),
    PID_FLOAT(rate_pitch, kp), PID_FLOAT(rate_pitch, ki), PID_FLOAT(rate_pitch, kd),
    PID_FLOAT(rate_pitch, out_min), PID_FLOAT(rate_pitch, out_max),
    PID_FLOAT(rate_pitch, int_limit), PID_FLOAT(rate_pitch, threshold),
    PID_ALPHA(rate_pitch), PID_ENABLED(rate_pitch),
    PID_FLOAT(rate_yaw, kp), PID_FLOAT(rate_yaw, ki), PID_FLOAT(rate_yaw, kd),
    PID_FLOAT(rate_yaw, out_min), PID_FLOAT(rate_yaw, out_max),
    PID_FLOAT(rate_yaw, int_limit), PID_FLOAT(rate_yaw, threshold),
    PID_ALPHA(rate_yaw), PID_ENABLED(rate_yaw),
    // ---- 垂直串级 ----
    PID_FLOAT(alt_pos, kp), PID_FLOAT(alt_pos, ki), PID_FLOAT(alt_pos, kd),
    PID_FLOAT(alt_pos, out_min), PID_FLOAT(alt_pos, out_max),
    PID_FLOAT(alt_pos, int_limit), PID_FLOAT(alt_pos, threshold),
    PID_ALPHA(alt_pos), PID_ENABLED(alt_pos),
    PID_FLOAT(alt_vel, kp), PID_FLOAT(alt_vel, ki), PID_FLOAT(alt_vel, kd),
    PID_FLOAT(alt_vel, out_min), PID_FLOAT(alt_vel, out_max),
    PID_FLOAT(alt_vel, int_limit), PID_FLOAT(alt_vel, threshold),
    PID_ALPHA(alt_vel), PID_ENABLED(alt_vel),
    // ---- 水平位置/速度串级 ----
    PID_FLOAT(pos_n, kp), PID_FLOAT(pos_n, ki), PID_FLOAT(pos_n, kd),
    PID_FLOAT(pos_n, out_min), PID_FLOAT(pos_n, out_max),
    PID_FLOAT(pos_n, int_limit), PID_FLOAT(pos_n, threshold),
    PID_ALPHA(pos_n), PID_ENABLED(pos_n),
    PID_FLOAT(pos_e, kp), PID_FLOAT(pos_e, ki), PID_FLOAT(pos_e, kd),
    PID_FLOAT(pos_e, out_min), PID_FLOAT(pos_e, out_max),
    PID_FLOAT(pos_e, int_limit), PID_FLOAT(pos_e, threshold),
    PID_ALPHA(pos_e), PID_ENABLED(pos_e),
    PID_FLOAT(vel_n, kp), PID_FLOAT(vel_n, ki), PID_FLOAT(vel_n, kd),
    PID_FLOAT(vel_n, out_min), PID_FLOAT(vel_n, out_max),
    PID_FLOAT(vel_n, int_limit), PID_FLOAT(vel_n, threshold),
    PID_ALPHA(vel_n), PID_ENABLED(vel_n),
    PID_FLOAT(vel_e, kp), PID_FLOAT(vel_e, ki), PID_FLOAT(vel_e, kd),
    PID_FLOAT(vel_e, out_min), PID_FLOAT(vel_e, out_max),
    PID_FLOAT(vel_e, int_limit), PID_FLOAT(vel_e, threshold),
    PID_ALPHA(vel_e), PID_ENABLED(vel_e),
    // ---- 控制滤波器 alpha ----
    ALPHA_ENTRY("spd_alpha", speed_filter_alpha, 0), ALPHA_ENTRY("spd_alpha", speed_filter_alpha, 1),
    ALPHA_ENTRY("spd_alpha", speed_filter_alpha, 2),
    ALPHA_ENTRY("ang_alpha", angle_out_filter_alpha, 0), ALPHA_ENTRY("ang_alpha", angle_out_filter_alpha, 1),
    ALPHA_ENTRY("ang_alpha", angle_out_filter_alpha, 2),
    ALPHA_ENTRY("out_alpha", output_filter_alpha, 0), ALPHA_ENTRY("out_alpha", output_filter_alpha, 1),
    ALPHA_ENTRY("out_alpha", output_filter_alpha, 2),
    // ★ 2026-08-10 惯量逆解交叉耦合前馈使能掩码（bit0 陀螺耦合 / bit1 转子陀螺；0=全关，在线 A/B）
    // 2026-08-10 全面审查修复：bptr(bool) → u8ptr(0-255)——双位掩码此前无法写 2/3、无法恢复 0x03
    PID_U8("inertia_comp_mask", kFlightCtrlParams.inertia_comp_mask),
    ALPHA_ENTRY("spd2_alpha", speed_filter_alpha2, 0), ALPHA_ENTRY("spd2_alpha", speed_filter_alpha2, 1),
    ALPHA_ENTRY("spd2_alpha", speed_filter_alpha2, 2),   // ★ 2026-08-09 二级滤波（级联二阶，抑 30-60Hz 桨振动）
    // ---- FPV 摇杆曲线（★2026-08-11 RATE_MODE 接入，Betaflight 三参数模型）----
    //   尾部追加保持既有 ID 不变；参数名 rc_rate / rc_expo[0..2] / rc_super[0..2]
    { "rc_rate", ANO_FLOAT, &kFlightCtrlParams.rc_rate, nullptr, nullptr },
    ALPHA_ENTRY("rc_expo", rc_expo, 0), ALPHA_ENTRY("rc_expo", rc_expo, 1), ALPHA_ENTRY("rc_expo", rc_expo, 2),
    ALPHA_ENTRY("rc_super", rc_super, 0), ALPHA_ENTRY("rc_super", rc_super, 1), ALPHA_ENTRY("rc_super", rc_super, 2),
};
static_assert(sizeof(kAnoParams) / sizeof(kAnoParams[0]) == ANO_PARAMS_COUNT,
              "ano_params: 注册表数量与 ANO_PARAMS_COUNT 不一致");

// 参数 ID → 表项（越界返回 nullptr）
static const AnoParamEntry *anoParamAt(uint16_t id)
{
    if (id >= ANO_PARAMS_COUNT)
        return nullptr;
    return &kAnoParams[id];
}

// 序列化单个参数值到 PAR_VAL（返回字节数，0 = 无效 ID）
static uint16_t anoParamSerialize(uint16_t id, uint8_t *out, uint16_t maxLen)
{
    const AnoParamEntry *e = anoParamAt(id);
    if (!e)
        return 0;
    if (e->type == ANO_FLOAT)
    {
        if (maxLen < 4)
            return 0;
        memcpy(out, e->fptr, 4);
        return 4;
    }
    if (maxLen < 1)
        return 0;
    out[0] = e->u8ptr ? *e->u8ptr : ((*e->bptr) ? 1 : 0);  // u8ptr 优先（0-255 值）
    return 1;
}

// 写入单个参数（长度须与类型严格匹配；无效 ID/长度返回 false 且不应用）
static bool anoParamDeserialize(uint16_t id, const uint8_t *val, uint16_t len)
{
    const AnoParamEntry *e = anoParamAt(id);
    if (!e)
        return false;
    if (e->type == ANO_FLOAT)
    {
        if (len != 4)
            return false;
        memcpy(e->fptr, val, 4);
        return true;
    }
    // ANO_UINT8：u8ptr（0-255 值，如 inertia_comp_mask）/ bptr（bool，enabled）
    if (len != 1)
        return false;
    if (e->u8ptr)
    {
        *e->u8ptr = val[0];   // ★ 2026-08-10 修复：掩码可写 0-255（此前 bool 化拒绝 2/3）
        return true;
    }
    if (val[0] > 1)
        return false;
    *e->bptr = val[0] ? true : false;
    return true;
}

// ==================================================================
//  MAVLink 参数桥统一 float 接口（2026-08-10，mavlink_bridge.cpp 复用）
// ==================================================================
uint16_t anoParamCount()
{
    return ANO_PARAMS_COUNT;
}

const char *anoParamNameAt(uint16_t id)
{
    const AnoParamEntry *e = anoParamAt(id);
    return e ? e->name : nullptr;
}

int16_t anoParamIdByName(const char *name)
{
    if (!name)
        return -1;
    for (uint16_t i = 0; i < ANO_PARAMS_COUNT; i++)
    {
        // ★ MAVLink param_id 是截断 15B 的名称——按前缀匹配即可
        // （截断唯一性已核实：int_lim vs thresho 第 2 字符即不同）
        if (strncmp(kAnoParams[i].name, name, 16) == 0)
            return (int16_t)i;
    }
    return -1;
}

bool anoParamReadFloat(uint16_t id, float *out)
{
    const AnoParamEntry *e = anoParamAt(id);
    if (!e || !out)
        return false;
    if (e->type == ANO_FLOAT)
    {
        *out = *e->fptr;
        return true;
    }
    *out = e->u8ptr ? (float)(*e->u8ptr) : ((*e->bptr) ? 1.0f : 0.0f);  // ANO_UINT8 → float
    return true;
}

bool anoParamWriteFloat(uint16_t id, float value)
{
    const AnoParamEntry *e = anoParamAt(id);
    if (!e)
        return false;
    if (e->type == ANO_FLOAT)
    {
        *e->fptr = value;
    }
    else
    {
        if (e->u8ptr)
        {
            // ★ 2026-08-10 修复：掩码按 0-255 值写入（此前钳 0/1 无法恢复 0x03）
            *e->u8ptr = (uint8_t)constrain(value, 0.0f, 255.0f);
        }
        else
        {
            // ANO_UINT8：0.0→false，其余→true（钳制语义，MAV_PARAM_TYPE_REAL32 来源）
            *e->bptr = (value != 0.0f);
        }
    }
    applyFlightCtrlParams();  // 无扰同步到全部 PID/滤波器实例
    return true;
}

bool anoParamsHandleRx(AnoComProtocol &ano, uint8_t funcCode,
                       const uint8_t *data, uint16_t len)
{
    uint8_t tx[32];
    uint16_t id;
    const AnoParamEntry *e;

    if (funcCode == ANO_FUNC_PARAM_CMD) // 0xE0 参数命令
    {
        if (len < 1)
            return true; // 空帧视为已消费（避免重复处理）
        const uint8_t cmd = data[0];
        switch (cmd)
        {
        case 0x01: // 读参数个数 → 回 0xE0 [0x01, count u32 LE]
            tx[0] = 0x01;
            tx[1] = ANO_PARAMS_COUNT & 0xFF;
            tx[2] = (ANO_PARAMS_COUNT >> 8) & 0xFF;
            tx[3] = 0;
            tx[4] = 0;
            ano.sendData(ANO_GND_ADDR, ANO_FUNC_PARAM_CMD, tx, 5);
            return true;

        case 0x02: // 读参数值（VAL=ID u16）→ 回 0xE1 [ID u16, PAR_VAL]
            if (len < 3)
                return true;
            id = (uint16_t)(data[1] | (data[2] << 8));
            if (!anoParamAt(id))
                return true;
            tx[0] = id & 0xFF;
            tx[1] = (id >> 8) & 0xFF;
            {
                uint16_t n = anoParamSerialize(id, &tx[2], sizeof(tx) - 2);
                if (n > 0)
                    ano.sendData(ANO_GND_ADDR, ANO_FUNC_PARAM_WRITE_READ, tx, 2 + n);
            }
            return true;

        case 0x03: // 读参数信息（VAL=ID u16）→ 回 0xE2 [ID u16, TYPE u8, NAME 20B]
            if (len < 3)
                return true;
            id = (uint16_t)(data[1] | (data[2] << 8));
            e = anoParamAt(id);
            if (!e)
                return true;
            tx[0] = id & 0xFF;
            tx[1] = (id >> 8) & 0xFF;
            tx[2] = e->type;
            memset(&tx[3], 0, 20); // PAR_NAME 定长 20 字节，不足补 0
            {
                size_t nameLen = strlen(e->name);
                if (nameLen > 20)
                    nameLen = 20;
                memcpy(&tx[3], e->name, nameLen);
            }
            ano.sendData(ANO_GND_ADDR, ANO_FUNC_PARAM_INFO, tx, 23);
            return true;

        case 0x10: // 命令类：VAL=0xAA 恢复默认 / 0xAB 保存
            if (len >= 2 && data[1] == 0xAA)
            {
                // 恢复出厂默认（kFlightCtrlParamsDefaults 为唯一默认值源）
                kFlightCtrlParams = kFlightCtrlParamsDefaults;
                applyFlightCtrlParams();
            }
            // 0xAB 保存：本工程参数无持久化（RAM 运行期值，重启回默认），
            // 仅回校验帧确认已接收（校验帧由 onAnoRxFrame 统一回传）
            return true;

        default:
            return true; // 未实现命令：已消费（校验帧已回传，不做额外响应）
        }
    }

    if (funcCode == ANO_FUNC_PARAM_WRITE_READ) // 0xE1 参数写入
    {
        if (len < 3)
            return true;
        id = (uint16_t)(data[0] | (data[1] << 8));
        if (anoParamDeserialize(id, &data[2], len - 2))
            applyFlightCtrlParams(); // 无扰同步到全部 PID/滤波器实例
        return true;
    }

    return false; // 非参数帧，未消费
}
