/**
 * @file AnoVars.h
 * @brief 通用变量注册表 + 上报集合（平台无关，host 可测）
 *
 * 设计目标：让固件任意内部变量（控制中间量/传感器/状态）注册后即可多协议上报：
 *   - AnoCom 0xF2 值帧（[id u16 LE] + [float LE]，watch 集合轮发）
 *   - AnoCom 0xF3 清单帧（仿 0xE0/0xE2：count + [id, type, name 16B]）
 *   - MAVLink NAMED_VALUE_FLOAT / DEBUG_VECT（QGC/MP 通道，复用同一 watch 集合）
 *   - DBG `vars` 命令（配置上报集合）
 *
 * 与 ano_params.cpp 同模式：注册表 = 宏展开静态表（顺序即变量 ID），
 * 加变量 = 注册表加一行宏。名称 ≤16 字符（0xF3 帧 PAR_NAME 定长 16B）。
 * 本头文件零 Arduino 依赖（仅 <cstdint>/<cstring>），host 测试可自行
 * 定义 kAnoVars 表直接编译验证序列化/上限/名称约束。
 */
#pragma once
#include <cstdint>
#include <cstring>

// ---- 变量类型（0xF3 清单帧 PAR_TYPE）----
enum AnoVarType : uint8_t
{
    ANO_VAR_FLOAT = 0,
    ANO_VAR_U8    = 1,
    ANO_VAR_U16   = 2,
    ANO_VAR_I16   = 3,
    ANO_VAR_I32   = 4,
    ANO_VAR_U32   = 5,
    ANO_VAR_BOOL  = 6,
};

// ---- 容量上限 ----
#define ANO_VARS_MAX_COUNT  64   // 注册表上限
#define ANO_VARS_MAX_WATCH  16   // 上报集合上限
#define ANO_VARS_NAME_LEN   16   // 名称定长（0xF3 帧 PAR_NAME 16B，含 NUL 截断约束）
#define ANO_VARS_MAV_NAME_LEN 10 // MAVLink NAMED_VALUE_FLOAT 名称 ≤9 字符 + NUL

struct AnoVarEntry
{
    const char *name;            // ≤16 字符（超长被截断，GCS 按名匹配会失败）
    uint8_t type;                // AnoVarType
    const volatile float *fptr;  // FLOAT 类型 → 直接取值（volatile 兼容 ISR 写入变量）
    const void *vptr;            // 其他类型 → 按 type 解引用
};

// 注册宏（与 ano_params PID_FLOAT 同模式）
#define ANO_VAR_FLOAT(name, var) { name, ANO_VAR_FLOAT, &(var), nullptr }
#define ANO_VAR_U8(name, var)    { name, ANO_VAR_U8,    nullptr, &(var) }
#define ANO_VAR_U16(name, var)   { name, ANO_VAR_U16,   nullptr, &(var) }
#define ANO_VAR_I16(name, var)   { name, ANO_VAR_I16,   nullptr, &(var) }
#define ANO_VAR_I32(name, var)   { name, ANO_VAR_I32,   nullptr, &(var) }
#define ANO_VAR_U32(name, var)   { name, ANO_VAR_U32,   nullptr, &(var) }
#define ANO_VAR_BOOL(name, var)  { name, ANO_VAR_BOOL,  nullptr, &(var) }

// 注册表（固件侧定义，顺序即变量 ID；host 测试可自定义）
extern const AnoVarEntry kAnoVars[];
extern const uint16_t ANO_VARS_COUNT;

// ---- 上报集合（watch）状态 ----
struct AnoVarWatchState
{
    uint8_t ids[ANO_VARS_MAX_WATCH];  // 注册表内变量 ID
    uint8_t count;                    // 当前个数（≤16）
    uint8_t rate_hz;                  // 上报频率（1-200，默认 50）
};

// 固件侧可变实例（ano_vars.cpp 定义）
extern AnoVarWatchState g_ano_var_watch;

// ------------------------------------------------------------------
//  纯函数/内联操作（host 可测）
// ------------------------------------------------------------------

// 统一取值：所有类型转 float 上报（u8/u16/bool 无损；u32 >2^24 有精度损失）
static inline float anoVarGetValue(const AnoVarEntry &e)
{
    switch (e.type)
    {
    case ANO_VAR_FLOAT: return e.fptr ? *e.fptr : 0.0f;
    case ANO_VAR_U8:    return e.vptr ? (float)(*(const uint8_t *)e.vptr) : 0.0f;
    case ANO_VAR_U16:   return e.vptr ? (float)(*(const uint16_t *)e.vptr) : 0.0f;
    case ANO_VAR_I16:   return e.vptr ? (float)(*(const int16_t *)e.vptr) : 0.0f;
    case ANO_VAR_I32:   return e.vptr ? (float)(*(const int32_t *)e.vptr) : 0.0f;
    case ANO_VAR_U32:   return e.vptr ? (float)(*(const uint32_t *)e.vptr) : 0.0f;
    case ANO_VAR_BOOL:  return e.vptr ? ((*(const bool *)e.vptr) ? 1.0f : 0.0f) : 0.0f;
    default:            return 0.0f;
    }
}

// 名称写入定长缓冲（0xF3 帧 PAR_NAME 16B / MAVLink 10B），超长截断补 0
static inline uint16_t anoVarCopyName(char *dst, uint16_t dstLen, const char *name)
{
    uint16_t n = 0;
    while (name && name[n] && n < dstLen - 1)
        n++;
    if (n) memcpy(dst, name, n);
    dst[n] = '\0';
    return n;
}

// ---- watch 集合操作（全部作用于 g_ano_var_watch）----

// 按名称查 ID（找不到返回 -1）
static inline int16_t anoVarIdByName(const char *name)
{
    if (!name) return -1;
    for (uint16_t i = 0; i < ANO_VARS_COUNT; i++)
        if (strncmp(kAnoVars[i].name, name, ANO_VARS_NAME_LEN) == 0)
            return (int16_t)i;
    return -1;
}

// 加入上报集合（已存在/越界/满返回 false）
static inline bool anoVarWatchAdd(uint16_t id)
{
    if (id >= ANO_VARS_COUNT)
        return false;
    AnoVarWatchState &w = g_ano_var_watch;
    for (uint8_t i = 0; i < w.count; i++)
        if (w.ids[i] == id)
            return false; // 已存在
    if (w.count >= ANO_VARS_MAX_WATCH)
        return false;
    w.ids[w.count++] = (uint8_t)id;
    return true;
}

// 按名称加入（返回 true 成功）
static inline bool anoVarWatchAddName(const char *name)
{
    int16_t id = anoVarIdByName(name);
    if (id < 0) return false;
    return anoVarWatchAdd((uint16_t)id);
}

// 按名称移除（返回 true 找到并移除）
static inline bool anoVarWatchRemoveName(const char *name)
{
    int16_t id = anoVarIdByName(name);
    if (id < 0) return false;
    AnoVarWatchState &w = g_ano_var_watch;
    for (uint8_t i = 0; i < w.count; i++)
    {
        if (w.ids[i] == id)
        {
            for (uint8_t j = i; j + 1 < w.count; j++)
                w.ids[j] = w.ids[j + 1];
            w.count--;
            return true;
        }
    }
    return false;
}

// 清空上报集合
static inline void anoVarWatchClear()
{
    g_ano_var_watch.count = 0;
}

// 设置上报频率（钳制 1-200Hz；uint16_t 防调用处字面量截断）
static inline void anoVarWatchSetRate(uint16_t hz)
{
    g_ano_var_watch.rate_hz = (uint8_t)((hz < 1) ? 1 : (hz > 200 ? 200 : hz));
}

// 上报集合是否为空
static inline bool anoVarWatchEmpty()
{
    return g_ano_var_watch.count == 0;
}
