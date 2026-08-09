// ============================================================
//  test_ano_vars.cpp — 通用变量注册表（AnoVars.h）host 测试
//
//  纯平台无关（无 Arduino 依赖）：验证注册表遍历、各类型→float
//  序列化、名称截断、watch 集合（增删清上限）、rate 钳制、越界容错。
//  测试自带 kAnoVars 表（模拟固件变量），链接时替代固件侧定义。
// ============================================================
#include <cstdio>
#include <cstring>
#include <cstdint>
#include "AnoVars.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)

// ---- 模拟固件变量 ----
static float    v_float = 3.14f;
static uint8_t  v_u8    = 200;
static uint16_t v_u16   = 40000;
static int16_t  v_i16   = -1234;
static int32_t  v_i32   = -100000;
static uint32_t v_u32   = 4000000000u;
static bool     v_bool  = true;
static float    v_arr[3] = {1.0f, -2.5f, 0.125f};

// 测试注册表（顺序即 ID：0-15）
// extern：const 定义默认 internal linkage，须与头文件 extern 声明一致；
// ★ 数组不带大小——带 ANO_VARS_MAX_COUNT 会让 sizeof 按 64 计（segfault 实测）
extern const AnoVarEntry kAnoVars[];
extern const uint16_t ANO_VARS_COUNT;
AnoVarWatchState g_ano_var_watch = { {0}, 0, 50 };

extern const AnoVarEntry kAnoVars[] = {
    ANO_VAR_FLOAT("float_var", v_float),
    ANO_VAR_U8("u8_var", v_u8),
    ANO_VAR_U16("u16_var", v_u16),
    ANO_VAR_I16("i16_var", v_i16),
    ANO_VAR_I32("i32_var", v_i32),
    ANO_VAR_U32("u32_var", v_u32),
    ANO_VAR_BOOL("bool_var", v_bool),
    ANO_VAR_FLOAT("arr_0", v_arr[0]),
    ANO_VAR_FLOAT("arr_1", v_arr[1]),
    ANO_VAR_FLOAT("arr_2", v_arr[2]),
    // 11-16：dummy 项，凑足 16 个合法 ID（watch 上限测试用）
    ANO_VAR_FLOAT("dummy_0", v_arr[0]),
    ANO_VAR_FLOAT("dummy_1", v_arr[0]),
    ANO_VAR_FLOAT("dummy_2", v_arr[0]),
    ANO_VAR_FLOAT("dummy_3", v_arr[0]),
    ANO_VAR_FLOAT("dummy_4", v_arr[0]),
    ANO_VAR_FLOAT("dummy_5", v_arr[0]),
};
extern const uint16_t ANO_VARS_COUNT = sizeof(kAnoVars) / sizeof(kAnoVars[0]);

int main()
{
    // ---- 1. 类型→float 序列化 ----
    CHECK(ANO_VARS_COUNT == 16, "注册表数量");
    CHECK(anoVarGetValue(kAnoVars[0]) == 3.14f, "float 直读");
    CHECK(anoVarGetValue(kAnoVars[1]) == 200.0f, "u8 → float");
    CHECK(anoVarGetValue(kAnoVars[2]) == 40000.0f, "u16 → float");
    CHECK(anoVarGetValue(kAnoVars[3]) == -1234.0f, "i16 → float");
    CHECK(anoVarGetValue(kAnoVars[4]) == -100000.0f, "i32 → float");
    CHECK(anoVarGetValue(kAnoVars[5]) == 4000000000.0f, "u32 → float（>2^24 有精度损失但近值）");
    CHECK(anoVarGetValue(kAnoVars[6]) == 1.0f, "bool true → 1");
    CHECK(anoVarGetValue(kAnoVars[7]) == 1.0f && anoVarGetValue(kAnoVars[9]) == 0.125f, "数组元素注册");

    // 变量值变更后取到新值（live-read 语义）
    v_float = 2.718f;
    CHECK(anoVarGetValue(kAnoVars[0]) == 2.718f, "live-read 变更可见");
    v_bool = false;
    CHECK(anoVarGetValue(kAnoVars[6]) == 0.0f, "bool false → 0");

    // ---- 2. 名称查找 / 截断 ----
    CHECK(anoVarIdByName("float_var") == 0, "按名称查 ID");
    CHECK(anoVarIdByName("arr_2") == 9, "按名称查 ID（末项）");
    CHECK(anoVarIdByName("nonexist") == -1, "未知名返回 -1");
    CHECK(anoVarIdByName(nullptr) == -1, "nullptr 名返回 -1");

    char buf[16];
    CHECK(anoVarCopyName(buf, sizeof(buf), "float_var") == 9 && strcmp(buf, "float_var") == 0, "名称拷贝");
    CHECK(anoVarCopyName(buf, sizeof(buf), "this_name_is_way_too_long") == 15 && strlen(buf) == 15, "名称截断 15 字符");
    char mav[10];
    anoVarCopyName(mav, sizeof(mav), "alpha_ref_x");
    CHECK(strlen(mav) == 9 && strcmp(mav, "alpha_ref") == 0, "MAVLink 名称截断 9 字符");

    // ---- 3. watch 集合 ----
    anoVarWatchClear();
    CHECK(anoVarWatchEmpty(), "清空后为空");
    CHECK(anoVarWatchAdd(0) && anoVarWatchAdd(3) && anoVarWatchAdd(9), "连续添加");
    CHECK(g_ano_var_watch.count == 3, "计数 3");
    CHECK(!anoVarWatchAdd(0), "重复添加拒绝");
    CHECK(anoVarWatchAddName("u8_var"), "按名称添加");
    CHECK(!anoVarWatchAddName("nonexist"), "未知名添加拒绝");
    CHECK(g_ano_var_watch.count == 4, "计数 4");
    CHECK(anoVarWatchRemoveName("arr_2"), "按名称移除");
    CHECK(anoVarWatchRemoveName("arr_2") == false, "重复移除失败");
    CHECK(g_ano_var_watch.count == 3, "移除后计数 3");
    anoVarWatchClear();
    CHECK(g_ano_var_watch.count == 0, "clear 归零");

    // 上限 16（注册表 16 个合法 ID）
    for (int i = 0; i < 10; i++) anoVarWatchAdd((uint16_t)i);
    CHECK(g_ano_var_watch.count == 10, "10 个加入成功");
    for (int i = 10; i < 16; i++) CHECK(anoVarWatchAdd((uint16_t)i), "继续加入至 16");
    CHECK(!anoVarWatchAdd(200), "越界 ID 拒绝");
    CHECK(g_ano_var_watch.count == 16, "上限 16 满");
    anoVarWatchClear();

    // ---- 4. rate 钳制 ----
    anoVarWatchSetRate(0);   CHECK(g_ano_var_watch.rate_hz == 1, "rate 下限 1");
    anoVarWatchSetRate(500); CHECK(g_ano_var_watch.rate_hz == 200, "rate 上限 200");
    anoVarWatchSetRate(30);  CHECK(g_ano_var_watch.rate_hz == 30, "rate 正常值");

    // ---- 5. 空 watch 语义 ----
    CHECK(anoVarWatchEmpty(), "最终空");

    if (g_fail == 0)
    {
        printf("test_ano_vars: ALL PASSED (%u vars)\n", (unsigned)ANO_VARS_COUNT);
        return 0;
    }
    printf("test_ano_vars: %d FAILED\n", g_fail);
    return 1;
}
