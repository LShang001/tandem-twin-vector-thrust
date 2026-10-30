/**
 * @file    test_coax_params.cpp
 * @brief   共轴双桨 VTVL 参数可标定化回归（默认值逐位 / 注册表顺序 / 逐拍输出锁定）
 *
 * 验证目标（对应 task-6/task-10）：
 *  1) CoaxialCtrlParams 的 18 项控制律默认值 + 13 项物理/输出/分配器常量逐位等于原
 *     flight_control.cpp 共轴分支内的硬编码字面量/分配器默认（不是"约等于"）；
 *  2) 参数注册表 coax.* 的 31 个名称、顺序与 BFS_COAX_PARAM_LIST 完全一致，
 *     名称 ≤20B，按 MAVLink 规则截断 15B 后唯一（param_id 冲突会让 QGC 写错参数）；
 *  3) 用默认参数构造的控制器跑 10 拍基准序列，输出位型逐拍等于 2026-10-30
 *     串级语义修正后的 golden（RATE 只复位角度外环、内环积分两模式使能、
 *     手动 TVC 输出过 α0.25 滤波），锁定默认参数路径不再漂移。
 *
 * 纯平台无关：只依赖 include/FlightCtrlParams.h + include/CoaxialVtvControl.h。
 */
#include <cstdio>
#include <cstring>
#include <cstdint>

#include "FlightCtrlParams.h"
#include "CoaxialVtvControl.h"

namespace
{
int g_fail = 0;
int g_checks = 0;

void check(bool ok, const char *what)
{
    ++g_checks;
    if (!ok)
    {
        ++g_fail;
        std::printf("  FAIL: %s\n", what);
    }
}

uint32_t bits(float f)
{
    uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}

bool same(float a, float b)
{
    return bits(a) == bits(b);
}

// ---- 改造前 flight_control.cpp 共轴分支的硬编码增益（逐字段抄写，作为参考真值）----
bfs::CoaxialVtvParams legacyParams()
{
    bfs::CoaxialVtvParams p;
    p.att_kp_roll = 4.25f;
    p.att_kp_pitch = 4.25f;
    p.rate_kp_roll = 0.25f;
    p.rate_ki_roll = 0.08f;
    p.rate_kp_pitch = 0.25f;
    p.rate_ki_pitch = 0.08f;
    p.rate_kp_yaw = 3.8f;
    p.rate_ki_yaw = 30.0f;
    p.max_target_rate_dps = 80.0f;
    p.max_correction_deg = 10.0f;
    p.max_yaw_diff_us = 180.0f;
    p.att_integral_limit = 40.0f;
    p.rate_integral_limit = 5.0f;
    p.yaw_integral_limit = 50.0f;
    p.yaw_integral_threshold = 40.0f;
    p.att_rate_filter_alpha = 0.85f;
    p.correction_filter_alpha = 0.25f;
    p.pid_derivative_filter_alpha = 0.2f;
    return p;
}

bool sameCtrl(const bfs::CoaxialVtvParams &a, const bfs::CoaxialVtvParams &b)
{
    return same(a.att_kp_roll, b.att_kp_roll) &&
           same(a.att_kp_pitch, b.att_kp_pitch) &&
           same(a.rate_kp_roll, b.rate_kp_roll) &&
           same(a.rate_ki_roll, b.rate_ki_roll) &&
           same(a.rate_kp_pitch, b.rate_kp_pitch) &&
           same(a.rate_ki_pitch, b.rate_ki_pitch) &&
           same(a.rate_kp_yaw, b.rate_kp_yaw) &&
           same(a.rate_ki_yaw, b.rate_ki_yaw) &&
           same(a.max_target_rate_dps, b.max_target_rate_dps) &&
           same(a.max_correction_deg, b.max_correction_deg) &&
           same(a.max_yaw_diff_us, b.max_yaw_diff_us) &&
           same(a.att_integral_limit, b.att_integral_limit) &&
           same(a.rate_integral_limit, b.rate_integral_limit) &&
           same(a.yaw_integral_limit, b.yaw_integral_limit) &&
           same(a.yaw_integral_threshold, b.yaw_integral_threshold) &&
           same(a.att_rate_filter_alpha, b.att_rate_filter_alpha) &&
           same(a.correction_filter_alpha, b.correction_filter_alpha) &&
           same(a.pid_derivative_filter_alpha, b.pid_derivative_filter_alpha);
}

// 参考真值：物理/输出常量（改造前散落在 run_coaxial_control 内）
struct LegacyPhysical
{
    float mass_kg;
    float max_total_thrust_n;
    float servo_half_angle_deg;
    float motor_min_us;
    float motor_max_us;
    float attitude_limit_deg;
    float dt_s;
    // ★ task-10 分配器参数默认值（旧行为 = alloc_mode 0，不启用 α+分配器路径）
    float alloc_mode;
    float lever_m;
    float yaw_moment_nm_per_us;
    float inertia_roll_kgm2;
    float inertia_pitch_kgm2;
    float inertia_yaw_kgm2;
};

LegacyPhysical legacyPhysical()
{
    return LegacyPhysical{0.84f, 14.0f, 15.0f, 1000.0f, 2000.0f, 45.0f, 0.005f,
                          0.0f, 0.15f, 1.0e-6f, 0.05f, 0.05f, 0.08f};
}

bool samePhysical(const CoaxialCtrlParams &p, const LegacyPhysical &l)
{
    return same(p.mass_kg, l.mass_kg) &&
           same(p.max_total_thrust_n, l.max_total_thrust_n) &&
           same(p.servo_half_angle_deg, l.servo_half_angle_deg) &&
           same(p.motor_min_us, l.motor_min_us) &&
           same(p.motor_max_us, l.motor_max_us) &&
           same(p.attitude_limit_deg, l.attitude_limit_deg) &&
           same(p.dt_s, l.dt_s) &&
           same(p.alloc_mode, l.alloc_mode) &&
           same(p.lever_m, l.lever_m) &&
           same(p.yaw_moment_nm_per_us, l.yaw_moment_nm_per_us) &&
           same(p.inertia_roll_kgm2, l.inertia_roll_kgm2) &&
           same(p.inertia_pitch_kgm2, l.inertia_pitch_kgm2) &&
           same(p.inertia_yaw_kgm2, l.inertia_yaw_kgm2);
}

// ---- 注册表顺序：与 FlightCtrlParams.h 的 BFS_COAX_PARAM_LIST 单一事实源比对 ----
// ★ 本列表必须与 GCS/server/params.py::expected_names() 尾部逐项一致
const char *const kExpectedNames[] = {
    "coax.att_kp_roll",
    "coax.att_kp_pitch",
    "coax.rate_kp_roll",
    "coax.rate_ki_roll",
    "coax.rate_kp_pitch",
    "coax.rate_ki_pitch",
    "coax.rate_kp_yaw",
    "coax.rate_ki_yaw",
    "coax.max_rate_dps",
    "coax.max_corr_deg",
    "coax.max_yaw_us",
    "coax.att_int_lim",
    "coax.rate_int_lim",
    "coax.yaw_int_lim",
    "coax.yaw_int_thr",
    "coax.att_flt_alpha",
    "coax.corr_flt_alpha",
    "coax.pid_deriv_alpha",
    "coax.mass_kg",
    "coax.max_thrust_n",
    "coax.servo_half_deg",
    "coax.motor_min_us",
    "coax.motor_max_us",
    "coax.att_limit_deg",
    "coax.dt_s",
    // ★ task-10 α+油门分配器参数（ID 158-163，GCS 同序）
    "coax.alloc_mode",
    "coax.lever_m",
    "coax.yaw_nm_per_us",
    "coax.inertia_roll",
    "coax.inertia_pitch",
    "coax.inertia_yaw",
};

const char *const kMacroNames[] = {
#define COAX_NAME_OF(name, member) #name,
    BFS_COAX_PARAM_LIST(COAX_NAME_OF)
#undef COAX_NAME_OF
};

// MAVLink 短名算法镜像（mavlink_bridge.cpp::mavParamIdOfName）：拆分 loop.field、
// loop 段截 12B、拼接后总长截 15B。coax.* 不命中缩写表。
void mavShortName(const char *name, char out[16])
{
    std::memset(out, 0, 16);
    const char *dot = std::strchr(name, '.');
    char cand[24];
    if (dot)
    {
        std::size_t loop_len = (std::size_t)(dot - name);
        if (loop_len > 12)
            loop_len = 12;
        std::memcpy(cand, name, loop_len);
        cand[loop_len] = '.';
        std::strcpy(cand + loop_len + 1, dot + 1);
    }
    else
    {
        std::strcpy(cand, name);
    }
    std::size_t n = 0;
    while (cand[n] && n < 15)
        ++n;
    std::memcpy(out, cand, n);
}

// ---- 逐拍基准序列（与改造前生成 golden 的序列完全一致）----
struct Step
{
    float er, ep, rr, rp, ry, yrt, rrt, prt;
    bool rate_mode, manual_tvc, att, yaw;
    float mr, mp, dt;
};

const Step kSeq[] = {
    {0, 0, 0, 0, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {5, 5, 0, 0, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {0, 0, 20, 20, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {-3, 2, -10, 5, 0, 10, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {0, 0, 0, 0, 0, 0, 30, -20, true, false, true, true, 0, 0, 0.005f},
    {0, 0, 0, 0, 0, 0, 0, 0, false, true, true, true, 4, -3, 0.005f},
    {30, -30, 0, 0, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {0, 0, 0, 0, 0, 50, 0, 0, false, false, true, true, 0, 0, 0.005f},
    {0, 0, 0, 0, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.0f},
    {1, -1, 0, 0, 0, 0, 0, 0, false, false, true, true, 0, 0, 0.005f},
};
const int kSeqLen = (int)(sizeof(kSeq) / sizeof(kSeq[0]));

struct OutBits
{
    int valid;
    uint32_t roll_bits;
    uint32_t pitch_bits;
    uint32_t yaw_bits;
    int saturated;
};

// golden：2026-10-30 串级语义修正后（默认参数）逐拍浮点位型。
// 与修正前的差异：RATE 步不再清零速率内环/两级滤波，手动 TVC 输出过 α0.25
// 滤波；由修正后的控制器生成后冻结，后续改动必须显式说明理由并重生成。
const OutBits kGolden[kSeqLen] = {
    {1, 0x00000000u, 0x00000000u, 0x00000000u, 0},
    {1, 0x3F90BB30u, 0x3F90BB30u, 0x00000000u, 0},
    {1, 0xBE6E2240u, 0xBE6E2240u, 0x00000000u, 0},
    {1, 0xBE4E27CDu, 0xBC1D7830u, 0x421E0000u, 0},
    {1, 0x3FAC7763u, 0xBF7FD948u, 0x3FBFFFFFu, 0},
    {1, 0x4000ACC5u, 0xBFBFF17Bu, 0x3FBFFFFFu, 0},
    {1, 0x408040CAu, 0xC067FA8Eu, 0x3FBFFFFFu, 0},
    {1, 0x406BDC57u, 0xC058BA9Fu, 0x43340000u, 1},
    {0, 0x00000000u, 0x00000000u, 0x00000000u, 0},
    {1, 0x3E6791E7u, 0xBE6791E7u, 0x00000000u, 0},
};

void runSequence(const bfs::CoaxialVtvParams &p, OutBits out[kSeqLen])
{
    bfs::CoaxialVtvController c;
    c.setParams(p);
    for (int i = 0; i < kSeqLen; ++i)
    {
        const Step &s = kSeq[i];
        bfs::CoaxialVtvInput in;
        in.error_roll_deg = s.er;
        in.error_pitch_deg = s.ep;
        in.rate_roll_dps = s.rr;
        in.rate_pitch_dps = s.rp;
        in.rate_yaw_dps = s.ry;
        in.yaw_rate_target_dps = s.yrt;
        in.roll_rate_target_dps = s.rrt;
        in.pitch_rate_target_dps = s.prt;
        in.rate_mode = s.rate_mode;
        in.manual_tvc = s.manual_tvc;
        in.attitude_active = s.att;
        // 旧语义映射（原始 manage_pid_integrals）：外环积分只在 ATTITUDE、
        // 内环两模式都积分、偏航按油门>1100。
        in.attitude_integral_active = s.att && !s.rate_mode;
        in.rate_integral_active = s.att;
        in.yaw_active = s.yaw;
        in.yaw_integral_active = s.yaw;
        in.manual_roll_deg = s.mr;
        in.manual_pitch_deg = s.mp;
        in.dt_s = s.dt;
        const bfs::CoaxialVtvOutput o = c.update(in);
        out[i].valid = o.valid ? 1 : 0;
        out[i].roll_bits = bits(o.roll_correction_deg);
        out[i].pitch_bits = bits(o.pitch_correction_deg);
        out[i].yaw_bits = bits(o.yaw_diff_us);
        out[i].saturated = o.saturated ? 1 : 0;
    }
}

bool sameBits(const OutBits &a, const OutBits &b)
{
    return a.valid == b.valid && a.roll_bits == b.roll_bits &&
           a.pitch_bits == b.pitch_bits && a.yaw_bits == b.yaw_bits &&
           a.saturated == b.saturated;
}

} // namespace

int main()
{
    std::printf("==== 共轴参数可标定化回归 ====\n");

    // ---- 1. 控制律默认值逐位等于旧硬编码 ----
    const bfs::CoaxialVtvParams legacy = legacyParams();
    const CoaxialCtrlParams def{};
    check(sameCtrl(def.ctrl, legacy), "CoaxialCtrlParams{}.ctrl 与旧硬编码逐位相等");
    check(sameCtrl(kFlightCtrlParamsDefaults.coax.ctrl, legacy),
          "kFlightCtrlParamsDefaults.coax.ctrl 与旧硬编码逐位相等");
    std::printf("  [1] 控制律 18 项默认值逐位比对完成\n");

    // ---- 2. 物理/输出常量逐位等于旧硬编码 ----
    const LegacyPhysical legacy_phys = legacyPhysical();
    check(samePhysical(def, legacy_phys), "CoaxialCtrlParams{} 物理/输出常量逐位相等");
    check(samePhysical(kFlightCtrlParamsDefaults.coax, legacy_phys),
          "kFlightCtrlParamsDefaults.coax 物理/输出常量逐位相等");
    std::printf("  [2] 物理/输出/分配器常量 13 项默认值逐位比对完成\n");

    // ---- 3. 注册表名称/顺序/长度/短名唯一 ----
    check(BFS_COAX_PARAM_COUNT == 31, "BFS_COAX_PARAM_COUNT == 31");
    check((int)(sizeof(kExpectedNames) / sizeof(kExpectedNames[0])) ==
              BFS_COAX_PARAM_COUNT,
          "期望名称表长度 == BFS_COAX_PARAM_COUNT");
    check((int)(sizeof(kMacroNames) / sizeof(kMacroNames[0])) ==
              BFS_COAX_PARAM_COUNT,
          "宏展开名称表长度 == BFS_COAX_PARAM_COUNT");
    for (int i = 0; i < BFS_COAX_PARAM_COUNT; ++i)
    {
        check(std::strcmp(kExpectedNames[i], kMacroNames[i]) == 0,
              "注册表名称/顺序与 BFS_COAX_PARAM_LIST 一致");
        check(std::strlen(kExpectedNames[i]) <= 20, "参数名 ≤20B");
    }
    for (int i = 0; i < BFS_COAX_PARAM_COUNT; ++i)
    {
        char a[16];
        mavShortName(kExpectedNames[i], a);
        check(std::strlen(a) <= 15, "MAVLink 短名 ≤15B");
        for (int j = i + 1; j < BFS_COAX_PARAM_COUNT; ++j)
        {
            char b[16];
            mavShortName(kExpectedNames[j], b);
            check(std::strcmp(a, b) != 0, "MAVLink 短名两两唯一");
        }
    }
    std::printf("  [3] 31 个参数名称/顺序/长度/短名唯一性检查完成\n");

    // ---- 4. 默认参数控制器输出逐拍等于改造前 golden ----
    OutBits got[kSeqLen];
    runSequence(def.ctrl, got);
    for (int i = 0; i < kSeqLen; ++i)
    {
        check(sameBits(got[i], kGolden[i]), "默认参数逐拍输出位型等于 golden");
    }
    // 再直接跑旧硬编码参数控制器：默认参数路径与旧路径必须逐拍位型相等
    OutBits old_out[kSeqLen];
    runSequence(legacy, old_out);
    for (int i = 0; i < kSeqLen; ++i)
    {
        check(sameBits(got[i], old_out[i]), "默认参数输出 == 旧硬编码参数输出");
        check(got[i].valid == kGolden[i].valid, "valid 位与 golden 一致");
        check(got[i].saturated == kGolden[i].saturated, "饱和位与 golden 一致");
    }
    std::printf("  [4] 10 拍基准序列输出位型逐拍比对完成\n");

    if (g_fail == 0)
        std::printf("PASS: %d/%d\n", g_checks, g_checks);
    else
        std::printf("FAIL: %d/%d 项失败\n", g_fail, g_checks);
    return g_fail == 0 ? 0 : 1;
}
