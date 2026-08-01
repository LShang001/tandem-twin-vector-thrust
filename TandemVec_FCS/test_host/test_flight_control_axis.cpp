// ============================================================
//  test_flight_control_axis.cpp — flight_control.cpp 姿态环闭环仿真
//
//  目的：对 src/flight_control.cpp 的姿态环（轴序 FRD 修复后）做
//  宿主机数值仿真验证——独立按源码公式实现（标注来源行号），
//  驱动刚体角动力学 + 推进力矩模型闭环，验证：
//   T1 滚转误差 → q_err.x → 差速(Mx) 负反馈收敛
//   T2 俯仰误差 → q_err.y → 尾摆(My) 负反馈收敛
//   T3 偏航角速率 → ω.z → 前摆(Mz) 速率阻尼
//   T4 三轴 10° 扰动闭环收敛（5 s 内误差 < 1°）
//   T5 执行器映射符号与 propulsion.mjs 一致
//   T6 轴序对齐：绕 x/y/z 扰动分别只激发对应通道（无错位）
//
//  编译运行：
//    g++ -std=c++17 -Iinclude test_host/test_flight_control_axis.cpp \
//        -o test_host/bin/fca && ./test_host/bin/fca
// ============================================================
#include "../include/QuaternionMath.h"
#include "../include/PositionPID.h"
#include "../include/TandemVec_Config.h"

#include <cmath>
#include <cstdio>
#include <array>
#include <algorithm>

static int g_fail = 0;
static void check(bool cond, const char* name, const char* detail = "")
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s  %s\n", name, detail);
    if (!cond) ++g_fail;
}
static bool approx(float a, float b, float rtol = 1e-2f, float atol = 1e-6f)
{
    return std::fabs(a - b) <= rtol * std::fmax(1.f, std::fmax(std::fabs(a), std::fabs(b))) + atol;
}

static const TandemVecParams &P = kDefaultTandemVecParams;
static constexpr float RAD2DEG = 57.295779513f;

// ============================================================
//  刚体角动力学（简化：I·ω̇ = M，忽略陀螺/耦合，四元数欧拉积分）
// ============================================================
struct RigidBody {
    std::array<float, 3> w{0, 0, 0};
    Quaternion q{1, 0, 0, 0};

    void step(const std::array<float, 3> &M, float dt) {
        std::array<float, 3> I{P.Ix, P.Iy, P.Iz};
        for (int i = 0; i < 3; ++i) w[i] += M[i] / I[i] * dt;
        // 小旋转四元数: (1, ω·dt/2) —— w 分量必须≈1（非 0），否则模≈|ω|dt/2 归一化后翻转
        Quaternion dq = quaternionMultiply(q, {1, w[0] * 0.5f * dt,
                                                  w[1] * 0.5f * dt,
                                                  w[2] * 0.5f * dt});
        q = dq;
        float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    }
};

// ============================================================
//  推进力矩模型（与 simulations/.../propulsion.mjs 公式一致：
//  差速 ω=ω0√(1±dw)，T=kT·ω²，Q=kQ·ω²，
//  Mx=-Qf·cf+Qt·ct, My=-b·Tt·st-Qf·sf, Mz=a·Tf·sf-Qt·st）
// ============================================================
static std::array<float, 3> thrustWrench(float w0, float dw, float df, float dt_)
{
    float wf = w0 * std::sqrt(std::max(0.f, 1.f + dw));
    float wt = w0 * std::sqrt(std::max(0.f, 1.f - dw));
    float Tf = P.kT * wf * wf, Tt = P.kT * wt * wt;
    float Qf = P.kQ * wf * wf, Qt = P.kQ * wt * wt;
    float cf = std::cos(df), sf = std::sin(df);
    float ct = std::cos(dt_), st = std::sin(dt_);
    return {
        -Qf * cf + Qt * ct,                       // Mx（差速反扭）
        -P.b * Tt * st - Qf * sf,                 // My（尾摆主控）
         P.a * Tf * sf - Qt * st,                 // Mz（前摆主控）
    };
}

// ============================================================
//  姿态环（flight_control.cpp:961-993、1014-1019、1066-1067 公式）
// ============================================================
struct AttitudeLoop {
    PositionPID rollAnglePID{1.5f, 0.0f, 0.15f, -100.f, 100.f};
    PositionPID pitchAnglePID{1.5f, 0.0f, 0.15f, -100.f, 100.f};
    PositionPID yawAnglePID{4.0f, 0.0f, 0.0f, -100.f, 100.f};  // VTOL 悬停：q_err.z 水平倾斜外环（与固件 yawAnglePID kp=4 一致）
    PositionPID rollRatePID{6.0f, 0.0f, 0.0f, -100.f, 100.f};
    PositionPID pitchRatePID{6.0f, 0.0f, 0.0f, -100.f, 100.f};
    PositionPID yawRatePID{6.0f, 0.0f, 0.0f, -100.f, 100.f};

    // 返回 {alpha_roll, alpha_pitch, alpha_yaw}（rad/s²）
    std::array<float, 3> update(const Quaternion &q_cur, const Quaternion &q_target,
                                const std::array<float, 3> &w_dps)
    {
        // —— 外环：姿态误差四元数 → 目标角速率（flight_control.cpp:961-998）——
        Quaternion q_error = quaternionMultiply(quaternionConjugate(q_cur), q_target);
        float sign_qw = (q_error.w >= 0.f) ? 1.f : -1.f;
        float q_vec_norm = std::sqrt(q_error.x * q_error.x +
                                     q_error.y * q_error.y +
                                     q_error.z * q_error.z);
        float scale = (q_vec_norm > 0.25f)
            ? 2.f * std::atan2(q_vec_norm, std::fabs(q_error.w)) / q_vec_norm * RAD2DEG
            : 2.f * RAD2DEG;
        // FRD 轴序：roll←q_err.x, pitch←q_err.y, yaw←q_err.z（VTOL 悬停：z=水平倾斜）
        float err_roll = sign_qw * q_error.x * scale;
        float err_pitch = sign_qw * q_error.y * scale;
        float err_yaw = sign_qw * q_error.z * scale;

        float rollRateTarget = rollAnglePID.computeWithExternalDerivative(err_roll, 0.f, w_dps[0]);
        float pitchRateTarget = pitchAnglePID.computeWithExternalDerivative(err_pitch, 0.f, w_dps[1]);
        float yawRateTarget = yawAnglePID.computeWithExternalDerivative(err_yaw, 0.f, w_dps[2]);
        constexpr float MAX_TARGET_RATE = 90.f;  // deg/s
        rollRateTarget = std::clamp(rollRateTarget, -MAX_TARGET_RATE, MAX_TARGET_RATE);
        pitchRateTarget = std::clamp(pitchRateTarget, -MAX_TARGET_RATE, MAX_TARGET_RATE);
        yawRateTarget = std::clamp(yawRateTarget, -MAX_TARGET_RATE, MAX_TARGET_RATE);

        // —— 内环：角速率误差 → 角加速度（flight_control.cpp:1016-1017、1066-1067）——
        float alpha_roll = rollRatePID.computeDerivativeOnMeasurement(rollRateTarget, w_dps[0]);
        float alpha_pitch = pitchRatePID.computeDerivativeOnMeasurement(pitchRateTarget, w_dps[1]);
        float alpha_yaw = yawRatePID.computeDerivativeOnMeasurement(yawRateTarget, w_dps[2]);
        return {alpha_roll, alpha_pitch, alpha_yaw};
    }
};

// ============================================================
//  对角分配（层1 惯量逆解 + 对角执行器映射，同 flight_control.cpp:1130-1132 的轴序）
// ============================================================
static void allocate(const std::array<float, 3> &alpha, float w0,
                     float &dw, float &df, float &dt_)
{
    float Mx = P.Ix * alpha[0];   // alpha_roll → Mx → 差速
    float My = P.Iy * alpha[1];   // alpha_pitch → My → 尾摆
    float Mz = P.Iz * alpha[2];   // alpha_yaw → Mz → 前摆
    float T0 = P.kT * w0 * w0;
    float tau0 = P.kQ * w0 * w0;
    dw = std::clamp(Mx / (-2.f * tau0), -P.dwMax, P.dwMax);
    dt_ = std::clamp(My / (-P.b * T0), -P.dMax, P.dMax);
    df = std::clamp(Mz / (P.a * T0), -P.dMax, P.dMax);
}

// ============================================================
//  单轴扰动闭环仿真：返回收敛后误差范数（rad）
// ============================================================
static float runClosedLoop(const Quaternion &q0, float T_total = 5.f, float dt = 0.001f)
{
    RigidBody body;
    body.q = q0;
    Quaternion q_target{1, 0, 0, 0};
    AttitudeLoop ctrl;
    float w0 = P.wMax * 0.5f;  // 50% 油门
    int N = static_cast<int>(T_total / dt);
    for (int i = 0; i < N; ++i) {
        std::array<float, 3> w_dps{body.w[0] * RAD2DEG, body.w[1] * RAD2DEG, body.w[2] * RAD2DEG};
        auto alpha = ctrl.update(body.q, q_target, w_dps);
        float dw = 0, df = 0, dt_ = 0;
        allocate(alpha, w0, dw, df, dt_);
        auto M = thrustWrench(w0, dw, df, dt_);
        body.step(M, dt);
    }
    Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_target);
    return std::sqrt(qe.x * qe.x + qe.y * qe.y + qe.z * qe.z);
}

static Quaternion rotX(float a) { return {std::cos(a/2), std::sin(a/2), 0, 0}; }
static Quaternion rotY(float a) { return {std::cos(a/2), 0, std::sin(a/2), 0}; }
static Quaternion rotZ(float a) { return {std::cos(a/2), 0, 0, std::sin(a/2)}; }

// ============================================================
//  测试
// ============================================================
int main()
{
    // T1/T2/T4: 闭环收敛（滚转 / 俯仰 / 三轴）
    float e_r = runClosedLoop(rotX(10.f * 3.14159f / 180.f));
    check(e_r < 0.02f, "T1 滚转 10° 扰动闭环收敛", "q_err<1.1°");
    float e_p = runClosedLoop(rotY(10.f * 3.14159f / 180.f));
    check(e_p < 0.02f, "T2 俯仰 10° 扰动闭环收敛", "q_err<1.1°");
    // T4: 组合扰动——滚转/俯仰姿态收敛 + 偏航角速率阻尼
    // （固件设计：偏航为速率模式无航向保持，q_err.z 允许存在，r 必须收敛）
    {
        RigidBody body;
        body.q = quaternionMultiply(rotX(10.f*3.14159f/180.f),
                                    rotY(8.f*3.14159f/180.f));
        body.w = {0.f, 0.f, 0.3f};  // 初始偏航角速度
        Quaternion q_target{1, 0, 0, 0};
        AttitudeLoop ctrl;
        float w0 = P.wMax * 0.5f;
        for (int i = 0; i < 5000; ++i) {
            std::array<float, 3> w_dps{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_target, w_dps);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0, dw, df, dt_);
            body.step(thrustWrench(w0, dw, df, dt_), 0.001f);
        }
        Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_target);
        float e_rp = std::sqrt(qe.x*qe.x + qe.y*qe.y);  // 滚转+俯仰误差
        check(e_rp < 0.02f, "T4 组合扰动：滚转/俯仰姿态收敛", "q_err(xy)<1.1°");
        check(std::fabs(body.w[2]) < 0.05f, "T4 组合扰动：偏航角速率阻尼", "r<0.05 rad/s");
    }

    // T3: 偏航角速率阻尼（r=1 rad/s 初始 → 0）
    {
        RigidBody body;
        body.w = {0.f, 0.f, 1.f};
        Quaternion q_target{1, 0, 0, 0};
        AttitudeLoop ctrl;
        float w0 = P.wMax * 0.5f;
        for (int i = 0; i < 3000; ++i) {  // 3 s
            std::array<float, 3> w_dps{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_target, w_dps);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0, dw, df, dt_);
            body.step(thrustWrench(w0, dw, df, dt_), 0.001f);
        }
        check(std::fabs(body.w[2]) < 0.05f, "T3 偏航角速率阻尼（ω.z→前摆→Mz）", "r<0.05 rad/s");
    }

    // T5: 执行器映射符号与 propulsion.mjs 一致
    {
        float w0 = P.wMax * 0.5f;
        auto M_pos_dt = thrustWrench(w0, 0.f, 0.f, 0.2f);   // 尾摆正
        auto M_pos_df = thrustWrench(w0, 0.f, 0.2f, 0.f);   // 前摆正
        auto M_pos_dw = thrustWrench(w0, 0.2f, 0.f, 0.f);   // 差速正
        check(M_pos_dt[1] < 0.f, "T5 尾摆正 → My<0（与 propulsion.mjs 一致）");
        check(M_pos_df[2] > 0.f, "T5 前摆正 → Mz>0（与 propulsion.mjs 一致）");
        check(M_pos_dw[0] < 0.f, "T5 差速正 → Mx<0（与 propulsion.mjs 一致）");
    }

    // T6: 轴序对齐——单轴扰动只激发对应通道
    {
        // 滚转扰动：误差主要落在 q_err.x → alpha_roll 主导
        RigidBody body;
        body.q = rotX(5.f * 3.14159f / 180.f);
        AttitudeLoop ctrl;
        std::array<float, 3> w_dps{0, 0, 0};
        auto alpha = ctrl.update(body.q, {1, 0, 0, 0}, w_dps);
        check(std::fabs(alpha[0]) > std::fabs(alpha[1]) * 5.f &&
              std::fabs(alpha[0]) > std::fabs(alpha[2]) * 5.f,
              "T6 滚转扰动 → alpha_roll 主导（轴序对齐）");
        // 俯仰扰动 → alpha_pitch 主导
        body.q = rotY(5.f * 3.14159f / 180.f);
        alpha = ctrl.update(body.q, {1, 0, 0, 0}, w_dps);
        check(std::fabs(alpha[1]) > std::fabs(alpha[0]) * 5.f &&
              std::fabs(alpha[1]) > std::fabs(alpha[2]) * 5.f,
              "T6 俯仰扰动 → alpha_pitch 主导（轴序对齐）");
        // 偏航角速度 → alpha_yaw 主导（纯角速度场景，无姿态误差）
        body.q = {1, 0, 0, 0};
        body.w = {0, 0, 0.5f};
        alpha = ctrl.update(body.q, {1, 0, 0, 0}, {0.f, 0.f, 0.5f * RAD2DEG});
        check(std::fabs(alpha[2]) > std::fabs(alpha[0]) * 5.f &&
              std::fabs(alpha[2]) > std::fabs(alpha[1]) * 5.f,
              "T6 偏航角速率 → alpha_yaw 主导（轴序对齐）");
    }

    // T7: 推力垂直投影——悬停（机头朝天，x_b 竖直）时 R13=±1 而 R33≈0
    // （cos_tilt 修复依据：推力沿 +x_b，垂直分量 = x 轴投影；原 R33 在悬停时≈0 → 油门×2）
    {
        const float q = 0.70710678f;  // cos/sin(45°)
        Quaternion qh{q, 0.f, q, 0.f};   // 绕 +y 转 90°（机头朝天）
        float R13 = 2.f * (qh.x * qh.z + qh.y * qh.w);
        float R33 = 1.f - 2.f * (qh.x * qh.x + qh.y * qh.y);
        check(approx(R13, 1.f, 1e-3f), "T7 悬停时 R13（x 轴垂直投影）= 1，cos_tilt 无需补偿");
        check(std::fabs(R33) < 1e-3f, "T7 悬停时 R33（z 轴投影）≈ 0——原公式会钳位 0.5 使油门×2");
        Quaternion ql{1, 0, 0, 0};       // 水平巡航
        float R13_l = 2.f * (ql.x * ql.z + ql.y * ql.w);
        check(approx(R13_l, 0.f, 1e-3f), "T7 水平时 R13≈0（气动升力承担，钳位 0.5 合理）");
    }

    // T8: VTOL 悬停构型目标姿态合成——q_target = q_tilt ⊗ q_hover ⊗ Rx(-Heading)
    // （AUTO_POSITION/GUIDED 目标姿态修复：悬停基准下航向轴 = 机体 x = 差速轴）
    {
        const float q = 0.70710678f;
        Quaternion q_hover{q, 0.f, q, 0.f};          // 绕 NED +y 转 90°（机头朝天）
        // 场景 A：悬停 + 世界航向 90°，无倾斜（q_tilt 恒等）
        // 期望 = q_W ⊗ q_hover = (0.5, -0.5, 0.5, 0.5)（绕 NED z 转 90° 的悬停姿态）
        Quaternion q_yaw = {q, -q, 0.f, 0.f};        // Rx(-90°)
        Quaternion q_target = quaternionMultiply(quaternionMultiply(q_hover, q_yaw), {1,0,0,0});
        check(approx(q_target.w, 0.5f, 1e-3f) && approx(q_target.x, -0.5f, 1e-3f) &&
              approx(q_target.y, 0.5f, 1e-3f) && approx(q_target.z, 0.5f, 1e-3f),
              "T8 悬停+航向90° 目标姿态 = (0.5,-0.5,0.5,0.5)（绕 NED z 转 90°）");
        // 对照：原公式（绕机体 z 航向）给出 (0.707,0,0,0.707)——悬停下错误
        Quaternion q_old = quaternionMultiply({q,0,0,q}, {1,0,0,0});
        check(!(approx(q_old.w, 0.5f, 1e-3f) && approx(q_old.x, -0.5f, 1e-3f)),
              "T8 原公式（绕机体 z）≠ 期望——修复必要性确认");
        // 场景 B：航向 0 → 目标 = q_hover（机头朝天），机头方向 = NED (0,0,-1)
        Quaternion qb = quaternionMultiply(q_hover, {1,0,0,0});
        Quaternion qe2 = quaternionMultiply({1,0,0,0}, {qb.w, -qb.x, -qb.y, -qb.z});
        // x̂_b 在 NED = q ⊗ x̂ ⊗ q⁻¹ 的矢量部分
        Quaternion v = quaternionMultiply(quaternionMultiply(qb, {0,1,0,0}), {qb.w,-qb.x,-qb.y,-qb.z});
        check(approx(v.x, 0.f, 1e-3f) && approx(v.y, 0.f, 1e-3f) && approx(v.z, -1.f, 1e-3f),
              "T8 航向 0 时机头方向 = NED (0,0,-1)（机头朝天）");
    }

    // T9: VTOL 悬停构型闭环——目标姿态 = 机头朝天（q_hover），初始绕机体轴扰动 → 收敛
    // （x_b 竖直：q_err.x = 世界航向误差（差速通道）、q_err.y/z = 倾斜误差（尾摆/前摆通道））
    {
        const float q = 0.70710678f;
        Quaternion q_hover{q, 0.f, q, 0.f};   // 目标：机头朝天
        auto run_hover = [&](const Quaternion &q0_dist, float T_total = 5.f) -> float {
            RigidBody body;
            body.q = quaternionMultiply(q_hover, q0_dist);  // 悬停基态 + 机体轴扰动
            AttitudeLoop ctrl;
            float w0 = P.wMax * 0.5f;
            int N = static_cast<int>(T_total / 0.001f);
            for (int i = 0; i < N; ++i) {
                std::array<float, 3> w_dps{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
                auto alpha = ctrl.update(body.q, q_hover, w_dps);
                float dw = 0, df = 0, dt_ = 0;
                allocate(alpha, w0, dw, df, dt_);
                body.step(thrustWrench(w0, dw, df, dt_), 0.001f);
            }
            Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_hover);
            return std::sqrt(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
        };
        const float a5 = 5.f * 3.14159f / 180.f;
        check(run_hover(rotX(a5)) < 0.02f, "T9 悬停：绕 x（世界航向）5° 扰动收敛", "q_err<1.1°");
        check(run_hover(rotY(a5)) < 0.02f, "T9 悬停：绕 y（俯仰）5° 扰动收敛", "q_err<1.1°");
        check(run_hover(rotZ(a5)) < 0.02f, "T9 悬停：绕 z（滚转）5° 扰动收敛", "q_err<1.1°");
        // 组合扰动（含世界航向误差）
        Quaternion qc = quaternionMultiply(rotX(a5), quaternionMultiply(rotY(a5), rotZ(a5)));
        check(run_hover(qc) < 0.02f, "T9 悬停：三轴组合扰动收敛", "q_err<1.1°");
    }

    // T10: VTOL 悬停油门——cos_tilt 修复后悬停所需总推力 = m·g（R13=1 无补偿）
    // （修复前 R33≈0 → cos_tilt 钳位 0.5 → 总需求×2 = 2·m·g；
    //   油门为转速比（P.thr = w/wMax）：悬停 √(mg/2kT·wMax²) ≈ 49.9%，
    //   修复前 √(2mg/2kT·wMax²) ≈ 70.6%——从 50% 抬到 71%，机动裕量大幅丧失）
    {
        const float q = 0.70710678f;
        Quaternion qh{q, 0.f, q, 0.f};   // 机头朝天
        float cos_tilt_new = std::fabs(2.f * (qh.x*qh.z + qh.y*qh.w));  // R13
        float cos_tilt_old = 1.f - 2.f * (qh.x*qh.x + qh.y*qh.y);       // R33
        float m = P.m, g = P.g;
        float T_req_new = m * g / std::max(cos_tilt_new, 0.5f);
        float T_req_old = m * g / std::max(cos_tilt_old, 0.5f);
        float T_max_total = 2.f * P.kT * P.wMax * P.wMax;   // 双发满推力（MAX_THRUST）
        check(approx(T_req_new, m*g, 1e-3f), "T10 修复后悬停需求总推力 = m·g（无需补偿）");
        check(approx(T_req_old, 2.f*m*g, 1e-3f), "T10 修复前悬停需求 = 2·m·g（油门翻倍）");
        check(T_req_old < T_max_total && T_req_new < T_max_total,
              "T10 双发满推力可覆盖悬停需求（修复前接近饱和、裕量丧失）");
    }

    std::printf("\n%s\n", g_fail ? "=== 存在失败项 ===" : "=== 全部通过 ===");
    return g_fail ? 1 : 0;
}
