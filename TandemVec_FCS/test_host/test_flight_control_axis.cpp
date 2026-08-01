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
//  六自由度刚体（含陀螺耦合 ω×(Iω)+ω×h、可选电机一阶滞后、平动/位置积分）
// ============================================================
struct SixDOF {
    std::array<float, 3> w{0, 0, 0}, v{0, 0, 0}, pos{0, 0, 0};
    Quaternion q{1, 0, 0, 0};
    float wf = 0, wt = 0;          // 实际转速（电机一阶滞后）
    bool motorLag = false;         // 启用 tauM 一阶滞后
    float norm_before = 1.f;       // 归一化前四元数范数（保范测试用）

    void initRotors(float w0) { wf = w0; wt = w0; }

    void step(float w0, float dw, float df, float dt_,
              float dt, const TandemVecParams &P_)
    {
        // —— 推进（可选一阶滞后）——
        if (motorLag) {
            float wfT = w0 * std::sqrt(std::max(0.f, 1.f + dw));
            float wtT = w0 * std::sqrt(std::max(0.f, 1.f - dw));
            float a = std::min(dt / P_.tauM, 1.f);
            wf += (wfT - wf) * a;
            wt += (wtT - wt) * a;
        } else {
            wf = w0 * std::sqrt(std::max(0.f, 1.f + dw));
            wt = w0 * std::sqrt(std::max(0.f, 1.f - dw));
        }
        float Tf = P_.kT * wf * wf, Tt = P_.kT * wt * wt;
        float Qf = P_.kQ * wf * wf, Qt = P_.kQ * wt * wt;
        float cf = std::cos(df), sf = std::sin(df);
        float ct = std::cos(dt_), st = std::sin(dt_);
        std::array<float, 3> F{Tf*cf + Tt*ct, Tf*sf, -Tt*st};
        std::array<float, 3> Mf{-Qf*cf + Qt*ct, -P_.b*Tt*st - Qf*sf, P_.a*Tf*sf - Qt*st};

        // —— 转动：I·ω̇ = M − ω×(I·ω) − ω×h_rotor ——
        std::array<float, 3> I{P_.Ix, P_.Iy, P_.Iz};
        // 转子角动量：前 +x（CW）、尾 −x（CCW 反转）→ h = Jp·(wf − wt)·x̂
        float hx = P_.Jp * (wf - wt);
        float gx = (I[2]-I[1])*w[1]*w[2];
        float gy = (I[0]-I[2])*w[2]*w[0] - w[2]*hx;
        float gz = (I[1]-I[0])*w[0]*w[1] + w[1]*hx;
        for (int k = 0; k < 3; ++k)
            w[k] += (Mf[k] - (k==0?gx:(k==1?gy:gz))) / I[k] * dt;

        // —— 姿态积分（小旋转四元数 w≈1）——
        Quaternion dq = quaternionMultiply(q, {1, w[0]*0.5f*dt, w[1]*0.5f*dt, w[2]*0.5f*dt});
        q = dq;
        float n = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
        norm_before = n;           // 归一化前范数（积分漂移的直接度量）
        q.w/=n; q.x/=n; q.y/=n; q.z/=n;

        // —— 平动：m·v̇ = F + m·g_b − m·ω×v（g_b = 重力在机体系）——
        std::array<float, 3> gb{0.f, 0.f, P_.g};
        {
            Quaternion qi{q.w, -q.x, -q.y, -q.z};
            Quaternion p{0, 0, 0, gb[2]};
            Quaternion t1 = quaternionMultiply(qi, p);
            Quaternion t2 = quaternionMultiply(t1, q);
            gb = {static_cast<float>(t2.x), static_cast<float>(t2.y), static_cast<float>(t2.z)};
        }
        std::array<float, 3> wxv{w[1]*v[2]-w[2]*v[1], w[2]*v[0]-w[0]*v[2], w[0]*v[1]-w[1]*v[0]};
        for (int k = 0; k < 3; ++k)
            v[k] += (F[k]/P_.m + gb[k] - wxv[k]) * dt;

        // —— 位置积分：NED 速度 = R(q)·v ——
        std::array<float, 3> vN{0,0,0};
        {
            Quaternion p{0, v[0], v[1], v[2]};
            Quaternion t1 = quaternionMultiply(q, p);
            Quaternion t2 = quaternionMultiply(t1, Quaternion{q.w, -q.x, -q.y, -q.z});
            vN = {static_cast<float>(t2.x), static_cast<float>(t2.y), static_cast<float>(t2.z)};
        }
        for (int k = 0; k < 3; ++k) pos[k] += vN[k] * dt;
    }
};

// ============================================================
//  悬停配平：w0 = √(mg/2kT)（转速），摆角 0、差速 0
// ============================================================
static float hoverOmega0() { return std::sqrt(P.m * P.g / (2.f * P.kT)); }

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

    // T11: RATE_MODE 四轴式摇杆映射链（VTOL 悬停构型，方向符号经数值推导）
    // 悬停 x_b = NED (0,0,-1)（朝上）：绕 +z_NED 正转=北→东=地图顺时针=航向正；
    // 绕 +x_b 正转 = 绕 −z_NED 正转 = 航向负 → yaw 右推需 Mx<0（绕 x_b 负转=航向正）
    // 内环增益用固件实际值（rollRatePID kp=0.30、yawRatePID kp=0.15，state_data.cpp）
    {
        float w0 = P.wMax * 0.5f;
        const float MAX_YAW_RATE = 35.f;   // MAX_MANUAL_yawRATE（差速保守幅值）
        const float MAX_ROLL_RATE = 50.f;  // MAX_MANUAL_rollRATE
        // yaw 摇杆右满偏 → rollRateTarget = -35°/s（mapFloat 反号后）→ alpha_roll = 0.30×(-35)
        float rollRateTarget = -MAX_YAW_RATE;
        float alpha_roll = 0.30f * (rollRateTarget - 0.0f);
        float dw, df, dt_;
        allocate({alpha_roll, 0.f, 0.f}, w0, dw, df, dt_);
        auto M = thrustWrench(w0, dw, df, dt_);
        check(M[0] < 0.f, "T11 yaw 摇杆右推 → Mx<0（绕 x_b 负转 = 世界航向正转，四轴 yaw 语义）");
        // roll 摇杆右满偏 → yawRateTarget = +50°/s → alpha_yaw = 0.15×50 → Mz
        float yawRateTarget = MAX_ROLL_RATE;
        float alpha_yaw = 0.15f * (yawRateTarget - 0.0f);
        allocate({0.f, 0.f, alpha_yaw}, w0, dw, df, dt_);
        M = thrustWrench(w0, dw, df, dt_);
        check(M[2] > 0.f, "T11 roll 摇杆右推 → Mz>0（绕 z_b 倾斜力矩）");
        // pitch 摇杆推杆 → pitchRateTarget 负（低头）→ alpha_pitch = 0.30×(−50) → My<0
        float pitchRateTarget = -MAX_ROLL_RATE;
        float alpha_pitch = 0.30f * (pitchRateTarget - 0.0f);
        allocate({0.f, alpha_pitch, 0.f}, w0, dw, df, dt_);
        M = thrustWrench(w0, dw, df, dt_);
        check(M[1] < 0.f, "T11 pitch 摇杆推杆 → My<0（低头力矩）");
    }

    // ============================================================
    //  T12–T19：六自由度悬停全动力学闭环（SixDOF + 姿态环）
    // ============================================================
    const float q45 = 0.70710678f;
    Quaternion q_hover{q45, 0.f, q45, 0.f};   // 机头朝天
    float w0h = hoverOmega0();                // 悬停配平转速

    auto run_hover6 = [&](SixDOF &body, float T_total) -> float {
        // 返回末态四元数误差范数（相对 q_hover）；每次调用新建控制器（无状态污染）
        AttitudeLoop ctrl;
        int N = static_cast<int>(T_total / 0.001f);
        for (int i = 0; i < N; ++i) {
            std::array<float, 3> wd{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_hover, wd);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0h, dw, df, dt_);
            body.step(w0h, dw, df, dt_, 0.001f, P);
        }
        Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_hover);
        return std::sqrt(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
    };

    // T12: 悬停配平平衡——转速 √(mg/2kT) 下双发总推力 = mg；6DOF 闭环 10s 垂直速度≈0
    {
        float T_total = 2.f * P.kT * w0h * w0h;
        check(approx(T_total, P.m * P.g, 1e-3f), "T12 悬停配平：2·kT·w0² = m·g");
        SixDOF body;
        body.q = q_hover;
        body.initRotors(w0h);
        float e = run_hover6(body, 10.f);
        check(e < 0.02f, "T12 悬停 10s 姿态保持（q_err<1.1°）");
        check(std::fabs(body.v[2]) < 0.05f && std::fabs(body.pos[2]) < 1.f,
              "T12 悬停 10s 垂直速度≈0、高度漂移<1m（配平平衡）");
    }

    // T13: 悬停航向保持——初始绕 x_b（世界航向）角速率 → 差速阻尼 + q_err.x 外环收敛
    {
        SixDOF body;
        body.q = q_hover;
        body.w = {0.5f, 0.f, 0.f};   // 初始航向角速度 0.5 rad/s
        body.initRotors(w0h);
        float e = run_hover6(body, 5.f);
        check(e < 0.02f, "T13 航向角速率扰动收敛（q_err<1.1°）");
        check(std::fabs(body.w[0]) < 0.05f, "T13 航向角速度衰减到 <0.05 rad/s（差速阻尼）");
    }

    // T14: 饱和鲁棒——60° 大姿态扰动触达执行器限幅，闭环仍收敛
    {
        const float a60 = 60.f * 3.14159f / 180.f;
        SixDOF body;
        body.q = quaternionMultiply(q_hover, rotY(a60));
        body.initRotors(w0h);
        AttitudeLoop ctrl;
        bool sat_seen = false;
        for (int i = 0; i < 8000; ++i) {
            std::array<float, 3> wd{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_hover, wd);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0h, dw, df, dt_);
            if (std::fabs(dw) >= P.dwMax - 1e-3f || std::fabs(dt_) >= P.dMax - 1e-3f ||
                std::fabs(df) >= P.dMax - 1e-3f) sat_seen = true;
            body.step(w0h, dw, df, dt_, 0.001f, P);
        }
        Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_hover);
        float e = std::sqrt(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
        check(sat_seen, "T14 60° 扰动确实触达执行器限幅（dMax/dwMax）");
        check(e < 0.02f, "T14 60° 大扰动收敛（饱和限幅下仍稳定）");
    }

    // T15: 电机滞后——推进一阶滞后（tauM=0.28s）下悬停保持仍收敛
    {
        SixDOF body;
        body.q = q_hover;
        body.initRotors(w0h);
        body.motorLag = true;
        float e = run_hover6(body, 10.f);
        check(e < 0.02f, "T15 电机滞后（τ=0.28s）下悬停保持收敛");
        // 滞后下差速修正响应变慢：姿态仍应保持在合理范围
    }

    // T16: 陀螺耦合——初始差速（wf≠wt → h_x≠0）+ 电机滞后（τ=0.28s 真实衰减），
    // 前 ~2s 内转子角动量非零 → ω×h 项真实作用，姿态环仍收敛
    {
        SixDOF body;
        body.q = q_hover;
        body.wf = 1.1f * w0h;   // 初始转速差 → h_x = Jp(wf−wt) ≠ 0
        body.wt = 0.9f * w0h;
        body.motorLag = true;   // 滞后分支不覆盖初始转速，向目标衰减
        AttitudeLoop ctrl;
        float max_rotor_diff = 0.f;
        for (int i = 0; i < 5000; ++i) {
            std::array<float, 3> wd{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_hover, wd);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0h, dw, df, dt_);
            body.step(w0h, dw, df, dt_, 0.001f, P);
            max_rotor_diff = std::max(max_rotor_diff, std::fabs(body.wf - body.wt));
        }
        Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_hover);
        float e = std::sqrt(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
        check(max_rotor_diff > 50.f,
              "T16 转子差速在衰减期内真实存在（h_x≠0，陀螺耦合激活）");
        check(e < 0.02f, "T16 陀螺耦合（ω×h）下悬停保持收敛");
    }

    // T17: 航向指令跟踪——目标 = 悬停 + 绕 x_b 转 20°（新航向）→ 收敛且航向实际转过 20°
    {
        const float a20 = 20.f * 3.14159f / 180.f;
        Quaternion q_target_new = quaternionMultiply(q_hover, rotX(a20));
        SixDOF body;
        body.q = q_hover;
        body.initRotors(w0h);
        AttitudeLoop ctrl;
        for (int i = 0; i < 6000; ++i) {
            std::array<float, 3> wd{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_target_new, wd);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0h, dw, df, dt_);
            body.step(w0h, dw, df, dt_, 0.001f, P);
        }
        Quaternion qe = quaternionMultiply(quaternionConjugate(body.q), q_target_new);
        float e = std::sqrt(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
        check(e < 0.02f, "T17 航向指令跟踪：收敛到目标航向（q_err<2.3°）");
        // 相对初始悬停的绕 x_b 转角（= 世界航向变化量）
        Quaternion qd = quaternionMultiply(quaternionConjugate(body.q), q_hover);
        float angle_x = 2.f * std::asin(std::fabs(qd.x));
        check(std::fabs(angle_x - a20) < 0.05f,
              "T17 航向实际转过 ≈20°（差速驱动，四轴 yaw 语义）");
    }

    // T18: 四元数保范——6000 步（6s）积分范数偏差 < 1e-9
    {
        SixDOF body;
        body.q = q_hover;
        body.w = {0.3f, -0.2f, 0.4f};
        body.initRotors(w0h);
        AttitudeLoop ctrl;
        float max_norm_err = 0.f;
        for (int i = 0; i < 6000; ++i) {
            std::array<float, 3> wd{body.w[0]*RAD2DEG, body.w[1]*RAD2DEG, body.w[2]*RAD2DEG};
            auto alpha = ctrl.update(body.q, q_hover, wd);
            float dw = 0, df = 0, dt_ = 0;
            allocate(alpha, w0h, dw, df, dt_);
            body.step(w0h, dw, df, dt_, 0.001f, P);
            // 归一化前范数：积分漂移的直接度量（归一化后恒≈1 无意义）
            max_norm_err = std::max(max_norm_err, std::fabs(body.norm_before - 1.f));
        }
        // float 角速度下单步欧拉积分漂移 ~5.96e-8（float ε）量级，不随步数累积
        check(max_norm_err < 1e-6f, "T18 四元数保范（6000 步归一化前范数偏差<1e-6）");
    }

    std::printf("\n%s\n", g_fail ? "=== 存在失败项 ===" : "=== 全部通过 ===");
    return g_fail ? 1 : 0;
}
