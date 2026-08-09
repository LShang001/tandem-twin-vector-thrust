// 宿主机测试：AirframeModel 通用框架跨机型验证（四旋翼 demo）
// 编译运行：
//   g++ -std=c++17 -Iinclude test_host/test_airframe_generic.cpp \
//       -o test_host/bin/ag && ./test_host/bin/ag
//
// ★2026-08-10 通用层验证：
//   用同一套 AirframeModel（执行器几何表 + 输入映射 + 数值 Jacobian 合成器）
//   定义四旋翼（与纵列双发不同的拓扑），验证"换机型 = 填几何数据，不写算法"。
//
// 覆盖：
//   A1 四旋翼正向映射（推力/反扭叠加）
//   A2 数值 Jacobian 生成（3×4）
//   A3 伪逆分配 → 正向往返（M_cmd → ω² → M ≈ M_cmd，相对误差 <1e-3）
//   A4 经典 Mixer 对照（Mx=左右差、My=前后差、Mz=CW/CCW 差）
#include "../include/AirframeModel.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_fail = 0;
static void check(bool cond, const std::string &name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail;
}
static bool approx(float a, float b, float tol = 1e-3f)
{
    float d = std::fabs(a - b);
    return d <= tol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

// ============================================================
//  四旋翼几何（十字布局，模型系 = 机体系 x 前 y 右 z 下）
//   电机1 前右 (a, a, 0)  CW   电机2 后左 (-a,-a,0) CW
//   电机3 前左 (a,-a,0)  CCW  电机4 后右 (-a,a,0) CCW
//   推力沿 -z（向上）；反扭沿 -z：CW=-kQ·w²（负 yaw）、CCW=+kQ·w²
// ============================================================
static const float kArm = 0.25f;   // 臂长 m
static const float kT4 = 1e-5f;    // 推力系数
static const float kQ4 = 2e-7f;    // 反扭系数
static const ActuatorDef kQuadActs[4] = {
    {ActuatorKind::kRotor, { kArm,  kArm, 0.f}, {0,0,-1}, {0,0,0},  1.f, 0, 0.f, -1},
    {ActuatorKind::kRotor, {-kArm, -kArm, 0.f}, {0,0,-1}, {0,0,0},  1.f, 1, 0.f, -1},
    {ActuatorKind::kRotor, { kArm, -kArm, 0.f}, {0,0,-1}, {0,0,0}, -1.f, 2, 0.f, -1},
    {ActuatorKind::kRotor, {-kArm,  kArm, 0.f}, {0,0,-1}, {0,0,0}, -1.f, 3, 0.f, -1},
};
static const AirframeModel kQuadModel = {kQuadActs, 4};

// 4×3 伪逆：B[3][4] → Bpinv[4][3] = Bᵀ(BBᵀ)⁻¹
static void pinv(const float B[3][4], float Bpinv[4][3])
{
    // S = B·Bᵀ（3×3）
    float S[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            S[i][j] = 0;
            for (int k = 0; k < 4; ++k) S[i][j] += B[i][k] * B[j][k];
        }
    // 3×3 逆（伴随矩阵）
    float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    float inv[3][3];
    inv[0][0] =  (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / det;
    inv[0][1] = -(S[0][1] * S[2][2] - S[0][2] * S[2][1]) / det;
    inv[0][2] =  (S[0][1] * S[2][1] - S[0][2] * S[1][1]) / det;
    inv[1][0] = -(S[1][0] * S[2][2] - S[1][2] * S[2][0]) / det;
    inv[1][1] =  (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / det;
    inv[1][2] = -(S[0][0] * S[2][1] - S[0][2] * S[1][0]) / det;
    inv[2][0] =  (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / det;
    inv[2][1] = -(S[0][0] * S[2][1] - S[0][1] * S[2][0]) / det;
    inv[2][2] =  (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / det;
    // Bpinv = Bᵀ · inv
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j)
        {
            Bpinv[i][j] = 0;
            for (int k = 0; k < 3; ++k) Bpinv[i][j] += B[k][i] * inv[k][j];
        }
}

int main()
{
    std::printf("══════════════════════════════════════════\n");
    std::printf(" AirframeModel 通用框架：四旋翼 demo（换机型=填数据）\n");
    std::printf("══════════════════════════════════════════\n");

    // A1: 正向映射（4 电机等速 w0 → 总推力 4T、零力矩）
    std::printf("\n-- A1: 正向映射 --\n");
    {
        const float w0 = 500.f;
        const float u[4] = {w0 * w0, w0 * w0, w0 * w0, w0 * w0};
        float out[6];
        computeWrenchGeneric(kQuadActs, 4, u, w0, kT4, kQ4, out);
        check(approx(out[2], -4.f * kT4 * w0 * w0, 1e-4f), "A1 等速 → 总推力 = 4T");
        check(approx(out[0], 0.f, 1e-4f) && approx(out[1], 0.f, 1e-4f), "A1 等速 → Fx=Fy=0");
        check(approx(out[3], 0.f, 1e-4f) && approx(out[4], 0.f, 1e-4f) && approx(out[5], 0.f, 1e-4f),
              "A1 等速 → 力矩全零（CW/CCW 反扭对消）");
    }

    // A2: 数值 Jacobian（3×4）
    std::printf("\n-- A2: 数值 Jacobian --\n");
    float B[3][4];
    {
        const float w0 = 500.f;
        const float u0[4] = {w0 * w0, w0 * w0, w0 * w0, w0 * w0};
        computeJacobianNumeric<4>(kQuadModel, u0, w0, kT4, kQ4, B);
        // Mx（roll）：右翼（电机2,4 r_y<0）减速 → 正 roll？
        //   Mx = Σ r_y·Fz：电机2 r_y=-a、电机4 r_y=+a
        //   ∂Mx/∂ω1² = r1_y·(-T) 系数 = (+a)·kT = a·kT（电机1）
        check(approx(B[0][0], kArm * kT4, 1e-3f), "A2 ∂Mx/∂ω1² = +a·kT");
        check(approx(B[0][1], -kArm * kT4, 1e-3f), "A2 ∂Mx/∂ω2² = -a·kT");
        check(approx(B[1][0], kArm * kT4, 1e-3f), "A2 ∂My/∂ω1² = +a·kT（前电机正 pitch）");
        check(approx(B[2][0], -kQ4, 1e-3f), "A2 ∂Mz/∂ω1² = -kQ（CW 负 yaw）");
        check(approx(B[2][2],  kQ4, 1e-3f), "A2 ∂Mz/∂ω3² = +kQ（CCW 正 yaw）");
    }

    // A3: 伪逆分配 → 正向往返
    std::printf("\n-- A3: 伪逆分配 → 正向往返 --\n");
    {
        const float w0 = 500.f;
        const float u0[4] = {w0 * w0, w0 * w0, w0 * w0, w0 * w0};
        computeJacobianNumeric<4>(kQuadModel, u0, w0, kT4, kQ4, B);
        float Bpinv[4][3];
        pinv(B, Bpinv);
        // 目标力矩（roll/pitch/yaw）
        const float M_cmd[3] = {0.01f, 0.01f, 0.001f};
        float du[4];
        for (int i = 0; i < 4; ++i)
        {
            du[i] = 0;
            for (int j = 0; j < 3; ++j) du[i] += Bpinv[i][j] * M_cmd[j];
        }
        // 正向映射回
        float u[4];
        for (int i = 0; i < 4; ++i) u[i] = u0[i] + du[i];
        float out[6];
        computeWrenchGeneric(kQuadActs, 4, u, w0, kT4, kQ4, out);
        check(approx(out[3], M_cmd[0], 1e-3f), "A3 往返 Mx ≈ M_cmd[0]");
        check(approx(out[4], M_cmd[1], 1e-3f), "A3 往返 My ≈ M_cmd[1]");
        check(approx(out[5], M_cmd[2], 1e-3f), "A3 往返 Mz ≈ M_cmd[2]");
    }

    // A4: Mixer 对照（经典四旋翼混合矩阵方向）
    std::printf("\n-- A4: Mixer 对照 --\n");
    {
        // 纯 roll 指令（Mx>0）→ 右翼电机（2,4 r_y<0 → 需 ω 减小产生正 Mx？
        //   Mx = Σ r_y·Fz = Σ r_y·(-kT·ω²)；Mx>0 → Σ r_y·ω² < 0 → r_y<0 电机（2,4）ω 增加
        //   ∂Mx/∂ω² = r_y·(-kT)：电机2 r_y=-a → +a·kT → ω2² 增 → Mx 增 ✓
        //   即 roll 指令 → 电机2/4 增、1/3 减
        const float w0 = 500.f;
        const float u0[4] = {w0 * w0, w0 * w0, w0 * w0, w0 * w0};
        float B3[3][4];
        computeJacobianNumeric<4>(kQuadModel, u0, w0, kT4, kQ4, B3);
        float Bpinv[4][3];
        pinv(B3, Bpinv);
        const float M_roll[3] = {0.01f, 0.f, 0.f};
        float du[4];
        for (int i = 0; i < 4; ++i) du[i] = Bpinv[i][0] * M_roll[0];
        // ∂Mx/∂ω² = -r_y·kT：电机2/3（r_y=-a，左翼）正贡献→增、1/4（r_y=+a，右翼）负→减
        check(du[1] > 0.f && du[2] > 0.f, "A4 正 roll → 电机2/3（左翼 r_y<0）加速");
        check(du[0] < 0.f && du[3] < 0.f, "A4 正 roll → 电机1/4（右翼 r_y>0）减速");
    }

    std::printf("\n");
    if (g_fail == 0) std::printf("=== 全部通过（通用框架跨机型验证）===\n");
    else             std::printf("=== %d 项失败 ===\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
