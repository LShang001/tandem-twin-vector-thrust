/**
 * @file    test_coaxial_dynamics_model.cpp
 * @brief   共轴双桨 VTVL rigorous 动力学模型宿主回归（2026-10-31）
 *
 * 覆盖：kT 反标定/悬停转速、推力平方律、反扭（ω² 与 Jp·ω̇ 两项）、
 *       陀螺耦合符号与等速反转抵消、摆座力矩、半隐式 6DOF 积分、
 *       自由落体能量守恒、地面弹簧-阻尼/摩擦、舵机速率限幅、非有限输入。
 */

#include <cmath>
#include <cstdio>

#include "coaxial_dynamics_model.h"

using namespace bfs::coax;

static int g_checks = 0;
static int g_fail = 0;

static void expect(const bool cond, const char *what)
{
    ++g_checks;
    if (!cond)
    {
        ++g_fail;
        std::printf("[FAIL] %s\n", what);
    }
}

static void expectNear(const float got, const float want, const float tol,
                       const char *what)
{
    ++g_checks;
    if (!(std::fabs(got - want) <= tol))
    {
        ++g_fail;
        std::printf("[FAIL] %s: got %.9g want %.9g tol %.3g\n", what,
                    static_cast<double>(got), static_cast<double>(want),
                    static_cast<double>(tol));
    }
}

static Vec3 zeroRate()
{
    return Vec3{0.0f, 0.0f, 0.0f};
}

int main()
{
    // ---- 反标定与悬停 ----
    Params p = makeParams(14.0f, 1150.0f);
    expectNear(p.kT_n_per_radps2, 14.0f / (2.0f * 1150.0f * 1150.0f),
               1.0e-13f, "kT = maxT/(2 wMax^2)");
    expectNear(p.kQ_nm_per_radps2, p.kT_n_per_radps2 * 0.027f, 1.0e-14f,
               "kQ = 0.027 kT");
    const float wh = hoverRotorSpeed(p);
    expect(wh > 100.0f && wh < p.w_max_radps, "hover speed in range");
    expectNear(2.0f * p.kT_n_per_radps2 * wh * wh, p.mass_kg * p.gravity_mps2,
               1.0e-3f, "2 kT wh^2 = mg");

    // ---- 悬停 wrench：推力抵消重力，反扭/陀螺对称抵消 ----
    const Wrench w_hover = computeWrench(p, wh, wh, 0.0f, 0.0f, 0.0f, 0.0f,
                                         zeroRate());
    expect(w_hover.valid, "hover wrench valid");
    expectNear(w_hover.thrust_n, p.mass_kg * p.gravity_mps2, 5.0e-3f,
               "hover thrust = mg");
    expectNear(w_hover.force_body.z, -p.mass_kg * p.gravity_mps2, 5.0e-3f,
               "hover Fz = -mg");
    expectNear(w_hover.reaction_nm, 0.0f, 1.0e-7f, "hover reaction 0");
    expectNear(w_hover.moment_body.z, 0.0f, 1.0e-7f, "hover Mz 0");
    expectNear(w_hover.gyro_moment_nm.y, 0.0f, 1.0e-9f, "hover gyro 0");

    // ---- 推力平方律：转速翻倍 → 推力 4 倍 ----
    const Wrench w_double = computeWrench(p, 2.0f * wh, 2.0f * wh, 0.0f, 0.0f,
                                          0.0f, 0.0f, zeroRate());
    expectNear(w_double.thrust_n, 4.0f * w_hover.thrust_n, 1.0e-2f,
               "thrust ~ w^2");

    // ---- 反扭：wf>wt → Mz<0；Jp·ω̇ 项独立可测 ----
    const Wrench w_diff = computeWrench(p, 1.1f * wh, 0.9f * wh, 0.0f, 0.0f,
                                        0.0f, 0.0f, zeroRate());
    expect(w_diff.moment_body.z < 0.0f, "wf>wt -> Mz negative");
    const Wrench w_wdot = computeWrench(p, wh, wh, 50.0f, -50.0f, 0.0f, 0.0f,
                                        zeroRate());
    expectNear(w_wdot.reaction_nm, p.rotor_inertia_kgm2 * 100.0f, 1.0e-6f,
               "Jp*(wdot_f-wdot_t) reaction");
    expectNear(w_wdot.moment_body.z, -p.rotor_inertia_kgm2 * 100.0f, 1.0e-6f,
               "reaction enters -Mz");

    // ---- 陀螺：等速反转严格抵消；差速 + 滚转角速率给出 My<0 ----
    const Vec3 roll_rate{1.0f, 0.0f, 0.0f};
    const Wrench w_gyro0 = computeWrench(p, wh, wh, 0.0f, 0.0f, 0.0f, 0.0f,
                                         roll_rate);
    expectNear(w_gyro0.gyro_moment_nm.x, 0.0f, 1.0e-9f, "gyro x 0 (equal)");
    expectNear(w_gyro0.gyro_moment_nm.y, 0.0f, 1.0e-9f, "gyro y 0 (equal)");
    expectNear(w_gyro0.gyro_moment_nm.z, 0.0f, 1.0e-9f, "gyro z 0 (equal)");
    const Wrench w_gyro = computeWrench(p, wh + 50.0f, wh - 50.0f, 0.0f, 0.0f,
                                        0.0f, 0.0f, roll_rate);
    expect(w_gyro.gyro_moment_nm.y < 0.0f,
           "differential rotor + roll rate -> My negative");

    // ---- 摆座几何：δf→+Mx、δt→+My，推力向 +x/+y 偏转 ----
    const Wrench w_servo = computeWrench(p, wh, wh, 0.0f, 0.0f, 0.10f, 0.10f,
                                         zeroRate());
    expect(w_servo.moment_body.x > 0.0f, "servo_f -> +Mx");
    expect(w_servo.moment_body.y > 0.0f, "servo_t -> +My");
    expect(w_servo.force_body.x > 0.0f, "servo_t -> +Fx");
    expect(w_servo.force_body.y > 0.0f, "servo_f -> +Fy");

    // ---- 非有限输入 ----
    const Wrench w_nan = computeWrench(p, NAN, wh, 0.0f, 0.0f, 0.0f, 0.0f,
                                       zeroRate());
    expect(!w_nan.valid, "NaN speed invalid");

    // ---- 自由落体：零推力、无地面、能量守恒 ----
    {
        Params pf = p;
        pf.enable_ground = false;
        pf.drag_linear_n_per_mps = 0.0f;
        pf.drag_quadratic_n_per_mps2 = 0.0f;
        State s;
        s.pos_ned.z = -10.0f;
        const float e0 = mechanicalEnergy(pf, s);
        Command c; // 全零：旋翼不转
        for (int i = 0; i < 1000; ++i)
        {
            step(pf, s, c, 0.001f, 10);
        }
        expectNear(s.vel_ned.z, pf.gravity_mps2, 0.05f, "free fall vz=g*t");
        expectNear(s.pos_ned.z, -10.0f + 0.5f * pf.gravity_mps2, 0.1f,
                   "free fall z");
        const float e1 = mechanicalEnergy(pf, s);
        expect(std::fabs(e1 - e0) <= std::fmax(0.02f * std::fabs(e0), 1.0f),
               "free fall energy conserved");
    }

    // ---- 悬停保持：给定 ω_h，1 s 内高度漂移小 ----
    {
        Params ph = p;
        ph.enable_ground = false;
        State s;
        s.pos_ned.z = -20.0f;
        s.wf = wh;
        s.wt = wh;
        Command c;
        c.wf_cmd = wh;
        c.wt_cmd = wh;
        for (int i = 0; i < 1000; ++i)
        {
            step(ph, s, c, 0.001f, 10);
        }
        expect(std::fabs(s.vel_ned.z) < 0.05f, "hover vz small");
        expect(std::fabs(s.pos_ned.z + 20.0f) < 0.05f, "hover altitude holds");
        expect(s.on_ground == false, "hover not on ground");
    }

    // ---- 地面接触：向下速度被法向弹簧-阻尼吸收，不穿地 ----
    {
        State s;
        s.pos_ned.z = 0.01f;
        s.vel_ned.z = 2.0f;
        Command c;
        c.wf_cmd = wh;
        c.wt_cmd = wh;
        for (int i = 0; i < 2000; ++i)
        {
            step(p, s, c, 0.001f, 10);
        }
        expect(std::isfinite(s.pos_ned.z), "ground pos finite");
        expect(std::fabs(s.vel_ned.z) < 0.2f, "ground absorbs vz");
        expect(s.pos_ned.z < 0.05f, "ground no deep penetration");
    }

    // ---- 地面摩擦：带法向载荷时水平速度衰减 ----
    {
        State s;
        s.pos_ned.z = 0.0f;
        s.vel_ned.x = 1.0f;
        Command c;
        c.wf_cmd = wh;
        c.wt_cmd = wh;
        step(p, s, c, 0.001f, 10);
        const float vx0 = s.vel_ned.x;
        for (int i = 0; i < 500; ++i)
        {
            step(p, s, c, 0.001f, 10);
        }
        expect(std::fabs(s.vel_ned.x) < std::fabs(vx0),
               "ground friction reduces horizontal speed");
    }

    // ---- 舵机速率限幅 ----
    {
        Params ps = p;
        ps.enable_ground = false;
        ps.enable_gyro = false;
        State s;
        Command c;
        c.servo_f_cmd = 1.0f;
        step(ps, s, c, 0.01f, 10);
        expect(s.servo_f <= 300.0f * 0.017453292f * 0.01f + 1.0e-6f,
               "servo rate limited");
        expect(s.servo_f > 0.0f, "servo moves");
    }

    // ---- dt<=0 不改状态 ----
    {
        State s;
        s.pos_ned.z = -1.0f;
        const float z0 = s.pos_ned.z;
        step(p, s, Command{}, 0.0f, 10);
        expectNear(s.pos_ned.z, z0, 0.0f, "dt=0 no-op");
    }

    std::printf("coaxial_dynamics_model: %d checks, %d failed\n", g_checks,
                g_fail);
    return (g_fail == 0) ? 0 : 1;
}
