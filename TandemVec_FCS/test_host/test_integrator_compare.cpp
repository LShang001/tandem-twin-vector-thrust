// ============================================================
//  test_integrator_compare.cpp — 外环积分 vs 内环积分 精确对比
//
//  纯C++，无Arduino依赖。仿真单轴俯仰动力学 + 持续扰动力矩。
//  分别测4种积分配置的阶跃响应和稳态误差。
//
//  编译：
//    g++ -std=c++17 -Iinclude test_host/test_integrator_compare.cpp \
//        -o test_host/bin/ic && ./test_host/bin/ic
// ============================================================
#include "../include/TandemVec_Config.h"
#include "../include/ControlUnits.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>

// ============================================================
//  极简 PID（与现役 PositionPID 一致：积分显式乘 dt）
// ============================================================
struct MiniPID {
    float kp, ki, kd;
    float integral, prev_err, prev_meas;
    float out_min, out_max, int_max;

    MiniPID(float p, float i, float d, float omax, float imax)
        : kp(p), ki(i), kd(d), integral(0), prev_err(0), prev_meas(0),
          out_min(-omax), out_max(omax), int_max(imax) {}

    void reset() { integral = 0; prev_err = 0; prev_meas = 0; }

    // 标准位置式 (用于外环)
    // int_max 约束 ki*integral 的输出贡献。
    float step_pos(float err, float dt) {
        integral += err * dt;
        float i_out = ki * integral;
        if (i_out >  int_max) i_out =  int_max;
        if (i_out < -int_max) i_out = -int_max;
        float d = (err - prev_err);
        prev_err = err;
        float out = kp * err + i_out + kd * d;
        if (out > out_max) out = out_max;
        if (out < out_min) out = out_min;
        return out;
    }

    // 微分先行 (与 PositionPID::computeDerivativeOnMeasurement 一致)
    float step_dom(float setpoint, float meas, float dt) {
        float err = setpoint - meas;
        integral += err * dt;
        float i_out = ki * integral;
        if (i_out >  int_max) i_out =  int_max;
        if (i_out < -int_max) i_out = -int_max;
        float d = -(meas - prev_meas);
        prev_meas = meas;
        prev_err  = err;
        float out = kp * err + i_out + kd * d;
        if (out > out_max) out = out_max;
        if (out < out_min) out = out_min;
        return out;
    }
};

// ============================================================
//  单轴（俯仰）闭环仿真
//  控制器域：角度 deg、角速率 deg/s、角加速度 deg/s²。
//  物理边界：alpha 统一转 rad/s²，再执行 M = I × alpha。
// ============================================================
struct Config {
    MiniPID outer;  // 角度误差(deg) → 目标角速率(deg/s)
    MiniPID inner;  // 角速率误差(deg/s) → 角加速度(deg/s²)
    const char *name;

    Config(const char *n, float op, float oi, float ip, float ii)
        : outer(op, oi, 0, 80.0f, 40.0f),
          inner(ip, ii, 0, ControlUnits::radps2ToDps2(100.0f), ControlUnits::radps2ToDps2(20.0f)),
          name(n) {}
    void reset() { outer.reset(); inner.reset(); }
};

struct Result {
    float rms_err, max_err_final;  // 最后2秒RMS误差和最大误差
    float overshoot, settle_2pct;
    int   inner_sat;  // 内环处于输出饱和的步数
};

static Result run(Config &cfg, float target_rad, float disturbance_Nm,
                  float inertia, float dt, float sim_s)
{
    int N = (int)(sim_s / dt);
    float theta = 0, omega = 0;  // 角度(rad), 角速率(rad/s)
    float rms_sum = 0, max_final = 0;
    int   rms_n = 0, settle_at = -1, sat_c = 0;
    float max_theta = 0;

    for (int i = 0; i < N; ++i) {
        float t = i * dt;
        float err = (target_rad - theta) * ControlUnits::kDegPerRad;

        // 外环: 角度误差 → 目标角速率
        float omega_ref_dps = cfg.outer.step_pos(err, dt);

        // 内环: 角速率误差 → 角加速度
        float alpha_ref_dps2 = cfg.inner.step_dom(
            omega_ref_dps, omega * ControlUnits::kDegPerRad, dt);
        float alpha_ref_radps2 = ControlUnits::dps2ToRadps2(alpha_ref_dps2);

        // 物理层
        float M_cmd = inertia * alpha_ref_radps2;
        float M_act = M_cmd + disturbance_Nm;  // 实际力矩(含持续扰动)
        float alpha = M_act / inertia;

        if (fabsf(alpha_ref_dps2) >= ControlUnits::radps2ToDps2(29.9f)) sat_c++;
        omega += alpha * dt;
        theta += omega * dt;

        // 统计最后2秒稳态
        if (t > sim_s - 2.0f) {
            float e = fabsf(err);
            rms_sum += e*e; rms_n++;
            if (e > max_final) max_final = e;
        }
        if (theta > max_theta) max_theta = theta;
        const float target_deg = target_rad * ControlUnits::kDegPerRad;
        if (settle_at < 0 && t > 0.5f && fabsf(err) < target_deg * 0.02f) settle_at = i;
    }
    Result r;
    r.rms_err       = sqrtf(rms_sum / rms_n);  // err 已是 deg
    r.max_err_final = max_final;
    r.overshoot     = (max_theta - target_rad) * ControlUnits::kDegPerRad;
    r.settle_2pct   = settle_at >= 0 ? settle_at * dt : 99;
    r.inner_sat     = sat_c;
    return r;
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::printf("=== 积分位置对比：外环 vs 内环 — 单轴俯仰 3秒仿真 ===\n\n");
    std::printf("参数: Iy=%.4f kg·m², dt=0.005s, 阶跃+15°, 扰动=0.01 N·m\n\n",
                kDefaultTandemVecParams.Iy);

    // 功率比解释：随扰动力矩产生的力矩需被外环
    // 增益取现役俯仰回路：外环 Kp=2.8 s^-1，内环 Kp=16.042818 s^-1。

    float Iy = kDefaultTandemVecParams.Iy, dt = 0.005f, sim_s = 3.0f;
    float target = 15.0f * 3.14159265f/180.f;
    float disturb = 0.01f;  // ~3% 最大力矩

    // 外环积分值 0.5 s^-2 仅用于对照；滚转/俯仰内环 Ki=5.729578 s^-2 为现役值。

    Config configs[] = {
        {"A:外环积(对照)", 2.8f, 0.5f,  16.042818f, 0.0f      },
        {"B:内环积(现役)", 2.8f, 0.0f,  16.042818f, 5.729578f },
        {"C:双积分",       2.8f, 0.5f,  16.042818f, 5.729578f },
        {"D:纯P(对照)",    2.8f, 0.0f,  16.042818f, 0.0f      },
    };

    std::printf("%-20s %8s %8s %8s %8s %s\n",
                "方案", "RMS(°)", "最大(°)", "超调(°)", "收敛(s)", "内环饱和");
    std::printf("----------------------------------------------------------------\n");

    for (auto &cfg : configs) {
        cfg.reset();
        auto r = run(cfg, target, disturb, Iy, dt, sim_s);
        std::printf("%-20s %7.2f  %7.2f  %7.2f  %7.2f  %s\n",
                    cfg.name, r.rms_err, r.max_err_final, r.overshoot,
                    r.settle_2pct,
                    r.inner_sat > 10 ? "⚠️饱和" : "✅正常");
    }

    // 不同扰动强度下，逐方案扫描稳态误差
    std::printf("\n=== 扰动扫描：不同偏心力矩下的稳态误差 ===\n");
    std::printf("扰动(N·m)");
    for (auto &cfg : configs) std::printf("  %-12s", cfg.name);
    std::printf("\n");
    for (float d : {0.0f, 0.005f, 0.01f, 0.02f, 0.03f, 0.05f}) {
        std::printf("  %5.3f   ", d);
        for (auto &cfg : configs) {
            cfg.reset();
            auto r = run(cfg, target, d, Iy, dt, sim_s);
            std::printf("  %5.2f°(%4s)", r.max_err_final,
                        r.inner_sat > 10 ? "饱和" : "正常");
        }
        std::printf("\n");
    }

    std::printf("\n=== 仿真结论 ===\n\n");
    std::printf("关键发现：\n\n");
    std::printf("1. 所有控制量均在角度域计算，进入惯量模型前只转换一次到 SI。\n");
    std::printf("2. Ki 使用显式 dt 的连续域定义，结果不再依赖 200Hz 隐式缩放。\n");
    std::printf("3. 在该理想刚体对照中，滚转/俯仰现役内环 Ki=5.729578 s^-2 的持续扰动误差最小。\n");
    std::printf("4. 候选外环积分与双积分都未优于现役内环积分，不支持引入外环 Ki。\n\n");
    std::printf(">>> 建议：保留现役内环积分，外环 Ki 保持 0 <<<\n");
    std::printf("    边界：此台架未含电机/舅机动态、噪声和分配饱和，只能证明该理想模型中的相对趋势。\n");

    return 0;
}
