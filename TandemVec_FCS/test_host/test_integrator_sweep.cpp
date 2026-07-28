// ============================================================
//  test_integrator_sweep.cpp — Ki参数扫描 + 最优配比
//
//  公平比较两种架构：各自扫Ki找最优，然后最优vs最优
//  评估指标：综合分数 = RMS×0.4 + 收敛×0.3 + (100-超调惩罚)×0.3
//
//  编译：
//    g++ -std=c++17 -Iinclude test_host/test_integrator_sweep.cpp \
//        -o test_host/bin/sw && ./test_host/bin/sw
// ============================================================
#include "../include/TandemVec_Config.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>

// ============================================================
//  极简 PID（与 test_integrator_compare.cpp 完全一致的实现）
// ============================================================
struct MiniPID {
    float kp, ki, kd;
    float integral, prev_err, prev_meas;
    float out_max, int_max;  // int_max约束的是ki*integral（与PositionPID一致）

    MiniPID(float p=0, float i=0, float d=0, float om=1e9f, float im=1e9f)
        : kp(p), ki(i), kd(d), integral(0), prev_err(0), prev_meas(0),
          out_max(om), int_max(im) {}
    void reset() { integral=prev_err=prev_meas=0; }

    float step_pos(float err, float) {
        integral += err;
        float io = ki*integral; if(io>int_max)io=int_max; if(io<-int_max)io=-int_max;
        float d = err-prev_err; prev_err=err;
        float out = kp*err + io + kd*d;
        if(out>out_max)out=out_max; if(out<-out_max)out=-out_max;
        return out;
    }
    float step_dom(float sp, float meas, float) {
        float err = sp-meas;
        integral += err;
        float io = ki*integral; if(io>int_max)io=int_max; if(io<-int_max)io=-int_max;
        float d = -(meas-prev_meas); prev_meas=meas; prev_err=err;
        float out = kp*err + io + kd*d;
        if(out>out_max)out=out_max; if(out<-out_max)out=-out_max;
        return out;
    }
};

// ============================================================
//  单次仿真
// ============================================================
struct Trace {
    float rms_err_deg, max_ss_err_deg;   // 最后25%时间的稳态指标
    float overshoot_deg, settle_2pct_s;
    bool  unstable;  // 误差从未<目标50% → 发散
};

static Trace simulate(float ki_outer, float ki_inner,
                      float target_rad, float disturb_Nm,
                      float I, float dt, float sim_s)
{
    MiniPID outer(4.25f, ki_outer, 0, 2.0f, 0.7f);   // ±2 rad/s, I_limit=0.7
    MiniPID inner(0.30f, ki_inner, 0, 30.0f, 5.0f);   // ±30 rad/s², I_limit=5

    int N=(int)(sim_s/dt);
    float theta=0, omega=0;
    float max_theta=0;
    int settle_at=-1, rms_n=0;
    float rms_sum=0, max_ss=0;

    for(int i=0;i<N;++i){
        float t=i*dt, err=target_rad-theta;
        float wref = outer.step_pos(err, dt);
        float aref = inner.step_dom(wref, omega, dt);
        float Mcmd = I*aref;
        omega += (Mcmd+disturb_Nm)/I * dt;
        theta += omega*dt;
        if(theta>max_theta)max_theta=theta;

        // 最后25%稳态
        if(t > sim_s*0.75f){
            float ae=fabsf(err);
            rms_sum+=ae*ae; rms_n++;
            if(ae>max_ss)max_ss=ae;
        }
        if(settle_at<0 && t>0.3f && fabsf(err)<target_rad*0.02f)settle_at=i;
    }

    float final_e = fabsf(target_rad-theta);
    float D2=57.29578f;
    Trace tr;
    tr.rms_err_deg     = rms_n>0? sqrtf(rms_sum/rms_n)*D2 : 99;
    tr.max_ss_err_deg  = max_ss*D2;
    tr.overshoot_deg   = (max_theta-target_rad)*D2;
    tr.settle_2pct_s   = settle_at>=0? settle_at*dt : 99;
    tr.unstable        = final_e > target_rad*0.5f;
    return tr;
}

// 综合分数：RMS 40% + 收敛速度 30% + 超调惩罚 30%（越小越好）
static float score(const Trace &t, float best_rms, float best_settle) {
    if(t.unstable) return 999;
    float s_rms    = (best_rms>0.1f)   ? t.rms_err_deg/best_rms : 1;
    float s_settle = (best_settle>0.01f)? t.settle_2pct_s/best_settle : 1;
    float s_over   = (t.overshoot_deg<0)?0 : t.overshoot_deg/5.0f; // 每5°超调=1分
    return 0.4f*s_rms + 0.3f*s_settle + 0.3f*s_over;
}

// ============================================================
//  main — 二维扫描
// ============================================================
int main()
{
    const float Iy=0.34f, dt=0.005f, sim_s=4.0f;
    const float target = 15.f*3.14159265f/180.f;
    const float disturb=0.01f;

    // 先跑一遍纯P获取baseline
    Trace p_only = simulate(0,0, target,disturb, Iy,dt,sim_s);
    float best_rms=p_only.rms_err_deg, best_settle=p_only.settle_2pct_s;

    std::printf("=== Ki 参数扫描：外环 vs 内环 vs 双积分 ===\n");
    std::printf("纯P baseline: RMS=%.2f° settle=%.2fs overshoot=%.2f°\n\n",
                p_only.rms_err_deg, p_only.settle_2pct_s, p_only.overshoot_deg);

    // 预扫描更新 best_rms / best_settle 归一化基准
    // 外环Ki扫描范围
    float ki_outer_vals[] = {0, 0.001f, 0.002f, 0.005f, 0.008f,  0.010f, 0.015f, 0.020f, 0.030f};
    // 内环Ki扫描范围
    float ki_inner_vals[] = {0, 0.00005f, 0.0001f, 0.0002f, 0.0005f, 0.001f, 0.002f, 0.005f};

    // Round 1: 纯外环积分扫描（内环Ki=0）
    std::printf("──────── 外环积分扫描 (内环Ki=0) ────────\n");
    std::printf("%8s  %8s  %8s  %8s  %8s\n","Ki_out","RMS(°)","settle","超调(°)","分数");
    float best_out_ki=0, best_out_score=999;
    for(float ki : ki_outer_vals){
        auto t = simulate(ki,0, target,disturb, Iy,dt,sim_s);
        float s = score(t, best_rms, best_settle);
        std::printf("%8.4f  %7.2f  %7.2f  %7.2f  %7.3f %s\n",
                    ki, t.rms_err_deg, t.settle_2pct_s, t.overshoot_deg, s,
                    t.unstable?"发散":"");
        if(!t.unstable && s<best_out_score){best_out_score=s; best_out_ki=ki;}
    }

    // Round 2: 纯内环积分扫描（外环Ki=0）
    std::printf("\n──────── 内环积分扫描 (外环Ki=0) ────────\n");
    std::printf("%8s  %8s  %8s  %8s  %8s\n","Ki_in","RMS(°)","settle","超调(°)","分数");
    float best_in_ki=0, best_in_score=999;
    for(float ki : ki_inner_vals){
        auto t = simulate(0,ki, target,disturb, Iy,dt,sim_s);
        float s = score(t, best_rms, best_settle);
        std::printf("%8.5f  %7.2f  %7.2f  %7.2f  %7.3f %s\n",
                    ki, t.rms_err_deg, t.settle_2pct_s, t.overshoot_deg, s,
                    t.unstable?"发散":"");
        if(!t.unstable && s<best_in_score){best_in_score=s; best_in_ki=ki;}
    }

    // Round 3: 双积分二维扫描（展示关键组合）
    std::printf("\n──────── 双积分关键组合 ────────\n");
    std::printf("%8s %8s  %8s  %8s  %8s  %8s\n","Ki_out","Ki_in","RMS(°)","settle","超调(°)","分数");
    float best_both_score=999, best_both_ko=0, best_both_ki=0;
    float test_ko[] = {0.001f, 0.002f, 0.005f};
    float test_ki[] = {0.0001f, 0.0002f, 0.0005f};
    for(float ko:test_ko) for(float ki:test_ki){
        auto t = simulate(ko,ki, target,disturb, Iy,dt,sim_s);
        float s = score(t, best_rms, best_settle);
        std::printf("%8.4f %8.5f  %7.2f  %7.2f  %7.2f  %7.3f\n",
                    ko, ki, t.rms_err_deg, t.settle_2pct_s, t.overshoot_deg, s);
        if(!t.unstable && s<best_both_score){best_both_score=s; best_both_ko=ko; best_both_ki=ki;}
    }

    // ================================================================
    // 裁决
    // ================================================================
    std::printf("\n═══════════════════════════════════════\n");
    std::printf("           最优方案对比\n");
    std::printf("═══════════════════════════════════════\n\n");

    std::printf("纯P baseline:           分数=1.000 (RMS=%.2f° settle=%.2fs)\n",
                p_only.rms_err_deg, p_only.settle_2pct_s);

    auto best_out = simulate(best_out_ki,0, target,disturb, Iy,dt,sim_s);
    std::printf("外环最优 Ki_out=%.4f:  分数=%.3f (RMS=%.2f° settle=%.2fs 超调=%.2f°)\n",
                best_out_ki, best_out_score,
                best_out.rms_err_deg, best_out.settle_2pct_s, best_out.overshoot_deg);

    auto best_in = simulate(0,best_in_ki, target,disturb, Iy,dt,sim_s);
    std::printf("内环最优 Ki_in=%.5f: 分数=%.3f (RMS=%.2f° settle=%.2fs 超调=%.2f°)\n",
                best_in_ki, best_in_score,
                best_in.rms_err_deg, best_in.settle_2pct_s, best_in.overshoot_deg);

    auto best_both = simulate(best_both_ko,best_both_ki, target,disturb, Iy,dt,sim_s);
    std::printf("双积分最优 (%.4f,%.5f): 分数=%.3f (RMS=%.2f° settle=%.2fs 超调=%.2f°)\n",
                best_both_ko, best_both_ki, best_both_score,
                best_both.rms_err_deg, best_both.settle_2pct_s, best_both.overshoot_deg);

    std::printf("\n>>> 裁决: ");
    if(best_in_score <= best_out_score && best_in_score <= best_both_score)
        std::printf("内环积分(Ki=%.5f)最优 — 微量积分不干扰瞬态+自动配平\n", best_in_ki);
    else if(best_out_score <= best_in_score && best_out_score <= best_both_score)
        std::printf("外环积分(Ki=%.4f)最优\n", best_out_ki);
    else
        std::printf("双积分(%.4f,%.5f)最优\n", best_both_ko, best_both_ki);

    return 0;
}
