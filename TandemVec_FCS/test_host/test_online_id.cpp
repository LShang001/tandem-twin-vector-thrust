// ============================================================
//  test_online_id.cpp — 在线参数辨识收敛性验证
//
//  场景: 真实惯量 = 名义值 ×0.7 (辨识目标: b=1/0.7=1.43)
//        飞行器做一系列15°阶跃机动提供激励
//        观察RLS的 θ_b 是否收敛到 1.43
//  编译: g++ -std=c++17 -Iinclude test_host/test_online_id.cpp -o test_host/bin/oi && ./test_host/bin/oi
// ============================================================
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#include "../include/TandemVec_OnlineID.h"
#include "../include/TandemVec_Config.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

// ---- 断言框架 ----
static int g_fail = 0;
static void check(bool cond, const std::string& name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail;
}

// 用给定的真实惯量比跑一次辨识，返回收敛后的估计
struct IdResult { float b_est, d_est, p11; };

static IdResult run_identification(float b_true, float noise_amp, int n_steps = 8)
{
    const float dt = 0.005f;
    RLS_2Param rls;
    float omega = 0, theta = 0;
    std::srand(12345);   // 固定种子，保证可复现

    for (int n = 0; n < n_steps; n++) {
        float target = (n % 2 == 0) ? 15.f : -15.f;
        for (int k = 0; k < 150; k++) {
            float err  = target - theta;
            float wref = 5.0f * constrain(err, -10.f, 10.f);
            float alpha_cmd = 0.30f * (wref - omega * 57.3f);
            float wdot = b_true * alpha_cmd
                       + ((float)std::rand()/RAND_MAX - 0.5f) * noise_amp;
            omega += wdot * dt;
            theta += omega * dt * 57.3f;
            rls.update(wdot, alpha_cmd);
        }
    }
    return { rls.theta_b, rls.theta_d, rls.p11 };
}

int main()
{
    const float I_true = 0.022f * 1.5f;  // 真实惯量 = 名义×1.5 → b=1/1.5=0.667
    const float b_true = 1.f / 1.5f;     // 0.667
    const float dt     = 0.005f;

    RLS_2Param rls;
    float omega = 0, theta = 0;
    int   step_counter = 0;
    std::srand(12345);

    printf("=== 在线参数辨识验证 ===\n");
    printf("真实 I=%.4f (名义×1.5), b_true=%.3f\n", I_true, b_true);
    printf("场景: 交替15°阶跃机动 → 提供激励 → RLS收敛\n\n");

    // 模拟: 交替 +/-15° 阶跃，内环P控制器
    for (int n = 0; n < 8; n++) {
        float target = (n % 2 == 0) ? 15.f : -15.f;
        for (int k = 0; k < 150; k++) {
            float err = target - theta;
            float wref = 5.0f * constrain(err, -10.f, 10.f);  // 外环P
            float alpha_cmd = 0.30f * (wref - omega * 57.3f); // 内环P (deg/s→rad/s²)
            // 真实物理: ω̇ = b_true × α_cmd + 噪声
            float wdot = b_true * alpha_cmd + ((float)rand()/RAND_MAX-0.5f)*0.5f;
            omega += wdot * dt;
            theta += omega * dt * 57.3f;  // rad/s → deg

            // RLS更新
            rls.update(wdot, alpha_cmd);
            step_counter++;

            if (step_counter % 150 == 0) {
                printf("  t=%.1fs b_est=%.3f (true=%.3f) err=%.1f%% P11=%.3f\n",
                       step_counter*dt, rls.theta_b, b_true,
                       fabsf(rls.theta_b-b_true)/b_true*100, rls.p11);
            }
        }
    }

    printf("\n最终: b_est=%.4f true=%.4f 误差=%.1f%%\n",
           rls.theta_b, b_true, fabsf(rls.theta_b-b_true)/b_true*100);
    printf("协方差 P11=%.4f (→0: 估计收敛)\n", rls.p11);
    printf("扰动项 d_est=%.3f (应≈0)\n\n", rls.theta_d);

    printf("=== 自适应增益调度演示 ===\n");
    float Kp_nominal = 0.30f;
    float Kp_adapted = Kp_nominal / sqrtf(fmaxf(rls.theta_b, 0.3f));
    printf("b=%.3f → Kp_r: %.3f → %.3f (增益补偿%.0f%%)\n",
           rls.theta_b, Kp_nominal, Kp_adapted,
           (Kp_adapted/Kp_nominal-1.f)*100);

    // ============================================================
    //  断言部分
    // ============================================================
    printf("\n=== 断言验证 ===\n");

    // A1: 主场景收敛精度（b_true=0.667）
    check(fabsf(rls.theta_b - b_true) / b_true < 0.05f,
          "A1 惯量比辨识误差 < 5%");
    check(rls.p11 < 0.01f,
          "A1 协方差 P11 < 0.01（估计已收敛）");
    check(fabsf(rls.theta_d) < 0.1f,
          "A1 扰动项 d_est ≈ 0（无偏估计）");

    // A2: 不同惯量比下的一致性（惯量偏小 → b > 1）
    {
        IdResult r = run_identification(1.f / 0.7f, 0.5f);   // I_true=名义×0.7 → b=1.43
        printf("  b_true=1.429 → b_est=%.3f (误差%.1f%%)\n",
               r.b_est, fabsf(r.b_est - 1.4286f) / 1.4286f * 100);
        check(fabsf(r.b_est - 1.4286f) / 1.4286f < 0.05f,
              "A2 惯量偏小(b=1.43)：辨识误差 < 5%");
    }

    // A3: 名义值场景（真实=名义 → b=1）
    {
        IdResult r = run_identification(1.0f, 0.5f);
        printf("  b_true=1.000 → b_est=%.3f (误差%.1f%%)\n",
               r.b_est, fabsf(r.b_est - 1.0f) * 100);
        check(fabsf(r.b_est - 1.0f) < 0.05f,
              "A3 名义惯量(b=1.0)：辨识误差 < 5%");
    }

    // A4: 抗噪声能力（噪声放大 4 倍）
    {
        IdResult r = run_identification(b_true, 2.0f);
        printf("  高噪声(×4) → b_est=%.3f (误差%.1f%%)\n",
               r.b_est, fabsf(r.b_est - b_true) / b_true * 100);
        check(fabsf(r.b_est - b_true) / b_true < 0.20f,
              "A4 高噪声下辨识误差 < 20%（鲁棒性）");
    }

    // A5: 无激励时不应污染估计（零输入 → 参数保持初值）
    {
        RLS_2Param idle;
        for (int i = 0; i < 500; ++i) idle.update(0.f, 0.f);
        check(std::isfinite(idle.theta_b) && std::isfinite(idle.theta_d),
              "A5 零激励下参数保持有限（无发散）");
        check(fabsf(idle.theta_b - 1.0f) < 0.2f,
              "A5 零激励下 b_est 停留在初值附近（不漂移）");
    }

    // A6: reset() 完整清零
    {
        RLS_2Param r;
        for (int i = 0; i < 100; ++i) r.update(0.5f, 1.0f);
        bool changed = (fabsf(r.theta_b - 1.0f) > 1e-4f);
        r.reset();
        check(changed, "A6 reset前参数已变化（前提成立）");
        check(r.theta_b == 1.0f && r.theta_d == 0.0f && r.p11 == 100.0f,
              "A6 reset() 后参数与协方差完全归零");
    }

    // A7: 自适应增益方向正确性
    //     b<1（真实惯量大于名义）→ 同样 α_cmd 产生的实际角加速度偏小
    //     → 需要提高 Kp 补偿 → Kp_adapted > Kp_nominal
    {
        OnlineID id;
        id.b_est[0] = 0.667f;   // 真实惯量偏大
        float kp_up = id.adaptKpR(0.30f, 0);
        id.b_est[0] = 1.429f;   // 真实惯量偏小
        float kp_dn = id.adaptKpR(0.30f, 0);
        printf("  b=0.667 → Kp=%.4f ; b=1.429 → Kp=%.4f\n", kp_up, kp_dn);
        check(kp_up > 0.30f, "A7 惯量偏大(b<1) → Kp 上调");
        check(kp_dn < 0.30f, "A7 惯量偏小(b>1) → Kp 下调");
    }

    // A8: 自适应约束边界（b 被硬约束在 [0.3, 3.0]）
    {
        OnlineID id;
        id.b_est[1] = 100.0f;   // 病态估计
        float kp = id.adaptKpR(0.30f, 1);
        id.b_est[1] = -5.0f;    // 非物理负值
        float kp2 = id.adaptKpR(0.30f, 1);
        check(std::isfinite(kp) && kp > 0.f,  "A8 b过大时 Kp 仍有限且为正");
        check(std::isfinite(kp2) && kp2 > 0.f, "A8 b为负时 Kp 仍有限且为正（约束生效）");
    }

    // ============================================================
    //  A9~A13: OnlineID::step() 完整管道（单位/门控/限速/首拍）
    // ============================================================
    printf("\n=== OnlineID::step() 管道验证 ===\n");

    // 用 step() 接口跑闭环辨识：输入 deg/s，内部应自行转 rad/s
    // 物理: ω̇ = b_true × α_cmd，α_cmd 用足够激励的方波
    auto run_pipeline = [](float b_true, float thr_pct, int n_sec,
                           OnlineID& id) -> float
    {
        const float dt = 0.005f;
        const float RAD2DEG = 57.29578f;
        float w_rps = 0.f;
        int   N = (int)(n_sec / dt);
        for (int k = 0; k < N; ++k) {
            // 方波 α_cmd = ±5 rad/s²，保证 alpha_var > 4
            float a = ((k / 100) % 2 == 0) ? 5.f : -5.f;
            float alpha_cmd[3] = { 0.f, a, 0.f };
            w_rps += (b_true * a) * dt;
            float omega_dps[3] = { 0.f, w_rps * RAD2DEG, 0.f };
            float i_term[3]    = { 0.f, 0.f, 0.f };
            id.step(alpha_cmd, omega_dps, i_term, thr_pct, dt);
        }
        return id.b_est[1];   // pitch 轴
    };

    // A9: 传 deg/s 应得到正确 b（验证内部单位转换）
    {
        OnlineID id;
        float b = run_pipeline(0.667f, 50.f, 8, id);
        printf("  step() 输入deg/s: b_est=%.3f (true=0.667)\n", b);
        check(fabsf(b - 0.667f) / 0.667f < 0.10f,
              "A9 step() 正确处理 deg/s 输入（误差<10%）");
    }

    // A10: 零激励时不更新（悬停无机动 → b 应保持初值 1.0）
    {
        OnlineID id;
        const float dt = 0.005f;
        for (int k = 0; k < 2000; ++k) {
            float alpha_cmd[3] = {0.f, 0.f, 0.f};
            float omega_dps[3] = {0.f, 0.f, 0.f};
            float i_term[3]    = {0.f, 0.f, 0.f};
            id.step(alpha_cmd, omega_dps, i_term, 50.f, dt);
        }
        printf("  零激励 10s: b_est=%.4f (应保持1.0)\n", id.b_est[1]);
        check(fabsf(id.b_est[1] - 1.0f) < 1e-5f,
              "A10 激励不足时冻结更新（b 保持名义值）");
    }

    // A11: 变化率限制生效 —— 单拍不应跳变超过 update_rate_limit
    {
        OnlineID id;
        const float dt = 0.005f;
        float b_prev = id.b_est[1];
        float max_jump = 0.f;
        float w_rps = 0.f;
        for (int k = 0; k < 1200; ++k) {
            float a = ((k / 50) % 2 == 0) ? 8.f : -8.f;
            float alpha_cmd[3] = {0.f, a, 0.f};
            w_rps += (0.4f * a) * dt;      // b_true=0.4，与初值1.0差距大
            float omega_dps[3] = {0.f, w_rps * 57.29578f, 0.f};
            float i_term[3] = {0.f,0.f,0.f};
            id.step(alpha_cmd, omega_dps, i_term, 50.f, dt);
            float jump = fabsf(id.b_est[1] - b_prev);
            if (jump > max_jump) max_jump = jump;
            b_prev = id.b_est[1];
        }
        printf("  单拍最大跳变=%.4f (限制≈%.3f)\n", max_jump, id.update_rate_limit * 3.f);
        check(max_jump < id.update_rate_limit * 3.f,
              "A11 变化率限制生效（单拍跳变受限）");
    }

    // A12: b 始终在物理约束 [0.3, 3.0] 内
    {
        OnlineID id;
        const float dt = 0.005f;
        float w_rps = 0.f;
        for (int k = 0; k < 2000; ++k) {
            // 极端 b_true=20（非物理），估计应被夹在 3.0
            float a = ((k / 50) % 2 == 0) ? 6.f : -6.f;
            float alpha_cmd[3] = {0.f, a, 0.f};
            w_rps += (20.f * a) * dt;
            float omega_dps[3] = {0.f, w_rps * 57.29578f, 0.f};
            float i_term[3] = {0.f,0.f,0.f};
            id.step(alpha_cmd, omega_dps, i_term, 50.f, dt);
        }
        printf("  非物理b_true=20 → b_est=%.3f (应夹在3.0)\n", id.b_est[1]);
        check(id.b_est[1] >= 0.3f && id.b_est[1] <= 3.0f,
              "A12 b_est 始终在物理范围 [0.3, 3.0]");
        check(std::isfinite(id.b_est[1]), "A12 极端输入下 b_est 有限");
    }

    // A13: CG 提取的悬停门控 —— 大角速率时不应更新
    {
        OnlineID id_hover, id_maneuver;
        const float dt = 0.005f;
        float i_term[3] = {0.f, 0.2f, 0.f};   // 俯仰积分项非零
        for (int k = 0; k < 1000; ++k) {
            float alpha_cmd[3] = {0.f, 0.f, 0.f};
            float slow[3] = {0.f, 2.f, 0.f};    // 2 deg/s → 悬停
            float fast[3] = {0.f, 80.f, 0.f};   // 80 deg/s → 机动中
            id_hover.step(alpha_cmd, slow, i_term, 50.f, dt);
            id_maneuver.step(alpha_cmd, fast, i_term, 50.f, dt);
        }
        printf("  悬停 cg=%.2fmm / 机动中 cg=%.2fmm\n",
               id_hover.cg_est_mm, id_maneuver.cg_est_mm);
        check(fabsf(id_hover.cg_est_mm) > 1e-3f,
              "A13 悬停条件下提取 CG 偏移");
        check(fabsf(id_maneuver.cg_est_mm) < 1e-6f,
              "A13 机动中冻结 CG 提取（门控生效）");
        // 油门超出 30~70% 也应冻结
        OnlineID id_lowthr;
        for (int k = 0; k < 1000; ++k) {
            float alpha_cmd[3] = {0.f,0.f,0.f};
            float slow[3] = {0.f, 2.f, 0.f};
            id_lowthr.step(alpha_cmd, slow, i_term, 10.f, dt);  // 10% 油门
        }
        check(fabsf(id_lowthr.cg_est_mm) < 1e-6f,
              "A13 低油门时冻结 CG 提取");
    }

    // A14: reset() 清空全部三轴（原版只清 pitch）
    {
        OnlineID id;
        const float dt = 0.005f;
        float w[3] = {0.f,0.f,0.f};
        for (int k = 0; k < 1200; ++k) {
            float a = ((k / 50) % 2 == 0) ? 6.f : -6.f;
            float alpha_cmd[3] = {a, a, a};
            for (int j=0;j<3;++j) w[j] += (0.6f * a) * dt;
            float omega_dps[3] = {w[0]*57.29578f, w[1]*57.29578f, w[2]*57.29578f};
            float i_term[3] = {0.f,0.f,0.f};
            id.step(alpha_cmd, omega_dps, i_term, 50.f, dt);
        }
        bool all_moved = true;
        for (int j=0;j<3;++j) if (fabsf(id.b_est[j]-1.0f) < 1e-4f) all_moved=false;
        check(all_moved, "A14 reset前三轴 b_est 均已变化（前提成立）");
        id.reset();
        bool all_clear = true;
        for (int j=0;j<3;++j)
            if (id.b_est[j]!=1.0f || id.rls[j].theta_b!=1.0f || id.rls[j].p11!=100.0f)
                all_clear=false;
        check(all_clear, "A14 reset() 清空全部三轴（含 roll/yaw 协方差）");
    }

    printf("\n");
    if (g_fail == 0) printf("=== 全部通过 ===\n");
    else             printf("=== %d 项失败 ===\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
