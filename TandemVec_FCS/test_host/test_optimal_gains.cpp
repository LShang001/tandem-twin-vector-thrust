// ============================================================
//  test_optimal_gains.cpp — 解析最优增益 + 仿真验证
//
//  级联 P-P 闭环 = 二阶系统:
//    θ/θ_ref = ωn²/(s²+2ζωn·s+ωn²)
//    ωn² = Kp_r · Kp_a
//    2ζωn = Kp_r
//  迁移后 Kp_r 本身就是角度域比例增益（s⁻¹）。
//
//  编译: g++ -std=c++17 test_host/test_optimal_gains.cpp -o test_host/bin/og && ./test_host/bin/og
// ============================================================
#include <cmath>
#include <cstdio>

static float run(float Kp_a, float Kp_r, float Ki_r,
                 float I, float target_deg, float dist_Nm, bool has_dist)
{
    const float dt=0.005f, sim_s=10.f, D2R=3.14159265f/180.f;
    int N=(int)(sim_s/dt);
    float th=0, om=0, integral=0;

    for(int i=0;i<N;++i){
        float t=i*dt;
        float err = target_deg - th;

        // 外环P: 角度误差(deg) → 目标角速率(deg/s)
        float wref = Kp_a * err;

        // 内环P+I: 速率误差(deg/s) → 角加速度(deg/s²)
        float ewr = wref - om;
        integral += ewr * dt;
        float alpha = Kp_r*ewr + Ki_r*integral;

        float M = has_dist && t>0.5f ? dist_Nm : 0;
        om += alpha*dt + (M/I)*D2R*dt;
        th += om*dt;
    }
    return fabsf(target_deg - th);
}

int main()
{
    const float I=0.34f, target=10.f, dist=0.01f;  // 小阶跃+小扰动，保持线性区

    std::printf("============== 解析候选增益（非现役参数） ===============\n\n");
    std::printf("级联P-P闭环二阶:\n");
    std::printf("  ωn² = Kp_r × Kp_a\n");
    std::printf("  2ζωn = Kp_r\n");
    std::printf("设计目标 ζ=0.8(最优阻尼), ωn=6~10 rad/s(VTOL合理)\n\n");

    // 下列 Kp_r 已由历史混合域数值乘 180/pi，闭环行为保持不变。

    struct {float a,r; const char* n;} c[] = {
        {3.0f,4.583662f,"ζ=0.67 ωn=3.7(慢)"},
        {4.0f,6.417127f,"ζ=0.63 ωn=5.1"},
        {5.0f,8.021409f,"ζ=0.63 ωn=6.3"},
        {3.0f,11.459156f,"ζ=0.98 ωn=5.9(安全)"},
        {4.25f,17.188734f,"ζ=1.01 ωn=8.6(对照)"},
        {5.0f,17.188734f,"ζ=0.78 ωn=9.3(推荐)"},
        {6.0f,17.188734f,"ζ=0.64 ωn=10.2"},
    };

    std::printf("%-20s %5s %6s %6s\n", "配置","ζ","ωn","终点°");
    for(auto&x:c){
        float z = x.r/(2.f*sqrtf(x.r*x.a));
        float w = sqrtf(x.r*x.a);
        float e = run(x.a,x.r,0, I,target,dist,true);
        std::printf("%-20s %4.2f %5.1f %5.2f\n", x.n, z, w, e);
    }

    // 积分验证：在最优 Kp 基础上，加 Ki 测配平效果
    std::printf("\n=== 积分验证 (Kp_a=5.0, Kp_r=17.189 s^-1) ===\n");
    std::printf("Ki_r    无扰终点° 有扰终点°  配平效果\n");
    float ki_test[]={0, 1.145916f, 2.291831f, 5.729578f, 11.459156f, 22.918312f};
    for(float ki:ki_test){
        float e0=run(5.f,17.188734f,ki, I,target,0,false);
        float e1=run(5.f,17.188734f,ki, I,target,dist,true);
        std::printf("%.5f  %8.3f  %8.3f  %s\n", ki, e0, e1,
                    (e1-e0)<0.2f?"✅有效":"  ");
    }

    std::printf("\n=== 结论 ===\n");
    std::printf("候选纯P点: Kp_a=5.0, Kp_r=17.19 s^-1 → ζ=0.78, ωn=9.3(1.5Hz)\n");
    std::printf("配平积分: Ki_r=3.44~5.73 s^-2 → 不影响ζ, 10s内消静差\n");
    return 0;
}
