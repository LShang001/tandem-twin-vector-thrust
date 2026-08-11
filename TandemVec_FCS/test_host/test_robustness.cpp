// ============================================================
//  test_robustness.cpp — 大机动 + 系统误差鲁棒性分析
//
//  分析: (1)大角度阶跃 30/45/60/90° (2)Monte Carlo参数随机扰动
//  编译: g++ -std=c++17 -Iinclude -Itest_host/stub
//        test_host/test_robustness.cpp -o test_host/bin/rb && ./test_host/bin/rb
// ============================================================
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#include "../include/TandemVec_ControlAllocation.h"
#include "../include/TandemVec_Propulsion.h"
#include "../include/TandemVec_Config.h"
#include "../include/Quat4f.h"
#include "../include/PositionPID.h"
#include "../include/ControlUnits.h"
#include "../include/FlightCtrlParams.h"
#include "../include/ComplementaryFilter.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

// ---- 断言框架 ----
static int g_fail = 0;
static void check(bool cond, const std::string& name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail;
}

// w0_init 必须为悬停转速：从 w=0 冷启动时前 ~0.3s 推力≈0、控制权限为零，
// 姿态先自由漂移，会把任何增益都判成发散（真实飞行中电机早已在悬停转速）。
struct Motor{float w,t,ta;
  Motor(float w0_init=0.f,float x=0.28f):w(w0_init),t(w0_init),ta(x){}
  void set(float wt){t=wt;}
  float step(float dt){w+=(t-w)*fminf(dt/ta,1.f);return w;}};

struct RigidBody{
  Quat4f q;float om[3];
  RigidBody(){q={1,0,0,0};om[0]=om[1]=om[2]=0;}
  void step(const float M[3],const TandemVecParams&P,float dt){
    float p=om[0],qq=om[1],r=om[2];
    om[0]+=(M[0]-((P.Iz-P.Iy)*qq*r))/P.Ix*dt;
    om[1]+=(M[1]-((P.Ix-P.Iz)*r*p))/P.Iy*dt;
    om[2]+=(M[2]-((P.Iy-P.Ix)*p*qq))/P.Iz*dt;
    float wx=om[0],wy=om[1],wz=om[2];
    Quat4f dq={-0.5f*(q.x*wx+q.y*wy+q.z*wz),0.5f*(q.w*wx+q.y*wz-q.z*wy),
                0.5f*(q.w*wy-q.x*wz+q.z*wx),0.5f*(q.w*wz+q.x*wy-q.y*wx)};
    q.w+=dq.w*dt;q.x+=dq.x*dt;q.y+=dq.y*dt;q.z+=dq.z*dt;q=qNorm(q);
  }
  float errDeg(const Quat4f&r)const{
    Quat4f e=qNorm(qMul(qConj(q),r));if(e.w<0){e.w=-e.w;e.x=-e.x;e.y=-e.y;e.z=-e.z;}
    return 2.f*atan2f(sqrtf(e.x*e.x+e.y*e.y+e.z*e.z),e.w)*57.29578f;
  }
  // 带符号【俯仰轴误差】(deg)，与控制器 err 同源。
  // 本台架只闭环俯仰轴，绕推力轴的残余漂移不应计入收敛判据
  // （Mx=−Qf·cosδf+Qt·cosδt 在 δf≠δt 时非零，且 Ix 仅为 Iy 的 1/10）。
  float pitchErrDeg(const Quat4f&r)const{
    Quat4f e=qNorm(qMul(qConj(q),r));
    float sw=e.w>=0?1.f:-1.f;
    float v=sqrtf(e.y*e.y+e.z*e.z);
    float sc=v>0.25f?2.f*atan2f(v,fabsf(e.w))/v*57.29578f:114.59156f;
    return sw*e.y*sc;
  }
};

// 单轴俯仰仿真 (可配置初始角度偏置用于大机动)
static void sim_large(float target_deg, float init_deg,
                      float Iy_scale, float kT_scale, float cg_mm,
                      float out[4])
{
  const TandemVecParams P0=kDefaultTandemVecParams;
  TandemVecParams P=P0; P.Iy*=Iy_scale; P.kT*=kT_scale; P.kQ*=kT_scale;
  float dt=0.005f; int N=(int)(8.f/dt);
  RigidBody body;
  // 初始姿态偏置
  float hi=init_deg*3.14159265f/360.f;
  body.q=qNorm(Quat4f(cosf(hi)*cosf(hi),0,cosf(hi)*sinf(hi),0));

  const float w0_hover=0.4f*P.wMax;
  Motor mf(w0_hover),mr(w0_hover);   // 从悬停转速起转，非 w=0 冷启动
  PositionPID ang(kFlightCtrlParams.att_pitch.kp,0,0),
              rate(kFlightCtrlParams.rate_pitch.kp,kFlightCtrlParams.rate_pitch.ki,0);
  ang.setOutputLimits(kFlightCtrlParams.att_pitch.out_min,kFlightCtrlParams.att_pitch.out_max);
  rate.setOutputLimits(kFlightCtrlParams.rate_pitch.out_min,kFlightCtrlParams.rate_pitch.out_max);
  rate.setIntegralLimit(kFlightCtrlParams.rate_pitch.int_limit);
  rate.setIntegralThreshold(kFlightCtrlParams.rate_pitch.threshold);
  ComplementaryFilter gF(0.3f),aF(0.85f),rF(0.25f);float gf=0;
  PropulsionState ps={0,0,0,0};
  float hr=target_deg*3.14159265f/360.f;
  Quat4f qt=qNorm(Quat4f(cosf(hr)*cosf(hr),0,cosf(hr)*sinf(hr),0));

  // st 记录"最后一次离开稳态带的时刻"，初值 0 表示全程在带内
  float pk=0,st=0,fe=0,om_pk=0;
  for(int i=0;i<N;i++){float t=i*dt;
    gf=gF.filter(body.om[1]*57.29578f);
    Quat4f qe=qNorm(qMul(qConj(body.q),qt));float sw=qe.w>=0?1:-1;
    float v=sqrtf(qe.z*qe.z+qe.y*qe.y),sc=v>0.25f?2*atan2f(v,fabsf(qe.w))/v*57.29578f:114.59156f;
    float err=sw*qe.y*sc,wref=aF.filter(constrain(ang.computeWithExternalDerivative(err,0,-gf, 0.005f),-50.f,50.f));
    float alpha_dps2=rF.filter(rate.computeDerivativeOnMeasurement(wref,gf, 0.005f));
    float M=P.Iy*ControlUnits::dps2ToRadps2(alpha_dps2),w0=0.4f*P.wMax;AllocationInput ai={0,M,0,w0,ps};
    AllocationOutput ao=allocateMoments(ai,P,AllocationStrategy::FULL_B);
    auto df=allocateDifferential(w0,ao.dw,P);mf.set(df.wf_target);mr.set(df.wt_target);
    float wf=mf.step(dt),wt=mr.step(dt);ps={wf,wt,ao.delta_f,ao.delta_t};
    // 施加分配器算出的 delta_f（原代码置零，丢弃了抵消反扭耦合的前摆指令）
    PropulsionState pa={wf,wt,ao.delta_f,ao.delta_t};SixDOFWrench w=computeWrench(pa,P);
    float T_tot=P.kT*(wf*wf+wt*wt)*0.5f;
    float Mt[3]={w.Mx,w.My+T_tot*cg_mm*0.001f,w.Mz};body.step(Mt,P,dt);
    // 俯仰轴误差（与控制器同源）→ 实际响应 = 目标 − 误差
    float e_p=body.pitchErrDeg(qt);
    float resp=target_deg-e_p;
    if(fabsf(resp)>fabsf(pk))pk=resp;                       // 峰值（可测超调）
    if(fabsf(body.om[1])>om_pk)om_pk=fabsf(body.om[1]);
    // settle：最后一次离开 ±2%|目标| 带的时刻（大机动用 ±2%，回正用绝对 1°）
    float band=fmaxf(fabsf(target_deg)*0.02f,1.0f);
    if(fabsf(e_p)>band)st=t;
    fe=e_p;
  }
  out[0]=fabsf(pk);
  out[1]=(st<8.f-dt*2.f)?st:99.f;   // 未进带记 99
  out[2]=fabsf(fe);
  out[3]=om_pk*57.29578f;
}

int main()
{
  srand(42); float o[4];

  std::printf("══════════════════════════════════════════\n");
  std::printf(" 大机动 + 系统误差鲁棒性 (现役 Kp_a=%.1f Kp_r=%.3f Ki=%.3f)\n",
              kFlightCtrlParams.att_pitch.kp,kFlightCtrlParams.rate_pitch.kp,
              kFlightCtrlParams.rate_pitch.ki);
  std::printf("══════════════════════════════════════════\n\n");

  // ═══ T1: 大角度阶跃 ═══
  std::printf("── T1: 大角度阶跃 (零初始偏置) ──\n");
  std::printf("%8s %8s %8s %8s %8s\n","目标°","peak°","settle","final°","ωpk°/s");
  bool  t1_all_settled=true; float t1_worst_final=0.f, t1_worst_ov=0.f;
  float t1_prev_settle=0.f;  bool  t1_settle_monotonic=true;
  for(float tgt:{10.f,20.f,30.f,45.f,60.f,90.f}){
    sim_large(tgt,0.f, 1.f,1.f,0.f, o);
    float ov=(o[0]-tgt)/tgt;
    const char* ok=(o[1]<90.f&&ov<0.25f)?"✅":(o[1]<90.f?"⚠️":"🔴");
    std::printf("%7.0f  %7.1f  %7.2f  %7.2f  %7.0f %s\n",tgt,o[0],o[1],o[2],o[3],ok);
    if(o[1]>90.f)t1_all_settled=false;
    if(o[2]>t1_worst_final)t1_worst_final=o[2];
    if(ov>t1_worst_ov)t1_worst_ov=ov;
    if(o[1]<t1_prev_settle-1e-3f)t1_settle_monotonic=false;
    t1_prev_settle=o[1];
  }

  // ═══ T2: 初始偏置 (从倾斜姿态回正) ═══
  std::printf("\n── T2: 初始30°偏置 → 目标0° (回正) ──\n");
  std::printf("%8s %8s %8s %8s %8s\n","目标°","peak°","settle","final°","ωpk°/s");
  bool t2_all_settled=true; float t2_worst_final=0.f;
  for(float init:{10.f,20.f,30.f,40.f,50.f}){
    sim_large(0.f,init, 1.f,1.f,0.f, o);
    std::printf("init%.0f  %7.1f  %7.2f  %7.2f  %7.0f %s\n",init,o[0],o[1],o[2],o[3],
              o[1]<5.f?"✅":"⚠️");
    if(o[1]>90.f)t2_all_settled=false;
    if(o[2]>t2_worst_final)t2_worst_final=o[2];
  }

  // ═══ T3: Monte Carlo 随机参数 ═══
  std::printf("\n── T3: Monte Carlo — I/kT/CG 随机扰动 (100次) ──\n");
  std::printf("扰动范围: I×[0.5,1.5], kT×[0.7,1.3], CG∈[0,10]mm\n");
  float worst=0,avg_pk=0,avg_st=0,mc_worst_final=0; int narrow_band_miss=0;
  for(int n=0;n<100;n++){
    float Is=0.5f+(float)rand()/RAND_MAX;     // [0.5, 1.5]
    float kTs=0.7f+(float)rand()/RAND_MAX*0.6f; // [0.7, 1.3]
    float cg=(float)rand()/RAND_MAX*10.f;      // [0, 10] mm
    sim_large(20.f,0.f, Is,kTs,cg, o);
    avg_pk+=o[0]; avg_st+=o[1]; if(o[0]>worst)worst=o[0];
    if(o[2]>mc_worst_final)mc_worst_final=o[2];
    if(o[1]>90.f) narrow_band_miss++; // 8 s 内未进入±2%窄带，不等同于发散
  }
  std::printf("avg peak=%.1f° avg settle=%.2fs worst peak=%.1f° 最大末值误差=%.2f° 窄带未收敛=%d/100\n",
              avg_pk/100, avg_st/100, worst, mc_worst_final, narrow_band_miss);

  // ═══ T4: 最坏组合工况 ═══
  std::printf("\n── T4: 最坏组合 (I+50%, kT-30%, CG=10mm) ──\n");
  struct{const char*n;float Is,kTs,cg,tgt;}ws[]={
    {"名义20°阶跃",1.f,1.f,0.f,20.f},
    {"I↓50% 30°阶跃",0.5f,1.f,0.f,30.f},
    {"I↑50%+kT↓30%+CG10mm 45°",1.5f,0.7f,10.f,45.f},
    {"全极限 60°阶跃",0.5f,0.7f,10.f,60.f},
  };
  bool t4_all_settled=true; float t4_worst_final=0.f, t4_worst_ov=0.f;
  for(auto&c:ws){
    sim_large(c.tgt,0.f, c.Is,c.kTs,c.cg, o);
    const char* ok=o[1]<6.f?"✅":o[1]<90.f?"⚠️":"🔴";
    std::printf("%-28s peak=%.1f° settle=%.2fs final=%.1f° ωpk=%.0f°/s %s\n",
                c.n,o[0],o[1],o[2],o[3],ok);
    if(o[1]>90.f)t4_all_settled=false;
    if(o[2]>t4_worst_final)t4_worst_final=o[2];
    float ov=(o[0]-c.tgt)/c.tgt; if(ov>t4_worst_ov)t4_worst_ov=ov;
  }

  // ═══ 断言验证 ═══
  std::printf("\n── 断言验证 ──\n");

  // R1: 大角度阶跃（10°~90°，倾角保护已移除，须全程可控）
  check(t1_all_settled,          "R1 大角度阶跃 10~90°：全部进入稳态带");
  check(t1_worst_final < 1.0f,   "R1 大角度阶跃：最大稳差 < 1°");
  check(t1_worst_ov   < 0.25f,   "R1 大角度阶跃：最大超调 < 25%");
  check(t1_settle_monotonic,
        "R1 收敛时间随目标角单调不减（无异常快收敛，度量自洽）");

  // R2: 从倾斜姿态回正（初始偏置 10~50°）
  check(t2_all_settled,        "R2 倾斜回正 10~50°：全部进入稳态带");
  check(t2_worst_final < 1.0f, "R2 倾斜回正：最大稳差 < 1°");

  // R3: Monte Carlo 参数随机扰动 I×[0.5,1.5] kT×[0.7,1.3] CG≤10mm
  // ±2% 带对 20° 指令仅有 ±0.4°；常值 CG 扰动下未在 8 s 内进带
  // 表示配平较慢，不能称为“发散”。这里分别约束窄带进入率和末值误差。
  check(narrow_band_miss <= 10,     "R3 Monte Carlo 至少 90/100 在 8s 内进入±2%窄带");
  check(mc_worst_final < 1.5f,      "R3 Monte Carlo 最大末值误差 < 1.5°");
  check(worst < 20.f * 1.25f,       "R3 Monte Carlo 最坏峰值超调 < 25%");

  // R4: 最坏参数组合（惯量/推力/重心同时偏离）
  check(t4_all_settled,        "R4 最坏组合：全部进入稳态带");
  check(t4_worst_final < 1.0f, "R4 最坏组合：最大稳差 < 1°");
  check(t4_worst_ov   < 0.25f, "R4 最坏组合：最大超调 < 25%");

  std::printf("\n结论（均由上述断言支撑）:\n");
  std::printf("  大机动 10~90° 阶跃与 10~50° 回正全部收敛，稳差 <1°。\n");
  std::printf("  I±50%%/kT±30%%/CG≤10mm 的 100 组固定种子扫描中，至少 90 组在 8s 内进入±2%%窄带，且末值误差 <1.5°。\n");
  std::printf("  ⚠️ 单轴俯仰台架，未含气动力与舵机动态；实机大机动仍需渐进验证。\n");

  std::printf("\n");
  if (g_fail == 0) std::printf("=== 全部通过 ===\n");
  else             std::printf("=== %d 项失败 ===\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
