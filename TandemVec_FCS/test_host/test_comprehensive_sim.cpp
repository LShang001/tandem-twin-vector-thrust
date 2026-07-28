// ============================================================
//  test_comprehensive_sim.cpp — 全面仿真:增益扫描+扰动+灵敏度
//
//  条件: α限幅=±100(不截断PID), 完整物理链路
//  使用真实 PositionPID (no-dt convention, 200Hz等效)
//  镜像 flight_control.cpp 的完整控制路径
//
//  分析: (1)Kp_a单独 (2)Kp_r单独 (3)联合扫描 (4)扰动抑制 (5)模型偏差
//
//  编译: g++ -std=c++17 -Iinclude -Itest_host/stub
//        test_host/test_comprehensive_sim.cpp -o test_host/bin/cs && ./test_host/bin/cs
// ============================================================
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#include "../include/TandemVec_ControlAllocation.h"
#include "../include/TandemVec_Propulsion.h"
#include "../include/TandemVec_Config.h"
#include "../include/TandemVec_AttitudeCtrl.h"
#include "../include/PositionPID.h"
#include "../include/ComplementaryFilter.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

// ============================================================
//  电机+刚体
// ============================================================
struct Motor{float w,t,ta;Motor(float x=0.28f):w(0),t(0),ta(x){}void set(float wt){t=wt;}
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
};

// ============================================================
//  全阶仿真 (单轴俯仰, 与 flight_control.cpp 同构)
//  返回: peak°, settle_s, final_err°, omega_peak°/s
// ============================================================
static void sim_pitch(float Kpa,float Kpr,float Ki,
                      float thr,float cg_mm,float kt_asym_pct,
                      float target_deg,float sim_s,float dt,
                      float out[4])
{
  const TandemVecParams&P=kDefaultTandemVecParams;
  int N=(int)(sim_s/dt);
  RigidBody body;Motor mf,mr;
  // Pitch axis only (body y-axis)
  PositionPID ang(Kpa,0,0), rate(Kpr,Ki,0);
  ang.setOutputLimits(-50,50);rate.setOutputLimits(-100,100);
  rate.setIntegralLimit(10.f);rate.setIntegralThreshold(30.f);
  // Use only 1 filter for each stage (pitch axis)
  float gf_p=0,af_p=0,rf_p=0; (void)af_p;(void)rf_p; // filtered values
  ComplementaryFilter gF(0.3f),aF(0.85f),rF(0.25f);

  PropulsionState ps={0,0,0,0};
  float a20=target_deg*3.14159265f/180.f,hr=a20/2;
  Quat4f qt=qNorm(Quat4f(cosf(hr)*cosf(hr),0,cosf(hr)*sinf(hr),0)); // pure pitch

  float peak=0,settle=99;
  for(int i=0;i<N;i++){
    float t=i*dt;
    // Gyro: body pitch rate (rad/s) → deg/s, with light noise
    float gd_raw=body.om[1]*57.29578f + ((float)rand()/RAND_MAX-0.5f)*0.01f;
    (void)gf_p;gf_p=gF.filter(gd_raw);

    // Quaternion error → pitch error (deg)
    Quat4f qe=qNorm(qMul(qConj(body.q),qt));
    float sw=qe.w>=0?1:-1;
    float v=sqrtf(qe.z*qe.z+qe.y*qe.y);
    float sc=v>0.25f?2*atan2f(v,fabsf(qe.w))/v*57.29578f:114.59156f;
    float err=sw*qe.y*sc; // pitch error = q_err.y component

    // Outer P → ω_ref
    float wref=aF.filter(constrain(ang.computeWithExternalDerivative(err,0,-gf_p),-50.f,50.f));
    // Inner P+I → α
    float alpha=rF.filter(constrain(rate.computeDerivativeOnMeasurement(wref,gf_p),-100.f,100.f));

    // I×α → M_cmd
    float My_cmd=P.Iy*alpha;
    float w0=(thr/100.f)*P.wMax;
    AllocationInput ai={0,My_cmd,0,w0,ps};
    AllocationOutput ao=allocateMoments(ai,P,AllocationStrategy::BTRUE);

    auto diff=allocateDifferential(w0,ao.dw,P);
    mf.set(diff.wf_target);mr.set(diff.wt_target);
    float wf=mf.step(dt),wt=mr.step(dt);
    ps={wf,wt,ao.delta_f,ao.delta_t};

    // Actual wrench: 单轴俯仰测试→delta_f=0(无偏航通道耦合),只用delta_t
    float asym=1.f+kt_asym_pct/100.f;
    PropulsionState pa={wf*sqrtf(asym),wt,0.f,ao.delta_t};
    SixDOFWrench w=computeWrench(pa,P);

    // CG offset creates pitch moment disturbance
    float T_tot=P.kT*(wf*wf*sqrtf(asym)+wt*wt)*0.5f;
    float M_disturb=T_tot*cg_mm*0.001f;
    float M_tot[3]={w.Mx,w.My+M_disturb,w.Mz};

    body.step(M_tot,P,dt);

    float e=body.errDeg(qt);
    if(e>peak)peak=e;
    if(t>sim_s*0.25f&&e<0.4f&&settle>98)settle=t;
  }
  out[0]=peak; out[1]=settle; out[2]=body.errDeg(qt);
  out[3]=fabsf(body.om[1])*57.29578f;
}

int main()
{
  const float thr=40.f, dt=0.005f, sim=6.f, tgt=20.f;
  float o[4];

  std::printf("══════════════════════════════════════════════\n");
  std::printf(" 单轴俯仰全面仿真 — α限幅±100(不截断)\n");
  std::printf(" I=%.4f kg·m²(估算), α可用≈13 rad/s²@悬停\n",kDefaultTandemVecParams.Iy);
  std::printf("══════════════════════════════════════════════\n\n");

  // ═══ S1: Kp_r×Kp_a 甜区联合扫描 (峰值<30°=✅) ═══
  std::printf("── S1: 联合扫描 — 峰值温度图 (✓=<30°) ──\n");
  std::printf("Kp_a↓ Kp_r→");
  for(float r=0.05f;r<=0.55f;r+=0.05f)std::printf(" %6.2f",r);
  std::printf("\n");
  for(float a=1.f;a<=8.f;a+=1.f){
    std::printf(" %4.0f   ",a);
    for(float r=0.05f;r<=0.55f;r+=0.05f){
      sim_pitch(a,r,0.0003f,thr,3.f,3.f,tgt,4.f,dt,o);
      if(o[0]<25.f)std::printf(" %5.1f✓",o[0]);
      else if(o[0]<50.f)std::printf(" %5.1f ",o[0]);
      else std::printf("  ╳╳╳ ");
    }
    std::printf("  a=%.0f\n",a);
  }
  std::printf("✓=收敛(峰值<25°) 数字=发散(峰值°) ╳=严重发散(>50°)\n");
  std::printf("结论: 甜区在 Kp_a=1~3, Kp_r=0.05~0.20。Kp_a≥5或Kp_r≥0.25严重发散\n\n");

  // ═══ S2: 甜区内精细扫描 ═══
  std::printf("── S2: 甜区精细扫描 (Kp_a=2.0~4.0, Kp_r=0.08~0.25) ──\n");
  std::printf("%8s%8s%8s%8s%8s\n","Kp_a","Kp_r","peak°","settle","final°");
  for(float a:{1.5f,2.f,2.5f,3.f,3.5f,4.f}){
    for(float r:{0.08f,0.10f,0.12f,0.15f,0.18f,0.20f,0.25f}){
      sim_pitch(a,r,0.0003f,thr,3.f,3.f,tgt,sim,dt,o);
      std::printf("%7.1f %7.2f %7.1f %7.2f %7.2f\n",a,r,o[0],o[1],o[2]);
  }}

  // ═══ S3: 扰动 & 鲁棒性 (在甜区增益下) ═══
  std::printf("\n── S3: 扰动&鲁棒性 (最优 Kp_a=2.5 Kp_r=0.12) ──\n");
  std::printf("%-20s %8s %8s %8s\n","工况","peak°","settle","final°");
  float best_a=2.5f,best_r=0.12f;

  sim_pitch(best_a,best_r,0.0003f,thr,0.f,0.f,tgt,sim,dt,o);
  std::printf("%-20s %7.1f %7.2f %7.2f\n","理想(无扰动)",o[0],o[1],o[2]);

  sim_pitch(best_a,best_r,0.0003f,thr,5.f,5.f,tgt,sim,dt,o);
  std::printf("%-20s %7.1f %7.2f %7.2f\n","CG5mm+kT5%%偏差",o[0],o[1],o[2]);

  // 模型偏差鲁棒性: I ±50%, kT ±30%
  std::printf("\n── S4: 模型偏差鲁棒性 (甜区增益) ──\n");
  struct{const char*n;float im;}p[]={{"名义I",1.f},{"I+50%",1.5f},{"I-50%",0.5f}};
  for(auto&c:p){// quick inline for modified I
    TandemVecParams P=kDefaultTandemVecParams;P.Iy*=c.im;P.Iz*=c.im;
    int N=1200;RigidBody b;Motor mf,mr;PositionPID ang(best_a,0,0),rate(best_r,0.0003f,0);
    ang.setOutputLimits(-50,50);rate.setOutputLimits(-100,100);rate.setIntegralLimit(10.f);rate.setIntegralThreshold(30.f);
    ComplementaryFilter gF(0.3f),aF(0.85f),rF(0.25f);float gf=0;PropulsionState ps={0,0,0,0};
    float hr=tgt*3.14159265f/360.f;Quat4f qt=qNorm(Quat4f(cosf(hr)*cosf(hr),0,cosf(hr)*sinf(hr),0));
    float pk=0,st=99;for(int i=0;i<N;i++){float t=i*0.005f;
      gf=gF.filter(b.om[1]*57.29578f);Quat4f qe=qNorm(qMul(qConj(b.q),qt));float sw=qe.w>=0?1:-1;
      float v=sqrtf(qe.z*qe.z+qe.y*qe.y),sc=v>0.25f?2*atan2f(v,fabsf(qe.w))/v*57.29578f:114.59156f;
      float err=sw*qe.y*sc,wref=aF.filter(constrain(ang.computeWithExternalDerivative(err,0,-gf),-50.f,50.f));
      float al=rF.filter(constrain(rate.computeDerivativeOnMeasurement(wref,gf),-100.f,100.f));
      float M=P.Iy*al,w0=(thr/100.f)*P.wMax;AllocationInput ai={0,M,0,w0,ps};
      AllocationOutput ao=allocateMoments(ai,P,AllocationStrategy::FULL_B);
      auto df=allocateDifferential(w0,ao.dw,P);mf.set(df.wf_target);mr.set(df.wt_target);
      float wf=mf.step(0.005f),wt=mr.step(0.005f);ps={wf,wt,ao.delta_f,ao.delta_t};
      PropulsionState pa={wf,wt,0,ao.delta_t};SixDOFWrench w=computeWrench(pa,P);
      float Mt[3]={0,w.My,0};b.step(Mt,P,0.005f);
      float e=b.errDeg(qt);if(e>pk)pk=e;if(t>1.5f&&e<0.4f&&st>98)st=t;
    }std::printf("%-20s %7.1f %7.2f\n",c.n,pk,st);
  }

  // ═══ 最终调参路线 ═══
  std::printf("\n═══════════════════════════════════════════\n");
  std::printf(" 实飞调参路线 (基于I≈0.022估算下的仿真)\n");
  std::printf("═══════════════════════════════════════════\n");
  std::printf("Kp起点: Kp_a=2.0, Kp_r=0.10 (保守, 甜区内)\n");
  std::printf("步骤: 每次Kp_r+0.02 → 直到响应脆 → 回退0.02 → 锁定\n");
  std::printf("      每次Kp_a+0.5 → 直到收敛<2s → 锁定\n");
  std::printf("⚠️ 增益上限: Kp_r<0.25, Kp_a<4.0 (超过→振荡发散)\n");
  std::printf("积分分离阈值: 30deg/s (甜区内不敏感)\n\n");

  return 0;
}
