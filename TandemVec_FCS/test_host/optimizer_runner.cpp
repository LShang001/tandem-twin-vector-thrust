// optimizer_runner.cpp — 20°阶跃+CG3mm+推力3%不对称, 8s仿真
// 被Python优化器调用: or.exe Kpa Kpr Ki sep → stdout: cost peak settle rms
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
struct Motor{float w,t,ta;Motor(float x=0.28f):w(0),t(0),ta(x){}void set(float wt){t=wt;}float step(float dt){w+=(t-w)*fminf(dt/ta,1.f);return w;}};
struct Body{Quat4f q;float o[3];Body(){q={1,0,0,0};o[0]=o[1]=o[2]=0;}
  void step(const float M[3],const TandemVecParams&P,float dt){float p=o[0],qq=o[1],r=o[2];o[0]+=(M[0]-((P.Iz-P.Iy)*qq*r))/P.Ix*dt;o[1]+=(M[1]-((P.Ix-P.Iz)*r*p))/P.Iy*dt;o[2]+=(M[2]-((P.Iy-P.Ix)*p*qq))/P.Iz*dt;float wx=o[0],wy=o[1],wz=o[2];Quat4f dq={-0.5f*(q.x*wx+q.y*wy+q.z*wz),0.5f*(q.w*wx+q.y*wz-q.z*wy),0.5f*(q.w*wy-q.x*wz+q.z*wx),0.5f*(q.w*wz+q.x*wy-q.y*wx)};q.w+=dq.w*dt;q.x+=dq.x*dt;q.y+=dq.y*dt;q.z+=dq.z*dt;q=qNorm(q);}
  float e(const Quat4f&r)const{Quat4f qe=qNorm(qMul(qConj(q),r));if(qe.w<0){qe.w=-qe.w;qe.x=-qe.x;qe.y=-qe.y;qe.z=-qe.z;}return 2.f*atan2f(sqrtf(qe.x*qe.x+qe.y*qe.y+qe.z*qe.z),qe.w)*57.29578f;}};
int main(int argc,char**argv){
  float Kpa=atof(argv[1]),Kpr=atof(argv[2]),Ki=atof(argv[3]),sep=atof(argv[4]);
  const auto&P=kDefaultTandemVecParams;
  Body b;Motor mf,mr;PositionPID ang(Kpa,0,0),rate(Kpr,Ki,0);
  ang.setOutputLimits(-50,50);rate.setOutputLimits(-100,100);rate.setIntegralLimit(10.f);rate.setIntegralThreshold(sep);
  ComplementaryFilter gF(0.3f),aF(0.85f),rF(0.25f);float gf=0;
  PropulsionState ps={0,0,0,0};
  float hr=10*3.14159265f/180.f;
  Quat4f qt=qNorm(Quat4f(cosf(hr)*cosf(hr),0,cosf(hr)*sinf(hr),0));
  float pk=0,st=99,fe=0,rms=0;int N=1600,rn=0; // 8s仿真
  for(int i=0;i<N;i++){float t=i*0.005f;
    gf=gF.filter(b.o[1]*57.29578f+((float)rand()/RAND_MAX-0.5f)*0.01f);
    Quat4f qe=qNorm(qMul(qConj(b.q),qt));float sw=qe.w>=0?1:-1;
    float v=sqrtf(qe.z*qe.z+qe.y*qe.y),sc=v>0.25f?2*atan2f(v,fabsf(qe.w))/v*57.29578f:114.59156f;
    float err=sw*qe.y*sc,wref=aF.filter(constrain(ang.computeWithExternalDerivative(err,0,-gf),-50.f,50.f));
    float al=rF.filter(constrain(rate.computeDerivativeOnMeasurement(wref,gf),-100.f,100.f));
    float M=P.Iy*al,w0=0.4f*P.wMax;AllocationInput ai={0,M,0,w0,ps};
    AllocationOutput ao=allocateMoments(ai,P,AllocationStrategy::FULL_B);
    auto df=allocateDifferential(w0,ao.dw,P);mf.set(df.wf_target);mr.set(df.wt_target);
    float wf=mf.step(0.005f),wt=mr.step(0.005f);ps={wf,wt,ao.delta_f,ao.delta_t};
    PropulsionState pa={wf*1.015f,wt,0,ao.delta_t};SixDOFWrench w=computeWrench(pa,P);
    float T_tot=P.kT*(wf*wf*1.03f+wt*wt)*0.5f;
    float Mt[3]={0,w.My+T_tot*0.003f,0};b.step(Mt,P,0.005f);
    float e=b.e(qt);if(e>pk)pk=e;if(t>2.0f&&e<0.4f&&st>98)st=t;
    if(t>5.0f){rn++; rms+=e*e;}
  }
  fe=b.e(qt);rms=rn>0?sqrtf(rms/rn):99;
  float cost=pk+(st<90?st*2.f:100.f)+rms*5.f; // 峰值+收敛(超时惩罚)+稳态RMS
  printf("%.3f %.1f %.2f %.2f\n",cost,pk,st,fe);
  return 0;
}
