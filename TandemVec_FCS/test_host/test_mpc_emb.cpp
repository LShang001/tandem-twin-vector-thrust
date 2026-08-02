// ============================================================
//  test_mpc_emb.cpp — 嵌入式 LMPC 求解器基准测试
//  1) 数值验证：与 Python 原型（mpc_proto.py）同场景对比收敛曲线
//  2) 性能基准：耗时（float/double、不同迭代数）、内存占用
//  3) 极限测试：N 可扩展性、数值稳定性（长时运行、NaN 检查）
// ============================================================
#include "mpc_emb.h"
#include "../include/TandemVec_Config.h"

#include <chrono>
#include <initializer_list>
#include <cstdio>
#include <cmath>

using Clock = std::chrono::high_resolution_clock;
static constexpr double PI = 3.14159265358979323846;

static const TandemVecParams &P = kDefaultTandemVecParams;

template <typename T, int N>
static void makeModel(typename EmbeddedLMPC<T, N>::Model &m) {
    m.Ix = P.Ix; m.Iy = P.Iy; m.Iz = P.Iz;
    m.tauM = P.tauM;
    // 名义 B_eff（悬停点）
    T w0 = std::sqrt(P.m * P.g / (2 * P.kT));
    T T0 = P.kT * w0 * w0, tau0 = P.kQ * w0 * w0;
    m.Beff[0] = -2 * tau0 / m.Ix;
    m.Beff[1] = 0; m.Beff[2] = 0;
    m.Beff[3] = 0; m.Beff[4] = -P.b * T0 / m.Iy; m.Beff[5] = -tau0 / m.Iy;
    m.Beff[6] = 0; m.Beff[7] = -tau0 / m.Iz; m.Beff[8] = P.a * T0 / m.Iz;
    m.u_max[0] = P.dwMax; m.u_max[1] = P.dMax; m.u_max[2] = P.dMax;
    for (int i = 0; i < 9; ++i) m.Q[i] = (i < 3) ? 4.0 : (i < 6 ? 1.0 : 0.0);
    m.R[0] = m.R[1] = m.R[2] = 0.1;
}

// 六自由度仿真：I·ω̇ = M − ω×Iω − ω×h，平动 v̇ = F/m + g_b − ω×v，
// 电机一阶滞后（与 MPC 模型一致）+ 四元数欧拉积分
template <typename T>
struct SimState {
    T q[4]; T w[3]; T v[3]; T wf, wt;
};

template <typename T>
static void simStep6(SimState<T> &s, const T u[3], T dt) {
    T w0 = std::sqrt(P.m * P.g / (2 * P.kT));
    T wfT = w0 * std::sqrt(std::max<T>(0, 1 + u[0]));
    T wtT = w0 * std::sqrt(std::max<T>(0, 1 - u[0]));
    T a = std::min<T>(dt / P.tauM, 1);
    s.wf += (wfT - s.wf) * a;              // 电机一阶滞后
    s.wt += (wtT - s.wt) * a;
    T Tf = P.kT * s.wf * s.wf, Tt = P.kT * s.wt * s.wt;
    T Qf = P.kQ * s.wf * s.wf, Qt = P.kQ * s.wt * s.wt;
    T cf = std::cos(u[2]), sf = std::sin(u[2]);
    T ct = std::cos(u[1]), st = std::sin(u[1]);
    T F[3] = {Tf*cf + Tt*ct, Tf*sf, -Tt*st};
    T M[3] = {-Qf*cf + Qt*ct, -P.b*Tt*st - Qf*sf, P.a*Tf*sf - Qt*st};
    T hx = P.Jp * (s.wf - s.wt);
    T g[3] = {(P.Iz - P.Iy) * s.w[1] * s.w[2],
              (P.Ix - P.Iz) * s.w[2] * s.w[0] - s.w[2] * hx,
              (P.Iy - P.Ix) * s.w[0] * s.w[1] + s.w[1] * hx};
    T I[3] = {P.Ix, P.Iy, P.Iz};
    for (int i = 0; i < 3; ++i) s.w[i] += (M[i] - g[i]) / I[i] * dt;
    // 平动：重力机体系分量 g_b = R(q)ᵀ·(0,0,g)
    T gb[3] = {0, 0, P.g};
    {
        // q⁻¹ ⊗ (0,0,g) ⊗ q
        T qi[4] = {s.q[0], -s.q[1], -s.q[2], -s.q[3]};
        T p[4] = {0, 0, 0, P.g};
        T t1[4] = {qi[0]*p[0]-qi[1]*p[1]-qi[2]*p[2]-qi[3]*p[3],
                   qi[0]*p[1]+qi[1]*p[0]+qi[2]*p[3]-qi[3]*p[2],
                   qi[0]*p[2]-qi[1]*p[3]+qi[2]*p[0]+qi[3]*p[1],
                   qi[0]*p[3]+qi[1]*p[2]-qi[2]*p[1]+qi[3]*p[0]};
        T t2[4] = {t1[0]*s.q[0]-t1[1]*s.q[1]-t1[2]*s.q[2]-t1[3]*s.q[3],
                   t1[0]*s.q[1]+t1[1]*s.q[0]+t1[2]*s.q[3]-t1[3]*s.q[2],
                   t1[0]*s.q[2]-t1[1]*s.q[3]+t1[2]*s.q[0]+t1[3]*s.q[1],
                   t1[0]*s.q[3]+t1[1]*s.q[2]-t1[2]*s.q[1]+t1[3]*s.q[0]};
        gb[0] = t2[1]; gb[1] = t2[2]; gb[2] = t2[3];
    }
    T wxv[3] = {s.w[1]*s.v[2]-s.w[2]*s.v[1],
                s.w[2]*s.v[0]-s.w[0]*s.v[2],
                s.w[0]*s.v[1]-s.w[1]*s.v[0]};
    for (int i = 0; i < 3; ++i)
        s.v[i] += (F[i] / P.m + gb[i] - wxv[i]) * dt;
    // 四元数积分（小旋转 w≈1）
    T dq[4] = {1, s.w[0]*0.5f*dt, s.w[1]*0.5f*dt, s.w[2]*0.5f*dt};
    T nq[4] = {s.q[0]*dq[0]-s.q[1]*dq[1]-s.q[2]*dq[2]-s.q[3]*dq[3],
               s.q[0]*dq[1]+s.q[1]*dq[0]+s.q[2]*dq[3]-s.q[3]*dq[2],
               s.q[0]*dq[2]-s.q[1]*dq[3]+s.q[2]*dq[0]+s.q[3]*dq[1],
               s.q[0]*dq[3]+s.q[1]*dq[2]-s.q[2]*dq[1]+s.q[3]*dq[0]};
    T n = std::sqrt(nq[0]*nq[0]+nq[1]*nq[1]+nq[2]*nq[2]+nq[3]*nq[3]);
    for (int i = 0; i < 4; ++i) s.q[i] = nq[i] / n;
}

template <typename T>
static void simStep(T q[4], T w[3], const T u[3], T dt) {
    SimState<T> s;
    for (int i = 0; i < 4; ++i) s.q[i] = q[i];
    for (int i = 0; i < 3; ++i) { s.w[i] = w[i]; s.v[i] = 0; }
    s.wf = s.wt = std::sqrt(P.m * P.g / (2 * P.kT));
    simStep6(s, u, dt);
    for (int i = 0; i < 4; ++i) q[i] = s.q[i];
    for (int i = 0; i < 3; ++i) w[i] = s.w[i];
}

template <typename T, int N>
static void runBenchmark(const char *tag, int n_iter, double &t_ms, int &n_iter_max) {
    typename EmbeddedLMPC<T, N>::Model m;
    makeModel<T, N>(m);
    static EmbeddedLMPC<T, N> mpc;   // static：N=60 时实例 ~2.2MB，栈上会溢出
    mpc.init(m);
    // 5° 扰动闭环 3s
    T q[4] = {std::cos(PI/4), 0, std::sin(PI/4), 0};
    T q_des[4] = {std::cos(PI/4), 0, std::sin(PI/4), 0};
    T w[3] = {0, 0, 0};
    // 初始绕 y 偏 5°
    T a5 = 5.0 * PI / 180 / 2;
    T q0[4] = {q[0]*std::cos(a5), q[2]*std::sin(a5), q[2]*std::cos(a5)+q[0]*std::sin(a5)*-1, q[3]*std::cos(a5)};
    // 简化：直接构造绕 y 5° 的扰动
    T qy[4] = {std::cos(a5), 0, std::sin(a5), 0};
    T qd[4] = {q[0]*qy[0]-q[1]*qy[1]-q[2]*qy[2]-q[3]*qy[3],
               q[0]*qy[1]+q[1]*qy[0]+q[2]*qy[3]-q[3]*qy[2],
               q[0]*qy[2]-q[1]*qy[3]+q[2]*qy[0]+q[3]*qy[1],
               q[0]*qy[3]+q[1]*qy[2]-q[2]*qy[1]+q[3]*qy[0]};
    for (int i = 0; i < 4; ++i) q[i] = qd[i];
    const T DT = 0.004;
    T eps_final = 0;
    bool nan = false;
    SimState<T> s;
    for (int i = 0; i < 4; ++i) s.q[i] = q[i];
    for (int i = 0; i < 3; ++i) { s.w[i] = w[i]; s.v[i] = 0; }
    s.wf = s.wt = std::sqrt(P.m * P.g / (2 * P.kT));
    auto t0 = Clock::now();
    for (int i = 0; i < 750; ++i) {   // 3s
        T u[3];
        mpc.update(s.q, q_des, s.w, u, n_iter);
        if (i < 3 || i == 749) std::printf("  step %d: u=(%.4f,%.4f,%.4f)\n", i, u[0], u[1], u[2]);
        simStep6(s, u, DT);
        T qe[4] = {q_des[0]*s.q[0]+q_des[1]*s.q[1]+q_des[2]*s.q[2]+q_des[3]*s.q[3],
                   q_des[0]*s.q[1]-q_des[1]*s.q[0]-q_des[2]*s.q[3]+q_des[3]*s.q[2],
                   q_des[0]*s.q[2]+q_des[1]*s.q[3]-q_des[2]*s.q[0]-q_des[3]*s.q[1],
                   q_des[0]*s.q[3]-q_des[1]*s.q[2]+q_des[2]*s.q[1]-q_des[3]*s.q[0]};
        if (qe[0] < 0) { qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]; }
        eps_final = std::sqrt(qe[1]*qe[1]+qe[2]*qe[2]+qe[3]*qe[3]);
        // NaN 检测：自不等（NaN != NaN）；w 为 3 元素，勿越界
        bool qnan = false;
        for (int k = 0; k < 4; ++k)
            if (s.q[k] != s.q[k]) { qnan = true; break; }
        for (int k = 0; k < 3 && !qnan; ++k)
            if (s.w[k] != s.w[k]) qnan = true;
        if (qnan) {
            nan = true;
            static int n_printed = 0;
            if (n_printed++ < 3)
                std::printf("  !! NaN at step %d: q=(%.4f,%.4f,%.4f,%.4f) w=(%.4f,%.4f,%.4f) u=(%.4f,%.4f,%.4f)\n",
                            i, s.q[0], s.q[1], s.q[2], s.q[3], s.w[0], s.w[1], s.w[2], u[0], u[1], u[2]);
        }
    }
    auto t1 = Clock::now();
    t_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / 750.0;
    n_iter_max = N;
    std::printf("[%s] N=%d 迭代=%d  单步均耗=%.4f ms  eps末值=%.4f  %s\n",
                tag, N, n_iter, t_ms, eps_final, nan ? "!! NaN !!" : "OK");
}

template <typename T, int N>
static void runBenchmarkFGM(const char *tag, int n_iter, double &t_ms, int &n_iter_max) {
    typename EmbeddedLMPC<T, N>::Model m;
    makeModel<T, N>(m);
    static EmbeddedLMPC<T, N> mpc;
    mpc.init(m);
    T q_des[4] = {std::cos(PI/4), 0, std::sin(PI/4), 0};
    T a5 = 5.0 * PI / 180 / 2;
    T qy[4] = {std::cos(a5), 0, std::sin(a5), 0};
    T qd[4] = {q_des[0]*qy[0]-q_des[1]*qy[1]-q_des[2]*qy[2]-q_des[3]*qy[3],
               q_des[0]*qy[1]+q_des[1]*qy[0]+q_des[2]*qy[3]-q_des[3]*qy[2],
               q_des[0]*qy[2]-q_des[1]*qy[3]+q_des[2]*qy[0]+q_des[3]*qy[1],
               q_des[0]*qy[3]+q_des[1]*qy[2]-q_des[2]*qy[1]+q_des[3]*qy[0]};
    T eps_final = 0;
    bool nan = false;
    SimState<T> s;
    for (int i = 0; i < 4; ++i) s.q[i] = qd[i];
    for (int i = 0; i < 3; ++i) { s.w[i] = 0; s.v[i] = 0; }
    s.wf = s.wt = std::sqrt(P.m * P.g / (2 * P.kT));
    const T DT = 0.004;
    auto t0 = Clock::now();
    for (int i = 0; i < 750; ++i) {
        T u[3];
        mpc.updateFGM(s.q, q_des, s.w, u, n_iter);
        simStep6(s, u, DT);
        T qe[4] = {q_des[0]*s.q[0]+q_des[1]*s.q[1]+q_des[2]*s.q[2]+q_des[3]*s.q[3],
                   q_des[0]*s.q[1]-q_des[1]*s.q[0]-q_des[2]*s.q[3]+q_des[3]*s.q[2],
                   q_des[0]*s.q[2]+q_des[1]*s.q[3]-q_des[2]*s.q[0]-q_des[3]*s.q[1],
                   q_des[0]*s.q[3]-q_des[1]*s.q[2]+q_des[2]*s.q[1]-q_des[3]*s.q[0]};
        if (qe[0] < 0) { qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]; }
        eps_final = std::sqrt(qe[1]*qe[1]+qe[2]*qe[2]+qe[3]*qe[3]);
        for (int k = 0; k < 4; ++k)
            if (s.q[k] != s.q[k]) nan = true;
        for (int k = 0; k < 3; ++k)
            if (s.w[k] != s.w[k]) nan = true;
    }
    auto t1 = Clock::now();
    t_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / 750.0;
    n_iter_max = N;
    std::printf("[%s] N=%d 迭代=%d  单步均耗=%.4f ms  eps末值=%.4f  %s\n",
                tag, N, n_iter, t_ms, eps_final, nan ? "!! NaN !!" : "OK");
}

int main() {
    std::printf("=== 嵌入式 LMPC 求解器基准（STM32H743 @480MHz 同构 g++ -O2）===\n");
    std::printf("内存（double, N=30）: %.1f KB\n", EmbeddedLMPC<double, 30>::memBytes() / 1024.0);
    std::printf("内存（float,  N=30）: %.1f KB\n", EmbeddedLMPC<float, 30>::memBytes() / 1024.0);

    double t; int n;
    // 收敛性（double，400 迭代）——6DOF 含电机滞后与平动
    runBenchmark<double, 30>("double 收敛基准(6DOF)", 400, t, n);
    // 迭代数扫描（warm start 下的最少迭代）
    for (int it : {400, 100, 50, 25}) {
        runBenchmark<double, 30>("double 迭代扫描", it, t, n);
    }
    // FGM 快速梯度法（O(1/k²)：25/50 迭代应达到 PGD 400 精度）
    runBenchmarkFGM<double, 30>("double FGM-50", 50, t, n);
    runBenchmarkFGM<double, 30>("double FGM-25", 25, t, n);
    runBenchmarkFGM<double, 30>("double FGM-10", 10, t, n);
    // float vs double
    runBenchmark<float, 30>("float 精度", 400, t, n);
    // N 扩展性（N=10/20/60）
    runBenchmark<double, 10>("double N=10", 400, t, n);
    runBenchmark<double, 20>("double N=20", 400, t, n);
    runBenchmark<double, 60>("double N=60", 400, t, n);
    return 0;
}
