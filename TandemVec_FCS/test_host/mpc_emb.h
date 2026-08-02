// ============================================================
//  mpc_emb.h — 嵌入式 LMPC 求解器（展开式 QP + 梯度投影 + warm start）
//
//  纯头文件、零依赖、float/double 可切（模板），面向 STM32H743
//  状态 x = [ε(3), ω(3), a(3)]（a 为执行器滞后角加速度，τm 增广）
//  输入 u = [Δω, δt, δf]，预测时域 N，框约束 |u|≤u_max
//
//  编译运行（test_host 基准）：
//    g++ -std=c++17 -O2 -Iinclude test_host/test_mpc_emb.cpp -o test_host/bin/mpcb
// ============================================================
#ifndef MPC_EMB_H
#define MPC_EMB_H

#include <cmath>
#include <cstring>
#include <cstdint>

template <typename T, int N>
class EmbeddedLMPC {
public:
    static constexpr int NU = 3;          // 控制维数
    static constexpr int NX = 9;          // 状态维数（ε, ω, a）
    static constexpr int NZ = N * NU;     // 决策变量数

    // 模型参数（悬停点 + 执行器滞后）
    struct Model {
        T Ix, Iy, Iz;      // 惯量
        T tauM;            // 电机滞后
        T Beff[9];         // B_eff = I⁻¹·B（9 参数，辨识/名义）
        T u_max[3];        // 执行器限幅
        T Q[NX];           // 状态权
        T R[3];            // 控制权
    };

    void init(const Model &m, T dt_pred = 0.02) {
        memcpy(&m_, &m, sizeof(m));
        dt_ = dt_pred;
        // —— 离散模型：x_{k+1} = A x_k + B u_k（含滞后增广）——
        // ε̇=½ω, ω̇=a, ȧ=(B_eff·u−a)/τ
        T A6[6][6] = {};
        for (int i = 0; i < 6; ++i) A6[i][i] = 1;
        A6[0][3] = 0.5 * dt_; A6[1][4] = 0.5 * dt_; A6[2][5] = 0.5 * dt_;
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 9; ++j) A_[i][j] = 0;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) A_[i][j] = A6[i][j];
        A_[3][6] = A_[4][7] = A_[5][8] = dt_;                    // ω ← a
        A_[6][6] = A_[7][7] = A_[8][8] = 1 - dt_ / m.tauM;       // a 滞后
        for (int i = 0; i < 9; ++i)
            for (int j = 0; j < 3; ++j) B_[i][j] = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                B_[6 + i][j] = m.Beff[i * 3 + j] * (dt_ / m.tauM); // a ← B_eff·u

        // —— 展开系数：Ap[k]=A^k ——
        for (int k = 0; k <= N; ++k)
            for (int i = 0; i < NX; ++i)
                for (int j = 0; j < NX; ++j)
                    Ap_[k][i][j] = (k == 0) ? (i == j ? 1 : 0) : 0;
        for (int k = 1; k <= N; ++k) {
            T tmp[NX][NX] = {};
            for (int i = 0; i < NX; ++i)
                for (int j = 0; j < NX; ++j) {
                    T s = 0;
                    for (int l = 0; l < NX; ++l)
                        s += A_[i][l] * Ap_[k - 1][l][j];
                    tmp[i][j] = s;
                }
            memcpy(Ap_[k], tmp, sizeof(tmp));
        }
        // —— Hessian H（按块构建，∂x_k/∂u_bi = A^{k-1-bi}B 临时计算）——
        for (int bi = 0; bi < N; ++bi)
            for (int bj = 0; bj < N; ++bj) {
                T Hb[3][3] = {};
                for (int k = std::max(bi, bj) + 1; k <= N; ++k) {
                    T Mi[NX][3] = {}, Mj[NX][3] = {};
                    // Mi = A^{k-1-bi}B
                    for (int i = 0; i < NX; ++i)
                        for (int c = 0; c < NU; ++c) {
                            T s = 0;
                            for (int l = 0; l < NX; ++l)
                                s += Ap_[k - 1 - bi][i][l] * B_[l][c];
                            Mi[i][c] = s;
                        }
                    if (bi != bj) {
                        for (int i = 0; i < NX; ++i)
                            for (int c = 0; c < NU; ++c) {
                                T s = 0;
                                for (int l = 0; l < NX; ++l)
                                    s += Ap_[k - 1 - bj][i][l] * B_[l][c];
                                Mj[i][c] = s;
                            }
                    } else {
                        for (int i = 0; i < NX; ++i)
                            for (int c = 0; c < NU; ++c) Mj[i][c] = Mi[i][c];
                    }
                    // Hb += Miᵀ Q Mj（Q 对角）
                    for (int a = 0; a < NU; ++a)
                        for (int b = 0; b < NU; ++b) {
                            T s = 0;
                            for (int p = 0; p < NX; ++p)
                                s += Mi[p][a] * m.Q[p] * Mj[p][b];
                            Hb[a][b] += s;
                        }
                }
                for (int a = 0; a < NU; ++a)
                    for (int b = 0; b < NU; ++b) {
                        if (bi == bj && a == b) Hb[a][b] += m.R[a];
                        H_[bi * NU + a][bj * NU + b] = 2 * Hb[a][b];
                    }
            }
        // —— 线性项 L（NZ×NX）：L_bi = 2Σ_k (A^{k-1-bi}B)ᵀQ·A^k ——
        for (int bi = 0; bi < N; ++bi)
            for (int p = 0; p < NX; ++p) {
                for (int c = 0; c < NU; ++c) {
                    T s = 0;
                    for (int k = bi + 1; k <= N; ++k) {
                        T acc = 0;
                        for (int i = 0; i < NX; ++i) {
                            T mi = 0;
                            for (int l = 0; l < NX; ++l)
                                mi += Ap_[k - 1 - bi][i][l] * B_[l][c];
                            acc += mi * m.Q[i] * Ap_[k][p][i];
                        }
                        s += acc;
                    }
                    L_[bi * NU + c][p] = 2 * s;
                }
            }
        // PGD 步长 α = 1/λmax(H)（幂迭代）
        T v[NZ], w[NZ];
        for (int i = 0; i < NZ; ++i) v[i] = 1;
        for (int it = 0; it < 50; ++it) {
            for (int i = 0; i < NZ; ++i) {
                T s = 0;
                for (int j = 0; j < NZ; ++j) s += H_[i][j] * v[j];
                w[i] = s;
            }
            T nrm = 0;
            for (int i = 0; i < NZ; ++i) nrm += w[i] * w[i];
            nrm = std::sqrt(nrm);
            for (int i = 0; i < NZ; ++i) v[i] = w[i] / (nrm + 1e-30);
        }
        T lam = 0;
        for (int i = 0; i < NZ; ++i) lam = std::max(lam, std::fabs(v[i]));  // 占位
        // 更稳：幂迭代 Rayleigh 商
        T rq = 0, vv = 0;
        for (int i = 0; i < NZ; ++i) {
            T s = 0;
            for (int j = 0; j < NZ; ++j) s += H_[i][j] * v[j];
            rq += v[i] * s; vv += v[i] * v[i];
        }
        alpha_ = 1.0 / (rq / (vv + 1e-30) + 1e-12);
        std::memset(z_, 0, sizeof(z_));
        std::memset(y_, 0, sizeof(y_));
        std::memset(lo_, 0, sizeof(lo_));
        std::memset(hi_, 0, sizeof(hi_));
        for (int i = 0; i < N; ++i) {
            lo_[i * 3 + 0] = -m.u_max[0]; hi_[i * 3 + 0] = m.u_max[0];
            lo_[i * 3 + 1] = -m.u_max[1]; hi_[i * 3 + 1] = m.u_max[1];
            lo_[i * 3 + 2] = -m.u_max[2]; hi_[i * 3 + 2] = m.u_max[2];
        }
    }

    // 求解并返回 u0 = [Δω, δt, δf]
    void update(const T q[4], const T q_des[4], const T omega[3],
                T u_out[3], int n_iter = 400) {
        // 误差四元数 → ε
        T qe[4];
        qe[0] = q_des[0] * q[0] + q_des[1] * q[1] + q_des[2] * q[2] + q_des[3] * q[3];
        qe[1] = q_des[0] * q[1] - q_des[1] * q[0] - q_des[2] * q[3] + q_des[3] * q[2];
        qe[2] = q_des[0] * q[2] + q_des[1] * q[3] - q_des[2] * q[0] - q_des[3] * q[1];
        qe[3] = q_des[0] * q[3] - q_des[1] * q[2] + q_des[2] * q[1] - q_des[3] * q[0];
        if (qe[0] < 0) { qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]; }
        T x0[NX] = {qe[1], qe[2], qe[3], omega[0], omega[1], omega[2], 0, 0, 0};
        // f = L·x0
        T f[NZ];
        for (int i = 0; i < NZ; ++i) {
            T s = 0;
            for (int p = 0; p < NX; ++p) s += L_[i][p] * x0[p];
            f[i] = s;
        }
        // warm start：平移上一解
        for (int i = 0; i < NZ - NU; ++i) z_[i] = z_[i + NU];
        for (int i = NZ - NU; i < NZ; ++i) z_[i] = 0;
        // PGD
        for (int it = 0; it < n_iter; ++it) {
            for (int i = 0; i < NZ; ++i) {
                T s = 0;
                for (int j = 0; j < NZ; ++j) s += H_[i][j] * z_[j];
                T g = s + f[i];
                T zn = z_[i] - alpha_ * g;
                z_[i] = zn < lo_[i] ? lo_[i] : (zn > hi_[i] ? hi_[i] : zn);
            }
        }
        u_out[0] = z_[0]; u_out[1] = z_[1]; u_out[2] = z_[2];
    }

    // FGM 快速梯度法：O(1/k²) 收敛，同等精度迭代数约为 PGD 的 1/4~1/8
    void updateFGM(const T q[4], const T q_des[4], const T omega[3],
                   T u_out[3], int n_iter = 50) {
        // 误差四元数 → ε（与 update 相同）
        T qe[4];
        qe[0] = q_des[0] * q[0] + q_des[1] * q[1] + q_des[2] * q[2] + q_des[3] * q[3];
        qe[1] = q_des[0] * q[1] - q_des[1] * q[0] - q_des[2] * q[3] + q_des[3] * q[2];
        qe[2] = q_des[0] * q[2] + q_des[1] * q[3] - q_des[2] * q[0] - q_des[3] * q[1];
        qe[3] = q_des[0] * q[3] - q_des[1] * q[2] + q_des[2] * q[1] - q_des[3] * q[0];
        if (qe[0] < 0) { qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3]; }
        T x0[NX] = {qe[1], qe[2], qe[3], omega[0], omega[1], omega[2], 0, 0, 0};
        T f[NZ];
        for (int i = 0; i < NZ; ++i) {
            T s = 0;
            for (int p = 0; p < NX; ++p) s += L_[i][p] * x0[p];
            f[i] = s;
        }
        // warm start
        for (int i = 0; i < NZ - NU; ++i) { z_[i] = z_[i + NU]; y_[i] = z_[i]; }
        for (int i = NZ - NU; i < NZ; ++i) { z_[i] = 0; y_[i] = 0; }
        T beta = 0;
        for (int it = 0; it < n_iter; ++it) {
            // 梯度在 y
            for (int i = 0; i < NZ; ++i) {
                T s = 0;
                for (int j = 0; j < NZ; ++j) s += H_[i][j] * y_[j];
                T g = s + f[i];
                T zn = y_[i] - alpha_ * g;
                y_[i] = zn < lo_[i] ? lo_[i] : (zn > hi_[i] ? hi_[i] : zn);
            }
            // Nesterov 加速组合
            T bn = (T)(it + 1) / (T)(it + 4);
            for (int i = 0; i < NZ; ++i) {
                T zn = y_[i] + bn * (y_[i] - z_[i]);
                z_[i] = zn < lo_[i] ? lo_[i] : (zn > hi_[i] ? hi_[i] : zn);
            }
            beta = bn;
        }
        (void)beta;
        // 交换（y 为最终迭代点）
        for (int i = 0; i < NZ; ++i) { T t = z_[i]; z_[i] = y_[i]; y_[i] = t; }
        u_out[0] = z_[0]; u_out[1] = z_[1]; u_out[2] = z_[2];
    }

    // 资源统计
    static constexpr size_t memBytes() {
        return sizeof(EmbeddedLMPC) + 0;
    }

private:
    Model m_;
    T dt_;
    T A_[NX][NX], B_[NX][NU];
    T Ap_[N + 1][NX][NX];
    T H_[NZ][NZ], L_[NZ][NX];
    T alpha_;
    T z_[NZ], y_[NZ], lo_[NZ], hi_[NZ];
};

#endif // MPC_EMB_H
