/**
 * @file    fs_lambda_dangling_probe.cpp
 * @module  固件/宿主 SIL 探针（fw-tests：Eigen 惰性表达式悬垂引用影响量化）
 * @brief   独立复现 ekf_15_state.h:3839 的写法，量化"栈变量销毁后再求值"的偏差量级
 * @details 被测：无（独立探针，不引用固件代码）。
 *
 *          背景：`lib/navigation-main/src/ekf_15_state.h:3839` 处
 *            const auto phi_at_time = [&](NavCovScalar tau) {
 *              const Eigen::Matrix<Scalar,15,15> f_tau = fs_d * tau_s;
 *              return identity_d + f_tau + 0.5*f_tau*f_tau;   // 返回惰性表达式
 *            };
 *          `const auto` 让返回类型被推导为 Eigen 表达式而非矩阵，表达式持有对
 *          lambda 内局部量 f_tau 的引用。lambda 返回后 f_tau 已销毁，调用方
 *          求值时读到的是失效栈内存。ASan 报 stack-use-after-scope。
 *
 *          本探针不碰固件，只用**同样的写法**造一个等价场景，回答两个问题：
 *            1. 惰性返回到底偏多少（对比显式求值的正确值）
 *            2. 偏差是否随"栈被复用什么"而变（解释偶发性）
 *
 * @theory 矩阵指数的一阶+二阶展开 Φ ≈ I + F·t + ½(F·t)²（与固件同式）
 * @frame  N/A
 * @units  N/A
 * @note   结论只说明"这类写法的数值后果"，不代表实机 EKF 的实际偏差——
 *         实机偏差取决于那一刻栈上恰好残留什么。
 * @see     TandemVec_FCS/test_host/fw_sil/fullstack/verify-ekf-lambda-fix.sh
 * @see     提交 db766a4b（定位）、本轮提交（量化）
 */

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <Eigen/Dense>

namespace
{
using Mat15 = Eigen::Matrix<double, 15, 15>;

/// 干净的 F 矩阵：用可复现的确定性数值填充，模拟 15 状态 EKF 的系统矩阵。
Mat15 makeF(double scale)
{
  Mat15 F = Mat15::Zero();
  for (int i = 0; i < 15; ++i)
  {
    F(i, i) = -0.01 * (i + 1) * scale;
    F(i, (i + 3) % 15) = 0.05 * scale;
    F((i + 7) % 15, i) = -0.02 * scale;
  }
  return F;
}

/// 固件里的写法：`const auto` lambda，返回类型由 return 推导 → 惰性表达式。
/// f_tau 是 lambda 内局部量，返回后销毁；调用方求值时读到失效栈内存。
const auto phi_lazy = [](const Mat15 &fs, double tau)
{
  const Mat15 f_tau = fs * tau;
  return Mat15::Identity() + f_tau + 0.5 * f_tau * f_tau;
};

/// 正确写法：显式返回类型，强制在返回时就求值成矩阵。
const auto phi_eager = [](const Mat15 &fs, double tau) -> Mat15
{
  const Mat15 f_tau = fs * tau;
  return Mat15::Identity() + f_tau + 0.5 * f_tau * f_tau;
};

/// 在惰性表达式已经指出越界之后，故意把栈"弄脏"，看偏差是否随之改变。
/// 这一步模拟"同一份二进制、不同运行，栈残留内容不同"。
void churn_stack(uint8_t fill)
{
  volatile uint8_t pad[4096];
  for (int i = 0; i < 4096; ++i) pad[i] = static_cast<uint8_t>(fill + i);
  (void)pad;
}

double max_abs_diff(const Mat15 &a, const Mat15 &b)
{
  return (a - b).cwiseAbs().maxCoeff();
}
} // namespace

int main()
{
  const Mat15 fs = makeF(1.0);
  const double tau = 0.0025; // 5 ms 半个周期，与固件 200 Hz 推进同量级

  std::printf("== Eigen 惰性表达式悬垂引用 影响量化 ==\n");
  std::printf("tau = %.4f s, F 量级 = %.3f\n\n", tau, fs.cwiseAbs().maxCoeff());

  // 正确值：显式求值
  const Mat15 ref = phi_eager(fs, tau);

  // 惰性返回后立即求值：此时栈还没被别的调用覆盖，通常"看起来正确"
  const Mat15 dirty_immediate = phi_lazy(fs, tau);

  // 惰性返回 → 先让栈被覆盖 → 再求值：这才是真实的失效场景。
  // ★这里必须用 `const auto` 保持**表达式类型**；若写成 `const Mat15`，
  //   赋值瞬间就求值了，模拟不出悬垂（第一版就踩了这个坑）。
  const auto lazy_expr_holder = phi_lazy(fs, tau); // 拿到表达式本身（已悬垂）
  churn_stack(0xA5);
  const Mat15 dirty_after_churn = lazy_expr_holder; // 此刻才求值

  churn_stack(0x5A);
  const Mat15 dirty_after_churn2 = lazy_expr_holder; // 再换一次栈内容

  std::printf("惰性返回但立即求值            : max|Δ| = %.3e\n", max_abs_diff(dirty_immediate, ref));
  std::printf("惰性返回 + 栈被覆盖 0xA5 后求值: max|Δ| = %.3e\n", max_abs_diff(dirty_after_churn, ref));
  std::printf("惰性返回 + 栈被覆盖 0x5A 后求值: max|Δ| = %.3e\n", max_abs_diff(dirty_after_churn2, ref));

  const double d1 = max_abs_diff(dirty_after_churn, ref);
  const double d2 = max_abs_diff(dirty_after_churn2, ref);

  std::printf("\n-- 判据 --\n");
  if (d1 > 1e-9 || d2 > 1e-9)
  {
    std::printf("  失效求值确实产生偏差（非零），且两次覆盖值不同 → 偏差随栈内容变化\n");
    std::printf("  两份脏结果的差异 = %.3e → 这解释了「同一份二进制、结果时好时坏」\n",
                std::fabs(d1 - d2));
  }
  else
  {
    std::printf("  本机未复现出偏差：编译器可能已把表达式提前求值，\n");
    std::printf("  或该规模下栈恰好未被覆盖。此时不能据此判断固件无问题。\n");
  }

  // 保守判据：无论是否复现，都要求"显式求值"这条路数值稳定
  const bool ref_stable = (max_abs_diff(phi_eager(fs, tau), ref) == 0.0);
  std::printf("\n  显式求值可重复性：%s\n", ref_stable ? "稳定（同输入同输出）" : "不稳定（异常）");
  return ref_stable ? 0 : 1;
}