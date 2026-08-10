// ============================================================
//  test_rc_curve.cpp — FPV 摇杆曲线（RcCurve.h）host 测试
//
//  纯平台无关：验证 Betaflight 三参数模型——
//    expo（中心压低，满杆增益恰 1）
//    super（边缘双曲放大，低杆量几乎无感、满杆 1/(1-super) 倍）
//    rc_rate（整条曲线线性缩放）
//    死区/限幅/极点保护
// ============================================================
#include <cstdio>
#include <cmath>
#include "RcCurve.h"

static int g_fail = 0;
#define CHECK(cond, msg) \
  do { if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)
#define CLOSE(a, b, tol) (fabsf((a) - (b)) <= (tol))

int main()
{
    // ---- 1. 归一化 ----
    CHECK(rcNormFromUs(1500.0f) == 0.0f, "中位 0");
    CHECK(CLOSE(rcNormFromUs(2012.0f), 1.0f, 1e-6), "满杆 +1");
    CHECK(CLOSE(rcNormFromUs(988.0f), -1.0f, 1e-6), "满杆 -1");

    // ---- 2. 死区 ----
    CHECK(rcRateCurve(0.01f, 1.0f, 0.0f, 0.0f, 450.0f) == 0.0f, "死区内归零");
    CHECK(rcRateCurve(0.03f, 1.0f, 0.0f, 0.0f, 450.0f) > 0.0f, "死区外生效");

    // ---- 3. 线性基线（无 expo/super）：rate = rc·rcRate·200 ----
    CHECK(CLOSE(rcRateCurve(1.0f, 1.0f, 0.0f, 0.0f, 450.0f), 200.0f, 1e-3), "满杆基准 200°/s");
    CHECK(CLOSE(rcRateCurve(0.5f, 1.0f, 0.0f, 0.0f, 450.0f), 100.0f, 1e-3), "半杆 100°/s");
    CHECK(CLOSE(rcRateCurve(0.5f, 2.0f, 0.0f, 0.0f, 450.0f), 200.0f, 1e-3), "rc_rate 线性缩放");
    CHECK(CLOSE(rcRateCurve(1.0f, 1.0f, 0.0f, 0.0f, 150.0f), 150.0f, 1e-3), "上限限幅");

    // ---- 4. expo 特性：中心压低、满杆增益恰 1 ----
    // expo=0.5：rc=0.5 → 0.5·(0.5·0.25+0.5)=0.5·0.625=0.3125 → 62.5°/s（线性为 100）
    float c = rcRateCurve(0.5f, 1.0f, 0.5f, 0.0f, 450.0f);
    CHECK(CLOSE(c, 62.5f, 1e-3), "expo=0.5 中心压低（半杆 62.5 vs 线性 100）");
    // 满杆增益恰 1：expo 不影响满杆
    CHECK(CLOSE(rcRateCurve(1.0f, 1.0f, 0.5f, 0.0f, 450.0f), 200.0f, 1e-3), "expo 满杆增益=1");
    // expo 单调性：中心增益 = (1-expo)
    float g_center = rcRateCurve(0.1f, 1.0f, 0.5f, 0.0f, 450.0f) / (0.1f * 200.0f);
    CHECK(CLOSE(g_center, 0.505f, 0.02f), "expo 中心增益=expo·rc²+(1-expo)=0.505");

    // ---- 5. super 特性：边缘放大、低杆量几乎无感 ----
    // super=0.7 满杆：200/(1-0.7) = 666.7
    CHECK(CLOSE(rcRateCurve(1.0f, 1.0f, 0.0f, 0.7f, 1000.0f), 666.67f, 1.0f), "super=0.7 满杆 667°/s");
    // 低杆量：super=0.7 时 rc=0.2 → 1/(1-0.14)=1.16（+16%）
    float low = rcRateCurve(0.2f, 1.0f, 0.0f, 0.7f, 1000.0f);
    CHECK(CLOSE(low, 40.0f * 1.1628f, 1.0f), "super 低杆量仅 +16%");
    // 中点：rc=0.5 → 1/(1-0.35)=1.54
    float mid = rcRateCurve(0.5f, 1.0f, 0.0f, 0.7f, 1000.0f);
    CHECK(CLOSE(mid, 100.0f * 1.5385f, 1.0f), "super 中杆量 +54%");
    // super 不影响中心（rc→0 时增益→1）
    CHECK(CLOSE(rcRateCurve(0.05f, 1.0f, 0.0f, 0.7f, 1000.0f), 10.0f * 1.036f, 0.5f), "super 中心≈线性");

    // ---- 6. 默认参数组合（expo=0.2, super 0.7）----
    // 满杆：expo 不影响满杆 → 667
    CHECK(CLOSE(rcRateCurve(1.0f, 1.0f, 0.2f, 0.7f, 1000.0f), 666.67f, 1.0f), "默认组合满杆 667");
    // 半杆：rcCmd=0.5·(0.2·0.25+0.8)=0.425 → 85·1/(1-0.425·0.7)=85·1.424=121
    float half = rcRateCurve(0.5f, 1.0f, 0.2f, 0.7f, 1000.0f);
    CHECK(CLOSE(half, 121.0f, 2.0f), "默认组合半杆 ≈121°/s");

    // ---- 7. 对称性（负杆量）----
    CHECK(CLOSE(rcRateCurve(-0.5f, 1.0f, 0.2f, 0.7f, 1000.0f),
                -rcRateCurve(0.5f, 1.0f, 0.2f, 0.7f, 1000.0f), 1e-3), "负杆量对称");

    // ---- 8. 极点保护（super 0.95 不发散）----
    float extreme = rcRateCurve(1.0f, 1.0f, 0.0f, 0.95f, 1000.0f);
    CHECK(extreme <= 1000.0f && extreme > 0.0f, "super 0.95 极点保护限幅");

    if (g_fail == 0)
    {
        printf("test_rc_curve: ALL PASSED\n");
        return 0;
    }
    printf("test_rc_curve: %d FAILED\n", g_fail);
    return 1;
}
