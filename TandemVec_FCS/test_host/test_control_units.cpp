#include "ControlUnits.h"
#include "FlightCtrlParams.h"
#include "PositionPID.h"
#include "ComplementaryFilter.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;

static void check(bool condition, const char *name)
{
    std::printf(condition ? "[PASS] %s\n" : "[FAIL] %s\n", name);
    if (!condition) ++g_fail;
}

static bool near(float a, float b, float tol = 2.0e-5f)
{
    return std::fabs(a - b) <= tol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

int main()
{
    check(near(ControlUnits::dps2ToRadps2(ControlUnits::kDegPerRad), 1.0f),
          "deg/s² 与 rad/s² 双向换算");
    check(near(kFlightCtrlParams.rate_roll.kp * ControlUnits::kRadPerDeg, 0.28f),
          "roll Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_pitch.kp * ControlUnits::kRadPerDeg, 0.28f),
          "pitch Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_yaw.kp * ControlUnits::kRadPerDeg, 0.20f),
          "yaw Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_roll.ki * ControlUnits::kRadPerDeg, 0.10f),
          "Ki 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_roll.out_max * ControlUnits::kRadPerDeg, 100.0f),
          "输出限幅迁移保持 rad/s² 物理上限");
    check(near(kFlightCtrlParams.rate_roll.int_limit * ControlUnits::kRadPerDeg, 20.0f),
          "积分限幅迁移保持 rad/s² 物理上限");

    PositionPID old_pid(0.28f, 0.10f, 0.0f, -100.0f, 100.0f,
                        true, 20.0f, 60.0f, 0.2f);
    PositionPID new_pid(kFlightCtrlParams.rate_roll.kp,
                        kFlightCtrlParams.rate_roll.ki,
                        kFlightCtrlParams.rate_roll.kd,
                        kFlightCtrlParams.rate_roll.out_min,
                        kFlightCtrlParams.rate_roll.out_max,
                        true,
                        kFlightCtrlParams.rate_roll.int_limit,
                        kFlightCtrlParams.rate_roll.threshold,
                        kFlightCtrlParams.rate_roll.filter_alpha);

    bool sequence_equal = true;
    const float refs[] = {0.0f, 15.0f, 40.0f, 40.0f, -20.0f, 0.0f};
    const float meas[] = {0.0f, 1.0f, 8.0f, 20.0f, 5.0f, -2.0f};
    for (int i = 0; i < 6; ++i)
    {
        const float old_radps2 = old_pid.computeDerivativeOnMeasurement(refs[i], meas[i], 0.005f);
        const float new_radps2 = ControlUnits::dps2ToRadps2(
            new_pid.computeDerivativeOnMeasurement(refs[i], meas[i], 0.005f));
        sequence_equal &= near(old_radps2, new_radps2, 5.0e-5f);
    }
    check(sequence_equal, "PID 动态序列迁移前后 rad/s² 输出等价");

    PositionPID old_long(0.28f, 0.10f, 0.0f, -100.0f, 100.0f,
                         true, 20.0f, 60.0f, 0.2f);
    PositionPID new_long(kFlightCtrlParams.rate_roll.kp,
                         kFlightCtrlParams.rate_roll.ki,
                         kFlightCtrlParams.rate_roll.kd,
                         kFlightCtrlParams.rate_roll.out_min,
                         kFlightCtrlParams.rate_roll.out_max,
                         true,
                         kFlightCtrlParams.rate_roll.int_limit,
                         kFlightCtrlParams.rate_roll.threshold,
                         kFlightCtrlParams.rate_roll.filter_alpha);
    ComplementaryFilter old_filter(0.9f), new_filter(0.9f);
    float max_abs_error = 0.0f;
    bool long_sequence_equal = true;
    for (int i = 0; i < 5000; ++i)
    {
        const float t = i * 0.005f;
        const float ref = ((i / 400) % 2 == 0 ? 35.0f : -20.0f) + 8.0f * std::sin(0.7f * t);
        const float meas = 18.0f * std::sin(1.3f * t) + 3.0f * std::cos(0.17f * t);
        const float old_radps2 = old_filter.filter(
            old_long.computeDerivativeOnMeasurement(ref, meas, 0.005f));
        const float new_radps2 = ControlUnits::dps2ToRadps2(new_filter.filter(
            new_long.computeDerivativeOnMeasurement(ref, meas, 0.005f)));
        const float abs_error = std::fabs(old_radps2 - new_radps2);
        if (abs_error > max_abs_error) max_abs_error = abs_error;
        long_sequence_equal &= near(old_radps2, new_radps2, 1.0e-4f);
    }
    std::printf("[INFO] 5000 拍含输出滤波最大物理输出差 = %.6g rad/s²\n", max_abs_error);
    check(long_sequence_equal, "PID+输出滤波长序列迁移前后物理输出等价");

    std::printf(g_fail == 0 ? "test_control_units: ALL PASSED\n"
                            : "test_control_units: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
