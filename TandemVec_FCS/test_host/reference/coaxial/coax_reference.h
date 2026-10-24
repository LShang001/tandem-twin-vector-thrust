/**
 * @file    coax_reference.h
 * @module  固件/宿主参考（实飞共轴 DirectTvcRate 输出级参考）
 * @brief   声明实飞共轴工程姿态/速率/偏航控制链的参考输入输出与入口
 * @details 实现见 coax_reference.cpp：逐行复刻外部 VTVL_ElectricDualRotor_FCS
 *          的 execute_attitude_controller/execute_yaw_controller（PositionPID +
 *          ComplementaryFilter + 固定增益/限幅）。仅用于与统一固件
 *          CoaxialVtvControl 做逐拍输出对比，不参与固件编译。
 * @theory  ω_ref=PID_att(e,−ω)；u=PID_rate(ω_ref,ω)；yaw=PID_yaw(ω_ref,ω_z)
 *          @ref CTL-003 §2.2
 * @frame   存档系 x=滚转、y=俯仰、z=推力轴
 * @units   deg、deg/s、deg、us、s
 * @note    参数/限幅/滤波全部来自外部实飞工程；参考实现不写 GPIO。
 * @test    test_host/test_coaxial_reference_equivalence.cpp
 */
#ifndef COAX_REFERENCE_H
#define COAX_REFERENCE_H

struct CoaxRefInput
{
    float error_roll_deg;
    float error_pitch_deg;
    float rate_roll_dps;
    float rate_pitch_dps;
    float rate_yaw_dps;
    float yaw_rate_target_dps;
    int rate_mode;
    float roll_rate_target_dps;
    float pitch_rate_target_dps;
    int attitude_active;
    int yaw_active;
    int manual_tvc;
    float manual_roll_deg;
    float manual_pitch_deg;
    float dt_s;
};

struct CoaxRefOutput
{
    float roll_correction_deg;
    float pitch_correction_deg;
    float yaw_diff_us;
};

void coax_ref_reset();
void coax_ref_update(const CoaxRefInput *in, CoaxRefOutput *out);

#endif // COAX_REFERENCE_H
