// ============================================================
//  TandemVec_Config.h — 纵列双发矢量推力飞行器机型参数
//
//  本文件是 TandemVec_FCS 飞控的单一参数源头文件。
//  所有数值均来源于上层项目 models/aircraft-model.json (PAR-C0-001)，
//  状态 MODEL-DEFAULT，未经台架标定。
//
//  修改规则：
//    ① 不得在本文件以外的地方硬编码任何飞行器参数；
//    ② 上层 aircraft-model.json 发生变化时同步更新本文件，
//       并在 commit 说明中注明来源版本；
//    ③ 非运行时可变量（标定系数）单独存入 EEPROM/FLASH，
//       通过 override 机制覆盖此处默认值。
//
//  使用方式：
//    TandemVecParams p = kDefaultTandemVecParams;
//    // 如有标定值：p.kT = calibrated_kT;
// ============================================================
#pragma once

struct TandemVecParams
{
    // ===== 推进系统 ===========================================
    // 来源: models/aircraft-model.json §推进
    float kT;    // N·s²     推力系数  T = kT·ω²
    float kQ;    // N·m·s²   反扭系数  τ = kQ·ω²  (稳态 ω̇=0)
    float Jp;    // kg·m²    桨+转子转动惯量（用于陀螺项与反扭瞬态 τ=kQ·ω²+Jp·ω̇）
    float wMax;  // rad/s    最大转速
    float a;     // m        前电机到质心距离（推力臂，正值）
    float b;     // m        尾电机到质心距离（推力臂，正值）
    float tauM;  // s        电机一阶滞后时间常数

    // ===== 惯量 / 执行机构 ====================================
    // 来源: models/aircraft-model.json §惯量 / 执行机构
    float Ix;    // kg·m²    机体滚转惯量
    float Iy;    // kg·m²    机体俯仰惯量
    float Iz;    // kg·m²    机体偏航惯量
    float dMax;  // rad      摆角限幅（±25°）

    // ===== 控制分配专用 =======================================
    // 来源: models/aircraft-model.json §SAS 增稳
    float dwMax; // -        差速指令限幅 Δω ∈ [-dwMax, +dwMax]

    // ===== 质量 / 重力 ========================================
    // 来源: models/aircraft-model.json §质量 / 气动
    float m;     // kg       质量
    float g;     // m/s²     重力加速度
};

// 默认参数（来源：2212 1400KV + 9047桨 + 3S锂电实测/估算，待台架标定精化）
// 更新记录：kT/kQ/wMax 根据官方推力数据(T_max=1.4kg)重算，20260728
static const TandemVecParams kDefaultTandemVecParams = {
    // 推进
    /* kT    */ 1.04e-5f,  // N·s²，由 T_max=13.73N @1150rad/s 反推
    /* kQ    */ 3.1e-7f,   // N·m·s²，kQ/kT≈0.030（9047桨 D=0.229m，CQ/CT≈0.13）
    /* Jp    */ 2.0e-4f,   // kg·m²，桨+转子惯量，待测
    /* wMax  */ 1150.0f,   // rad/s，≈11000RPM，1400KV×11.1V×0.75（带桨负载）
    /* a     */ 0.62f,     // m，前电机到质心距离，待测量实机
    /* b     */ 0.62f,     // m，尾电机到质心距离，待测量实机
    /* tauM  */ 0.28f,     // s，电机一阶滞后，待测
    // 惯量 / 执行机构
    /* Ix    */ 0.09f,
    /* Iy    */ 0.34f,
    /* Iz    */ 0.36f,
    /* dMax  */ 0.4363323f,   // 25° in rad
    // 控制分配
    /* dwMax */ 0.7f,
    // 质量 / 重力
    /* m     */ 2.6f,
    /* g     */ 9.81f,
};
