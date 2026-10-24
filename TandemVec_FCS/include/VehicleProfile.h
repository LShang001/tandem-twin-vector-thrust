/**
 * @file    VehicleProfile.h
 * @module  固件/控制（机型与执行器输出描述，通用层）
 * @brief   用一张机型输出表描述“执行器类型/硬件槽位/量程”，支持纵列双发、
 *          多旋翼 VTVL、液体火箭等机型扩展；本层不依赖 Arduino/Eigen
 * @details 输入：编译期 `VehicleProfile`（机型名、类型、能力位、输出通道表）；
 *          处理：`validateVehicleProfile()` 检查数量/槽位/量程；
 *          输出：供分配器输出映射、遥测和地面站读取的机型描述。
 *          当前固件仍使用 `kTandemVecProfile`；其他 profile 是后续机型扩展的
 *          数据骨架，未接入控制链前不得声称已支持对应飞行器。
 * @theory  机型 = 执行器拓扑 + 输出量纲；控制律与分配器不应知道电机/舵机数量。
 *          @ref THY-003 §2–§3（执行器映射/控制分配）、CTL-001 §2（分配策略边界）
 * @frame   N/A（输出表不含坐标系；物理几何仍由 AirframeModel 的 ActuatorDef 描述）
 * @units   输出百分比 %、布尔点火 0/1；量程/零点按通道类型解释
 * @note    输出槽位是逻辑硬件槽位，不是 GPIO；GPIO 映射留在固件输出驱动层。
 *          能力位只声明“具备该能力”，不表示当前固件已实现对应模式/制导。
 * @test    test_host/test_vehicle_profile.cpp
 * @see     include/ActuatorOutputMap.h、include/AirframeModel.h、src/flight_control.cpp
 */

#ifndef TANDEMVEC_VEHICLE_PROFILE_H
#define TANDEMVEC_VEHICLE_PROFILE_H

#include <cstdint>

namespace bfs
{

constexpr uint8_t kMaxVehicleOutputs = 16;

enum class VehicleKind : uint8_t
{
    TandemVec = 0,   // 纵列双发矢量推力（当前实机）
    MultirotorVTVL,  // 多旋翼/共轴 VTVL
    LiquidRocket,    // 液体火箭（发动机+摆座/阀门/RCS）
    Count
};

enum class CapabilityFlag : uint16_t
{
    None = 0,
    DifferentialYaw = 1U << 0, // 差速偏航
    GimbalServo = 1U << 1,     // 机械摆座舵机
    EscThrottle = 1U << 2,     // 电调油门
    ThrustValve = 1U << 3,     // 推进剂/发动机阀门
    Ignition = 1U << 4,        // 点火
    Rcs = 1U << 5,             // RCS/燃气舵
    StageSeparation = 1U << 6, // 级间分离
    VariableMass = 1U << 7,    // 变质量/变惯量
};

inline uint16_t capabilityBit(const CapabilityFlag flag)
{
    return static_cast<uint16_t>(flag);
}

enum class OutputKind : uint8_t
{
    ServoPwmPct = 0, // 舵机 PWM 百分比 0–100
    EscPwmPct,       // 电调 PWM 百分比 0–100
    ValvePwmPct,     // 阀门 0–100
    IgnitionBool,    // 点火 0/1
    RcsPwmPct,       // RCS 0–100
    Count
};

struct OutputChannelSpec
{
    OutputKind kind = OutputKind::EscPwmPct;
    uint8_t hardware_slot = 0; // 逻辑硬件槽位（GPIO 映射在驱动层）
    float min_value = 0.0f;
    float max_value = 100.0f;
    float zero_value = 0.0f;   // 归一化输入为 0 时的输出
    float span = 100.0f;       // 归一化输入 1 对应的增量
};

struct VehicleProfile
{
    const char *name = "";
    VehicleKind kind = VehicleKind::TandemVec;
    uint16_t capabilities = 0;
    const OutputChannelSpec *outputs = nullptr;
    uint8_t output_count = 0;
};

enum class VehicleProfileError : uint8_t
{
    Ok = 0,
    NullName,
    NoOutputs,
    TooManyOutputs,
    DuplicateSlot,
    InvalidKind,
    InvalidRange,
};

inline const char *vehicleProfileErrorName(const VehicleProfileError error)
{
    switch (error)
    {
    case VehicleProfileError::Ok:
        return "ok";
    case VehicleProfileError::NullName:
        return "null_name";
    case VehicleProfileError::NoOutputs:
        return "no_outputs";
    case VehicleProfileError::TooManyOutputs:
        return "too_many_outputs";
    case VehicleProfileError::DuplicateSlot:
        return "duplicate_slot";
    case VehicleProfileError::InvalidKind:
        return "invalid_kind";
    case VehicleProfileError::InvalidRange:
        return "invalid_range";
    default:
        return "unknown";
    }
}

/**
 * @brief 校验机型输出表：名称、数量、槽位唯一、类型和量程
 * @param[in] profile 机型描述
 * @return 错误码；Ok 才能用于输出映射
 * @note 纯函数、无堆分配；输出槽位必须唯一，避免两路执行器写同一硬件槽。
 */
inline VehicleProfileError validateVehicleProfile(const VehicleProfile &profile)
{
    if (profile.name == nullptr || profile.name[0] == '\0')
    {
        return VehicleProfileError::NullName;
    }
    if (profile.outputs == nullptr || profile.output_count == 0U)
    {
        return VehicleProfileError::NoOutputs;
    }
    if (profile.output_count > kMaxVehicleOutputs)
    {
        return VehicleProfileError::TooManyOutputs;
    }
    bool used[kMaxVehicleOutputs] = {};
    for (uint8_t i = 0; i < profile.output_count; ++i)
    {
        const OutputChannelSpec &channel = profile.outputs[i];
        if (channel.kind >= OutputKind::Count)
        {
            return VehicleProfileError::InvalidKind;
        }
        if (channel.hardware_slot >= kMaxVehicleOutputs ||
            used[channel.hardware_slot])
        {
            return VehicleProfileError::DuplicateSlot;
        }
        used[channel.hardware_slot] = true;
        if (!(channel.min_value <= channel.zero_value &&
              channel.zero_value <= channel.max_value) ||
            channel.span < 0.0f)
        {
            return VehicleProfileError::InvalidRange;
        }
    }
    return VehicleProfileError::Ok;
}

// ---- 当前纵列双发实机：2 舵机 + 2 电调 ----
static constexpr OutputChannelSpec kTandemVecOutputs[] = {
    {OutputKind::ServoPwmPct, 0U, 0.0f, 100.0f, 50.0f, 50.0f}, // 上摆/滚转
    {OutputKind::ServoPwmPct, 1U, 0.0f, 100.0f, 50.0f, 50.0f}, // 下摆/俯仰
    {OutputKind::EscPwmPct, 2U, 0.0f, 100.0f, 0.0f, 100.0f},   // 前电机
    {OutputKind::EscPwmPct, 3U, 0.0f, 100.0f, 0.0f, 100.0f},   // 尾电机
};

static constexpr VehicleProfile kTandemVecProfile = {
    "TandemVec",
    VehicleKind::TandemVec,
    static_cast<uint16_t>(CapabilityFlag::DifferentialYaw) |
        static_cast<uint16_t>(CapabilityFlag::GimbalServo) |
        static_cast<uint16_t>(CapabilityFlag::EscThrottle),
    kTandemVecOutputs,
    static_cast<uint8_t>(sizeof(kTandemVecOutputs) / sizeof(kTandemVecOutputs[0])),
};

// ---- 多旋翼 VTVL 示例：4 电调、无舵机；仅用于扩展表/测试，当前固件未接 ----
static constexpr OutputChannelSpec kQuadVtvOutputs[] = {
    {OutputKind::EscPwmPct, 0U, 0.0f, 100.0f, 0.0f, 100.0f},
    {OutputKind::EscPwmPct, 1U, 0.0f, 100.0f, 0.0f, 100.0f},
    {OutputKind::EscPwmPct, 2U, 0.0f, 100.0f, 0.0f, 100.0f},
    {OutputKind::EscPwmPct, 3U, 0.0f, 100.0f, 0.0f, 100.0f},
};

static constexpr VehicleProfile kQuadVtvProfile = {
    "QuadVTVL",
    VehicleKind::MultirotorVTVL,
    static_cast<uint16_t>(CapabilityFlag::EscThrottle),
    kQuadVtvOutputs,
    static_cast<uint8_t>(sizeof(kQuadVtvOutputs) / sizeof(kQuadVtvOutputs[0])),
};

// ---- 液体火箭输出示例：阀门/点火/双摆座/RCS；当前固件未接 ----
static constexpr OutputChannelSpec kLiquidRocketOutputs[] = {
    {OutputKind::ValvePwmPct, 0U, 0.0f, 100.0f, 0.0f, 100.0f},
    {OutputKind::IgnitionBool, 1U, 0.0f, 1.0f, 0.0f, 1.0f},
    {OutputKind::ServoPwmPct, 2U, 0.0f, 100.0f, 50.0f, 50.0f},
    {OutputKind::ServoPwmPct, 3U, 0.0f, 100.0f, 50.0f, 50.0f},
    {OutputKind::RcsPwmPct, 4U, 0.0f, 100.0f, 0.0f, 100.0f},
    {OutputKind::RcsPwmPct, 5U, 0.0f, 100.0f, 0.0f, 100.0f},
};

static constexpr VehicleProfile kLiquidRocketProfile = {
    "LiquidRocket",
    VehicleKind::LiquidRocket,
    static_cast<uint16_t>(CapabilityFlag::ThrustValve) |
        static_cast<uint16_t>(CapabilityFlag::Ignition) |
        static_cast<uint16_t>(CapabilityFlag::GimbalServo) |
        static_cast<uint16_t>(CapabilityFlag::Rcs) |
        static_cast<uint16_t>(CapabilityFlag::VariableMass),
    kLiquidRocketOutputs,
    static_cast<uint8_t>(sizeof(kLiquidRocketOutputs) /
                         sizeof(kLiquidRocketOutputs[0])),
};

} // namespace bfs

#endif // TANDEMVEC_VEHICLE_PROFILE_H
