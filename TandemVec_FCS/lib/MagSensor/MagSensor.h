/**
 * @file MagSensor.h
 * @brief 磁力计统一管理层：自动探测 + 非阻塞轮询 + 标定 + 统一 FRD 输出
 *
 * 解决"接入不同磁力计后都能直接用起来"的问题：
 *   - 上电时按顺序探测 IST8310 (0x0E) -> QMC5883L (0x0D)，命中即用；
 *   - 两个芯片驱动各自负责芯片坐标系 uT 数据采集；
 *   - 本层统一完成 芯片系 -> 机体 FRD(前-右-下) 的轴对齐（矩阵见 MagMath.h）
 *     与椭球标定（MagCalMath.h），上层（Madgwick / EKF 磁矢量融合）只看到 FRD 数据。
 *
 * 标定管线（数据顺序，均在芯片系进行，与轴映射解耦）：
 *   原始芯片样本 --(MagCalCollector 采集)--> 椭球拟合(硬铁+软铁)
 *   芯片数据 --(applyCal)--> 校正芯片数据 --(轴对齐)--> 机体 FRD 输出
 *
 * 更换/新增磁力计型号只需：
 *   1. lib/ 下新增芯片驱动（begin/update/get_data 三件套，非阻塞）；
 *   2. MagMath.h 增加该芯片的轴映射常量表；
 *   3. begin() 增加一路探测。
 *
 * 注意：默认只挂载一个磁力计模块。若两个芯片同时挂在同一 I2C 总线上，
 * 探测顺序优先 IST8310（本机默认硬件）；需要改优先级时调整 begin() 内顺序。
 */

#ifndef MAG_SENSOR_MANAGER_H
#define MAG_SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "IST8310.h"
#include "QMC5883L.h"
#include "MagMath.h"
#include "MagCalMath.h"
#include "MagCalGuide.h"

/**
 * @brief 当前接线的磁力计型号
 */
enum class MagChip : uint8_t
{
    NONE = 0,   //< 未检测到任何磁力计
    IST8310,    //< iSentek IST8310 (0x0E)
    QMC5883L,   //< QST QMC5883L (0x0D)
};

/**
 * @brief 统一磁力计样本（机体 FRD 系，uT）
 */
struct MagSample
{
    float x = 0.0f;             //< 机体前 (uT)
    float y = 0.0f;             //< 机体右 (uT)
    float z = 0.0f;             //< 机体下 (uT)
    uint32_t seq = 0;           //< 样本序号（自增；0 = 从未采样）
    uint32_t timestamp_us = 0;  //< 采样时刻 micros()，用于导航新鲜度门控
};

class MagSensor
{
public:
    MagSensor();

    /**
     * @brief 上电自动探测并初始化（阻塞，仅初始化阶段调用）
     * @param wire I2C 总线（本项目为 Wire1，400kHz）
     * @return true 探测到受支持的磁力计并初始化成功
     */
    bool begin(TwoWire *wire);

    /**
     * @brief 非阻塞轮询（主循环高频调用；内部驱动 FSM 限速）
     * @return true 本帧产生新样本
     */
    bool update();

    MagChip chip() const { return _chip; }
    bool isPresent() const { return _chip != MagChip::NONE; }

    /**
     * @brief 型号名称（日志打印用）
     */
    const char *chipName() const;

    /**
     * @brief 最新样本（机体 FRD，uT，已含标定；seq==0 表示尚无有效数据）
     */
    MagSample sampleFRD() const { return _last; }
    bool fresh() const { return _last.seq != 0; }
    uint32_t lastUpdateUs() const { return _last.timestamp_us; }

#ifdef FW_SIL_REAL_NAV
    /**
     * @brief SIL 专用：注入已标定/已轴对齐的 FRD 样本（生产路径不调用）
     * @param frd          机体系 FRD 磁场 [uT]
     * @param timestamp_us 虚拟/真实 micros() 时间戳
     * @param cal_valid    是否模拟“已有有效 Flash 标定”
     * @return 输入有限且注入成功
     * @note 仅用于真实固件 SIL 把传感器模型接到 MagSensor，绕过 I2C 驱动；
     *       生产固件不编译此接口。
     */
    bool silInjectFrdSample(const float frd[3], uint32_t timestamp_us,
                            bool cal_valid)
    {
        if (!std::isfinite(frd[0]) || !std::isfinite(frd[1]) ||
            !std::isfinite(frd[2]))
        {
            return false;
        }
        _chip = MagChip::IST8310; // 标记“已探测到”，供导航层走真实路径
        _cal.identity();
        _cal.scale = 50.0f; // 模拟一次通过质量门的物理场强标定
        _cal.valid = cal_valid;
        _last = {frd[0], frd[1], frd[2], ++_seq, timestamp_us};
        return true;
    }
#endif

    // ========================================================================
    // 椭球标定（芯片系；与 MagCalStore 持久化配合使用）
    // ========================================================================

    /**
     * @brief 开始采样采集（清空旧样本；采样随 update() 自动进行，
     *        抽稀：≥20ms 间隔 + ≥0.5uT 间距，上限 400 点）
     */
    bool calStart();

    void calStop() { _calCollecting = false; }
    bool calCollecting() const { return _calCollecting; }
    uint16_t calSampleCount() const { return _collector.count(); }
    void calSampleMinMax(float mn[3], float mx[3]) const { _collector.sampleMinMax(mn, mx); }

    /**
     * @brief 椭球拟合（质量门全过才返回 true；不自动应用，由调用方 setCal）
     */
    bool calFit(magcal::MagCalParams &out, magcal::MagCalStats &stats) const
    {
        return _collector.fit(out, stats);
    }

    /**
     * @brief 应用标定（参数合理性校验后生效；立即影响 sampleFRD 输出）
     */
    bool setCal(const magcal::MagCalParams &cal);

    magcal::MagCalParams calParams() const { return _cal; }
    bool calValid() const { return _cal.valid; }

    /**
     * @brief 清空标定（恢复未标定直通状态）
     */
    void calClear();

    // ========================================================================
    // 大疆式两圈引导校准（DBG `magcal guided`；状态机见 MagCalGuide.h）
    // ========================================================================

    /**
     * @brief 启动引导校准（清空采集器，进入阶段1 水平旋转；与自由采集互斥）
     */
    bool guideStart();

    void guideStop();
    bool guideActive() const { return _guideActive; }

    /**
     * @brief 引导状态机推进（每 IMU 帧调用，通常挂在 readIMUData 内）
     * @param accFrd  机体 FRD 加速度 (m/s²)
     * @param gyroFrd 机体 FRD 角速度 (rad/s)
     * @param nowUs   时间戳（采样抽稀用）
     * 两圈转满后自动执行椭球拟合并应用（一次性 ~ms 级，仅完成瞬间）。
     */
    void guideUpdate(const float accFrd[3], const float gyroFrd[3], uint32_t nowUs);

    magcal::GuideStage guideStage() const { return _guide.stage(); }
    uint8_t guideSectorsDone() const { return _guide.sectorsDone(); }

    /** DONE/FAILED 后为 true：拟合已执行，结果可读 */
    bool guideResultReady() const { return _guideFitReady; }
    bool guideFitOk() const { return _guideFitOk; }
    const magcal::MagCalStats &guideFitStats() const { return _guideFitStats; }

    // 诊断计数透传（QMC5883L；IST8310 无对应项时恒 0）
    uint32_t overflowCount() const;
    uint32_t dorCount() const;
    uint32_t reinitCount() const;

private:
    TwoWire *_wire = nullptr;
    MagChip _chip = MagChip::NONE;
    IST8310 _ist;
    QMC5883L _qmc;
    MagSample _last{};
    uint32_t _seq = 0;

    // 椭球标定（芯片系）
    magcal::MagCalCollector _collector{};
    magcal::MagCalParams _cal{};
    bool _calCollecting = false;

    // 大疆式引导校准
    magcal::MagCalGuide _guide{};
    bool _guideActive = false;
    bool _guideFitReady = false;
    bool _guideFitOk = false;
    magcal::MagCalStats _guideFitStats{};
    float _lastChip[3] = {0.0f, 0.0f, 0.0f}; //< 最近一次芯片系原始样本（引导采集用）
    bool _lastChipValid = false;
};

#endif // MAG_SENSOR_MANAGER_H
