/**
 * @file QMC5883L.h
 * @brief QST QMC5883L 三轴磁力计高性能非阻塞驱动
 *
 * 设计对齐本项目 IST8310 驱动的"有限状态机 + 非阻塞"风格：
 *   - begin()   阻塞式初始化（仅上电阶段调用）：软复位 -> 芯片 ID 校验 -> 配置回读校验
 *   - update()  必须在主循环中高频调用；内部按 poll 间隔轮询 DRDY，
 *               数据就绪才发起 I2C 突发读，任何路径都不阻塞主循环。
 *   - 看门狗    连续 300ms 无新数据（芯片锁死/OVL 卡死）自动软复位重配置。
 *
 * 寄存器与配置取值参照以下工程级实现交叉验证：
 *   - QMC5883L 数据手册（QST v1.0）：寄存器表、灵敏度、CTRL1 位域
 *   - Betaflight compass_qmc5883l.c（连续模式 + DRDY 轮询 + 0x0B 置 0x01 + 溢出处理，
 *     及 PR #13467 "Fix qmc5883l lockup" 的软复位防锁死策略）
 *   - MechaQMC5883 / mprograms QMC5883LCompass（Arduino 生态常用库，初始化为参考）
 *
 * 默认配置（飞行控制器场景）：
 *   ODR=100Hz、RNG=±8G、OSR=512 —— ±8G 量程防电机强磁干扰溢出(OVL)，
 *   灵敏度 3000 LSB/Gauss = 0.03333 uT/LSB，OSR=512 为最低噪声档。
 *
 * 输出：芯片坐标系 uT 向量（轴对齐到机体 FRD 由 lib/MagSensor 统一完成）。
 */

#ifndef QMC5883L_DRIVER_H
#define QMC5883L_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "MagMath.h"

/* ============================================================================
   寄存器与位域定义（参考 QMC5883L 数据手册 v1.0）
   ============================================================================ */
#define QMC5883L_I2C_ADDR_DEFAULT 0x0D //< 7 位默认地址（SETC 悬空）
#define QMC5883L_CHIP_ID_VAL 0xFF      //< 芯片 ID 寄存器 (0x0D) 内容

#define QMC5883L_REG_X_LSB 0x00 //< 数据起始地址（连续可读 6 字节）
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CTRL1 0x09
#define QMC5883L_REG_CTRL2 0x0A
#define QMC5883L_REG_SET_RESET 0x0B
#define QMC5883L_REG_CHIP_ID 0x0D

// CTRL1 (0x09) 位域
#define QMC5883L_MODE_STANDBY 0x00        //< [1:0]=00 待机
#define QMC5883L_MODE_CONTINUOUS 0x01     //< [1:0]=01 连续测量
#define QMC5883L_ODR_10HZ (0x00U << 2)    //< [3:2] 输出数据率
#define QMC5883L_ODR_50HZ (0x01U << 2)
#define QMC5883L_ODR_100HZ (0x02U << 2)
#define QMC5883L_ODR_200HZ (0x03U << 2)
#define QMC5883L_RNG_2G (0x00U << 4)      //< [5:4] 量程
#define QMC5883L_RNG_8G (0x01U << 4)
#define QMC5883L_OSR_512 (0x00U << 6)     //< [7:6] 过采样率（噪声）
#define QMC5883L_OSR_256 (0x01U << 6)
#define QMC5883L_OSR_128 (0x02U << 6)
#define QMC5883L_OSR_64 (0x03U << 6)

// CTRL2 (0x0A) 位域
#define QMC5883L_SOFT_RST 0x80 //< [7] 写 1 触发软复位（自清零）

// STATUS (0x06) 位域
#define QMC5883L_STATUS_DRDY 0x01 //< [0] 数据就绪（读数据后自动清零）
#define QMC5883L_STATUS_OVL 0x02  //< [1] 量程溢出
#define QMC5883L_STATUS_DOR 0x04  //< [2] 数据被覆盖（丢样）

/* ============================================================================
   数据类型定义
   ============================================================================ */

struct QMC5883L_Vector
{
    float x; //< 芯片 X 轴 (uT)
    float y; //< 芯片 Y 轴 (uT)
    float z; //< 芯片 Z 轴 (uT)
};

/**
 * @brief 驱动运行配置（可选的初始化参数）
 */
struct QMC5883L_Config
{
    uint8_t ctrl1 = QMC5883L_MODE_CONTINUOUS |
                    QMC5883L_ODR_100HZ |
                    QMC5883L_RNG_8G |
                    QMC5883L_OSR_512;     //< 0x19：100Hz / ±8G / OSR512
    uint8_t range_index = 1;              //< 与 ctrl1 的 RNG 位保持一致（1=±8G）
    uint16_t poll_interval_us = 3000;     //< DRDY 轮询间隔（ODR100Hz 时 3ms 足够）
    uint32_t watchdog_us = 300000;        //< 无新数据超时 -> 软复位重配置
};

class QMC5883L
{
public:
    QMC5883L();

    /**
     * @brief 初始化传感器（阻塞，仅上电阶段调用）
     * @param wire I2C 总线对象（本项目为 Wire1）
     * @param addr 7 位 I2C 地址
     * @param cfg  运行配置
     * @return true 芯片 ID 校验 + 配置回读校验通过
     */
    bool begin(TwoWire *wire = &Wire,
               uint8_t addr = QMC5883L_I2C_ADDR_DEFAULT,
               const QMC5883L_Config &cfg = QMC5883L_Config());

    /**
     * @brief 非阻塞状态机更新（主循环高频调用）
     * @return true  当且仅当本帧完成一次采样并更新数据
     * @return false 未到轮询点 / 数据未就绪 / I2C 错误（均不阻塞）
     */
    bool update();

    /**
     * @brief 获取最新磁场数据 (uT，芯片坐标系)
     */
    QMC5883L_Vector get_data() const { return _data; }

    // 诊断计数（DBG/遥测可用）
    uint32_t overflow_count() const { return _ovl_count; }  //< 量程溢出次数
    uint32_t dor_count() const { return _dor_count; }       //< 丢样次数
    uint32_t reinit_count() const { return _reinit_count; } //< 看门狗重配置次数

private:
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_reg(uint8_t reg, uint8_t &val);
    bool read_bytes(uint8_t start_reg, uint8_t *buf, uint8_t len);

    bool softReset();      //< 写 CTRL2=0x80
    bool probeChipId();    //< 读 0x0D == 0xFF
    bool applyConfig();    //< 写 0x0B=0x01 + CTRL1，并回读校验
    void armWatchdogReinit(); //< 软复位后延迟 10ms 再补配置（非阻塞）

    TwoWire *_wire;
    uint8_t _addr;
    QMC5883L_Config _cfg;

    QMC5883L_Vector _data;

    uint32_t _last_poll_us;   //< 上次 DRDY 轮询时刻
    uint32_t _last_data_us;   //< 上次成功采样时刻（0=尚未采样）
    uint32_t _reinit_pending_us; //< 非 0 时表示复位已发、等待补配置

    uint32_t _ovl_count;
    uint32_t _dor_count;
    uint32_t _reinit_count;
};

#endif // QMC5883L_DRIVER_H
