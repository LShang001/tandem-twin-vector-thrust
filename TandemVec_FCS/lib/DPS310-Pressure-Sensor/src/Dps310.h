#ifndef DPS310_H_INCLUDED // 防止头文件重复包含的预处理指令，如果 DPS310_H_INCLUDED 尚未定义
#define DPS310_H_INCLUDED // 定义 DPS310_H_INCLUDED 宏

#include "DpsClass.h"           // 包含 DpsClass 类的头文件，DpsClass 是气压传感器的抽象基类
#include "util/dps310_config.h" // 包含 DPS310 相关的配置信息

// 定义 DPS310 类，该类继承自 DpsClass 抽象基类
class Dps310 : public DpsClass
{
public:
  /**
   * @brief 获取连续测量模式下的温度和压力数据。
   *
   * 从传感器的 FIFO 中读取连续测量模式下的温度和压力数据，
   * 将原始数据转换为实际的物理量（例如温度单位为摄氏度，压力单位为帕斯卡），
   * 并将数据存储在用户提供的缓冲区中。
   *
   * @param tempBuffer 指向用于存储温度数据的浮点数数组的指针。
   * @param tempCount  引用类型，表示温度数据的数量，同时作为返回值表示实际读取到的温度数据数量。
   * @param prsBuffer  指向用于存储压力数据的浮点数数组的指针。
   * @param prsCount   引用类型，表示压力数据的数量，同时作为返回值表示实际读取到的压力数据数量。
   * @return int16_t  返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t getContResults(float *tempBuffer, uint8_t &tempCount, float *prsBuffer, uint8_t &prsCount);

  /**
   * @brief 获取最新的温度和压力测量结果。
   *
   * 此函数从 DPS310 传感器的 FIFO 中读取最新的温度和压力数据，即使 FIFO 中有多个数据，也只返回最新的结果。
   * 函数会先检查 FIFO 是否溢出，如果溢出，则返回错误代码。
   * 然后检查 FIFO 是否有新数据，如果有，则持续读取直到 FIFO 为空，并更新最新的温度和压力值及其可用标志。
   * 最后，根据标志位判断是否成功读取到数据，并计算实际的温度和压力值。
   * 如果没有任何可用的新数据，则返回 DPS__FAIL_UNFINISHED。
   * 此函数适用于低延迟获取最新测量结果的应用场景，
   * 能够兼容不同的温度和压力采样率，避免 FIFO 积压时持续读到旧样本。
   *
   * @param temp 引用类型，用于存储最新的温度值，单位：摄氏度 (℃)。
   * @param pressure 引用类型，用于存储最新的压力值，单位：帕斯卡 (Pa)。
   *
   * @return int16_t 返回操作结果状态码：
   *         - DPS__SUCCEEDED (0): 成功获取到最新的温度和压力数据。
   *         - DPS__FAIL_INIT_FAILED (-2): DPS310 传感器初始化失败。
   *         - DPS__FAIL_TOOBUSY (-3): DPS310 传感器不在后台模式。
   *         - DPS__FAIL_UNFINISHED (-4): 没有新的数据可用。
   *         - DPS__FAIL_UNKNOWN (-1): 读取数据失败。
   *         - DPS__FAIL_OVERFLOW (-5): FIFO 溢出。
   *
   * @note
   *   - 此函数假设 DPS310 传感器已经正确初始化并在后台模式下运行。
   *   - 调用此函数前，应确保已根据需要配置好温度和压力的采样率。
   *   - 如果 FIFO 发生溢出，函数返回错误，由上层决定是否清空 FIFO。
   *   - 即使温度和压力采样率不同，此函数也能正常工作。
   *   - 如果应用场景需要更高的实时性，建议考虑使用中断方式读取数据。
   */
  int16_t getLatestResults(float &temp, float &pressure);

#ifdef BFS_DPS310_DIAG
  /**
   * @brief 诊断构建下读取原始寄存器值。
   *
   * 仅供上板 profile 定位 I2C/FIFO 状态使用，正式固件不启用。
   */
  int16_t readDebugRegister(uint8_t regAddress);
#endif

  /**
   * @brief 设置中断源和中断极性。
   *
   * 配置 DPS310 传感器的中断功能，例如在 FIFO 满或新的测量值可用时产生中断信号。
   * 设置中断信号的极性，例如高电平有效或低电平有效。
   *
   * @param intr_source 表示中断源的无符号 8 位整数，通常使用枚举类型定义，例如 Interrupt_source_310_e。
   * @param polarity    表示中断极性的无符号 8 位整数，默认为 1，通常 1 表示高电平有效，0 表示低电平有效。
   * @return int16_t    返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t setInterruptSources(uint8_t intr_source, uint8_t polarity = 1);

  /**
   * @brief 清空 FIFO 缓冲区
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 用于清空传感器内部的 FIFO 缓冲区。
   *
   * @return int16_t 返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t flushFIFO();

  /**
   * @brief 根据当前气压和温度计算海拔高度（考虑温度梯度的公式）。
   *
   * 使用气压高度公式并考虑温度梯度校正。
   *
   * @param pressure 当前气压值，单位：帕斯卡。
   * @param temperature 当前温度值，单位：摄氏度。
   * @return 海拔高度，单位：米。
   */
  float calculateAltitude(float pressure, float temperature);

  /**
   * 计算简化版的海拔高度
   *
   * 此函数基于气压值计算海拔高度，采用了一种简化的模型
   * 适用于对精度要求不高的应用场景
   *
   * @param pressure 海平面气压值，单位为百帕（hPa）
   * @return 海拔高度，单位为米（m）
   */
  float calculateAltitudeSimplified(float pressure);

protected:
  uint8_t m_tempSensor; // 成员变量，用于存储温度传感器的选择信息

  // 补偿系数，用于校准传感器读数
  int32_t m_c0Half; // 温度补偿系数 C0 的一半
  int32_t m_c1;     // 温度补偿系数 C1

  /////// 实现纯虚函数 ///////

  /**
   * @brief 初始化 DPS310 传感器。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 用于执行 DPS310 传感器的初始化操作，例如设置通信接口、配置寄存器等。
   */
  void init(void);

  /**
   * @brief 配置温度测量参数。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 用于配置温度测量的采样率和过采样率。
   *
   * @param temp_mr 温度测量的采样率 (Measurement Rate)。
   * @param temp_osr 温度测量的过采样率 (Oversampling Rate)。
   * @return int16_t 返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);

  /**
   * @brief 配置压力测量参数。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 用于配置压力测量的采样率和过采样率。
   *
   * @param prs_mr 压力测量的采样率 (Measurement Rate)。
   * @param prs_osr 压力测量的过采样率 (Oversampling Rate)。
   * @return int16_t 返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);

  /**
   * @brief 读取校准系数。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 用于从传感器内部存储器中读取校准系数，这些系数用于将原始测量值转换为精确的物理量。
   *
   * @return int16_t 返回一个 16 位有符号整数，表示函数执行的状态。通常情况下，0 表示成功，非零值表示错误代码。
   */
  int16_t readcoeffs(void);

public:
  /**
   * @brief 根据原始温度值计算实际温度值（摄氏度）。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 使用校准系数和特定的公式，将原始温度测量值转换为以摄氏度为单位的实际温度值。
   *
   * @param raw 原始温度测量值。
   * @return float 计算得到的实际温度值（摄氏度）。
   */
  float calcTemp(int32_t raw);

  /**
   * @brief 根据原始压力值计算实际压力值（帕斯卡）。
   *
   * 该函数是 DpsClass 抽象基类中定义的纯虚函数，在 DPS310 类中必须实现。
   * 使用校准系数和特定的公式，将原始压力测量值转换为以帕斯卡为单位的实际压力值。
   *
   * @param raw 原始压力测量值。
   * @return float 计算得到的实际压力值（帕斯卡）。
   */
  float calcPressure(int32_t raw);
};

#endif // 结束条件编译，如果定义了 DPS310_H_INCLUDED，则不再编译该部分
