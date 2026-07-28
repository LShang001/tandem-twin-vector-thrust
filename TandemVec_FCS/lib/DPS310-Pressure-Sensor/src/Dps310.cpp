#include "Dps310.h"           // 包含 Dps310 类的头文件
#include "Dps310FifoPolicy.h" // FIFO 最新样本读取策略

#ifndef BFS_DPS310_FIFO_MAX_READS
#define BFS_DPS310_FIFO_MAX_READS 1U
#endif

using namespace dps;	// 使用 dps 命名空间
using namespace dps310; // 使用 dps310 命名空间

/**
 * @brief 获取连续测量模式下的温度和压力结果。
 *
 * 该函数重写了 DpsClass 基类的 getContResults 函数，
 * 用于获取连续测量模式下的温度和压力结果。
 * 它调用基类的 getContResults 函数，并使用 registers[FIFO_EMPTY] 作为参数，
 * 这可能表示从 FIFO 非空的状态下读取数据。
 *
 * @param tempBuffer 指向存储温度结果的浮点数组的指针。
 * @param tempCount  引用一个无符号 8 位整数，表示温度结果的数量。
 * @param prsBuffer  指向存储压力结果的浮点数组的指针。
 * @param prsCount   引用一个无符号 8 位整数，表示压力结果的数量。
 * @return int16_t   返回一个 16 位有符号整数，表示操作的状态。通常，0 表示成功，非零值表示失败或错误代码。
 */
int16_t Dps310::getContResults(float *tempBuffer,
							   uint8_t &tempCount,
							   float *prsBuffer,
							   uint8_t &prsCount)
{
	return DpsClass::getContResults(tempBuffer, tempCount, prsBuffer, prsCount, registers[FIFO_EMPTY]);
}

/**
 * @brief 获取最新的温度和压力测量结果。
 *
 * 此函数从 DPS310 传感器的 FIFO 中读取数据，清空 FIFO 后返回最后一组可用的温度和压力数据。
 * 函数会先检查 FIFO 是否溢出，如果溢出，则返回错误代码。
 * 然后检查 FIFO 是否有新数据，如果有，则读取直到 FIFO 为空，并返回最新可用的温度和压力数据及其可用标志。
 * 最后，根据标志位判断是否成功读取到数据，并计算实际的温度和压力值。
 * 如果没有任何可用的新数据，则返回 DPS__FAIL_UNFINISHED。
 * 此函数适用于需要低延迟获取最新样本的应用场景，避免任务消费速率低于 FIFO 生产速率时读到旧样本。
 *
 * @param temp 引用类型，用于存储温度值，单位：摄氏度 (℃)。
 * @param pressure 引用类型，用于存储压力值，单位：帕斯卡 (Pa)。
 *
 * @return int16_t 返回操作结果状态码：
 *
 * @note
 *   - 此函数假设 DPS310 传感器已经正确初始化并在后台模式下运行。
 *   - 调用此函数前，应确保已根据需要配置好温度和压力的采样率。
 *   - 此函数会清空 FIFO。
 *   - 即使温度和压力采样率不同，此函数也能正常工作。
 *   - 如果应用场景需要更高的实时性，建议考虑使用中断方式读取数据。
 */
int16_t Dps310::getLatestResults(float &temp, float &pressure)
{
	if (m_initFail)
	{
		return DPS__FAIL_INIT_FAILED;
	}
	// abort if device is not in background mode
	if (!(m_opMode & 0x04))
	{
		return DPS__FAIL_TOOBUSY;
	}

	const int16_t fifo_status_raw = readByte(registers[FIFO_EMPTY].regAddress);
	if (fifo_status_raw < 0)
	{
		return DPS__FAIL_UNKNOWN;
	}
	const FifoStatus fifo_status = decodeDps310FifoStatus(static_cast<uint8_t>(fifo_status_raw));

	if (!fifo_status.has_sample)
	{
		return DPS__FAIL_UNFINISHED;
	}
	// fifo_status.full: DPS310 FIFO 是环形缓冲，满时最老样本被覆盖，最新样本仍有效。
	// 不在这里直接返回 OVERFLOW——先读完再根据读取结果决定是否上报错误。

	LatestFifoRawResult latest;
#if BFS_DPS310_FIFO_MAX_READS <= 1U
	// 单样本路径用于保守回退或对比 profile：读取前已经确认 FIFO 非空，
	// 因此不再重复查询 empty bit，避免多一次 I2C 事务。
	const int16_t read_status = readLatestRawFromFifo(
		[]() { return false; },
		[this](int32_t &raw_result) { return getFIFOvalue(&raw_result); },
		latest,
		1U);
#else
	bool first_fifo_check = true;
	// 上板实测的正式路径：最多读取两个 FIFO 样本，并在拿到压力样本后停止。
	// 目标不是排空 FIFO，而是在 128Hz 压力输出附近保持最新压力样本不过期。
	const int16_t read_status = readLatestRawFromFifo(
		[this, &first_fifo_check, &latest]() {
			if (first_fifo_check)
			{
				first_fifo_check = false;
				return false;
			}
#ifdef BFS_DPS310_STOP_AFTER_PRESSURE_SAMPLE
			if (shouldStopFifoReadAfterSample(latest, true))
			{
				return true;
			}
#endif
			return readByteBitfield(registers[FIFO_EMPTY]) != 0;
		},
		[this](int32_t &raw_result) { return getFIFOvalue(&raw_result); },
		latest,
		BFS_DPS310_FIFO_MAX_READS);
#endif
	if (read_status != DPS__SUCCEEDED)
	{
		// FIFO 满时读取仍失败说明状态异常，上报 OVERFLOW 触发上层清空；
		// 正常情况下（FIFO 不满）直接透传错误码。
		return fifo_status.full ? static_cast<int16_t>(DPS__FAIL_OVERFLOW) : read_status;
	}

	// 计算并返回数据
	if (latest.temp_available)
	{
		temp = calcTemp(latest.raw_temp);
	}

	if (latest.pressure_available)
	{
		pressure = calcPressure(latest.raw_pressure);
	}

	return pressureUpdateStatus(latest);
}

#ifdef BFS_DPS310_DIAG
int16_t Dps310::readDebugRegister(uint8_t regAddress)
{
	return readByte(regAddress);
}
#endif

#ifndef DPS_DISABLESPI
/**
 * @brief 设置中断源和极性。
 *
 * 该函数用于设置中断源和极性，仅在非 SPI 模式下可用。
 * 首先检查是否支持中断设置（不支持 4 线 SPI 模式）。
 * 如果支持，通过写入 registers[INT_SEL] 和 registers[INT_HL] 来配置中断源和极性。
 *
 * @param intr_source 中断源。
 * @param polarity    中断信号的极性。
 * @return int16_t    操作状态。DPS__FAIL_UNKNOWN 表示未知失败，其他值由 writeByteBitfield 函数返回。
 */
int16_t Dps310::setInterruptSources(uint8_t intr_source, uint8_t polarity)
{
	// Interrupts are not supported with 4 Wire SPI
	// 如果不是 I2C 模式且不是三线 SPI 模式，则返回错误
	if (!m_SpiI2c & !m_threeWire)
	{
		return DPS__FAIL_UNKNOWN;
	}
	// 设置中断源和极性
	return writeByteBitfield(intr_source, registers[INT_SEL]) || writeByteBitfield(polarity, registers[INT_HL]);
}
#endif

/**
 * @brief 初始化 DPS310 传感器。
 *
 * 该函数执行 DPS310 传感器的初始化操作。包括：
 * 1. 读取产品 ID 和修订 ID，验证设备是否为 DPS310。
 * 2. 读取温度传感器校准信息，并选择用于温度测量的传感器。
 * 3. 读取校准系数。
 * 4. 设置设备为待机模式，并配置温度和压力的测量精度和速率。
 * 5. 执行一次温度测量，以获取最新的温度数据用于压力补偿。
 * 6. 最后，将设备设置回待机模式，并执行一个温度校正操作（针对存在 fuse bit 问题的 IC）。
 */
void Dps310::init(void)
{
	// 读取产品 ID
	int16_t prodId = readByteBitfield(registers[PROD_ID]);
	if (prodId < 0)
	{
		// Connected device is not a Dps310
		// 如果读取产品 ID 失败，则说明连接的设备不是 DPS310
		m_initFail = 1U;
		return;
	}
	m_productID = prodId; // 保存产品 ID

	// 读取修订 ID
	int16_t revId = readByteBitfield(registers[REV_ID]);
	if (revId < 0)
	{
		// 如果读取修订 ID 失败，则设置初始化失败标志
		m_initFail = 1U;
		return;
	}
	m_revisionID = revId; // 保存修订 ID

	// find out which temperature sensor is calibrated with coefficients...
	// 读取温度传感器校准信息，确定哪个温度传感器已经用系数校准过
	int16_t sensor = readByteBitfield(registers[TEMP_SENSORREC]);
	if (sensor < 0)
	{
		// 如果读取温度传感器校准信息失败，则设置初始化失败标志
		m_initFail = 1U;
		return;
	}

	// 并使用该传感器进行温度测量
	m_tempSensor = sensor;
	// 设置温度传感器
	if (writeByteBitfield((uint8_t)sensor, registers[TEMP_SENSOR]) < 0)
	{
		// 如果设置温度传感器失败，则设置初始化失败标志
		m_initFail = 1U;
		return;
	}

	// 读取校准系数
	if (readcoeffs() < 0)
	{
		// 如果读取校准系数失败，则设置初始化失败标志
		m_initFail = 1U;
		return;
	}

	// 设置设备为待机模式，以便进行进一步的配置
	standby();

	// 将测量精度和速率设置为标准值
	configTemp(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);
	configPressure(DPS__MEASUREMENT_RATE_4, DPS__OVERSAMPLING_RATE_8);

	// 执行第一次温度测量，最新的温度将保存在内部，并在计算压力时用于补偿
	float trash;
	measureTempOnce(trash);

	// 确保 DPS310 在初始化后处于待机模式
	standby();

	// 修复具有熔丝位问题的 IC，该问题会导致错误的温度
	// 对于没有此问题的 IC，不应影响
	correctTemp();
}

/**
 * @brief 读取校准系数。
 *
 * 该函数用于从传感器内部存储器中读取校准系数。
 * 首先从 coeffBlock 地址读取 18 个字节的数据到缓冲区。
 * 然后，根据数据手册中的公式，从缓冲区中提取并计算出各个校准系数
 * （m_c0Half, m_c1, m_c00, m_c10, m_c01, m_c11, m_c20, m_c21, m_c30）。
 * 最后，对每个系数执行二进制补码转换。
 *
 * @return int16_t 操作状态，DPS__SUCCEEDED 表示成功。
 */
int16_t Dps310::readcoeffs(void)
{
	// 待办事项：删除魔术数字 18
	uint8_t buffer[18]; // 缓冲区，用于存储从传感器读取的校准系数数据
	// 从 coeffBlock 地址读取数据到缓冲区
	int16_t ret = readBlock(coeffBlock, buffer);
	if (ret != coeffBlock.length)
	{
		return DPS__FAIL_UNKNOWN; // 校准系数不完整时直接失败，避免解析未初始化缓冲区
	}

	// 从缓冲区内容中组合系数
	// 根据数据手册中的公式计算 m_c0Half
	m_c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
	getTwosComplement(&m_c0Half, 12);
	// c0 仅用作 c0*0.5，因此立即计算 c0_half
	m_c0Half = m_c0Half / 2U;

	// 现在对所有其他系数执行相同的操作
	// 根据数据手册中的公式计算 m_c1
	m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	getTwosComplement(&m_c1, 12);
	// 根据数据手册中的公式计算 m_c00
	m_c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
	getTwosComplement(&m_c00, 20);
	// 根据数据手册中的公式计算 m_c10
	m_c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
	getTwosComplement(&m_c10, 20);

	// 根据数据手册中的公式计算 m_c01
	m_c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
	getTwosComplement(&m_c01, 16);

	// 根据数据手册中的公式计算 m_c11
	m_c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
	getTwosComplement(&m_c11, 16);
	// 根据数据手册中的公式计算 m_c20
	m_c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
	getTwosComplement(&m_c20, 16);
	// 根据数据手册中的公式计算 m_c21
	m_c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
	getTwosComplement(&m_c21, 16);
	// 根据数据手册中的公式计算 m_c30
	m_c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
	getTwosComplement(&m_c30, 16);
	return DPS__SUCCEEDED; // 返回成功
}

/**
 * @brief 配置温度测量的相关参数。
 *
 * 该函数用于配置温度测量的采样率、过采样率和温度传感器选择。
 * 首先调用基类的 configTemp 函数。
 * 然后设置温度传感器选择。
 * 最后，根据过采样率 (OSR) 设置温度测量结果是否需要进行偏移使能。
 *
 * @param tempMr 温度测量速率。
 * @param tempOsr 温度过采样率。
 * @return int16_t 操作状态。
 */
int16_t Dps310::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
	int16_t ret = DpsClass::configTemp(tempMr, tempOsr); // 调用基类的 configTemp 函数

	// 设置温度传感器
	writeByteBitfield(m_tempSensor, registers[TEMP_SENSOR]);
	// 如果过采样率高于 8(2^3)，则设置 TEMP SHIFT ENABLE
	if (tempOsr > DPS310__OSR_SE)
	{
		ret = writeByteBitfield(1U, registers[TEMP_SE]);
	}
	else
	{
		ret = writeByteBitfield(0U, registers[TEMP_SE]);
	}
	return ret;
}

/**
 * @brief 配置压力测量的相关参数。
 *
 * 该函数用于配置压力测量的采样率和过采样率。
 * 首先调用基类的 configPressure 函数。
 * 然后，根据过采样率 (OSR) 设置压力测量结果是否需要进行偏移使能。
 *
 * @param prsMr 压力测量速率。
 * @param prsOsr 压力过采样率。
 * @return int16_t 操作状态。
 */
int16_t Dps310::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
	int16_t ret = DpsClass::configPressure(prsMr, prsOsr); // 调用基类的 configPressure 函数
	// 如果过采样率高于 8(2^3)，则设置 PM SHIFT ENABLE
	if (prsOsr > DPS310__OSR_SE)
	{
		ret = writeByteBitfield(1U, registers[PRS_SE]);
	}
	else
	{
		ret = writeByteBitfield(0U, registers[PRS_SE]);
	}
	return ret;
}

/**
 * @brief 根据原始温度值计算实际的温度值（摄氏度）。
 *
 * 该函数使用校准系数和特定的公式，将原始温度测量值转换为以摄氏度为单位的实际温度值。
 * 首先，根据过采样率对原始温度值进行缩放。
 * 然后，更新 m_lastTempScal，用于压力补偿。
 * 最后，使用公式 temp = m_c0Half + m_c1 * temp 计算补偿后的温度值。
 *
 * @param raw 原始温度测量值。
 * @return float 计算得到的实际温度值（摄氏度）。
 */
float Dps310::calcTemp(int32_t raw)
{
	float temp = raw; // 将原始温度值赋给临时变量 temp

	// 根据缩放表和过采样率对温度进行缩放
	temp /= scaling_facts[m_tempOsr];

	// 更新最后测量的温度，它将用于压力补偿
	m_lastTempScal = temp;

	// 计算补偿后的温度
	temp = m_c0Half + m_c1 * temp;

	return temp; // 返回计算得到的温度值
}

/**
 * @brief 根据原始压力值计算实际的压力值（帕斯卡）。
 *
 * 该函数使用校准系数和特定的公式，将原始压力测量值转换为以帕斯卡为单位的实际压力值。
 * 首先，根据过采样率对原始压力值进行缩放。
 * 然后，使用公式
 * prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21))
 * 计算补偿后的压力值。
 *
 * @param raw 原始压力测量值。
 * @return float 计算得到的实际压力值（帕斯卡）。
 */
float Dps310::calcPressure(int32_t raw)
{
	float prs = raw; // 将原始压力值赋给临时变量 prs

	// 根据缩放表和过采样率对压力进行缩放
	prs /= scaling_facts[m_prsOsr];

	// 计算补偿后的压力
	prs = m_c00 + prs * (m_c10 + prs * (m_c20 + prs * m_c30)) + m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

	return prs; // 返回计算得到的压力值
}

/**
 * @brief 清空 FIFO 缓冲区。
 *
 * 该函数用于清空传感器内部的 FIFO 缓冲区。
 * 通过向 registers[FIFO_FL] 写入 1 来清空 FIFO。
 *
 * @return int16_t 操作状态。
 */
int16_t Dps310::flushFIFO()
{
	return writeByteBitfield(1U, registers[FIFO_FL]); // 向 FIFO_FL 寄存器写入 1，清空 FIFO
}

/**
 * @brief 根据当前气压和温度计算海拔高度（考虑温度梯度的公式）
 *
 * 使用气压高度公式并考虑温度梯度校正。
 *
 * @param pressure 当前气压值，单位：帕斯卡
 * @param temperature 当前温度值，单位：摄氏度
 * @return 海拔高度，单位：米
 */
float Dps310::calculateAltitude(float pressure, float temperature)
{
	// 常数定义
	const float P0 = 101325.0f; // 标准大气压，单位：帕斯卡
	const float L = 0.0065f;	// 温度梯度，单位：K/m
	const float R = 8.3144598f; // 通用气体常数，单位：J/(mol·K)
	const float g = 9.80665f;	// 重力加速度，单位：m/s²
	const float M = 0.0289644f; // 空气的摩尔质量，单位：kg/mol

	// 将温度从摄氏度转换为开尔文
	float T = temperature + 273.15f;

	// 计算指数部分
	// 指数 = (R * L) / (g * M)
	float exponent = (R * L) / (g * M);

	// 计算海拔高度
	// h = (T / L) * [1 - (P / P0)^(exponent)]
	float altitude = (T / L) * (1.0f - pow(pressure / P0, exponent));

	return altitude;
}

/**
 * @brief 根据当前气压计算海拔高度（简化的公式，未考虑温度的影响）。
 *
 * 使用简化的气压高度公式，仅根据气压值计算海拔高度。
 * 此公式未考虑温度的影响，因此精度较低，仅适用于对精度要求不高的场景。
 *
 * @param pressure 当前气压值，单位：帕斯卡
 * @return 海拔高度，单位：米
 */
float Dps310::calculateAltitudeSimplified(float pressure)
{
	const float P0 = 101325.0f;			  // 标准大气压 (海平面)，单位：帕斯卡
	const float exponent = 1.0f / 5.255f; // 简化后的指数部分

	// 计算海拔高度
	// h = 44330 * (1 - (P / P0)^(exponent))
	float altitude = 44330.0f * (1.0f - pow(pressure / P0, exponent));

	return altitude;
}
