//=============================================================================================
// MadgwickAHRS.cpp
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef 100.0f // sample frequency in Hz
#define betaDef 0.1f		 // 2 * proportional gain

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Madgwick::Madgwick()
{
	beta = betaDef;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	anglesComputed = 0;
}

/**
 * @brief 通过加速度计数据初始化 Madgwick 滤波器姿态。
 *
 * @param ax 加速度计 X 轴测量值。
 * @param ay 加速度计 Y 轴测量值。
 * @param az 加速度计 Z 轴测量值。
 *
 * @details 将加速度向量归一化后，直接构建四元数，使初始姿态与重力方向对齐。
 */
void Madgwick::initializeFromAccelerometer(double ax, double ay, double az)
{
	// 归一化加速度矢量
	double recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	// 处理零向量的情况
	if (!std::isfinite(recipNorm))
	{
		q0 = 1.0;
		q1 = 0.0;
		q2 = 0.0;
		q3 = 0.0;
		anglesComputed = 0;
		return;
	}
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// 将归一化的加速度向量视为重力方向在机体坐标系下的表示
	// 将此向量旋转到世界坐标系的 Z 轴方向 (即 [0, 0, 1])
	// 构造一个旋转四元数，将归一化加速度向量旋转到 [0, 0, 1]
	double dotProduct = az; // 加速度向量与世界坐标系 Z 轴的点积

	if (dotProduct >= 1.0)
	{
		// 加速度向量已经与 Z 轴对齐，无需旋转
		q0 = 1.0;
		q1 = 0.0;
		q2 = 0.0;
		q3 = 0.0;
	}
	else if (dotProduct <= -1.0)
	{
		q0 = 0.0;
		q1 = 1.0;
		q2 = 0.0;
		q3 = 0.0;
	}
	else
	{
		double rotationAngle = acos(dotProduct); // 旋转角度
		double rotationAxisX = ay;
		double rotationAxisY = -ax; // 叉乘 [ax, ay, az] x [0, 0, 1] 的结果
		double rotationAxisZ = 0;
		double sinHalfAngle = sin(rotationAngle / 2.0);
		double invSinHalfAngleNorm = 1.0 / sqrt(rotationAxisX * rotationAxisX + rotationAxisY * rotationAxisY + rotationAxisZ * rotationAxisZ);

		rotationAxisX *= invSinHalfAngleNorm;
		rotationAxisY *= invSinHalfAngleNorm;
		rotationAxisZ *= invSinHalfAngleNorm;

		q0 = cos(rotationAngle / 2.0);	   // 四元数的标量部分
		q1 = rotationAxisX * sinHalfAngle; // 四元数的 X 分量
		q2 = rotationAxisY * sinHalfAngle; // 四元数的 Y 分量
		q3 = rotationAxisZ * sinHalfAngle; // 四元数的 Z 分量
	}

	// 标记角度未计算
	anglesComputed = 0;
}

/**
 * @brief 结合加速度计和磁力计数据初始化四元数 (消除初始 Yaw 误差)。
 *
 * @details
 * 该函数使用代数方法（类似于 TRIAD 算法）直接构建旋转矩阵。
 * 它利用加速度计确定“上”(Z轴)，利用磁力计和加速度计的叉积确定“西”(Y轴)，
 * 最后通过叉积确定“北”(X轴)。
 * 这样可以立即得到准确的 Pitch, Roll 和 Yaw，无需等待滤波器收敛。
 *
 * @note 必须确保传入的磁力计数据已经过硬铁/软铁校准，否则初始 Yaw 会有偏差。
 *
 * @param ax, ay, az 加速度计测量值
 * @param mx, my, mz 磁力计测量值
 */
void Madgwick::initializeFromAccelMag(double ax, double ay, double az, double mx, double my, double mz)
{
	// 1. 归一化加速度计 (作为 "Up" 向量, Z轴)
	double normAcc = sqrt(ax * ax + ay * ay + az * az);
	if (normAcc == 0.0f)
		return; // 防止除零
	double invNormAcc = 1.0f / normAcc;
	double downX = ax * invNormAcc;
	double downY = ay * invNormAcc;
	double downZ = az * invNormAcc;

	// 2. 归一化磁力计
	double normMag = sqrt(mx * mx + my * my + mz * mz);
	if (normMag == 0.0f)
		return;
	double invNormMag = 1.0f / normMag;
	double magX = mx * invNormMag;
	double magY = my * invNormMag;
	double magZ = mz * invNormMag;

	// 3. 计算 "West" 向量 (Y轴) = Up x Mag
	// Madgwick 内部系是 NWU (北西天)。
	// 叉乘：Z(Up) x X(North_ish) = Y(West)
	double westX = downY * magZ - downZ * magY;
	double westY = downZ * magX - downX * magZ;
	double westZ = downX * magY - downY * magX;

	// 归一化 West 向量
	double normWest = sqrt(westX * westX + westY * westY + westZ * westZ);
	if (normWest == 0.0f)
	{
		// 奇异点处理：如果磁场和重力平行（如在磁极），无法确定航向。
		// 回退到只用加速度计初始化
		initializeFromAccelerometer(ax, ay, az);
		return;
	}
	double invNormWest = 1.0f / normWest;
	westX *= invNormWest;
	westY *= invNormWest;
	westZ *= invNormWest;

	// 4. 计算 "North" 向量 (X轴) = West x Up
	// 确保正交系：Y(West) x Z(Up) = X(North)
	double northX = westY * downZ - westZ * downY;
	double northY = westZ * downX - westX * downZ;
	double northZ = westX * downY - westY * downX;

	// 5. 构建旋转矩阵并转换为四元数
	// 我们构建的是从 [Sensor] 到 [Earth] 的旋转矩阵 R。
	// R 的行向量分别是 Sensor 坐标系下的 North, West, Up。
	// R = [ North.x  North.y  North.z ]
	//     [ West.x   West.y   West.z  ]
	//     [ Down.x   Down.y   Down.z  ]

	double t0 = northX;
	double t1 = westY;
	double t2 = downZ;

	double trace = t0 + t1 + t2;
	double S;

	// 使用 Shepperd 算法 (或常见的四元数转换算法) 避免除零，保证数值稳定性
	if (trace > 0.0f)
	{
		S = 0.5f / sqrt(trace + 1.0f);
		q0 = 0.25f / S;
		q1 = (westZ - downY) * S;
		q2 = (downX - northZ) * S;
		q3 = (northY - westX) * S;
	}
	else
	{
		if (t0 > t1 && t0 > t2)
		{
			S = 2.0f * sqrt(1.0f + t0 - t1 - t2);
			double invS = 1.0f / S;
			q0 = (westZ - downY) * invS;
			q1 = 0.25f * S;
			q2 = (westX + northY) * invS;
			q3 = (northZ + downX) * invS;
		}
		else if (t1 > t2)
		{
			S = 2.0f * sqrt(1.0f + t1 - t0 - t2);
			double invS = 1.0f / S;
			q0 = (downX - northZ) * invS;
			q1 = (westX + northY) * invS;
			q2 = 0.25f * S;
			q3 = (downY + westZ) * invS;
		}
		else
		{
			S = 2.0f * sqrt(1.0f + t2 - t0 - t1);
			double invS = 1.0f / S;
			q0 = (northY - westX) * invS;
			q1 = (northZ + downX) * invS;
			q2 = (downY + westZ) * invS;
			q3 = 0.25f * S;
		}
	}

	// 归一化四元数（防御性编程）
	double recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// 重置标志位
	anglesComputed = 0;
}

void Madgwick::update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double invSampleFreq)
{
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double hx, hy;
	double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// 如果磁力计测量无效，使用IMU算法（避免磁力计归一化时出现NaN）
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		updateIMU(gx, gy, gz, ax, ay, az, invSampleFreq);
		return;
	}

	// 将陀螺仪的度/秒转换为弧度/秒
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;

	// 来自陀螺仪的四元数变化率
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// 仅当加速度计测量有效时才计算反馈（避免加速度计归一化时出现NaN）
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// 归一化加速度计测量值
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 归一化磁力计测量值
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// 辅助变量以避免重复计算
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// 地球磁场的参考方向
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// 梯度下降算法校正步骤
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化步长大小
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// 应用反馈步骤
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// 积分四元数变化率以产生四元数
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// 归一化四元数
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/**
 * @brief 更新IMU数据并计算四元数
 *
 * 该函数使用Madgwick滤波器算法更新IMU（惯性测量单元）数据，并计算新的四元数。
 * 主要步骤包括：
 * 1. 将陀螺仪的度/秒转换为弧度/秒。
 * 2. 从陀螺仪数据计算四元数的变化率。
 * 3. 如果加速度计测量值有效，进行梯度下降校正。
 * 4. 积分四元数的变化率以得到新的四元数。
 * 5. 对新的四元数进行归一化处理。
 *
 * @param gx 陀螺仪X轴角速度（度/秒）
 * @param gy 陀螺仪Y轴角速度（度/秒）
 * @param gz 陀螺仪Z轴角速度（度/秒）
 * @param ax 加速度计X轴加速度（重力加速度g）
 * @param ay 加速度计Y轴加速度（重力加速度g）
 * @param az 加速度计Z轴加速度（重力加速度g）
 * @param invSampleFreq 采样频率的倒数，即采样周期（秒）
 */
void Madgwick::updateIMU(double gx, double gy, double gz, double ax, double ay, double az, double invSampleFreq)
{
	double recipNorm;
	double s0, s1, s2, s3;
	double qDot1, qDot2, qDot3, qDot4;
	double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// 将陀螺仪的度/秒转换为弧度/秒
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;

	// 从陀螺仪计算四元数的变化率
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// 只有当加速度计测量值有效时才进行反馈计算（避免加速度计归一化中的NaN）
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// 归一化加速度计测量值
		recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 辅助变量，避免重复计算
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// 梯度下降算法的校正步骤
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化步长
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// 应用反馈步骤
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// 积分四元数的变化率以得到四元数
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// 归一化四元数
	recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

/**
 * 使用Madgwick算法更新四元数，基于陀螺仪数据。
 *
 * @param gx 陀螺仪x轴角速度（度/秒）
 * @param gy 陀螺仪y轴角速度（度/秒）
 * @param gz 陀螺仪z轴角速度（度/秒）
 * @param invSampleFreq 采样频率的倒数（秒/次）
 */
void Madgwick::updateGyro(double gx, double gy, double gz, double invSampleFreq)
{
	double qDot1, qDot2, qDot3, qDot4;

	// 将陀螺仪的角速度从度/秒转换为弧度/秒
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;

	// 根据陀螺仪数据计算四元数的变化率
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// 将四元数的变化率积分以更新四元数
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// 规范化四元数
	double recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// 重置角度计算完成标志
	anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
/**
 * @brief 计算一个浮点数的倒数平方根 (快速算法版本)。
 *
 * @param x 待求倒数平方根的浮点数。
 * @return x的倒数平方根的近似值。
 *
 * @details 使用快速平方根倒数算法，速度快但精度较低。
 */
double Madgwick::fastInvSqrt(double x)
{
	// 检查输入是否为正数，如果不是则返回NaN
	if (x <= 0.0)
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	// 原版快速算法按 float 的 32 位魔数实现，直接套在 double 上会触发越界和严格别名未定义行为。
	return 1.0 / sqrt(x);
}

/**
 * @brief 计算一个浮点数的倒数平方根 (高精度版本)。
 *
 * @param x 待求倒数平方根的浮点数。
 * @return x的倒数平方根。
 *
 * @details 使用标准库函数 `sqrt` 计算平方根，然后取倒数，精度高。
 */
double Madgwick::invSqrt(double x)
{
	// 检查输入是否为正数，如果不是则返回NaN
	if (x <= 0.0)
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	return 1.0 / sqrt(x); // 使用标准库函数计算平方根并取倒数
}

//-------------------------------------------------------------------------------------------

/**
 * 计算Madgwick算法中的角度值(前-左-上坐标系下欧拉角)
 * 该函数不接受参数，也不直接返回值，但会更新类成员变量roll、pitch和yaw，分别表示欧拉角的翻滚角（范围：-π至π）、俯仰角（范围：-π/2至π/2）和偏航角（范围：-π至π）。
 * 成员变量anglesComputed会被设置为1，表示角度计算完成。
 */
void Madgwick::computeAngles()
{
	// 计算翻滚角，使用atan2f函数获取范围在-π至π之间的角度值
	roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
	// 计算俯仰角，使用asinf函数获取范围在-π/2至π/2之间的弧度值（转换为角度后也是同样的范围）
	pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
	// 计算偏航角，同样使用atan2f函数获取范围在-π至π之间的角度值
	yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
	// 标记角度计算完成
	anglesComputed = 1;
}

/**
 * @brief [Madgwick 内部坐标系专用] 从四元数计算欧拉角。
 *
 * @details
 * **警告：这是一个专用函数，请勿用于通用坐标系转换！**
 *
 * 本函数的功能是，将一个描述从 Madgwick 内部导航系 (NWU: 北-西-天) 到
 * 其内部机体系 (FLU: 前-左-上) 旋转的四元数，转换为对应的欧拉角。
 *
 * 欧拉角的物理定义遵循 Madgwick 的内部约定：
 * - **Roll (横滚角, phi):** 绕机体的 **前 (Front, X_FLU)** 轴旋转。
 * - **Pitch (俯仰角, theta):** 绕机体的 **左 (Left, Y_FLU)** 轴旋转。
 * - **Yaw (偏航角, psi):** 绕导航系的 **天 (Up, Z_NWU)** 轴旋转。
 *
 * 旋转顺序为 Z-Y'-X'' (偏航-俯仰-横滚) 内旋。
 *
 * 该函数的实现与 Madgwick 库自带的 getRoll(), getPitch(), getYaw() 内部
 * 使用的 `computeAngles()` 方法在数学上是等价的，提供了独立的、清晰的实现。
 *
 * @param roll  [out] 输出的横滚角 (phi, 绕 FLU 的 X 轴)，单位：弧度。
 * @param pitch [out] 输出的俯仰角 (theta, 绕 FLU 的 Y 轴)，单位：弧度。
 * @param yaw   [out] 输出的偏航角 (psi, 绕 NWU 的 Z 轴)，单位：弧度。
 */
void Madgwick::getEulerAngles_NWU_FLU(double &roll, double &pitch, double &yaw)
{
	// --- 俯仰角 (Pitch, theta, 绕'左'轴) ---
	// 从旋转矩阵 R_FLU_<-_NWU 的 R(3,1) 元素提取
	// R(3,1) = 2 * (q1*q3 - q0*q2) = -sin(theta)
	double sin_pitch = 2.0 * (q0 * q2 - q1 * q3);
	// 处理万向节死锁 (Gimbal Lock)
	if (std::fabs(sin_pitch) >= 1.0)
	{
		pitch = std::copysign(M_PI / 2.0, sin_pitch);
	}
	else
	{
		pitch = std::asin(sin_pitch);
	}
	// --- 横滚角 (Roll, phi, 绕'前'轴) ---
	// 从旋转矩阵的 R(3,2) 和 R(3,3) 元素提取
	// R(3,2) = 2 * (q0*q1 + q2*q3) = cos(theta) * sin(phi)
	// R(3,3) = 1 - 2 * (q1*q1 + q2*q2) = cos(theta) * cos(phi)
	double sin_roll_cos_pitch = 2.0 * (q0 * q1 + q2 * q3);
	double cos_roll_cos_pitch = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
	roll = std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);
	// --- 偏航角 (Yaw, psi, 绕'天'轴) ---
	// 从旋转矩阵的 R(1,1) 和 R(2,1) 元素提取
	// R(1,1) = 1 - 2 * (q2*q2 + q3*q3) = cos(psi) * cos(theta)
	// R(2,1) = 2 * (q0*q3 + q1*q2) = sin(psi) * cos(theta)
	double sin_yaw_cos_pitch = 2.0 * (q0 * q3 + q1 * q2);
	double cos_yaw_cos_pitch = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
	yaw = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
}

/**
 * @brief [标准航空接口] 从内部四元数计算符合 NED->FRD 约定的欧拉角。
 *
 * @details
 * **这是推荐给最终用户使用的标准姿态输出函数。**
 *
 * 本函数处理了所有底层的坐标系转换，将 Madgwick 算法内部计算出的、
 * 代表 NWU->FLU 旋转的四元数，转换为用户期望的、符合标准航空航天
 * 约定的 NED->FRD (北-东-地 -> 前-右-下) 欧拉角。
 *
 * 转换原理基于 NWU->FLU 与 NED->FRD 坐标系之间的对称关系。
 *
 * @param roll  [out] 输出的横滚角 (phi, 绕 FRD 的 X 轴 '前')，单位：弧度。
 * @param pitch [out] 输出的俯仰角 (theta, 绕 FRD 的 Y 轴 '右')，单位：弧度。
 * @param yaw   [out] 输出的偏航角 (psi, 绕 NED 的 Z 轴 '地')，单位：弧度。
 */
void Madgwick::getEulerAngles_NED_FRD(double &roll, double &pitch, double &yaw)
{
	// 1. 获取 Madgwick 内部坐标系 (NWU->FLU) 下的欧拉角。
	//    这些数值在数学上是正确的，但其物理含义与标准航空约定不同。
	//    - roll_nwu_flu:  绕 '前' 轴旋转
	//    - pitch_nwu_flu: 绕 '左' 轴旋转
	//    - yaw_nwu_flu:   绕 '天' 轴旋转
	double roll_nwu_flu, pitch_nwu_flu, yaw_nwu_flu;
	getEulerAngles_NWU_FLU(roll_nwu_flu, pitch_nwu_flu, yaw_nwu_flu);
	// 2. 进行坐标系转换，将 NWU->FLU 欧拉角映射到 NED->FRD 欧拉角。
	//    这个转换是基于两个坐标系框架之间轴的对应关系。
	//    - 标准 Roll (绕'右'轴)  -> 在 FLU 中没有直接对应，但 Pitch (绕'左'轴) 是其反向。
	//    - 标准 Pitch (绕'前'轴) -> 在 FLU 中对应 Roll (绕'前'轴)。
	//    - 标准 Yaw (绕'地'轴)   -> 在 NWU 中 Yaw (绕'天'轴) 是其反向。

	// - Pitch: 绕'右'轴为正 vs 绕'左'轴为正。根据右手定则，两者效果相反。
	// - Yaw: 绕'地'(下)为正 vs 绕'天'(上)为正。根据右手定则，两者效果相反。
	// - Roll: 绕'前'轴为正。两者定义相同。
	roll = roll_nwu_flu;
	pitch = -pitch_nwu_flu;
	yaw = -yaw_nwu_flu;
}
