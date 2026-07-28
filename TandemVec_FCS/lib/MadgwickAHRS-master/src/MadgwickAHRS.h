//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// 实现 Madgwick 的 IMU 和 AHRS 算法
// 详细信息请参考: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// 来自 x-io 网站的信息: “该网站上的开源资源在未提供其他许可的情况下，均按照 GNU General Public License 提供。”
//
// 日期           作者               备注
// 29/09/2011     SOH Madgwick       初始版本发布
// 02/10/2011     SOH Madgwick       优化以降低 CPU 负载
//
//=============================================================================================

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#include <Arduino.h>

//--------------------------------------------------------------------------------------------
// 变量声明
class Madgwick
{
private:
    /**
     * @brief 计算浮点数的倒数平方根
     * @param x 输入的浮点数
     * @return 倒数平方根
     */
    static double invSqrt(double x);

    /**
     * @brief 使用快速算法计算倒数平方根
     * @param x 输入的浮点数
     * @return 倒数平方根
     */
    static double fastInvSqrt(double x);

    double beta;             // 算法增益
    double q0, q1, q2, q3;   // 表示传感器相对于辅助框架的四元数
    double invSampleFreq;    // 采样频率的倒数
    double roll, pitch, yaw; // 欧拉角：滚转角、俯仰角、偏航角
    char anglesComputed;     // 标志，指示是否已经计算出欧拉角

    /**
     * @brief 计算欧拉角
     * 通过当前的四元数计算滚转角、俯仰角和偏航角。
     */
    void computeAngles();

public:
    /**
     * @brief 构造函数，初始化四元数和算法增益
     */
    Madgwick(void);

    /**
     * @brief 设置采样频率
     * @param sampleFrequency 采样频率，单位 Hz
     */
    void begin(double sampleFrequency)
    {
        invSampleFreq = 1.0 / sampleFrequency;
    }

    /**
     * @brief 使用加速度计数据初始化姿态
     * @param ax 加速度计 x 轴加速度（单位：g）
     * @param ay 加速度计 y 轴加速度（单位：g）
     * @param az 加速度计 z 轴加速度（单位：g）
     */
    void initializeFromAccelerometer(double ax, double ay, double az);

    /**
     * @brief 使用加速度计与磁力计数据初始化姿态
     * @param ax 加速度计 x 轴加速度（单位：g）
     * @param ay 加速度计 y 轴加速度（单位：g）
     * @param az 加速度计 z 轴加速度（单位：g）
     * @param mx 磁力计 x 轴磁场（单位：任意）
     * @param my 磁力计 y 轴磁场（单位：任意）
     * @param mz 磁力计 z 轴磁场（单位：任意）
     */
    void initializeFromAccelMag(double ax, double ay, double az, double mx, double my, double mz);

    /**
     * @brief 使用陀螺仪、加速度计和磁力计数据更新四元数
     * @param gx 陀螺仪 x 轴角速度（单位：度/秒）
     * @param gy 陀螺仪 y 轴角速度（单位：度/秒）
     * @param gz 陀螺仪 z 轴角速度（单位：度/秒）
     * @param ax 加速度计 x 轴加速度（单位：g）
     * @param ay 加速度计 y 轴加速度（单位：g）
     * @param az 加速度计 z 轴加速度（单位：g）
     * @param mx 磁力计 x 轴磁场（单位：任意）
     * @param my 磁力计 y 轴磁场（单位：任意）
     * @param mz 磁力计 z 轴磁场（单位：任意）
     * @param invSampleFreq 采样频率的倒数，即采样周期（秒）
     */
    void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double invSampleFreq);

    /**
     * @brief 仅使用陀螺仪和加速度计更新四元数
     * @param gx 陀螺仪 x 轴角速度（单位：度/秒）
     * @param gy 陀螺仪 y 轴角速度（单位：度/秒）
     * @param gz 陀螺仪 z 轴角速度（单位：度/秒）
     * @param ax 加速度计 x 轴加速度（单位：g）
     * @param ay 加速度计 y 轴加速度（单位：g）
     * @param az 加速度计 z 轴加速度（单位：g）
     */
    void updateIMU(double gx, double gy, double gz, double ax, double ay, double az, double invSampleFreq);

    /**
     * @brief 仅使用陀螺仪数据更新四元数
     * @param gx 陀螺仪 x 轴角速度（单位：度/秒）
     * @param gy 陀螺仪 y 轴角速度（单位：度/秒）
     * @param gz 陀螺仪 z 轴角速度（单位：度/秒）
     */
    void updateGyro(double gx, double gy, double gz, double invSampleFreq);

    /**
     * @brief 获取滚转角（单位：度）
     * @return 滚转角
     */
    double getRoll()
    {
        if (!anglesComputed)
            computeAngles();
        return roll * RAD_TO_DEG;
    }

    /**
     * @brief 获取俯仰角（单位：度）
     * @return 俯仰角
     */
    double getPitch()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch * RAD_TO_DEG;
    }

    /**
     * @brief 获取偏航角（单位：度）
     * @return 偏航角
     */
    double getYaw()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw * RAD_TO_DEG;
    }

    /**
     * @brief 获取滚转角（单位：弧度）
     * @return 滚转角
     */
    double getRollRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return roll;
    }

    /**
     * @brief 获取俯仰角（单位：弧度）
     * @return 俯仰角
     */
    double getPitchRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch;
    }

    /**
     * @brief 获取偏航角（单位：弧度）
     * @return 偏航角
     */
    double getYawRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw;
    }

    /**
     * @brief 获取当前四元数
     * @param q 四元数数组，q[0]=q0, q[1]=q1, q[2]=q2, q[3]=q3。此四元数表示从导航坐标系到机体坐标系的旋转 (qNB)。
     */
    void getQuaternion(double q[4])
    {
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }

    // 使用这个函数来获取符合 NWU->FLU 标准的欧拉角
    void getEulerAngles_NWU_FLU(double &roll, double &pitch, double &yaw);

    //================================================================================
    // 【新增函数】
    // 使用这个函数来获取符合 NED->FRD 标准的欧拉角
    //================================================================================
    /**
     * @brief 根据标准的NED->FRD约定，从内部姿态四元数计算欧拉角。
     * @param roll 输出的横滚角 (绕FRD X轴)，单位：弧度。
     * @param pitch 输出的俯仰角 (绕FRD Y轴)，单位：弧度。
     * @param yaw 输出的偏航角 (绕NED Z轴)，单位：弧度。
     */
    void getEulerAngles_NED_FRD(double &roll, double &pitch, double &yaw);
};

#endif
