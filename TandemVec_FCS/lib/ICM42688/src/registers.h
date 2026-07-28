#ifndef ICM42688_REGISTERS_H_
#define ICM42688_REGISTERS_H_

#include <cstdint>

namespace ICM42688reg
{

  // **公共寄存器**：所有用户寄存器组都可以访问
  static constexpr uint8_t REG_BANK_SEL = 0x76; // 寄存器寄存器组选择，用于切换到不同的用户寄存器组

  // **用户寄存器组 0 (User Bank 0)** 寄存器定义
  static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;      // 设备配置寄存器
  static constexpr uint8_t UB0_REG_DRIVE_CONFIG = 0x13;       // 驱动配置
  static constexpr uint8_t UB0_REG_INT_CONFIG = 0x14;         // 中断配置
  static constexpr uint8_t UB0_REG_FIFO_CONFIG = 0x16;        // FIFO 配置
  static constexpr uint8_t UB0_REG_TEMP_DATA1 = 0x1D;         // 温度高字节数据
  static constexpr uint8_t UB0_REG_TEMP_DATA0 = 0x1E;         // 温度低字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_X1 = 0x1F;      // X轴加速度高字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_X0 = 0x20;      // X轴加速度低字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Y1 = 0x21;      // Y轴加速度高字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Y0 = 0x22;      // Y轴加速度低字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Z1 = 0x23;      // Z轴加速度高字节数据
  static constexpr uint8_t UB0_REG_ACCEL_DATA_Z0 = 0x24;      // Z轴加速度低字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_X1 = 0x25;       // X轴陀螺仪高字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_X0 = 0x26;       // X轴陀螺仪低字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_Y1 = 0x27;       // Y轴陀螺仪高字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_Y0 = 0x28;       // Y轴陀螺仪低字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_Z1 = 0x29;       // Z轴陀螺仪高字节数据
  static constexpr uint8_t UB0_REG_GYRO_DATA_Z0 = 0x2A;       // Z轴陀螺仪低字节数据
  static constexpr uint8_t UB0_REG_TMST_FSYNCH = 0x2B;        // 时间戳高字节
  static constexpr uint8_t UB0_REG_TMST_FSYNCL = 0x2C;        // 时间戳低字节
  static constexpr uint8_t UB0_REG_INT_STATUS = 0x2D;         // 中断状态寄存器
  static constexpr uint8_t UB0_REG_FIFO_COUNTH = 0x2E;        // FIFO 计数高字节
  static constexpr uint8_t UB0_REG_FIFO_COUNTL = 0x2F;        // FIFO 计数低字节
  static constexpr uint8_t UB0_REG_FIFO_DATA = 0x30;          // FIFO 数据寄存器
  static constexpr uint8_t UB0_REG_APEX_DATA0 = 0x31;         // APEX 数据寄存器 0
  static constexpr uint8_t UB0_REG_APEX_DATA1 = 0x32;         // APEX 数据寄存器 1
  static constexpr uint8_t UB0_REG_APEX_DATA2 = 0x33;         // APEX 数据寄存器 2
  static constexpr uint8_t UB0_REG_APEX_DATA3 = 0x34;         // APEX 数据寄存器 3
  static constexpr uint8_t UB0_REG_APEX_DATA4 = 0x35;         // APEX 数据寄存器 4
  static constexpr uint8_t UB0_REG_APEX_DATA5 = 0x36;         // APEX 数据寄存器 5
  static constexpr uint8_t UB0_REG_INT_STATUS2 = 0x37;        // 中断状态寄存器 2
  static constexpr uint8_t UB0_REG_INT_STATUS3 = 0x38;        // 中断状态寄存器 3
  static constexpr uint8_t UB0_REG_SIGNAL_PATH_RESET = 0x4B;  // 信号路径重置
  static constexpr uint8_t UB0_REG_INTF_CONFIG0 = 0x4C;       // 接口配置寄存器 0
  static constexpr uint8_t UB0_REG_INTF_CONFIG1 = 0x4D;       // 接口配置寄存器 1
  static constexpr uint8_t UB0_REG_PWR_MGMT0 = 0x4E;          // 电源管理寄存器
  static constexpr uint8_t UB0_REG_GYRO_CONFIG0 = 0x4F;       // 陀螺仪配置寄存器 0
  static constexpr uint8_t UB0_REG_ACCEL_CONFIG0 = 0x50;      // 加速度计配置寄存器 0
  static constexpr uint8_t UB0_REG_GYRO_CONFIG1 = 0x51;       // 陀螺仪配置寄存器 1
  static constexpr uint8_t UB0_REG_GYRO_ACCEL_CONFIG0 = 0x52; // 陀螺仪和加速度计联合配置
  static constexpr uint8_t UB0_REG_ACCEL_CONFIG1 = 0x53;      // 加速度计配置寄存器 1
  static constexpr uint8_t UB0_REG_TMST_CONFIG = 0x54;        // 时间戳配置
  static constexpr uint8_t UB0_REG_APEX_CONFIG0 = 0x56;       // APEX 配置寄存器 0
  static constexpr uint8_t UB0_REG_SMD_CONFIG = 0x57;         // 显著运动检测配置
  static constexpr uint8_t UB0_REG_FIFO_CONFIG1 = 0x5F;       // FIFO 配置寄存器 1
  static constexpr uint8_t UB0_REG_FIFO_CONFIG2 = 0x60;       // FIFO 配置寄存器 2
  static constexpr uint8_t UB0_REG_FIFO_CONFIG3 = 0x61;       // FIFO 配置寄存器 3
  static constexpr uint8_t UB0_REG_FSYNC_CONFIG = 0x62;       // 同步配置寄存器
  static constexpr uint8_t UB0_REG_INT_CONFIG0 = 0x63;        // 中断配置寄存器 0
  static constexpr uint8_t UB0_REG_INT_CONFIG1 = 0x64;        // 中断配置寄存器 1
  static constexpr uint8_t UB0_REG_INT_SOURCE0 = 0x65;        // 中断源寄存器 0
  static constexpr uint8_t UB0_REG_INT_SOURCE1 = 0x66;        // 中断源寄存器 1
  static constexpr uint8_t UB0_REG_INT_SOURCE3 = 0x68;        // 中断源寄存器 3
  static constexpr uint8_t UB0_REG_INT_SOURCE4 = 0x69;        // 中断源寄存器 4
  static constexpr uint8_t UB0_REG_FIFO_LOST_PKT0 = 0x6C;     // FIFO 丢失数据包计数高字节
  static constexpr uint8_t UB0_REG_FIFO_LOST_PKT1 = 0x6D;     // FIFO 丢失数据包计数低字节
  static constexpr uint8_t UB0_REG_SELF_TEST_CONFIG = 0x70;   // 自检配置寄存器
  static constexpr uint8_t UB0_REG_WHO_AM_I = 0x75;           // 设备身份寄存器

  // **用户寄存器组 1 (User Bank 1)** 寄存器定义
  static constexpr uint8_t UB1_REG_SENSOR_CONFIG0 = 0x03;       // 传感器配置寄存器 0
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC2 = 0x0B;  // 陀螺仪静态配置 2
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC3 = 0x0C;  // 陀螺仪静态配置 3
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC4 = 0x0D;  // 陀螺仪静态配置 4
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC5 = 0x0E;  // 陀螺仪静态配置 5
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC6 = 0x0F;  // 陀螺仪静态配置 6
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC7 = 0x10;  // 陀螺仪静态配置 7
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC8 = 0x11;  // 陀螺仪静态配置 8
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC9 = 0x12;  // 陀螺仪静态配置 9
  static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC10 = 0x13; // 陀螺仪静态配置 10
  static constexpr uint8_t UB1_REG_XG_ST_DATA = 0x5F;           // X轴陀螺仪自检数据
  static constexpr uint8_t UB1_REG_YG_ST_DATA = 0x60;           // Y轴陀螺仪自检数据
  static constexpr uint8_t UB1_REG_ZG_ST_DATA = 0x61;           // Z轴陀螺仪自检数据
  static constexpr uint8_t UB1_REG_TMSTVAL0 = 0x62;             // 时间戳值低字节
  static constexpr uint8_t UB1_REG_TMSTVAL1 = 0x63;             // 时间戳值中字节
  static constexpr uint8_t UB1_REG_TMSTVAL2 = 0x64;             // 时间戳值高字节
  static constexpr uint8_t UB1_REG_INTF_CONFIG4 = 0x7A;         // 接口配置寄存器 4
  static constexpr uint8_t UB1_REG_INTF_CONFIG5 = 0x7B;         // 接口配置寄存器 5
  static constexpr uint8_t UB1_REG_INTF_CONFIG6 = 0x7C;         // 接口配置寄存器 6

  // **用户寄存器组 2 (User Bank 2)** 寄存器定义
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC2 = 0x03; // 加速度计静态配置 2
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC3 = 0x04; // 加速度计静态配置 3
  static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC4 = 0x05; // 加速度计静态配置 4
  static constexpr uint8_t UB2_REG_XA_ST_DATA = 0x3B;           // X轴加速度计自检数据
  static constexpr uint8_t UB2_REG_YA_ST_DATA = 0x3C;           // Y轴加速度计自检数据
  static constexpr uint8_t UB2_REG_ZA_ST_DATA = 0x3D;           // Z轴加速度计自检数据

  // **用户寄存器组 4 (User Bank 4)** 寄存器定义
  static constexpr uint8_t UB4_REG_APEX_CONFIG1 = 0x40; // APEX 配置寄存器 1，控制 APEX 功能的基本设置
  static constexpr uint8_t UB4_REG_APEX_CONFIG2 = 0x41; // APEX 配置寄存器 2，进一步设置 APEX 参数
  static constexpr uint8_t UB4_REG_APEX_CONFIG3 = 0x42; // APEX 配置寄存器 3，扩展配置选项
  static constexpr uint8_t UB4_REG_APEX_CONFIG4 = 0x43; // APEX 配置寄存器 4，用于活动检测
  static constexpr uint8_t UB4_REG_APEX_CONFIG5 = 0x44; // APEX 配置寄存器 5，支持倾斜检测
  static constexpr uint8_t UB4_REG_APEX_CONFIG6 = 0x45; // APEX 配置寄存器 6，控制步态检测相关功能
  static constexpr uint8_t UB4_REG_APEX_CONFIG7 = 0x46; // APEX 配置寄存器 7，进一步优化唤醒检测
  static constexpr uint8_t UB4_REG_APEX_CONFIG8 = 0x47; // APEX 配置寄存器 8，用于 Tap 检测
  static constexpr uint8_t UB4_REG_APEX_CONFIG9 = 0x48; // APEX 配置寄存器 9，扩展 APEX 功能的设置
  // break
  static constexpr uint8_t UB4_REG_ACCEL_WOM_X_THR = 0x4A; // X 轴加速度计唤醒运动阈值
  static constexpr uint8_t UB4_REG_ACCEL_WOM_Y_THR = 0x4B; // Y 轴加速度计唤醒运动阈值
  static constexpr uint8_t UB4_REG_ACCEL_WOM_Z_THR = 0x4C; // Z 轴加速度计唤醒运动阈值
  static constexpr uint8_t UB4_REG_INT_SOURCE6 = 0x4D;     // 中断源 6，报告唤醒事件
  static constexpr uint8_t UB4_REG_INT_SOURCE7 = 0x4E;     // 中断源 7，报告显著运动事件
  static constexpr uint8_t UB4_REG_INT_SOURCE8 = 0x4F;     // 中断源 8，与加速度计相关的中断
  static constexpr uint8_t UB4_REG_INT_SOURCE9 = 0x50;     // 中断源 9，与陀螺仪相关的中断
  static constexpr uint8_t UB4_REG_INT_SOURCE10 = 0x51;    // 中断源 10，其他事件中断
  // break
  static constexpr uint8_t UB4_REG_OFFSET_USER0 = 0x77; // 用户偏移寄存器 0，X 轴陀螺仪的低字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER1 = 0x78; // 用户偏移寄存器 1，X 轴陀螺仪的高字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER2 = 0x79; // 用户偏移寄存器 2，Y 轴陀螺仪的低字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER3 = 0x7A; // 用户偏移寄存器 3，Y 轴陀螺仪的高字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER4 = 0x7B; // 用户偏移寄存器 4，Z 轴陀螺仪的低字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER5 = 0x7C; // 用户偏移寄存器 5，Z 轴陀螺仪的高字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER6 = 0x7D; // 用户偏移寄存器 6，X 轴加速度计的低字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER7 = 0x7E; // 用户偏移寄存器 7，X 轴加速度计的高字节偏移
  static constexpr uint8_t UB4_REG_OFFSET_USER8 = 0x7F; // 用户偏移寄存器 8，Z 轴加速度计的低字节偏移

} // ns ICM42688reg

#endif // ICM42688_REGISTERS_H_
