/**
 * @file DETA100_MODULE.h
 * @brief DETA100 惯性导航模块驱动程序
 *
 * 本文件实现了与 DETA100 惯性导航模块的串口通信协议解析。
 * DETA100 模块提供 AHRS 姿态、INS/GNSS 组合导航、大地坐标位置和系统状态数据。
 *
 * 串口配置: Serial4, 921600 波特率
 * 协议格式: [帧头0xFC(1B)] [类型(1B)] [长度(1B)] [数据(N B)] [CRC8(1B)] [CRC16(2B)] [帧尾0xFD(1B)]
 *
 * 注意: 本文件包含函数实现（非 inline），只能在 main.cpp 中包含，
 * 不能被多个 .cpp 文件包含，否则会导致链接重复定义错误。
 *
 * @version 2.0
 * @date [2025.4.29]
 */

 #include "MedianFilter.h"
#include "deta100_types.h"

 #ifndef IMU_MODULE_H
 #define IMU_MODULE_H

 // [模块化] 滤波器和数据包实例已迁移至 state_data.h/cpp，此处不再定义
 
 // [模块化] 协议常量、枚举、结构体定义已迁移至 deta100_types.h
 // ================== 枚举类型定义 ==================
 /**
  * @brief GNSS定位状态枚举
  * @details 定义了GNSS定位的不同状态，对应于StatusPacket中filter_status的位[7:4]
  */

 // [模块化] 类型定义和变量定义已迁移至 deta100_types.h 和 state_data.h/cpp

 // ================== 函数实现 ==================
 
 // CRC8查找表
 static const uint8_t CRC8Table[256] = {
     0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
     157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
     35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
     190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
     70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
     219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
     101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
     248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
     140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
     17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
     175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
     50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
     202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
     87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
     233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
     116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};
 
 /**
  * @brief 计算 CRC8 校验值
  *
  * 使用预计算的 CRC8 查找表快速计算数据的 CRC8 校验值。
  * 用于 DETA100 协议帧头校验 (前4字节: 帧头+类型+长度)。
  *
  * @param data   待校验数据指针
  * @param length 待校验数据长度 (字节)
  * @return CRC8 校验值
  */
 uint8_t calculateCRC8(const uint8_t *data, uint8_t length)
 {
     uint8_t crc = 0; // CRC 初始值为 0
     for (uint8_t i = 0; i < length; i++)
     {
         crc = CRC8Table[crc ^ data[i]]; // 查表法: 当前CRC与数据字节异或后查表
     }
     return crc;
 }
 
 // CRC16查找表
 static const uint16_t CRC16Table[256] = {
     0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
     0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
     0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
     0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
     0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
     0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
     0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
     0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
     0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
     0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
     0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
     0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
     0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
     0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
     0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
     0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78};
 
 /**
  * @brief 计算CRC16校验
  * @param data 数据指针
  * @param length 数据长度
  * @return 计算得到的CRC16校验值
  */
 uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
 {
     uint16_t crc = 0; // 初始值为0
     for (uint16_t i = 0; i < length; i++)
     {
         crc = CRC16Table[((crc >> 8) ^ data[i]) & 0xFF] ^ (crc << 8);
     }
     return crc;
 }
 
 /**
  * @brief 将4字节十六进制数据转换为浮点数
  * @param data 指向数据的指针
  * @param isLittleEndian 是否为小端序
  * @return float 转换后的浮点数
  */
 float hexToFloat(const uint8_t *data, bool isLittleEndian)
 {
     union
     {
         float f;
         uint8_t bytes[4];
     } converter;
 
     if (isLittleEndian)
     {
         // 小端序：直接复制字节数据
         memcpy(converter.bytes, data, 4);
     }
     else
     {
         // 大端序：反转字节顺序
         for (int i = 0; i < 4; i++)
         {
             converter.bytes[i] = data[3 - i];
         }
     }
     return converter.f; // 返回转换得到的浮点数
 }
 
 /**
  * @brief 将8字节十六进制数据转换为双精度浮点数
  * @param data 指向数据的指针
  * @param isLittleEndian 是否为小端序
  * @return double 转换后的双精度浮点数
  */
 double hexToDouble(const uint8_t *data, bool isLittleEndian)
 {
     union
     {
         double d;
         uint8_t bytes[8];
     } converter;
 
     if (isLittleEndian)
     {
         // 小端序：直接复制字节数据
         memcpy(converter.bytes, data, 8);
     }
     else
     {
         // 大端序：反转字节顺序
         for (int i = 0; i < 8; i++)
         {
             converter.bytes[i] = data[7 - i];
         }
     }
     return converter.d; // 返回转换得到的双精度浮点数
 }
 
//  /**
//   * @brief 解析时间戳
//   * @param data 指向时间戳数据的指针
//   * @return uint32_t 解析后的时间戳（微秒）
//   */
//  uint32_t parseTimestamp(const uint8_t *data)
//  {
//      uint32_t timestamp = 0;
//      for (int i = 0; i < 4; i++)
//      {
//          timestamp |= (uint32_t)data[i] << (i * 8); // 逐字节拼接为32位时间戳
//      }
//      return timestamp; // 返回解析后的时间戳
//  }
 
/**
 * @brief 解析8字节小端序时间戳（微秒）
 * @param data 指向8字节时间戳数据的指针（小端序，低位在前）
 * @return uint64_t 解析后的时间戳（单位：微秒）
 */
uint64_t parseTimestamp(const uint8_t *data) {
    uint64_t timestamp = 0; // 使用64位变量存储8字节数据
    
    // 小端序拼接：低位字节（data[0]）对应0-7位，依次递增
    for (int i = 0; i < 8; i++) {
        timestamp |= (uint64_t)data[i] << (i * 8); // 左移i*8位，拼接字节
    }
    
    return timestamp; // 返回64位无符号整数
}

 
 /**
  * @brief 从数据包中解析浮点数
  * @param data 数据包指针
  * @param offset 数据偏移量
  * @return float 解析后的浮点数
  */
 float parseFloatFromData(const uint8_t *data, int offset)
 {
     return hexToFloat(data + offset, true); // DETA100 统一为小端序，低位在前高位在后
 }
 
 /**
  * @brief 从数据包中解析双精度浮点数
  * @param data 数据包指针
  * @param offset 数据偏移量
  * @return double 解析后的双精度浮点数
  */
 double parseDoubleFromData(const uint8_t *data, int offset)
 {
     return hexToDouble(data + offset, true); // DETA100 统一为小端序，低位在前高位在后
 }
 
 /**
  * @brief 帧缓冲区重同步
  *
  * 当帧校验失败或类型未知时，在已接收的缓冲区中搜索下一个帧头 (0xFC)。
  * 找到后将后续数据前移，保留可能的下一帧数据，避免丢弃有效字节。
  *
  * @param Count 当前缓冲区中的字节数 (会被修改)
  * @return true 找到下一个帧头并已前移, false 未找到需从头开始
  */
 bool resyncBuffer(uint16_t &Count)
 {
     for (uint16_t i = 1; i < Count; i++)
     {
         if (Fd_data[i] == FRAME_HEAD)
         {
             memmove(Fd_data, Fd_data + i, Count - i);
             Count -= i;
             return true;
         }
     }
     Count = 0;
     return false;
 }
 
 /**
  * @brief 读取并解析 DETA100 串口数据帧
  *
  * 实现了高效的串口协议解析状态机：
  * - WAIT_FOR_HEAD: 等待帧头 (0xFC)
  * - COLLECT_DATA: 收集数据直到完整一帧
  *
  * 解析流程：等待帧头 -> 读取类型+长度确定帧长 -> 收集数据 ->
  * 校验帧尾(0xFD)+CRC8+CRC16 -> 拷贝数据到原始缓冲区 -> 设置就绪标志
  *
  * @param serial DETA100 连接的串口对象引用 (Serial4)
  */
 void Read_DETA100Data(HardwareSerial &serial)
 {
     static uint16_t Count = 0;
     static uint16_t expectedLength = 0;
     static enum { WAIT_FOR_HEAD,
                   COLLECT_DATA } state = WAIT_FOR_HEAD;
 
     while (serial.available() > 0)
     {
         uint8_t b = serial.read();
 
         if (state == WAIT_FOR_HEAD)
         {
             if (b == FRAME_HEAD)
             {
                 Fd_data[0] = b;
                 Count = 1;
                 expectedLength = 0;
                 state = COLLECT_DATA;
             }
             continue;
         }
 
         // COLLECT_DATA
         Fd_data[Count++] = b;
 
         // 第三字节到达后确定 expectedLength
         if (Count == 3)
         {
             uint8_t type = Fd_data[1];
             uint8_t payloadLen = Fd_data[2];
             uint8_t fixedLen = 0;
 
             switch (type)
             {
             case TYPE_IMU:
                 fixedLen = IMU_LEN;
                 break;
             case TYPE_AHRS:
                 fixedLen = AHRS_LEN;
                 break;
             case TYPE_INS_GNSS:
                 fixedLen = INS_GNSS_LEN;
                 break;
             case TYPE_GEODETIC_POS:
                 fixedLen = GEODETIC_POS_LEN;
                 break;
             case TYPE_STATUS:
                 fixedLen = STATUS_LEN;
                 break;
             default:
                 if (!resyncBuffer(Count))
                     state = WAIT_FOR_HEAD;
                 continue;
             }
 
             if (payloadLen != fixedLen)
             {
                 if (!resyncBuffer(Count))
                     state = WAIT_FOR_HEAD;
                 continue;
             }
 
             expectedLength = payloadLen + 8;
         }
 
         // 收齐一帧
         if (expectedLength && Count == expectedLength)
         {
             bool ok = (Fd_data[Count - 1] == FRAME_END);
 
             // 校验 CRC8
             if (ok)
             {
                 uint8_t crc8 = calculateCRC8(Fd_data, 4);
                 if (crc8 != Fd_data[4])
                 {
                     ok = false;
                 }
             }
 
             // 校验 CRC16
             if (ok)
             {
                 uint16_t crc16 = calculateCRC16(&Fd_data[5], expectedLength - 8);
                 uint16_t recv = (Fd_data[expectedLength - 3] << 8) | Fd_data[expectedLength - 2];
                 if (crc16 != recv)
                 {
                     // ok = false;
                 }
             }
 
             // 通过则拷贝
             if (ok)
             {
                 switch (Fd_data[1])
                 {
                 case TYPE_IMU:
                     memcpy(IMU_Data, Fd_data, IMU_TYPE_LEN);
                     Data_of_IMU = true;
                     break;
                 case TYPE_AHRS:
                     memcpy(AHRS_Data, Fd_data, AHRS_TYPE_LEN);
                     Data_of_AHRS = true;
                     break;
                 case TYPE_INS_GNSS:
                     memcpy(INS_GNSS_Data, Fd_data, INS_GNSS_TYPE_LEN);
                     Data_of_INS_GNSS = true;
                     break;
                 case TYPE_GEODETIC_POS:
                     memcpy(Geodetic_Pos_Data, Fd_data, GEODETIC_POS_TYPE_LEN);
                     Data_of_Geodetic_Pos = true;
                     break;
                 case TYPE_STATUS:
                     memcpy(Status_Data, Fd_data, STATUS_TYPE_LEN);
                     Data_of_Status = true;
                     break;
                 }
             }
 
             // 重同步
             if (!resyncBuffer(Count))
                 state = WAIT_FOR_HEAD;
             else
                 state = COLLECT_DATA;
         }
         else if (expectedLength && Count > expectedLength)
         {
             // 超长直接重同步
             if (!resyncBuffer(Count))
                 state = WAIT_FOR_HEAD;
             else
                 state = COLLECT_DATA;
         }
     }
 }
 
 /**
  * @brief 数据解包函数
  *
  * 将原始字节缓冲区中的数据解析为结构化的数据包：
  * - IMU_Data -> IMU_Packet: 陀螺仪、加速度计、磁力计、温度、气压、时间戳
  * - AHRS_Data -> AHRS_Packet: 角速度、欧拉角(带中值滤波)、四元数、时间戳
  * - INS_GNSS_Data -> INS_GNSS_Packet: 机体系/NED系速度/加速度/位置、气压高度
  * - Geodetic_Pos_Data -> Geodetic_Pos_Packet: 经纬高(双精度)、定位精度
  * - Status_Data -> Status_Packet: 系统故障状态、滤波器状态位域
  *
  * 所有浮点数使用小端序解析 (DETA100协议统一小端序)。
  * 解析完成后清除对应的 Data_of_* 标志位。
  */
 void DataUnpacking()
 {
     if (Data_of_IMU)
     {
         // 解析IMU数据包中的各项数据
         IMU_Packet.gyroscope_x = parseFloatFromData(IMU_Data, 7);
         IMU_Packet.gyroscope_y = parseFloatFromData(IMU_Data, 11);
         IMU_Packet.gyroscope_z = parseFloatFromData(IMU_Data, 15);
         IMU_Packet.accelerometer_x = parseFloatFromData(IMU_Data, 19);
         IMU_Packet.accelerometer_y = parseFloatFromData(IMU_Data, 23);
         IMU_Packet.accelerometer_z = parseFloatFromData(IMU_Data, 27);
         IMU_Packet.magnetometer_x = parseFloatFromData(IMU_Data, 31);
         IMU_Packet.magnetometer_y = parseFloatFromData(IMU_Data, 35);
         IMU_Packet.magnetometer_z = parseFloatFromData(IMU_Data, 39);
         IMU_Packet.imu_temperature = parseFloatFromData(IMU_Data, 43);
         IMU_Packet.Pressure = parseFloatFromData(IMU_Data, 47);
         IMU_Packet.pressure_temperature = parseFloatFromData(IMU_Data, 51);
         IMU_Packet.Timestamp = parseTimestamp(&IMU_Data[55]);
         Data_of_IMU = false; // 清除数据就绪标志
     }
 
     if (Data_of_AHRS)
     {
         // 解析AHRS数据包中的各项数据
         AHRS_Packet.RollSpeed = parseFloatFromData(AHRS_Data, 7);
         AHRS_Packet.PitchSpeed = parseFloatFromData(AHRS_Data, 11);
         AHRS_Packet.HeadingSpeed = parseFloatFromData(AHRS_Data, 15);
         AHRS_Packet.Roll = parseFloatFromData(AHRS_Data, 19);
         AHRS_Packet.Roll = RollFilter.filter(AHRS_Packet.Roll); // 对滚转角进行中值滤波处理
         AHRS_Packet.Pitch = parseFloatFromData(AHRS_Data, 23);
         AHRS_Packet.Pitch = PitchFilter.filter(AHRS_Packet.Pitch); // 对俯仰角进行中值滤波处理
         AHRS_Packet.Heading = parseFloatFromData(AHRS_Data, 27);
         AHRS_Packet.Heading = HeadingFilter.filter(AHRS_Packet.Heading); // 对航向角进行中值滤波处理
         AHRS_Packet.Qw = parseFloatFromData(AHRS_Data, 31);
         AHRS_Packet.Qx = parseFloatFromData(AHRS_Data, 35);
         AHRS_Packet.Qy = parseFloatFromData(AHRS_Data, 39);
         AHRS_Packet.Qz = parseFloatFromData(AHRS_Data, 43);
         AHRS_Packet.Timestamp = parseTimestamp(&AHRS_Data[47]);
         Data_of_AHRS = false; // 清除数据就绪标志
     }
 
     if (Data_of_INS_GNSS)
     {
         // 解析INS/GNSS数据包中的各项数据
         INS_GNSS_Packet.body_velocity_x = parseFloatFromData(INS_GNSS_Data, 7);
         INS_GNSS_Packet.body_velocity_y = parseFloatFromData(INS_GNSS_Data, 11);
         INS_GNSS_Packet.body_velocity_z = parseFloatFromData(INS_GNSS_Data, 15);
         INS_GNSS_Packet.body_acceleration_x = parseFloatFromData(INS_GNSS_Data, 19);
         INS_GNSS_Packet.body_acceleration_y = parseFloatFromData(INS_GNSS_Data, 23);
         INS_GNSS_Packet.body_acceleration_z = parseFloatFromData(INS_GNSS_Data, 27);
         INS_GNSS_Packet.location_north = parseFloatFromData(INS_GNSS_Data, 31);
         INS_GNSS_Packet.location_east = parseFloatFromData(INS_GNSS_Data, 35);
         INS_GNSS_Packet.location_down = parseFloatFromData(INS_GNSS_Data, 39);
         INS_GNSS_Packet.velocity_north = parseFloatFromData(INS_GNSS_Data, 43);
         INS_GNSS_Packet.velocity_east = parseFloatFromData(INS_GNSS_Data, 47);
         INS_GNSS_Packet.velocity_down = parseFloatFromData(INS_GNSS_Data, 51);
         INS_GNSS_Packet.acceleration_north = parseFloatFromData(INS_GNSS_Data, 55);
         INS_GNSS_Packet.acceleration_east = parseFloatFromData(INS_GNSS_Data, 59);
         INS_GNSS_Packet.acceleration_down = parseFloatFromData(INS_GNSS_Data, 63);
         INS_GNSS_Packet.pressure_altitude = parseFloatFromData(INS_GNSS_Data, 67);
         INS_GNSS_Packet.timestamp = parseTimestamp(&INS_GNSS_Data[71]);
         Data_of_INS_GNSS = false; // 清除数据就绪标志
     }
 
     if (Data_of_Geodetic_Pos)
     {
         // 解析 GeodeticPos 数据包 (0x5C)
         Geodetic_Pos_Packet.latitude = parseDoubleFromData(Geodetic_Pos_Data, 7);   // 纬度, 注意这里是双精度
         Geodetic_Pos_Packet.longitude = parseDoubleFromData(Geodetic_Pos_Data, 15); // 经度, 注意这里是双精度
         Geodetic_Pos_Packet.height = parseDoubleFromData(Geodetic_Pos_Data, 23);    // 高度, 注意这里是双精度
         Geodetic_Pos_Packet.hAcc = parseFloatFromData(Geodetic_Pos_Data, 31);       // 水平定位精度
         Geodetic_Pos_Packet.vAcc = parseFloatFromData(Geodetic_Pos_Data, 35);       // 垂直定位精度
         Data_of_Geodetic_Pos = false;                                               // 清除数据就绪标志
     }
 
     if (Data_of_Status)
     {
         // 解析 Status 数据包 (0x53)
         uint16_t systemStatus = (Status_Data[7] << 8) | Status_Data[8];
         uint16_t filterStatus = (Status_Data[9] << 8) | Status_Data[10];
 
         // 解析系统状态 (11.4.2 节)
         Status_Packet.system_status.system_failure = (systemStatus >> 0) & 0x01;
         Status_Packet.system_status.accelerometer_sensor_failure = (systemStatus >> 1) & 0x01;
         Status_Packet.system_status.gyroscope_sensor_failure = (systemStatus >> 2) & 0x01;
         Status_Packet.system_status.magnetometer_sensor_failure = (systemStatus >> 3) & 0x01;
         Status_Packet.system_status.pressure_sensor_failure = (systemStatus >> 4) & 0x01;
         Status_Packet.system_status.gnss_failure = (systemStatus >> 5) & 0x01;
         Status_Packet.system_status.accelerometer_over_range = (systemStatus >> 6) & 0x01;
         Status_Packet.system_status.gyroscope_over_range = (systemStatus >> 7) & 0x01;
         Status_Packet.system_status.magnetometer_over_range = (systemStatus >> 8) & 0x01;
         Status_Packet.system_status.pressure_over_range = (systemStatus >> 9) & 0x01;
         Status_Packet.system_status.minimum_temperature_alarm = (systemStatus >> 10) & 0x01;
         Status_Packet.system_status.maximum_temperature_alarm = (systemStatus >> 11) & 0x01;
         Status_Packet.system_status.low_voltage_alarm = (systemStatus >> 12) & 0x01;
         Status_Packet.system_status.high_voltage_alarm = (systemStatus >> 13) & 0x01;
         Status_Packet.system_status.gnss_antenna_disconnected = (systemStatus >> 14) & 0x01;
         Status_Packet.system_status.data_output_overflow_alarm = (systemStatus >> 15) & 0x01;
 
         // 解析滤波器状态 (11.4.3 节)
         Status_Packet.filter_status.orientation_filter_initialised = (filterStatus >> 0) & 0x01;
         Status_Packet.filter_status.navigation_filter_initialised = (filterStatus >> 1) & 0x01;
         Status_Packet.filter_status.heading_initialised = (filterStatus >> 2) & 0x01;
         Status_Packet.filter_status.utc_time_initialised = (filterStatus >> 3) & 0x01;
         Status_Packet.filter_status.gnss_fix_status = static_cast<GPSFixType>((filterStatus & 0xF0) >> 4);
         Status_Packet.filter_status.event_occurred = (filterStatus >> 8) & 0x01;
         Status_Packet.filter_status.internal_gnss_enabled = (filterStatus >> 9) & 0x01;
         Status_Packet.filter_status.magnetic_heading_active = (filterStatus >> 10) & 0x01;
         Status_Packet.filter_status.velocity_heading_enabled = (filterStatus >> 11) & 0x01;
         Status_Packet.filter_status.atmospheric_altitude_enabled = (filterStatus >> 12) & 0x01;
         Status_Packet.filter_status.external_position_active = (filterStatus >> 13) & 0x01;
         Status_Packet.filter_status.external_velocity_active = (filterStatus >> 14) & 0x01;
         Status_Packet.filter_status.external_heading_active = (filterStatus >> 15) & 0x01;
 
         Data_of_Status = false; // 清除数据就绪标志
     }
 }
 
 #endif // IMU_MODULE_H
 
