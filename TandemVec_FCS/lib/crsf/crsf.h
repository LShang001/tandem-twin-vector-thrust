/**
 * @file crsf.h
 *
 * CRSF（TBS Crossfire）遥控通信协议定义。
 * 这是一种非反转信号的通信协议，波特率为420000。
 *
 * 遥控通道的数据传输频率为150Hz。
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <stdint.h>

// CRSF帧的最大长度是30，实际最大长度是64，但为了减小缓冲区大小，只关注遥控通道数据
#define CRSF_FRAME_SIZE_MAX 30
#define CRSF_PAYLOAD_SIZE_MAX (CRSF_FRAME_SIZE_MAX - 4) // 有效载荷的最大大小

struct crsf_frame_header_t
{
	uint8_t device_address; ///< 设备地址，参见 crsf_address_t
	uint8_t length;			///< crsf_frame_t的长度（包括CRC）减去sizeof(crsf_frame_header_t)
};

struct crsf_frame_t
{
	crsf_frame_header_t header;					///< 帧头
	uint8_t type;								///< 帧类型，参见 crsf_frame_type_t
	uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; ///< 有效载荷数据，包括末尾的1字节CRC
};

#define RC_INPUT_MAX_CHANNELS 16 // 系统中R/C输入通道的最大数量。例如，S.Bus最多有18个通道。

/**
 * 配置一个UART端口用于CRSF
 * @param uart_fd UART文件描述符
 * @return 成功时返回0，否则返回-errno
 */
int crsf_config(int uart_fd);

/**
 * 解析CRSF协议，提取RC通道数据。
 *
 * @param frame 待解析的数据帧
 * @param len 数据帧长度
 * @param values 输出通道值，每个通道范围 [1000, 2000]
 * @param num_values 设置为在values中解析到的通道数量
 * @param max_channels values的最大长度
 * @return 如果成功解码通道，则返回true
 */
bool crsf_parse(const uint8_t *frame, unsigned len, uint16_t *values,
				uint16_t *num_values, uint16_t max_channels);

/**
 * 发送遥测电池信息
 * @param uart_fd UART文件描述符
 * @param voltage 电压 [0.1V单位]
 * @param current 电流 [0.1A单位]
 * @param fuel 已消耗的mAh
 * @param remaining 电池剩余百分比
 * @return 成功时返回true
 */
bool crsf_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining);

/**
 * 发送遥测GPS信息
 * @param uart_fd UART文件描述符
 * @param latitude 纬度 [度 * 1e7]
 * @param longitude 经度 [度 * 1e7]
 * @param groundspeed 地面速度 [km/h * 10]
 * @param gps_heading GPS航向 [度 * 100]
 * @param altitude 高度 [米 + 1000米偏移]
 * @param num_satellites 使用的卫星数
 * @return 成功时返回true
 */
bool crsf_send_telemetry_gps(int uart_fd, int32_t latitude, int32_t longitude, uint16_t groundspeed,
							 uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);

/**
 * 发送遥测姿态信息
 * @param uart_fd UART文件描述符
 * @param pitch 俯仰角 [弧度 * 1e4]
 * @param roll 滚转角 [弧度 * 1e4]
 * @param yaw 偏航角 [弧度 * 1e4]
 * @return 成功时返回true
 */
bool crsf_send_telemetry_attitude(int uart_fd, int16_t pitch, int16_t roll, int16_t yaw);

/**
 * 发送遥测飞行模式信息
 * @param uart_fd UART文件描述符
 * @param flight_mode 飞行模式字符串（最大长度=15）
 * @return 成功时返回true
 */
bool crsf_send_telemetry_flight_mode(int uart_fd, const char *flight_mode);