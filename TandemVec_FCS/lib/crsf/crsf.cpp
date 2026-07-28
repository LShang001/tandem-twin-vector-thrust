/****************************************************************************
 *
 *   版权所有 2018 PX4 开发团队。版权所有。
 *
 * 根据下述条件，无论是否经过修改，均授权使用源代码和二进制形式的再分发和使用：
 *
 * 1. 源代码的再分发必须保留上述版权声明、此条件列表和以下免责声明。
 * 2. 以二进制形式再分发必须复制上述版权声明、此条件列表以及随分发提供的文档和/或其他材料中的免责声明。
 * 3. 未经具体事先书面许可，不得使用PX4的名称及其贡献者的名称来支持或宣传源自本软件的产品。
 *
 * 本软件由版权持有人和贡献者“按原样”提供，任何明示或暗示的保证，包括但不限于对商业性和特定目的适用性的暗示保证，均不予承认。
 * 在任何情况下，版权持有者或贡献者均不对任何直接、间接、偶然、特殊、惩罚性或后果性损害（包括但不限于替代商品或服务的购买；
 * 使用、数据或利润的损失；或业务中断）负责，无论这些损害是如何引起的，以及根据任何责任理论，包括合同、严格责任或侵权行为（包括疏忽或其他）在任何使用本软件的情况下所引起的。
 *
 ****************************************************************************/

// 防止头文件重复包含的预处理指令
#ifndef CRSF_HEADER_INCLUDED
#define CRSF_HEADER_INCLUDED 1

#if 0 // 启用非详细调试
#define CRSF_DEBUG PX4_WARN
#else
// 定义空的调试宏函数，以便在不需要调试输出时，减少输出量并提高效率
#define CRSF_DEBUG(...)
#endif

#if 0 // 启用详细调试。注意: 启用时可能会因为大量输出导致丢失字节
#define CRSF_VERBOSE PX4_WARN
#else
// 定义空的详细调试宏函数
#define CRSF_VERBOSE(...)
#endif

// 引入所需的头文件
// #include <termios.h>
#include <string.h>
#include <unistd.h>

// 引入其他相关头文件
#include "crsf.h"
#include "common_rc.h"

// 最小值和最大值的定义
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

// 定义CRSF协议的波特率
#define CRSF_BAUDRATE 420000

// CRSF同步字节的定义
#define CRSF_SYNC_BYTE 0xC8

// 定义CRSF帧类型的枚举
enum class crsf_frame_type_t : uint8_t
{
	gps = 0x02,
	battery_sensor = 0x08,
	link_statistics = 0x14,
	rc_channels_packed = 0x16,
	attitude = 0x1E,
	flight_mode = 0x21,

	// 扩展头帧范围: 0x28 to 0x96
	device_ping = 0x28,
	device_info = 0x29,
	parameter_settings_entry = 0x2B,
	parameter_read = 0x2C,
	parameter_write = 0x2D,
	command = 0x32
};

// 定义CRSF负载大小的枚举
enum class crsf_payload_size_t : uint8_t
{
	gps = 15,
	battery_sensor = 8,
	link_statistics = 10,
	rc_channels = 22, ///< 11位每通道 * 16通道 = 22字节。
	attitude = 6,
};

// 定义CRSF设备地址的枚举
enum class crsf_address_t : uint8_t
{
	broadcast = 0x00,
	usb = 0x10,
	tbs_core_pnp_pro = 0x80,
	reserved1 = 0x8A,
	current_sensor = 0xC0,
	gps = 0xC2,
	tbs_blackbox = 0xC4,
	flight_controller = 0xC8,
	reserved2 = 0xCA,
	race_tag = 0xCC,
	radio_transmitter = 0xEA,
	crsf_receiver = 0xEC,
	crsf_transmitter = 0xEE
};

// 打包数据结构，确保没有字节对齐的空隙
#pragma pack(push, 1)
struct crsf_payload_RC_channels_packed_t
{
	// 176 bits的数据（11 bits 每通道 * 16通道）= 22字节
	unsigned chan0 : 11;
	unsigned chan1 : 11;
	unsigned chan2 : 11;
	unsigned chan3 : 11;
	unsigned chan4 : 11;
	unsigned chan5 : 11;
	unsigned chan6 : 11;
	unsigned chan7 : 11;
	unsigned chan8 : 11;
	unsigned chan9 : 11;
	unsigned chan10 : 11;
	unsigned chan11 : 11;
	unsigned chan12 : 11;
	unsigned chan13 : 11;
	unsigned chan14 : 11;
	unsigned chan15 : 11;
};
#pragma pack(pop)

// 定义CRSF解析器状态的枚举
enum class crsf_parser_state_t : uint8_t
{
	unsynced = 0,
	synced
};

// 静态变量定义
static crsf_frame_t &crsf_frame = rc_decode_buf.crsf_frame;				 // CRFS帧数据
static unsigned current_frame_position = 0;								 // 当前帧位置
static crsf_parser_state_t parser_state = crsf_parser_state_t::unsynced; // 解析器状态

/**
 * 解析当前的crsf帧缓冲区
 */
static bool crsf_parse_buffer(uint16_t *values, uint16_t *num_values, uint16_t max_channels);

// CRSF帧CRC计算函数的声明
uint8_t crsf_frame_CRC(const crsf_frame_t &frame);

// int crsf_config(int uart_fd)
// {
// 	struct termios t;

// 	/* no parity, one stop bit */
// 	tcgetattr(uart_fd, &t);
// 	t.c_cflag &= ~(CSTOPB | PARENB);
// 	return tcsetattr(uart_fd, TCSANOW, &t);
// }

/**
 * 解析CRSF帧数据。
 *
 * @param frame 指向待解析CRSF帧数据的指针。
 * @param len 待解析数据的长度。
 * @param values 解析出的值存储数组的指针。
 * @param num_values 解析出的值的数量指针。
 * @param max_channels 最多能解析的通道数。
 * @return 如果成功解析至少一个值，则返回true；否则返回false。
 */
bool crsf_parse(const uint8_t *frame, unsigned len, uint16_t *values,
				uint16_t *num_values, uint16_t max_channels)
{
	bool ret = false;								  // 默认返回值为false
	uint8_t *crsf_frame_ptr = (uint8_t *)&crsf_frame; // 用于处理CRSF帧数据的指针

	while (len > 0) // 遍历输入数据直到处理完毕
	{

		// 将输入数据填充到crsf_frame缓冲区，尽量多填充
		const unsigned current_len = MIN(len, sizeof(crsf_frame_t) - current_frame_position);
		memcpy(crsf_frame_ptr + current_frame_position, frame, current_len);
		current_frame_position += current_len; // 更新当前处理位置

		// 确保解析过程有进展，防止死循环
		if (current_len == 0)
		{
			CRSF_DEBUG("========== parser bug: no progress (%i) ===========", len);

			for (unsigned i = 0; i < current_frame_position; ++i)
			{
				CRSF_DEBUG("crsf_frame_ptr[%i]: 0x%x", i, (int)crsf_frame_ptr[i]);
			}

			// 重置解析器状态
			current_frame_position = 0;
			parser_state = crsf_parser_state_t::unsynced;
			return false; // 发生错误，退出解析
		}

		len -= current_len;	  // 更新剩余待处理数据长度
		frame += current_len; // 更新待处理数据的指针

		// 尝试从缓冲区解析值
		if (crsf_parse_buffer(values, num_values, max_channels))
		{
			ret = true; // 成功解析至少一个值
		}
	}

	return ret; // 返回解析结果
}

/**
 * 计算给定crsf_frame_t结构的CRC校验值。
 *
 * @param frame crsf_frame_t结构，包含类型、负载和头部信息。
 * @return uint8_t类型的CRC校验值。
 */
uint8_t crsf_frame_CRC(const crsf_frame_t &frame)
{
	// CRC包括类型和负载
	uint8_t crc = crc8_dvb_s2(0, frame.type);

	for (int i = 0; i < frame.header.length - 2; ++i)
	{
		crc = crc8_dvb_s2(crc, frame.payload[i]);
	}

	return crc;
}

/**
 * 将RC（遥控器控制）值转换为PWM（脉宽调制）值
 * @param chan_value 遥控器通道值，有效范围为[172, 1811]
 * @return PWM通道值，有效范围为[988, 2012]
 */
static uint16_t convert_channel_value(unsigned chan_value)
{
	/*
	 *       RC     PWM
	 * min  172 ->  988us
	 * mid  992 -> 1500us
	 * max 1811 -> 2012us
	 */
	// 计算转换比例和偏移量，实现从RC值到PWM值的线性转换
	static constexpr float scale = (2012.f - 988.f) / (1811.f - 172.f); // 转换比例
	static constexpr float offset = 988.f - 172.f * scale;				// 转换偏移量
	return (scale * chan_value) + offset;								// 应用转换公式得到PWM值
}

static bool crsf_parse_buffer(uint16_t *values, uint16_t *num_values, uint16_t max_channels)
{
	uint8_t *crsf_frame_ptr = (uint8_t *)&crsf_frame;

	if (parser_state == crsf_parser_state_t::unsynced)
	{
		// there is no sync byte, try to find an RC packet by searching for a matching frame length and type
		for (unsigned i = 1; i < current_frame_position - 1; ++i)
		{
			if (crsf_frame_ptr[i] == (uint8_t)crsf_payload_size_t::rc_channels + 2 &&
				crsf_frame_ptr[i + 1] == (uint8_t)crsf_frame_type_t::rc_channels_packed)
			{
				parser_state = crsf_parser_state_t::synced;
				unsigned frame_offset = i - 1;
				CRSF_VERBOSE("RC channels found at offset %i", frame_offset);

				// move the rest of the buffer to the beginning
				if (frame_offset != 0)
				{
					memmove(crsf_frame_ptr, crsf_frame_ptr + frame_offset, current_frame_position - frame_offset);
					current_frame_position -= frame_offset;
				}

				break;
			}
		}
	}

	if (parser_state != crsf_parser_state_t::synced)
	{
		if (current_frame_position >= sizeof(crsf_frame_t))
		{
			// discard most of the data, but keep the last 3 bytes (otherwise we could miss the frame start)
			current_frame_position = 3;

			for (unsigned i = 0; i < current_frame_position; ++i)
			{
				crsf_frame_ptr[i] = crsf_frame_ptr[sizeof(crsf_frame_t) - current_frame_position + i];
			}

			CRSF_VERBOSE("Discarding buffer");
		}

		return false;
	}

	if (current_frame_position < 3)
	{
		// wait until we have the header & type
		return false;
	}

	// Now we have at least the header and the type

	const unsigned current_frame_length = crsf_frame.header.length + sizeof(crsf_frame_header_t);

	if (current_frame_length > sizeof(crsf_frame_t) || current_frame_length < 4)
	{
		// frame too long or bogus -> discard everything and go into unsynced state
		current_frame_position = 0;
		parser_state = crsf_parser_state_t::unsynced;
		CRSF_DEBUG("Frame too long/bogus (%i, type=%i) -> unsync", current_frame_length, crsf_frame.type);
		return false;
	}

	if (current_frame_position < current_frame_length)
	{
		// we don't have the full frame yet -> wait for more data
		CRSF_VERBOSE("waiting for more data (%i < %i)", current_frame_position, current_frame_length);
		return false;
	}

	bool ret = false;

	// Now we have the full frame

	if (crsf_frame.type == (uint8_t)crsf_frame_type_t::rc_channels_packed &&
		crsf_frame.header.length == (uint8_t)crsf_payload_size_t::rc_channels + 2)
	{
		const uint8_t crc = crsf_frame.payload[crsf_frame.header.length - 2];

		if (crc == crsf_frame_CRC(crsf_frame))
		{
			const crsf_payload_RC_channels_packed_t *const rc_channels =
				(crsf_payload_RC_channels_packed_t *)&crsf_frame.payload;
			*num_values = MIN(max_channels, 16);

			if (max_channels > 0)
			{
				values[0] = convert_channel_value(rc_channels->chan0);
			}

			if (max_channels > 1)
			{
				values[1] = convert_channel_value(rc_channels->chan1);
			}

			if (max_channels > 2)
			{
				values[2] = convert_channel_value(rc_channels->chan2);
			}

			if (max_channels > 3)
			{
				values[3] = convert_channel_value(rc_channels->chan3);
			}

			if (max_channels > 4)
			{
				values[4] = convert_channel_value(rc_channels->chan4);
			}

			if (max_channels > 5)
			{
				values[5] = convert_channel_value(rc_channels->chan5);
			}

			if (max_channels > 6)
			{
				values[6] = convert_channel_value(rc_channels->chan6);
			}

			if (max_channels > 7)
			{
				values[7] = convert_channel_value(rc_channels->chan7);
			}

			if (max_channels > 8)
			{
				values[8] = convert_channel_value(rc_channels->chan8);
			}

			if (max_channels > 9)
			{
				values[9] = convert_channel_value(rc_channels->chan9);
			}

			if (max_channels > 10)
			{
				values[10] = convert_channel_value(rc_channels->chan10);
			}

			if (max_channels > 11)
			{
				values[11] = convert_channel_value(rc_channels->chan11);
			}

			if (max_channels > 12)
			{
				values[12] = convert_channel_value(rc_channels->chan12);
			}

			if (max_channels > 13)
			{
				values[13] = convert_channel_value(rc_channels->chan13);
			}

			if (max_channels > 14)
			{
				values[14] = convert_channel_value(rc_channels->chan14);
			}

			if (max_channels > 15)
			{
				values[15] = convert_channel_value(rc_channels->chan15);
			}

			CRSF_VERBOSE("Got Channels");

			ret = true;
		}
		else
		{
			CRSF_DEBUG("CRC check failed");
		}
	}
	else
	{
		CRSF_DEBUG("Got Non-RC frame (len=%i, type=%i)", current_frame_length, crsf_frame.type);
		// We could check the CRC here and reset the parser into unsynced state if it fails.
		// But in practise it's robust even without that.
	}

	// Either reset or move the rest of the buffer
	if (current_frame_position > current_frame_length)
	{
		CRSF_VERBOSE("Moving buffer (%i > %i)", current_frame_position, current_frame_length);
		memmove(crsf_frame_ptr, crsf_frame_ptr + current_frame_length, current_frame_position - current_frame_length);
		current_frame_position -= current_frame_length;
	}
	else
	{
		current_frame_position = 0;
	}

	return ret;
}

/**
 * write an uint8_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint8_t(uint8_t *buf, int &offset, uint8_t value)
{
	buf[offset++] = value;
}
/**
 * write an uint16_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint16_t(uint8_t *buf, int &offset, uint16_t value)
{
	// Big endian
	buf[offset] = value >> 8;
	buf[offset + 1] = value & 0xff;
	offset += 2;
}
/**
 * write an uint24_t value to a buffer at a given offset and increment the offset
 */
static inline void write_uint24_t(uint8_t *buf, int &offset, int value)
{
	// Big endian
	buf[offset] = value >> 16;
	buf[offset + 1] = (value >> 8) & 0xff;
	buf[offset + 2] = value & 0xff;
	offset += 3;
}

/**
 * write an int32_t value to a buffer at a given offset and increment the offset
 */
static inline void write_int32_t(uint8_t *buf, int &offset, int32_t value)
{
	// Big endian
	buf[offset] = value >> 24;
	buf[offset + 1] = (value >> 16) & 0xff;
	buf[offset + 2] = (value >> 8) & 0xff;
	buf[offset + 3] = value & 0xff;
	offset += 4;
}

static inline void write_frame_header(uint8_t *buf, int &offset, crsf_frame_type_t type, uint8_t payload_size)
{
	write_uint8_t(buf, offset, CRSF_SYNC_BYTE); // this got changed from the address to the sync byte
	write_uint8_t(buf, offset, payload_size + 2);
	write_uint8_t(buf, offset, (uint8_t)type);
}
static inline void write_frame_crc(uint8_t *buf, int &offset, int buf_size)
{
	// CRC does not include the address and length
	write_uint8_t(buf, offset, crc8_dvb_s2_buf(buf + 2, buf_size - 3));

	// check correctness of buffer size (only needed during development)
	// if (buf_size != offset) { PX4_ERR("frame size mismatch (%i != %i)", buf_size, offset); }
}

bool crsf_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int fuel, uint8_t remaining)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::battery_sensor + 4];
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::battery_sensor, (uint8_t)crsf_payload_size_t::battery_sensor);
	write_uint16_t(buf, offset, voltage);
	write_uint16_t(buf, offset, current);
	write_uint24_t(buf, offset, fuel);
	write_uint8_t(buf, offset, remaining);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

bool crsf_send_telemetry_gps(int uart_fd, int32_t latitude, int32_t longitude, uint16_t groundspeed,
							 uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::gps + 4];
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::gps, (uint8_t)crsf_payload_size_t::gps);
	write_int32_t(buf, offset, latitude);
	write_int32_t(buf, offset, longitude);
	write_uint16_t(buf, offset, groundspeed);
	write_uint16_t(buf, offset, gps_heading);
	write_uint16_t(buf, offset, altitude);
	write_uint8_t(buf, offset, num_satellites);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

bool crsf_send_telemetry_attitude(int uart_fd, int16_t pitch, int16_t roll, int16_t yaw)
{
	uint8_t buf[(uint8_t)crsf_payload_size_t::attitude + 4];
	int offset = 0;
	write_frame_header(buf, offset, crsf_frame_type_t::attitude, (uint8_t)crsf_payload_size_t::attitude);
	write_uint16_t(buf, offset, pitch);
	write_uint16_t(buf, offset, roll);
	write_uint16_t(buf, offset, yaw);
	write_frame_crc(buf, offset, sizeof(buf));
	return write(uart_fd, buf, offset) == offset;
}

/**
 * 向CRSF接收机发送遥测飞行模式信息。
 *
 * @param uart_fd UART文件描述符，用于串行通信。
 * @param flight_mode 指向包含飞行模式名称的字符数组的指针。
 * @return 如果成功发送，则返回true；否则返回false。
 */
bool crsf_send_telemetry_flight_mode(int uart_fd, const char *flight_mode)
{
	const int max_length = 16;			  // 定义最大长度以防止过长的飞行模式字符串
	int length = strlen(flight_mode) + 1; // 计算飞行模式字符串的长度，包括终止符

	if (length > max_length)
	{
		length = max_length; // 限制长度不超过最大值
	}

	uint8_t buf[max_length + 4]; // 创建缓冲区以包含帧头、帧数据和帧校验和
	int offset = 0;
	// 写入帧头
	write_frame_header(buf, offset, crsf_frame_type_t::flight_mode, length);
	memcpy(buf + offset, flight_mode, length); // 复制飞行模式字符串到缓冲区
	offset += length;
	buf[offset - 1] = 0; // 确保字符串以空字符终止
	// 写入帧CRC
	write_frame_crc(buf, offset, length + 4);
	// 尝试发送缓冲区中的数据
	return write(uart_fd, buf, offset) == offset;
}
#endif