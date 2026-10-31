/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "imu/inv_imu_transport.h"
#include "imu/inv_imu_defs.h"

/*
 * ICM45686 传输层封装。
 *
 * 上层驱动统一调用 inv_imu_read_reg()/inv_imu_write_reg()，本文件根据地址范围自动选择：
 * - DREG：0x00~0xFF 的直接寄存器，通过用户提供的 read_reg/write_reg 回调访问。
 * - MREG/SRAM：超过 0xFF 的内部寄存器或 SRAM 地址，通过 IREG_ADDR/IREG_DATA 间接窗口访问。
 *
 * 这里不直接关心底层是 SPI 还是 I2C，真正的总线事务由 inv_imu_transport_t 中的回调实现。
 */

/* 静态函数声明 */

/* DREG：直接访问寄存器。 */
static int write_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, const uint8_t *buf);
static int read_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, uint8_t *buf);

/* MREG/SRAM：通过内部间接地址窗口访问的 MCLK 域寄存器或内存。 */
static int check_out_of_bounds_mreg(uint32_t reg, uint32_t len);
static int write_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, const uint8_t *buf);
static int read_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, uint8_t *buf);

int inv_imu_read_reg(void *t, uint32_t reg, uint32_t len, uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	/* 大于 0xFF 的地址视为 MREG/SRAM 地址，必须走间接访问窗口。 */
	if (reg > 0xFF)
		return read_mreg(tr, reg & 0xFFFF, len, buf);
	else
		return read_dreg(tr, reg, len, buf);
}

int inv_imu_write_reg(void *t, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	/* 写寄存器同样按地址范围分发，保持上层 API 简洁。 */
	if (reg > 0xFF)
		return write_mreg(tr, reg, len, buf);
	else
		return write_dreg(tr, reg, len, buf);
}

int inv_imu_read_sram(void *t, uint32_t addr, uint32_t len, uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	/* SRAM 地址总是通过 MREG 间接窗口访问。 */
	return read_mreg(tr, addr, len, buf);
}

int inv_imu_write_sram(void *t, uint32_t addr, uint32_t len, const uint8_t *buf)
{
	inv_imu_transport_t *tr = (inv_imu_transport_t *)t;
	/* SRAM 写入会先检查禁访地址区，再逐字节写入 IREG_DATA。 */
	return write_mreg(tr, addr, len, buf);
}

/*
 * 静态函数实现
 */

static int read_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, uint8_t *buf)
{
	/* 调用平台回调读取直接寄存器；非 0 返回统一映射为传输错误。 */
	if (t->read_reg(t->context, reg, buf, len) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int write_dreg(inv_imu_transport_t *t, uint8_t reg, uint32_t len, const uint8_t *buf)
{
	/* 调用平台回调写直接寄存器；这里不做重试，重试策略应放在平台回调或上层。 */
	if (t->write_reg(t->context, reg, buf, len) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int check_out_of_bounds_mreg(uint32_t reg, uint32_t len)
{
	uint32_t min_addr = reg;
	uint32_t max_addr = reg + len - 1;

	/*
	 * AN-000364 禁访区检查：
	 * 用户不得访问以下地址空间，否则芯片可能 stall，且需要 soft reset 才能恢复。
	 * - 0x000023FF ~ 0x00003FFF
	 * - 0x000083FF ~ 0x00009FFF
	 * - 0x0000AFFF ~ 0xFFFFFFFF
	 *
	 * 判断条件同时覆盖起始地址落入、结束地址落入、访问区间跨越禁区三种情况。
	 */
	if (((min_addr > 0x000023FF) && (min_addr <= 0x00003FFF)) ||
	    ((max_addr > 0x000023FF) && (max_addr <= 0x00003FFF)) ||
	    ((min_addr <= 0x000023FF) && (max_addr > 0x00003FFF)))
		return INV_IMU_ERROR_TRANSPORT;
	if (((min_addr > 0x000083FF) && (min_addr <= 0x00009FFF)) ||
	    ((max_addr > 0x000083FF) && (max_addr <= 0x00009FFF)) ||
	    ((min_addr <= 0x000083FF) && (max_addr > 0x00009FFF)))
		return INV_IMU_ERROR_TRANSPORT;
	if (max_addr > 0x0000AFFF)
		return INV_IMU_ERROR_TRANSPORT;

	return INV_IMU_OK;
}

static int read_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, uint8_t *buf)
{
	int     status = INV_IMU_OK;
	uint8_t data[2];

	/* MREG/SRAM 访问必须先过滤禁访地址，避免芯片进入 stall 状态。 */
	status |= check_out_of_bounds_mreg(reg, len);
	if (status)
		return status;

	/* 先把 16 位目标地址写入 IREG_ADDR_15_8/IREG_ADDR_7_0。 */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	/* 地址窗口切换需要少量等待，官方底层保留 4 us 间隔。 */
	t->sleep_us(4);
	status |= write_dreg(t, IREG_ADDR_15_8, 2, data);

	if (status)
		return status;

	/* 间接数据窗口每次读取一个字节，内部地址会自动递增。 */
	for (uint32_t i = 0; i < len; i++) {
		t->sleep_us(4);
		status |= read_dreg(t, IREG_DATA, 1, &buf[i]);
	}

	return status;
}

static int write_mreg(inv_imu_transport_t *t, uint32_t reg, uint32_t len, const uint8_t *buf)
{
	int     status = INV_IMU_OK;
	uint8_t data[3];

	/* 写入前同样做禁访区检查，保护芯片状态。 */
	status |= check_out_of_bounds_mreg(reg, len);
	if (status)
		return status;

	/* 前两个字节为目标地址，高字节在前。 */
	data[0] = (reg & 0xFF00) >> 8;
	data[1] = reg & 0xFF;
	/* 第三个字节为首个待写数据；地址和首字节可以一次 burst 写入。 */
	data[2] = buf[0];

	/* 写地址和首字节后，后续数据通过 IREG_DATA 逐字节追加。 */
	t->sleep_us(4);
	status |= write_dreg(t, IREG_ADDR_15_8, 3, data);
	t->sleep_us(4);

	if (status)
		return status;

	/* 写剩余字节；间接窗口内部地址自动递增。 */
	for (uint32_t i = 1; i < len; i++) {
		status |= write_dreg(t, IREG_DATA, 1, &buf[i]);
		t->sleep_us(4);
	}

	return status;
}
