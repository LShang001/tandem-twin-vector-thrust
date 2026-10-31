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

#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_edmp_wearable.h"
#include "imu/inv_imu_edmp_defs.h"

/*
 * eDMP Wearable/B2S 扩展功能实现。
 *
 * B2S 通常指 Bring-to-See/抬腕亮屏一类可穿戴场景算法。该功能依赖 eDMP 扩展 RAM 镜像，
 * 需要先加载 image，再配置安装矩阵和功能使能位。这里保留官方底层流程，只补充调用约束说明。
 */

/* B2S 专用 eDMP SRAM 参数地址：用于选择算法内部使用的安装方向矩阵。 */
#define EDMP_B2S_MOUNTING_MATRIX      0x8FE
#define EDMP_B2S_MOUNTING_MATRIX_SIZE 0x1

int inv_imu_edmp_b2s_init(inv_imu_device_t *s)
{
	int            status    = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_extended_features_image.h"
	};

	/* 初始化 APEX/eDMP 基础环境，后续才能安全写入扩展功能 RAM image。 */
	status |= inv_imu_edmp_init_apex(s);

	/* 把 B2S 所需的扩展算法 image 写入 eDMP 程序 RAM。 */
	status |=
	    inv_imu_write_sram(s, (uint32_t)EDMP_RAM_FEATURE_PRGM_RAM_BASE, sizeof(ram_img), ram_img);

	/* B2S 与 R2W 同属可穿戴姿态事件类功能，这里先关闭 R2W，避免事件源互相影响。 */
	status |= inv_imu_edmp_disable_r2w(s);

	return status;
}

int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params)
{
	int status = INV_IMU_OK;

	/* 从 eDMP SRAM 参数区读取当前 B2S 安装矩阵，供上层确认或做增量修改。 */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
	                                 (uint8_t *)&b2s_params->b2s_mounting_matrix);

	return status;
}

int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params)
{
	int status = INV_IMU_OK;

	/*
	 * 写入 B2S 安装矩阵。
	 * 该参数决定算法如何解释传感器轴向，接入飞控或可穿戴产品前必须按实际安装方向验证。
	 */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
	                                  (uint8_t *)&b2s_params->b2s_mounting_matrix);

	return status;
}

int inv_imu_edmp_b2s_enable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	fifo_config0_t  fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	/*
	 * 官方限制：FIFO 深度配置为 MAX 时不能启用该扩展 feature。
	 * 这里提前返回错误，避免 eDMP 输出路径和 FIFO 配置冲突。
	 */
	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_MAX)
		return INV_IMU_ERROR;

	/* feature3_en 对应 B2S 扩展功能使能位；采用读-改-写保护同寄存器其他位。 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_b2s_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* 关闭 B2S 功能位，但不清除 SRAM 中的参数，便于后续再次 enable。 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}
