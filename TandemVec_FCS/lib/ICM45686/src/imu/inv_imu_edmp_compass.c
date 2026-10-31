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
#include "imu/inv_imu_edmp_compass.h"
#include "imu/inv_imu_edmp_defs.h"

/*
 * eDMP Compass 扩展功能实现。
 *
 * ICM45686 本体没有磁力计，这里的 compass 指的是通过 AUX/I2CM 接入的外部磁力计数据。
 * 本文件负责把 eDMP 扩展 RAM 镜像加载到芯片 SRAM，并配置 eDMP 从外部传感器通道获取磁力计数据。
 * 这些函数只处理 eDMP 侧的使能和速率配置；外部磁力计的具体 I2C 地址、寄存器和读写事务
 * 仍需要由 I2CM/外部磁力计驱动完成。
 */

int inv_imu_edmp_compass_init(inv_imu_device_t *s)
{
	int            status    = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "imu/edmp_ram_extended_features_image.h"
	};

	/* 先初始化 APEX/eDMP 基础配置，确保 SRAM 起始地址和内部执行环境处于可加载状态。 */
	status |= inv_imu_edmp_init_apex(s);

	/*
	 * 将扩展功能 RAM 镜像写入 eDMP 程序 RAM。
	 * 注意：这里写入的是整段二进制 image，不能按普通参数区局部修改。
	 */
	status |=
	    inv_imu_write_sram(s, (uint32_t)EDMP_RAM_FEATURE_PRGM_RAM_BASE, sizeof(ram_img), ram_img);

	return status;
}

int inv_imu_edmp_compass_enable_soft_iron_cor(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* 读-改-写 EDMP_APEX_EN1，避免覆盖同一寄存器里的其他 APEX/eDMP 功能位。 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	/* 使能软/硬铁校正通道，校正算法实际运行依赖前面已加载的 eDMP 扩展 image。 */
	edmp_apex_en1.soft_hard_iron_corr_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_compass_disable_soft_iron_cor(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* 只清除软/硬铁校正使能位，不影响外部磁力计采样和其他 APEX 功能。 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.soft_hard_iron_corr_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_compass_set_frequency(inv_imu_device_t *                  s,
                                       const dmp_ext_sen_odr_cfg_ext_odr_t frequency)
{
	int                   status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;
	int_i2cm_source_t     int_i2cm_source;

	/* 使能 eDMP/FIFO 外部传感器通道，并写入外部传感器 ODR 枚举。 */
	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	dmp_ext_sen_odr_cfg.ext_sensor_en = 1;
	dmp_ext_sen_odr_cfg.ext_odr       = frequency;
	status |= inv_imu_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	/*
	 * 让内部 ODR 事件触发 I2CM 事务。
	 * int_status_i2cm_smc_ext_odr_en = 1 表示使能外部传感器 ODR 中断源。
	 */
	status |= inv_imu_read_reg(s, INT_I2CM_SOURCE, 1, (uint8_t *)&int_i2cm_source);
	int_i2cm_source.int_status_i2cm_smc_ext_odr_en = 1;
	status |= inv_imu_write_reg(s, INT_I2CM_SOURCE, 1, (uint8_t *)&int_i2cm_source);

	return status;
}

int inv_imu_edmp_compass_enable_es(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	uint8_t         data;
	edmp_apex_enx_t cfg;

	/* 在 eDMP SRAM 参数区打开外部传感器 RAM image 和 ES0 compass 通道。 */
	data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_RAM_IMAGE_EN, &data);
	data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES0_COMPASS_EN, &data);

	/*
	 * 当没有启用 pedometer/tilt/free-fall/SMD/tap/R2W 等 APEX 功能时，
	 * 可以进入更省电的外部传感器工作模式；如果其他 APEX 功能正在运行，则保持兼容模式。
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 2, (uint8_t *)&cfg);
	if (cfg.edmp_apex_en0.pedo_en || cfg.edmp_apex_en0.tilt_en || cfg.edmp_apex_en0.ff_en ||
	    cfg.edmp_apex_en0.smd_en || cfg.edmp_apex_en0.tap_en || cfg.edmp_apex_en0.r2w_en ||
	    cfg.edmp_apex_en1.basic_smd_en)
		data = 0;
	else
		data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_POWER_MODE, &data);

	return status;
}

int inv_imu_edmp_compass_disable_es(inv_imu_device_t *s)
{
	int     status = INV_IMU_OK;
	uint8_t data;

	/* 关闭 eDMP 外部传感器 RAM image 和 ES0 compass 通道；不会复位 I2CM 硬件配置。 */
	data = 0;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_RAM_IMAGE_EN, &data);
	data = 0;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES0_COMPASS_EN, &data);

	return status;
}
