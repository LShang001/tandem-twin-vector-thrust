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

#include "imu/inv_imu_selftest.h"
#include "imu/inv_imu_edmp.h"

/*
 * ICM45686 内置自检流程。
 *
 * 自检由 eDMP/STC 逻辑执行：上层先准备阈值和平均时间参数，本文件把参数写入 eDMP SRAM，
 * 触发 on-demand eDMP 运行，然后等待 selftest_done 中断并读取结果位。自检过程中会复位设备，
 * 因此不要在正常高速采样或控制闭环运行期间直接调用。
 */

/* 内部静态函数声明：分别负责写入自检参数、触发自检、解析自检结果。 */
static int set_selftest_parameters(inv_imu_device_t *                   s,
                                   const inv_imu_selftest_parameters_t *st_params);
static int run_internal_selftest(inv_imu_device_t *s);
static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                               inv_imu_selftest_output_t *st_output);

/* API implementation */

int inv_imu_selftest_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;

	/* 默认同时测试加速度计和陀螺仪，平均时间与阈值采用官方推荐的 320 ms / 50%。 */
	st_params->accel_en       = INV_IMU_ENABLE;
	st_params->gyro_en        = INV_IMU_ENABLE;
	st_params->avg_time       = SELFTEST_AVG_TIME_320_MS;
	st_params->accel_limit    = SELFTEST_ACCEL_THRESHOLD_50_PERCENT;
	st_params->gyro_limit     = SELFTEST_GYRO_THRESHOLD_50_PERCENT;
	st_params->patch_settings = 0;

	return rc;
}

int inv_imu_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                     inv_imu_selftest_output_t *st_output)
{
	int rc = INV_IMU_OK;

	/* 至少需要启用一个传感器通道；两个都关闭时没有可执行的自检任务。 */
	if (!(st_params->accel_en || st_params->gyro_en))
		return INV_IMU_ERROR;

	/*
	 * 自检前复位设备，清理前序配置对 eDMP/STC 的影响。
	 * 注意：这会打断当前采样配置，调用方需要在自检后重新初始化所需 ODR/FSR/FIFO。
	 */
	rc |= inv_imu_adv_device_reset(s);

	/* 复位后等待内部电源域和寄存器状态稳定。 */
	inv_imu_sleep_us(s, 10000);

	/* 设备复位会清掉 eDMP 地址配置，因此这里重新配置 eDMP 起始地址。 */
	rc |= inv_imu_edmp_configure(s);

	/* 把用户选择的自检通道、阈值、平均时间和补丁开关写入 eDMP SRAM。 */
	rc |= set_selftest_parameters(s, st_params);

	/* 触发内部自检并阻塞等待完成或超时。 */
	rc |= run_internal_selftest(s);

	/* 读取 eDMP SRAM 中的 STC 结果位，并转换成逐轴状态。 */
	rc |= get_selftest_output(s, st_params, st_output);

	/* 自检完成后再次复位，确保芯片回到干净状态，避免残留 STC 配置影响普通采样。 */
	rc |= inv_imu_adv_device_reset(s);

	return rc;
}

/* 静态函数实现 */

static int set_selftest_parameters(inv_imu_device_t *                   s,
                                   const inv_imu_selftest_parameters_t *st_params)
{
	int      rc             = INV_IMU_OK;
	uint32_t tmp_stc_params = 0;
	int      init_en;

	/* 写 SRAM 参数前必须打开 SRAM 电源域。 */
	rc |= inv_imu_adv_power_up_sram(s);

	/* 组合 STC_CONFIGPARAMS 位域：初始化开关、通道开关、加速度/陀螺阈值、平均时间。 */
	init_en = (st_params->accel_en || st_params->gyro_en);
	tmp_stc_params |= (init_en ? SELFTEST_INIT_EN : SELFTEST_INIT_DIS);
	tmp_stc_params |= (st_params->accel_en ? SELFTEST_ACCEL_EN : SELFTEST_ACCEL_DIS);
	tmp_stc_params |= (st_params->gyro_en ? SELFTEST_GYRO_EN : SELFTEST_GYRO_DIS);
	tmp_stc_params |= (uint32_t)(st_params->accel_limit & SELFTEST_ACCEL_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->gyro_limit & SELFTEST_GYRO_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->avg_time & SELFTEST_AVERAGE_TIME_MASK);
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);

	/* 禁用 self-test stop point，保证自检按完整流程运行到结果输出。 */
	tmp_stc_params = 0;
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_DEBUG_EN, (uint8_t *)&tmp_stc_params);

	/* patch_settings 预留给官方 STC patch/调试场景，默认值通常为 0。 */
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_PATCH_EN, (uint8_t *)&(st_params->patch_settings));

	return rc;
}

static int run_internal_selftest(inv_imu_device_t *s)
{
	int                rc = INV_IMU_OK;
	reg_host_msg_t     reg_host_msg;
	int_apex_config1_t int_apex_config1;
	int                timeout_us = 3000000; /* 3 seconds */

	/* 打开 testopenable，允许芯片进入内部测试路径。 */
	rc |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.testopenable = INV_IMU_ENABLE;
	rc |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* 解除 selftest_done 中断屏蔽，用它作为自检完成标志。 */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 0;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	/* 以 on-demand 方式运行 eDMP 自检流程，并把完成事件路由到 INT2。 */
	rc |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT2);

	/* 轮询 selftest_done 状态位；每 100 us 检查一次，最多等待 3 s。 */
	while (1) {
		int_apex_status1_t int_apex_status1;
		rc |= inv_imu_read_reg(s, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		if (int_apex_status1.int_status_selftest_done)
			break;

		inv_imu_sleep_us(s, 100);
		timeout_us -= 100;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	/* 自检完成后重新屏蔽 selftest_done，避免后续 APEX 中断被旧配置影响。 */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 1;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	return rc;
}

static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                               inv_imu_selftest_output_t *st_output)
{
	int      rc = INV_IMU_OK;
	uint32_t stc_results;

	/* 读取 STC 结果位。各轴 mask 为 0 表示通过，置位表示对应轴失败。 */
	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_RESULTS, (uint8_t *)&stc_results);

	if (!st_params->accel_en) {
		/* 未启用加速度计自检时，总体和三轴状态都标记为 NOT_RUN。 */
		st_output->accel_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ax_status    = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ay_status    = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->az_status    = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		/* 加速度计总状态同时检查 X/Y/Z 轴失败位和 STC 总失败位。 */
		st_output->accel_status =
		    ((stc_results & (STC_RESULTS_ACCEL_X_MASK | STC_RESULTS_ACCEL_Y_MASK |
		                     STC_RESULTS_ACCEL_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) == 0) ?
                INV_IMU_ST_STATUS_SUCCESS :
                INV_IMU_ST_STATUS_FAIL;
		st_output->ax_status = ((stc_results & STC_RESULTS_ACCEL_X_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->ay_status = ((stc_results & STC_RESULTS_ACCEL_Y_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->az_status = ((stc_results & STC_RESULTS_ACCEL_Z_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
	}

	if (!st_params->gyro_en) {
		/* 未启用陀螺仪自检时，总体和三轴状态都标记为 NOT_RUN。 */
		st_output->gyro_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gx_status   = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gy_status   = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gz_status   = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		/* 陀螺仪总状态同时检查 X/Y/Z 轴失败位和 STC 总失败位。 */
		st_output->gyro_status =
		    ((stc_results & (STC_RESULTS_GYRO_X_MASK | STC_RESULTS_GYRO_Y_MASK |
		                     STC_RESULTS_GYRO_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) == 0) ?
                INV_IMU_ST_STATUS_SUCCESS :
                INV_IMU_ST_STATUS_FAIL;
		st_output->gx_status = ((stc_results & STC_RESULTS_GYRO_X_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->gy_status = ((stc_results & STC_RESULTS_GYRO_Y_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->gz_status = ((stc_results & STC_RESULTS_GYRO_Z_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
	}

	return rc;
}
