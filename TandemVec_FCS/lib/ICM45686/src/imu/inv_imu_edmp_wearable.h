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

/** @defgroup WEARABLE EDMP Wearable
 *  @brief High-level functions to drive eDMP Wearable features
 *  @{
 */

/** @file inv_imu_edmp_wearable.h */
/*
 * 中文说明：
 * 本头文件声明 eDMP 可穿戴算法中的 B2S 功能接口。B2S 依赖 eDMP 扩展 RAM image，
 * 主要用于抬腕/抬起查看等姿态事件检测。参数中的安装矩阵会直接影响算法对 X/Y/Z 轴的理解，
 * 因此移植到新硬件时必须结合实际安装方向验证事件触发方向。
 */

#ifndef _INV_IMU_WEARABLE_H_
#define _INV_IMU_WEARABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/** @brief IMU B2S 参数定义。
 */
typedef struct {
	/* B2S 安装矩阵位域，用于描述算法输入轴向的交换和翻转关系。 */
	uint32_t b2s_mounting_matrix; /**< Specifies mounting matrix to be applied to B2S raw data
                                       Set bit 2 : swap X/Y ; flip Z
                                       Set bit 1 : flip X ; flip Z
                                       Set bit 0 : flip Y ; flip Z */
} inv_imu_edmp_b2s_parameters_t;

/** @brief  初始化 B2S 算法，会加载扩展 RAM image 并关闭可能冲突的 R2W 功能。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_init(inv_imu_device_t *s);

/** @brief  读取当前 B2S 配置参数。
 *  @param[in]  s           Pointer to device.
 *  @param[out] b2s_params  Pointer to B2S configuration structure, which will hold current B2S configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params);

/** @brief  写入新的 B2S 配置参数，尤其是安装方向矩阵。
 *  @param[in]  s           Pointer to device.
 *  @param[in]  b2s_params  Pointer to B2S configuration structure, which contains new B2S configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params);

/** @brief  使能 APEX/eDMP B2S 算法输出。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_enable(inv_imu_device_t *s);

/** @brief  关闭 APEX/eDMP B2S 算法输出，但保留已写入的参数。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_disable(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_WEARABLE_H_ */

/** @} */
