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

/** @defgroup EDMPMag EDMP Compass
 *  @brief High-level functions to drive eDMP Compass features
 *  @{
 */

/** @file inv_imu_edmp_compass.h */
/*
 * 中文说明：
 * 本头文件声明 eDMP 外部磁力计/罗盘相关接口。ICM45686 芯片本身不集成磁力计，
 * 这些接口主要用于把外部磁力计数据接入 eDMP，并控制软/硬铁校正和外部传感器输出频率。
 * 使用前应先完成 AUX I2CM 或外部磁力计驱动配置，否则 eDMP 侧使能后也不会产生有效罗盘数据。
 */

#ifndef _INV_IMU_COMPASS_H_
#define _INV_IMU_COMPASS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/** @brief  初始化 eDMP 罗盘扩展功能，会加载扩展 RAM image 并准备相关算法环境。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_init(inv_imu_device_t *s);

/** @brief  使能 eDMP/APEX 软铁/硬铁校正功能。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_enable_soft_iron_cor(inv_imu_device_t *s);

/** @brief  关闭 eDMP/APEX 软铁/硬铁校正功能，不会关闭外部磁力计采样。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_disable_soft_iron_cor(inv_imu_device_t *s);

/** @brief 配置 eDMP 外部罗盘数据输出频率。
 *  @param[in] s         Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_set_frequency(inv_imu_device_t *                  s,
                                       const dmp_ext_sen_odr_cfg_ext_odr_t frequency);

/** @brief  把罗盘作为 eDMP external sensor 使能，让 eDMP/FIFO 可以接收外部磁力计数据。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_enable_es(inv_imu_device_t *s);

/** @brief  关闭 eDMP external sensor 罗盘通道。
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_disable_es(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_COMPASS_H_ */

/** @} */
