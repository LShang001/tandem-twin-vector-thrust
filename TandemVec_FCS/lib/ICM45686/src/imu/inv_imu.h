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

#ifndef _INV_IMU_H_
#define _INV_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IMU IMU
 *  @brief Describes IMU
 *  @{
 */

/** @file inv_imu.h */

/*
 * 中文说明：当前编译目标芯片能力描述。
 *
 * 本库当前固定面向 ICM45686：
 * - WHO_AM_I 预期值为 0xE9。
 * - 支持高量程、FSYNC、INT2、I2C master、CLKIN、AUX1。
 * - 不支持 AUX2。
 *
 * 这些宏会影响大量条件编译，切换到其他 ICM456xx 型号时必须同步核对手册和编译结果。
 */

/* Device ID */
#define ICM45686

/* Device description */
#define INV_IMU_STRING_ID            "ICM45686"
#define INV_IMU_WHOAMI               0xE9
#define INV_IMU_HIGH_FSR_SUPPORTED   1
#define INV_IMU_FSYNC_SUPPORTED      1
#define INV_IMU_USE_BASIC_SMD        0
#define INV_IMU_INT2_PIN_SUPPORTED   1
#define INV_IMU_I2C_MASTER_SUPPORTED 1
#define INV_IMU_CLKIN_SUPPORTED      1
#define INV_IMU_AUX1_SUPPORTED       1
#define INV_IMU_AUX2_SUPPORTED       0

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
