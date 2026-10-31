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

#ifndef _INV_IMU_VERSION_H_
#define _INV_IMU_VERSION_H_

/*
 * 底层 IMU 驱动版本号。
 *
 * 该字符串用于标识当前移植进 Arduino/PlatformIO 库的 InvenSense 底层驱动版本。
 * 如果未来整体同步官方底层驱动，应同时更新这里、README 和手册对照审查记录。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 当前底层驱动版本字符串。 */
#define INV_IMU_VERSION_STRING "3.0.0"

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_VERSION_H_ */
