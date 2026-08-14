/**
 * @file MagCalStore.h
 * @brief 磁力计标定参数的内部 Flash 持久化（STM32H743 bank2 末扇区）
 *
 * 存储位置：0x081E0000-0x081FFFFF（bank2 最后一个 128KB 扇区，扇区号 7）。
 * 选型依据：
 *   - 应用固件 ~277KB 全部落在 bank1（扇区 0-2），该区域与运行代码零重叠；
 *   - H7 双 Bank 架构允许在 bank1 执行的同时擦写 bank2（写期间代码不受影响）；
 *   - stm32duino 的 EEPROM 仿真库也默认占用"最后扇区"，本项目不使用该库，
 *     故独占此区域（★ 固件中禁止再引入 EEPROM.h 或其它占用末扇区的机制）；
 *   - 整扇区擦除后再写：记录仅 96B，磨损影响可忽略（扇区擦写寿命 ~1 万次，
 *     标定保存频率远低于此）。
 *
 * 记录格式（96B 窗口，3 个 256-bit FlashWord）：
 *   [0..3]   magic  "MAGC"
 *   [4..5]   version (1)
 *   [6..7]   sampleCount
 *   [8..19]  hardIron[3]  (float)
 *   [20..55] softIron[3][3] (float)
 *   [56..59] scale (float)
 *   [60..63] residualRms (float)
 *   [64..67] crc32（覆盖 [0..63]）
 *   [68..95] 保留（写 0xFF）
 *
 * 写入路径在 DBG 控制台 `magcal save` 触发（地面操作，非飞行路径）：
 *   - 擦除期间轮询喂 IWDG（2.99s 窗口内完成，实测 128KB 扇区擦除 ~1s）；
 *   - 写完做 DCache 失效 + 读回校验（magic/版本/CRC/参数逐项比对）。
 * 读取路径在 initMagnetometer() 上电时执行：DCache 失效后直读 + CRC 校验，
 * 校验失败静默回退未标定状态（不影响飞行）。
 */

#ifndef MAG_CAL_STORE_H
#define MAG_CAL_STORE_H

#include <stdint.h>
#include "MagCalMath.h"

namespace magcal {

// bank2 末扇区起始地址（2MB 器件：bank2 = 0x08100000，8×128KB，扇区7 = +0xE0000）
#define MAGCAL_FLASH_ADDR 0x081E0000UL
#define MAGCAL_FLASH_SECTOR_IN_BANK 7U   // bank2 内扇区号（0..7）
#define MAGCAL_FLASH_WORDS 3U            // 3 × 256bit = 96B 编程窗口

/**
 * @brief 保存标定到内部 Flash（阻塞 ~1-2s，含扇区擦除；仅地面 DBG 命令调用）
 * @param p           待保存标定（须 valid）
 * @param sampleCount 参与拟合样本数（记录用）
 * @param residualRms 拟合残差（记录用）
 * @return true 写入并读回校验通过
 */
bool storeSave(const MagCalParams &p, uint16_t sampleCount, float residualRms);

/**
 * @brief 从内部 Flash 加载标定（上电/`magcal load` 调用，~µs 级）
 * @param p 输出参数；失败时内容不变
 * @return true 记录完整（magic/版本/CRC/参数合理性全部通过）
 */
bool storeLoad(MagCalParams &p);

/**
 * @brief 擦除存储扇区（`magcal erase`：清空持久化标定，恢复未标定状态）
 */
bool storeErase();

} // namespace magcal

#endif // MAG_CAL_STORE_H
