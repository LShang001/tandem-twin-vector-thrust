/**
 * @file MagCalStore.cpp
 * @brief 磁力计标定参数内部 Flash 持久化实现（H7 寄存器级，双 Bank 安全）
 *
 * 关键实现要点（均有注释依据，勿改）：
 *   - H743 双 Bank：bank1/bank2 有独立的 KEYR/CR/SR 寄存器组
 *     （FLASH->CR2/SR2/KEYR2 控制 bank2），在 bank1 执行代码的同时
 *     擦写 bank2 是官方支持的用法（RM0433）。
 *   - 扇区擦除用寄存器级流程而非 HAL_FLASHEx_Erase：HAL 内部阻塞轮询
 *     不喂狗，128KB 擦除最坏 4s 会撞 2.99s IWDG；本实现每 1ms 喂一次狗。
 *   - 编程用 256-bit FlashWord：先置 CR2.PG，向目标地址连写 8 个 32 位字，
 *     硬件自动打包编程（与 HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD) 一致）。
 *   - 擦写完成后必须 SCB_InvalidateDCache_by_Addr：H7 D-Cache 写回模式下
 *     可能缓存旧的扇区内容，不失效则读回校验/上电加载会读到脏数据。
 */

#include "MagCalStore.h"
#include <Arduino.h>
#include <string.h>

// IWDG 喂狗（main.cpp 暴露；test 环境 BFS_DISABLE_IWDG 时为 no-op）：
// 擦除/编程阻塞期间轮询喂狗，防 2.99s 硬件看门狗复位打断写扇区。
#ifdef BFS_DISABLE_IWDG
static inline void magCalKickIwdg(void) {}
#else
void kickIwdg(void);
#define magCalKickIwdg() kickIwdg()
#endif

// ST 通用 FLASH 解锁键（KEYR1/KEYR2 相同）
#define MAGCAL_FLASH_KEY1 0x45670123UL
#define MAGCAL_FLASH_KEY2 0xCDEF89ABUL

namespace magcal {

namespace {

constexpr char kMagic[4] = {'M', 'A', 'G', 'C'};
constexpr uint16_t kVersion = 1;

struct FlashRecord
{
    char magic[4];
    uint16_t version;
    uint16_t sampleCount;
    float hardIron[3];
    float softIron[3][3];
    float scale;
    float residualRms;
    uint32_t crc32; // 覆盖 [0 .. offsetof(crc32))
};

static_assert(sizeof(FlashRecord) <= 96, "record must fit 96B window");

void fillRecord(FlashRecord &rec, const MagCalParams &p, uint16_t sampleCount, float residualRms)
{
    memset(&rec, 0xFF, sizeof(rec));
    memcpy(rec.magic, kMagic, 4);
    rec.version = kVersion;
    rec.sampleCount = sampleCount;
    memcpy(rec.hardIron, p.hardIron, sizeof(rec.hardIron));
    memcpy(rec.softIron, p.softIron, sizeof(rec.softIron));
    rec.scale = p.scale;
    rec.residualRms = residualRms;
    rec.crc32 = crc32(&rec, offsetof(FlashRecord, crc32));
}

void invalidateRegion()
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)MAGCAL_FLASH_ADDR, 128);
    __DSB();
    __ISB();
}

bool unlockBank2()
{
    FLASH->KEYR2 = MAGCAL_FLASH_KEY1;
    FLASH->KEYR2 = MAGCAL_FLASH_KEY2;
    __ISB();
    __DSB();
    return (FLASH->CR2 & FLASH_CR_LOCK) == 0U;
}

void lockBank2()
{
    FLASH->CR2 |= FLASH_CR_LOCK;
}

/**
 * @brief 寄存器级 bank2 扇区擦除（喂狗轮询，超时 8s 兜底）
 */
bool eraseSectorWithKick()
{
    // 清 PG，配置 SER + SNB（bank2 内扇区号 7）
    FLASH->CR2 &= ~(FLASH_CR_PG);
    FLASH->CR2 = (FLASH->CR2 & ~(FLASH_CR_SER | FLASH_CR_SNB)) |
                 FLASH_CR_SER |
                 (MAGCAL_FLASH_SECTOR_IN_BANK << FLASH_CR_SNB_Pos);
    FLASH->CR2 |= FLASH_CR_START;
    __ISB();
    __DSB();

    const uint32_t t0 = millis();
    while (FLASH->SR2 & FLASH_SR_BSY)
    {
        magCalKickIwdg();
        delay(1);
        if ((uint32_t)(millis() - t0) > 8000UL)
        {
            FLASH->CR2 &= ~(FLASH_CR_SER);
            return false; // 擦除超时（扇区损坏或时钟异常）
        }
    }

    const bool err = (FLASH->SR2 & (FLASH_SR_WRPERR | FLASH_SR_PGSERR)) != 0U;
    FLASH->CR2 &= ~(FLASH_CR_SER);
    return !err;
}

/**
 * @brief 编程 96B（3 × 256-bit FlashWord），喂狗轮询
 * @param buf 32 字节对齐的 96B 数据（内部再按 32B 切分）
 */
bool programWithKick(const uint8_t *buf)
{
    FLASH->CR2 |= FLASH_CR_PG;
    __ISB();
    __DSB();

    volatile uint32_t *dst = (volatile uint32_t *)MAGCAL_FLASH_ADDR;
    const uint32_t *src = (const uint32_t *)buf;
    bool ok = true;

    for (uint32_t w = 0; w < MAGCAL_FLASH_WORDS && ok; w++)
    {
        for (uint32_t k = 0; k < 8; k++)
        {
            dst[w * 8 + k] = src[w * 8 + k];
        }
        __ISB();
        __DSB();

        const uint32_t t0 = millis();
        while (FLASH->SR2 & FLASH_SR_BSY)
        {
            magCalKickIwdg();
            if ((uint32_t)(millis() - t0) > 1000UL)
            {
                ok = false;
                break;
            }
        }
        if (FLASH->SR2 & (FLASH_SR_WRPERR | FLASH_SR_PGSERR))
        {
            ok = false;
        }
    }

    FLASH->CR2 &= ~(FLASH_CR_PG);
    return ok;
}

} // namespace

bool storeSave(const MagCalParams &p, uint16_t sampleCount, float residualRms)
{
    if (!p.valid)
        return false;

    FlashRecord rec;
    fillRecord(rec, p, sampleCount, residualRms);

    // 打包进 32B 对齐的 96B 编程缓冲（FlashWord 源地址对齐保险）
    alignas(32) uint8_t buf[96];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &rec, sizeof(rec));

    if (!unlockBank2())
        return false;
    if (!eraseSectorWithKick())
    {
        lockBank2();
        return false;
    }
    if (!programWithKick(buf))
    {
        lockBank2();
        return false;
    }
    lockBank2();

    // 读回校验（先失效 D-Cache，确保读到 Flash 真值）
    invalidateRegion();
    FlashRecord back;
    memcpy(&back, (const void *)MAGCAL_FLASH_ADDR, sizeof(back));
    if (memcmp(back.magic, kMagic, 4) != 0 || back.version != kVersion)
        return false;
    if (back.crc32 != crc32(&back, offsetof(FlashRecord, crc32)))
        return false;
    if (memcmp(&back, &rec, offsetof(FlashRecord, crc32)) != 0)
        return false;
    return true;
}

bool storeLoad(MagCalParams &p)
{
    invalidateRegion();

    FlashRecord rec;
    memcpy(&rec, (const void *)MAGCAL_FLASH_ADDR, sizeof(rec));
    if (memcmp(rec.magic, kMagic, 4) != 0)
        return false;
    if (rec.version != kVersion)
        return false;
    if (rec.crc32 != crc32(&rec, offsetof(FlashRecord, crc32)))
        return false;

    MagCalParams cand;
    cand.identity();
    memcpy(cand.hardIron, rec.hardIron, sizeof(cand.hardIron));
    memcpy(cand.softIron, rec.softIron, sizeof(cand.softIron));
    cand.scale = rec.scale;
    cand.valid = true;
    if (!paramsSanity(cand))
        return false;

    p = cand;
    return true;
}

bool storeErase()
{
    if (!unlockBank2())
        return false;
    const bool ok = eraseSectorWithKick();
    lockBank2();
    invalidateRegion();
    return ok;
}

} // namespace magcal
