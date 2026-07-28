#ifndef DPS_CONSTS_H_ // 如果未定义 DPS_CONSTS_H_ 宏
#define DPS_CONSTS_H_ // 定义 DPS_CONSTS_H_ 宏，防止头文件重复包含

#include "util/DpsRegister.h" // 包含 DpsRegister.h 头文件，该文件定义了寄存器操作相关的结构体或类

///////////     DPS310    ///////////
// DPS310 传感器相关常量定义

#define DPS310__PROD_ID 0x00          // 产品ID寄存器的地址，读取产品ID的命令
#define DPS310__SPI_WRITE_CMD 0x00U   // SPI 写命令，写操作的地址位最高位为 0
#define DPS310__SPI_READ_CMD 0x80U    // SPI 读命令，读操作的地址位最高位为 1
#define DPS310__SPI_RW_MASK 0x80U     // SPI 读写位掩码，用于提取 SPI 通信地址字节的最高位
#define DPS310__SPI_MAX_FREQ 1000000U // SPI 通信的最大频率，1MHz

#define DPS310__OSR_SE 3U // 过采样率设置的阈值，当过采样率大于 2^3 = 8 时，需要使能温度和压力的偏移

// DPS310 在每次同步测量或每秒异步测量时，有 10 毫秒的空闲时间
// 这个值用于防止在某些情况下出现错误，可以设置为 0，但不建议这样做
#define DPS310__BUSYTIME_FAILSAFE 10U
// 最大的忙碌时间，计算公式为 (1000 - DPS310__BUSYTIME_FAILSAFE) * DPS__BUSYTIME_SCALING，单位为 0.1 毫秒
#define DPS310__MAX_BUSYTIME ((1000U - DPS310__BUSYTIME_FAILSAFE) * DPS__BUSYTIME_SCALING)

#define DPS310__REG_ADR_SPI3W 0x09U     // 在三线 SPI 模式下，配置寄存器的地址
#define DPS310__REG_CONTENT_SPI3W 0x01U // 在三线 SPI 模式下，配置寄存器的内容,用来使能三线SPI模式

///////////     DPS422    ///////////
// DPS422 传感器相关常量定义

#define DPS422__PROD_ID 0x0A // DPS422 的产品 ID 的读取命令

///////////     common    ///////////
// DPS310 和 DPS422 通用的常量定义

// 从机地址对于 422 和 310 是相同的（未来可能会有变动）
#define DPS__FIFO_SIZE 32            // FIFO 的大小，可以存储 32 个测量结果
#define DPS__STD_SLAVE_ADDRESS 0x77U // 标准的 I2C 从机地址
#define DPS__RESULT_BLOCK_LENGTH 3   // 每个测量结果的块长度，每个压力或温度测量结果占用 3 个字节
#define NUM_OF_COMMON_REGMASKS 16    // 通用寄存器掩码的数量

// 测量速率的定义
#define DPS__MEASUREMENT_RATE_1 0   // 1 次/秒
#define DPS__MEASUREMENT_RATE_2 1   // 2 次/秒
#define DPS__MEASUREMENT_RATE_4 2   // 4 次/秒
#define DPS__MEASUREMENT_RATE_8 3   // 8 次/秒
#define DPS__MEASUREMENT_RATE_16 4  // 16 次/秒
#define DPS__MEASUREMENT_RATE_32 5  // 32 次/秒
#define DPS__MEASUREMENT_RATE_64 6  // 64 次/秒
#define DPS__MEASUREMENT_RATE_128 7 // 128 次/秒

// 过采样率的定义
#define DPS__OVERSAMPLING_RATE_1 DPS__MEASUREMENT_RATE_1     // 1 倍过采样
#define DPS__OVERSAMPLING_RATE_2 DPS__MEASUREMENT_RATE_2     // 2 倍过采样
#define DPS__OVERSAMPLING_RATE_4 DPS__MEASUREMENT_RATE_4     // 4 倍过采样
#define DPS__OVERSAMPLING_RATE_8 DPS__MEASUREMENT_RATE_8     // 8 倍过采样
#define DPS__OVERSAMPLING_RATE_16 DPS__MEASUREMENT_RATE_16   // 16 倍过采样
#define DPS__OVERSAMPLING_RATE_32 DPS__MEASUREMENT_RATE_32   // 32 倍过采样
#define DPS__OVERSAMPLING_RATE_64 DPS__MEASUREMENT_RATE_64   // 64 倍过采样
#define DPS__OVERSAMPLING_RATE_128 DPS__MEASUREMENT_RATE_128 // 128 倍过采样

// 使用 0.1 毫秒为时间单位进行计算，所以 10 个单位是 1 毫秒
#define DPS__BUSYTIME_SCALING 10U

#define DPS__NUM_OF_SCAL_FACTS 8 // 缩放因子的数量

// 状态码定义
#define DPS__SUCCEEDED 0         // 成功
#define DPS__FAIL_UNKNOWN -1     // 未知失败
#define DPS__FAIL_INIT_FAILED -2 // 初始化失败
#define DPS__FAIL_TOOBUSY -3     // 过于繁忙
#define DPS__FAIL_UNFINISHED -4  // 未完成
#define DPS__FAIL_OVERFLOW -5    // FIFO 溢出

namespace dps // 命名空间 dps，用于避免命名冲突
{

    /**
     * @brief 工作模式枚举类型
     *
     */
    enum Mode
    {
        IDLE = 0x00,     // 空闲模式
        CMD_PRS = 0x01,  // 单次压力测量命令模式
        CMD_TEMP = 0x02, // 单次温度测量命令模式
        CMD_BOTH = 0x03, // 单次压力和温度测量命令模式 (仅适用于 DPS422)
        CONT_PRS = 0x05, // 连续压力测量模式
        CONT_TMP = 0x06, // 连续温度测量模式
        CONT_BOTH = 0x07 // 连续压力和温度测量模式
    };

    /**
     * @brief 寄存器块枚举类型
     *
     */
    enum RegisterBlocks_e
    {
        PRS = 0,  // 压力值寄存器块
        TEMP = 1, // 温度值寄存器块
    };

    /**
     * @brief 寄存器块结构体数组
     *
     * 该数组定义了压力值和温度值寄存器块的起始地址和长度。
     */
    const RegBlock_t registerBlocks[2] = {
        {0x00, 3}, // 压力值寄存器块，起始地址为 0x00，长度为 3 字节。这里根据dps310数据手册，压力值寄存器块起始地址应为0x00到0x02
        {0x03, 3}, // 温度值寄存器块，起始地址为 0x03，长度为 3 字节。这里根据dps310数据手册，温度值寄存器块起始地址应为0x03到0x05
    };

    /**
     * @brief 配置寄存器枚举类型
     *
     * 该枚举类型定义了配置寄存器的类型。
     */
    enum Config_Registers_e
    {
        TEMP_MR = 0,   // 温度测量速率
        TEMP_OSR,      // 温度测量分辨率
        PRS_MR,        // 压力测量速率
        PRS_OSR,       // 压力测量分辨率
        MSR_CTRL,      // 测量控制
        FIFO_EN,       // FIFO 使能
        TEMP_RDY,      // 温度就绪标志
        PRS_RDY,       // 压力就绪标志
        INT_FLAG_FIFO, // 中断标志 - FIFO
        INT_FLAG_TEMP, // 中断标志 - 温度
        INT_FLAG_PRS,  // 中断标志 - 压力
    };

    /**
     * @brief 配置寄存器掩码结构体数组
     *
     * 该数组定义了各个配置寄存器的地址、掩码和位偏移量。
     */
    const RegMask_t config_registers[NUM_OF_COMMON_REGMASKS] = {
        {0x07, 0x70, 4}, // TEMP_MR (温度测量速率) 寄存器，地址为 0x07，掩码为 0x70，位偏移量为 4。这表示 TEMP_MR 位于 0x07 寄存器的第 4-6 位。
        {0x07, 0x07, 0}, // TEMP_OSR (温度测量分辨率) 寄存器，地址为 0x07，掩码为 0x07，位偏移量为 0。这表示 TEMP_OSR 位于 0x07 寄存器的第 0-2 位。
        {0x06, 0x70, 4}, // PRS_MR (压力测量速率) 寄存器，地址为 0x06，掩码为 0x70，位偏移量为 4。这表示 PRS_MR 位于 0x06 寄存器的第 4-6 位。
        {0x06, 0x07, 0}, // PRS_OSR (压力测量分辨率) 寄存器，地址为 0x06，掩码为 0x07，位偏移量为 0。这表示 PRS_OSR 位于 0x06 寄存器的第 0-2 位。
        {0x08, 0x07, 0}, // MSR_CTRL (测量控制) 寄存器，地址为 0x08，掩码为 0x07，位偏移量为 0。这表示 MSR_CTRL 位于 0x08 寄存器的第 0-2 位。
        {0x09, 0x02, 1}, // FIFO_EN (FIFO 使能) 寄存器，地址为 0x09，掩码为 0x02，位偏移量为 1。这表示 FIFO_EN 位于 0x09 寄存器的第 1 位。
        {0x08, 0x20, 5}, // TEMP_RDY (温度就绪标志) 寄存器，地址为 0x08，掩码为 0x20，位偏移量为 5。这表示 TEMP_RDY 位于 0x08 寄存器的第 5 位。
        {0x08, 0x10, 4}, // PRS_RDY (压力就绪标志) 寄存器，地址为 0x08，掩码为 0x10，位偏移量为 4。这表示 PRS_RDY 位于 0x08 寄存器的第 4 位。
        {0x0A, 0x04, 2}, // INT_FLAG_FIFO (中断标志 - FIFO) 寄存器，地址为 0x0A，掩码为 0x04，位偏移量为 2。这表示 INT_FLAG_FIFO 位于 0x0A 寄存器的第 2 位。
        {0x0A, 0x02, 1}, // INT_FLAG_TEMP (中断标志 - 温度) 寄存器，地址为 0x0A，掩码为 0x02，位偏移量为 1。这表示 INT_FLAG_TEMP 位于 0x0A 寄存器的第 1 位。
        {0x0A, 0x01, 0}, // INT_FLAG_PRS (中断标志 - 压力) 寄存器，地址为 0x0A，掩码为 0x01，位偏移量为 0。这表示 INT_FLAG_PRS 位于 0x0A 寄存器的第 0 位。
    };

} // namespace dps
#endif /* DPS_CONSTS_H_ */
