/**
 * @file CCAEGT.h
 * @module 固件/宿主参考（CC-AEGT 实飞制导库，逐字复制，仅测试用）
 * @brief CC-AEGT 制导算法主头文件
 * @details 提供完整的 API 接口用于 Arduino 飞控项目
 *
 * @section usage 使用示例
 * @code
 * #include <CCAEGT.h>
 *
 * CCAEGT_State_t state = {0};
 * CCAEGT_Output_t output = {0};
 *
 * void setup() {
 *     CCAEGT_Init(&state);
 * }
 *
 * void loop() {
 *     // 更新传感器数据
 *     state.h = getAltitude();
 *     state.v = getVelocity();
 *     state.m = estimateMass();
 *
 *     // 计算制导指令
 *     CCAEGT_Compute(&state, &output);
 *
 *     // 输出推力指令
 *     setEngineThrust(output.T_cmd);
 *
 *     delayMicroseconds(5000); // 5ms 控制周期
 * }
 * @endcode
 */

#ifndef CCAEGT_H
#define CCAEGT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "CCAEGT_Config.h"
#include "CCAEGT_Types.h"
#include <stddef.h>

    /**
     * @brief 初始化制导算法
     * @param state 飞行器状态指针 (输入/输出)
     * @return 0 成功，-1 失败
     */
    int CCAEGT_Init(CCAEGT_State_t *state);

    /**
     * @brief 初始化制导算法 (带自定义参数)
     * @param state 飞行器状态指针
     * @param delay_queue 延迟队列缓冲区 (需预先分配)
     * @param delay_steps 延迟队列长度
     * @return 0 成功，-1 失败
     */
    int CCAEGT_Init_Custom(CCAEGT_State_t *state, float *delay_queue, uint16_t delay_steps);

    /**
     * @brief 执行一次制导计算
     * @param state 飞行器状态指针 (输入/输出)
     * @param output 制导输出指针 (输出)
     * @return 0 成功，-1 失败
     */
    int CCAEGT_Compute(CCAEGT_State_t *state, CCAEGT_Output_t *output);

    /**
     * @brief 获取当前飞行阶段
     * @param state 飞行器状态指针
     * @return 飞行阶段枚举值
     */
    CCAEGT_Phase_t CCAEGT_GetPhase(const CCAEGT_State_t *state);

    /**
     * @brief 手动设置飞行阶段
     * @param state 飞行器状态指针
     * @param phase 目标飞行阶段
     * @return 0 成功，-1 失败
     */
    int CCAEGT_SetPhase(CCAEGT_State_t *state, CCAEGT_Phase_t phase);

    /**
     * @brief 重置积分器 (用于阶段切换)
     * @param state 飞行器状态指针
     */
    void CCAEGT_ResetIntegrator(CCAEGT_State_t *state);

    /**
     * @brief 检查是否已着陆
     * @param state 飞行器状态指针
     * @return true 已着陆，false 未着陆
     */
    bool CCAEGT_IsLanded(const CCAEGT_State_t *state);

    /**
     * @brief 检查是否坠毁
     * @param state 飞行器状态指针
     * @return true 坠毁，false 正常
     */
    bool CCAEGT_IsCrashed(const CCAEGT_State_t *state);

    /**
     * @brief 获取算法统计信息
     * @param stats 统计信息结构体指针 (输出)
     */
    void CCAEGT_GetStats(CCAEGT_Stats_t *stats);

    /**
     * @brief 获取版本号
     * @return 版本字符串
     */
    const char *CCAEGT_GetVersion(void);

#ifdef __cplusplus
}
#endif

#endif // CCAEGT_H