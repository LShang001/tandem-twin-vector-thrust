/**
 * @file CCAEGT.cpp
 * @module 固件/宿主参考（CC-AEGT 实飞制导库，逐字复制，仅测试用）
 * @brief CC-AEGT 制导算法核心实现
 */

#include "CCAEGT.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

// ============================================================================
// 静态内部状态 (避免动态内存分配)
// ============================================================================
static CCAEGT_Internal_t s_internal = {CCAEGT_PHASE_IDLE, 0.0f, NULL, 0, 0, false};
static CCAEGT_Stats_t s_stats = {0};
static float s_default_delay_queue[32] = {0}; // 默认延迟队列 (0.1s/0.005s = 20 步)

// ============================================================================
// 前向声明
// ============================================================================
static CCAEGT_PredictResult_t predictCriticalCapture(const CCAEGT_State_t *state);
static void computeAdaptiveEnvelope(float h, float m, float *v_lim, float *a_cap);
static void computeControlLaw(const CCAEGT_State_t *state, float v_lim, float a_cap,
                              CCAEGT_Output_t *output);
static void applyActuatorDynamics(CCAEGT_State_t *state, float T_cmd);
static uint32_t micros_wrapper(void);

// ============================================================================
// 公共 API 实现
// ============================================================================

int CCAEGT_Init(CCAEGT_State_t *state)
{
    if (state == NULL)
        return -1;

    // 初始化状态
    state->h = 0.0f;
    state->v = 0.0f;
    state->m = CCAEGT_M0;
    state->prop = (CCAEGT_M0 - CCAEGT_M_DRY);
    state->T = 0.0f;
    state->t = 0.0f;

    // 初始化内部状态
    s_internal.phase = CCAEGT_PHASE_IDLE;
    s_internal.integrator = 0.0f;
    s_internal.delay_queue = s_default_delay_queue;
    s_internal.delay_steps = sizeof(s_default_delay_queue) / sizeof(s_default_delay_queue[0]);
    s_internal.delay_idx = 0;
    s_internal.initialized = true;

    // 初始化延迟队列
    for (uint16_t i = 0; i < s_internal.delay_steps; i++)
    {
        s_internal.delay_queue[i] = 0.0f;
    }

    // 重置统计
    memset(&s_stats, 0, sizeof(s_stats));

    return 0;
}

int CCAEGT_Init_Custom(CCAEGT_State_t *state, float *delay_queue, uint16_t delay_steps)
{
    if (state == NULL || delay_queue == NULL || delay_steps == 0)
        return -1;

    int ret = CCAEGT_Init(state);
    if (ret != 0)
        return ret;

    s_internal.delay_queue = delay_queue;
    s_internal.delay_steps = delay_steps;

    // 强制清零用户传入的数组，防止带有随机垃圾数据导致推力异常
    for (uint16_t i = 0; i < delay_steps; i++)
    {
        s_internal.delay_queue[i] = 0.0f;
    }

    return 0;
}

int CCAEGT_Compute(CCAEGT_State_t *state, CCAEGT_Output_t *output)
{
    if (state == NULL || output == NULL || !s_internal.initialized)
        return -1;

    uint32_t start_time = micros_wrapper();

    // 初始化输出
    memset(output, 0, sizeof(CCAEGT_Output_t));

    // 计算自适应包络
    computeAdaptiveEnvelope(state->h, state->m, &output->v_lim, &output->a_cap);

    // 状态机逻辑
    switch (s_internal.phase)
    {
    case CCAEGT_PHASE_ASCENT:
    {
        // ==========================================================
        // 真实飞行逻辑：上升段推力完全由飞控主程序（遥控器）决定
        // 算法内部只做状态跟随，不干预油门
        // ==========================================================
        output->T_cmd = state->T; // 追踪真实的当前推力
        output->T_ff = state->T;
        output->T_fb = 0.0f;

        // ==========================================================
        // 检测物理顶点 (Apogee) - 彻底移除 5 秒死板限制！
        // 只要火箭离开地面一定高度，并且垂直速度开始朝下 ( < -0.2m/s 防噪声)，
        // 就自动切入自由落体回收序列
        // ==========================================================
        if (state->h > CCAEGT_H_MARGIN && state->v < -0.2f)
        {
            s_internal.phase = CCAEGT_PHASE_FREEFALL;
        }
        break;
    }

    case CCAEGT_PHASE_FREEFALL:
    {
        // 自由落体段：维持最小推力 + 临界预测
        output->T_cmd = CCAEGT_T_MIN; // 【修改】不再是 0.0f
        output->T_ff = 0.0f;
        output->T_fb = 0.0f;

        // 运行临界捕获预测器
        CCAEGT_PredictResult_t pred = predictCriticalCapture(state);

        if (pred == CCAEGT_PRED_CRITICAL)
        {
            // 触发点火
            s_internal.phase = CCAEGT_PHASE_BRAKE;
            s_internal.integrator = 0.0f;

            // 填充延迟队列
            for (uint16_t i = 0; i < s_internal.delay_steps; i++)
            {
                s_internal.delay_queue[i] = state->T;
            }
        }
        break;
    }

    case CCAEGT_PHASE_BRAKE:
    {
        // 制动段：复合控制
        computeControlLaw(state, output->v_lim, output->a_cap, output);

        // 触地检测：只要高度到了必须关机，再根据速度判断是安全着陆还是坠毁
        if (state->h <= 0.2f)
        {
            if (fabsf(state->v) <= CCAEGT_LAND_VELOCITY_TOL)
            {
                s_internal.phase = CCAEGT_PHASE_LANDED;
            }
            else
            {
                s_internal.phase = CCAEGT_PHASE_CRASHED;
            }
            output->T_cmd = 0.0f;
            output->T_ff = 0.0f;
            output->T_fb = 0.0f;
        }
        break;
    }

    case CCAEGT_PHASE_LANDED:
    case CCAEGT_PHASE_CRASHED:
    {
        output->T_cmd = 0.0f;
        output->T_ff = 0.0f;
        output->T_fb = 0.0f;
        break;
    }

    default:
        break;
    }

    // 应用执行机构动态
    applyActuatorDynamics(state, output->T_cmd);

    // 更新状态统计
    s_stats.compute_count++;
    if (output->a_cap > s_stats.max_a_cap)
    {
        s_stats.max_a_cap = output->a_cap;
    }
    if (state->h < s_stats.min_altitude || s_stats.min_altitude == 0)
    {
        s_stats.min_altitude = state->h;
    }

    // 记录计算耗时
    s_stats.last_compute_time_us = micros_wrapper() - start_time;

    return 0;
}

CCAEGT_Phase_t CCAEGT_GetPhase(const CCAEGT_State_t *state)
{
    (void)state;
    return s_internal.phase;
}

int CCAEGT_SetPhase(CCAEGT_State_t *state, CCAEGT_Phase_t phase)
{
    if (state == NULL)
        return -1;
    s_internal.phase = phase;
    return 0;
}

void CCAEGT_ResetIntegrator(CCAEGT_State_t *state)
{
    (void)state;
    s_internal.integrator = 0.0f;
}

bool CCAEGT_IsLanded(const CCAEGT_State_t *state)
{
    (void)state;
    return (s_internal.phase == CCAEGT_PHASE_LANDED);
}

bool CCAEGT_IsCrashed(const CCAEGT_State_t *state)
{
    (void)state;
    return (s_internal.phase == CCAEGT_PHASE_CRASHED);
}

void CCAEGT_GetStats(CCAEGT_Stats_t *stats)
{
    if (stats != NULL)
    {
        memcpy(stats, &s_stats, sizeof(CCAEGT_Stats_t));
    }
}

const char *CCAEGT_GetVersion(void)
{
    return "1.0.0";
}

// ============================================================================
// 内部函数实现
// ============================================================================

/**
 * @brief 临界捕获预测器
 */
static CCAEGT_PredictResult_t predictCriticalCapture(const CCAEGT_State_t *state)
{
    // 悲观参数
    float T_pred = CCAEGT_T_MAX * CCAEGT_PRED_T_FACTOR;
    float m_pred = state->m * CCAEGT_PRED_M_FACTOR;
    float delay_eff = CCAEGT_DELAY * CCAEGT_PRED_DELAY_FACTOR;

    // 预测状态
    float h = state->h;
    float v = state->v;
    float m = m_pred;
    float prop = state->prop;
    float T = state->T;

    // 延迟队列 (本地副本)
    float pred_queue[32];
    uint16_t delay_steps = (uint16_t)(delay_eff / CCAEGT_PRED_DT);
    if (delay_steps == 0)
        delay_steps = 1; // 【修复 1】：防止 delay_eff 为 0 时触发除零异常导致 MCU 死机
    if (delay_steps > 32)
        delay_steps = 32;

    for (uint16_t i = 0; i < delay_steps; i++)
    {
        pred_queue[i] = T;
    }

    // 预测步数
    uint16_t steps = (uint16_t)(CCAEGT_PRED_HORIZON / CCAEGT_PRED_DT);

    for (uint16_t i = 0; i < steps; i++)
    {
        // 撞地检查
        if (h <= 0.0f)
        {
            return CCAEGT_PRED_CRITICAL;
        }

        // 请求最大推力 (如果 prop 没燃料了，请求最小维持推力)
        float T_cmd = (prop > 0) ? T_pred : CCAEGT_T_MIN;

        // 模拟延迟：先取延迟后的值，再将新值入队
        uint16_t idx = i % delay_steps;
        float T_delayed = pred_queue[idx];
        pred_queue[idx] = T_cmd;

        // 推力变化率限制
        float dT = T_delayed - T;
        float max_dT = CCAEGT_T_RATE_LIMIT * CCAEGT_PRED_DT;
        if (dT > max_dT)
            dT = max_dT;
        if (dT < -max_dT)
            dT = -max_dT;
        T = T + dT;
        if (T > CCAEGT_T_MAX)
            T = CCAEGT_T_MAX;
        if (T < 0.0f)
            T = 0.0f;

        // 动力学积分 (Euler)
        float actual_T = (prop > 0) ? T : 0.0f;
        float mdot = (actual_T > 0) ? actual_T / (CCAEGT_GRAVITY * CCAEGT_Isp) : 0.0f;
        float acc = (actual_T / m) - CCAEGT_GRAVITY;

        v = v + acc * CCAEGT_PRED_DT;
        h = h + v * CCAEGT_PRED_DT;
        m = m - mdot * CCAEGT_PRED_DT;
        prop = prop - mdot * CCAEGT_PRED_DT;

        // 检查包络捕获
        float v_lim_pred, a_cap_pred;
        computeAdaptiveEnvelope(h, m, &v_lim_pred, &a_cap_pred);

        // 【修复 2】：预测轨迹一旦跌穿包络，说明当前状态已危险，必须立即点火
        if (v < v_lim_pred)
        {
            return CCAEGT_PRED_CRITICAL;
        }

        // 【修复 3】：如果在未跌穿包络的情况下，成功将速度减到了 0（或正数），说明安全
        if (v >= 0.0f)
        {
            return CCAEGT_PRED_SAFE;
        }
    }

    // 如果跑完了整个预测时域，既没有跌穿包络，也没把速度降到0，默认当前是安全的
    return CCAEGT_PRED_SAFE;
}

/**
 * @brief 计算自适应动态包络
 */
static void computeAdaptiveEnvelope(float h, float m, float *v_lim, float *a_cap)
{
    // 计算当前制动能力
    *a_cap = (CCAEGT_ETA * CCAEGT_T_MAX / m) - CCAEGT_GRAVITY;
    if (*a_cap < 0.1f)
        *a_cap = 0.1f;

    // 计算交接速度
    float v_touch_sq = CCAEGT_V_FINAL * CCAEGT_V_FINAL + (*a_cap) * CCAEGT_H_MARGIN;

    if (h <= CCAEGT_H_MARGIN)
    {
        // 低空段：抛物线柔和接入
        float stiff = (*a_cap) / CCAEGT_H_MARGIN;
        *v_lim = -sqrtf(CCAEGT_V_FINAL * CCAEGT_V_FINAL + stiff * h * h);
    }
    else
    {
        // 高空段：全能力制动
        float v_mag = sqrtf(v_touch_sq + 2.0f * (*a_cap) * (h - CCAEGT_H_MARGIN));
        *v_lim = -v_mag;
        if (*v_lim < -CCAEGT_V_MAX_DESC)
            *v_lim = -CCAEGT_V_MAX_DESC;
    }
}

/**
 * @brief 复合控制律计算
 */
static void computeControlLaw(const CCAEGT_State_t *state, float v_lim, float a_cap,
                              CCAEGT_Output_t *output)
{
    output->v_lim = v_lim;
    output->a_cap = a_cap;

    // 速度误差
    output->v_err = v_lim - state->v;

    // --- 前馈项 ---
    if (state->h > CCAEGT_H_MARGIN)
    {
        output->T_ff = CCAEGT_ETA * CCAEGT_T_MAX;
    }
    else
    {
        float a_req = a_cap * (state->h / CCAEGT_H_MARGIN);
        output->T_ff = state->m * (CCAEGT_GRAVITY + a_req);
    }

    // --- 反馈项 ---
    // 抗积分饱和检查
    // 获取上一周期最新计算出的指令，而不是即将出队的最老指令
    uint16_t last_idx = (s_internal.delay_idx + s_internal.delay_steps - 1) % s_internal.delay_steps;
    float prev_cmd = s_internal.delay_queue[last_idx];
    bool saturated = false;

    if (prev_cmd >= CCAEGT_ETA * CCAEGT_T_MAX && output->v_err > 0)
    {
        saturated = true;
    }
    // 防积分饱和的下限必须与当前的物理最小油门对齐
    if (prev_cmd <= CCAEGT_T_MIN && output->v_err < 0)
    {
        saturated = true;
    }

    if (!saturated)
    {
        s_internal.integrator = s_internal.integrator + output->v_err * CCAEGT_DT;
    }

    output->T_fb = state->m * (CCAEGT_Kp * output->v_err + CCAEGT_Ki * s_internal.integrator);

    // --- 合成指令 ---
    output->T_cmd = output->T_ff + output->T_fb;

    // 限幅
    float T_max_use = (state->prop > 0) ? CCAEGT_ETA * CCAEGT_T_MAX : 0.0f;
    if (output->T_cmd > T_max_use)
        output->T_cmd = T_max_use;
    if (output->T_cmd < CCAEGT_T_MIN)
        output->T_cmd = CCAEGT_T_MIN; // 【修改】限幅下限
}

/**
 * @brief 执行机构动态模拟
 */
static void applyActuatorDynamics(CCAEGT_State_t *state, float T_cmd)
{
    // 更新延迟队列
    s_internal.delay_queue[s_internal.delay_idx] = T_cmd;
    s_internal.delay_idx = (s_internal.delay_idx + 1) % s_internal.delay_steps;

    float T_delayed = s_internal.delay_queue[s_internal.delay_idx];

    // 推力变化率限制
    float dT = T_delayed - state->T;
    float max_dT = CCAEGT_T_RATE_LIMIT * CCAEGT_DT;

    if (dT > max_dT)
        dT = max_dT;
    if (dT < -max_dT)
        dT = -max_dT;

    state->T = state->T + dT;

    if (state->T > CCAEGT_T_MAX)
        state->T = CCAEGT_T_MAX;
    if (state->T < 0.0f)
        state->T = 0.0f;

    // 无燃料时推力为零
    if (state->prop <= 0.0f)
    {
        state->T = 0.0f;
    }
}

/**
 * @brief 微秒计时器包装 (兼容不同平台)
 */
static uint32_t micros_wrapper(void)
{
#ifdef ARDUINO
    return micros();
#else
    // 非 Arduino 平台返回 0
    return 0;
#endif
}