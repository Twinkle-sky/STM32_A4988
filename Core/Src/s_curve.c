/**
 *******************************************************************************
 * @file    s_curve.c
 * @brief   S曲线加减速算法实现
 * 
 * @details 
 * 本文件实现了完整的七段式S曲线加减速控制算法，包括：
 * - 初始化：预计算各阶段步数
 * - 启动：设置运动参数并启动状态机
 * - 更新：每个控制周期计算当前速度和状态
 * - 脉冲处理：生成精确的步进脉冲
 * 
 * S曲线原理说明：
 * S曲线的核心思想是用"加加速度(jerk)"来控制加速度的变化，
 * 使得加速度本身也是线性变化的，从而避免速度曲线出现突变点。
 * 
 * 数学基础：
 * - jerk (j) = da/dt  (加速度的变化率)
 * - accl (a) = ∫j·dt  (加速度是jerk的积分)
 * - velo (v) = ∫a·dt  (速度是加速度的积分)
 * - pos   (s) = ∫v·dt  (位移是速度的积分)
 * 
 * @version V1.0
 * @author  Twinkle-sky
 * @date    2026-05-12
 *******************************************************************************
 */

#include "s_curve.h"
#include <stdlib.h>

/*============================================================================*
 *                          内部辅助函数
 *============================================================================*/

/**
 * @brief       取三个浮点数的最小值
 * @details     比较三个浮点数，返回最小值
 *              注意：浮点数比较可能存在精度问题，此处简化处理
 * 
 * @param[in]   a   第一个浮点数
 * @param[in]   b   第二个浮点数
 * @param[in]   c   第三个浮点数
 * @return      三个数中的最小值
 */
static float min3(float a, float b, float c) {
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

/*============================================================================*
 *                          核心函数实现
 *============================================================================*/

/**
 * @brief       初始化S曲线控制器
 * @details     根据配置参数初始化S曲线控制器，主要完成以下工作：
 *              1. 保存配置参数
 *              2. 预计算加速阶段各子段时间和步数
 *              3. 预计算减速阶段各子段时间和步数
 * 
 *              加速段分析：
 *              - 情况A（速度增量足够大）：包含加加速、匀加速、减加速三个子段
 *              - 情况B（速度增量较小）：只有加加速和减加速（纯S曲线）
 * 
 *              判断条件：(v_target - v_start) > A²/J
 * 
 * @param[in]   h   S曲线句柄指针
 * @param[in]   cfg S曲线配置参数指针
 * @return      无
 */
void SCurve_Init(SCurve_Handle_t *h, SCurveConfig_t *cfg) 
{
    /* 保存配置参数到句柄 */
    h->cfg = *cfg;
    h->state = STATE_IDLE;
    h->running = false;
    
    /* 读取配置参数，便于后续计算 */
    float J = cfg->jerk;      /* 加加速度 */
    float A = cfg->accl;      /* 最大加速度 */
    float vs = cfg->v_start;  /* 起始速度 */
    float vt = cfg->v_target; /* 目标速度 */
    
    /**
     * ================================================================
     *                      加速阶段步数计算
     * ================================================================
     * 
     * 判断条件：(vt - vs) > A²/J
     * - 如果满足：速度增量足够大，可以达到最大加速度A
     * - 如果不满足：速度增量较小，无法达到最大加速度
     */
    
    if ((vt - vs) > (A * A) / J) 
    {
        /*------------------------------------------------------------
         * 情况A：能达到最大加速度（包含匀加速段）
         * 
         * 速度曲线：
         *        ^
         *        |     /--------
         *        |    /         
         *        |   /          
         *        |  /           
         *        | /            
         *        +--------------->
         * 
         * 三段时间相等：t_jinc = t_aconst = t_jdec = A/J
         *------------------------------------------------------------*/
        
        float t_jinc = A / J;          /* 加加速段时间：加速度从0增加到A的时间 */
        float t_jdec = A / J;          /* 减加速段时间：加速度从A减小到0的时间 */
        
        /* 计算加加速段结束时的速度 */
        float v_jinc_end = vs + 0.5f * J * t_jinc * t_jinc;
        /* 计算减加速段开始时的速度（进入匀速前的速度） */
        float v_jdec_start = vt - 0.5f * J * t_jdec * t_jdec;
        /* 计算匀加速段时间 */
        float t_aconst = (v_jdec_start - v_jinc_end) / A;
        
        // 加加速段步数（精确：θ = vs·t + J·t³/6）
        h->steps.jerk_inc = (uint32_t)(vs * t_jinc + J * t_jinc * t_jinc * t_jinc / 6.0f);

        // 匀加速段步数（精确：θ = v₀·t + ½A·t²，其中v₀=v_jinc_end）
        h->steps.accel_const = (uint32_t)(v_jinc_end * t_aconst + 0.5f * A * t_aconst * t_aconst);

        // 减加速段步数（精确：θ = v₀·t + ½A·t² - J·t³/6，或从终点反推）
        // 从起点积分的精确公式：
        h->steps.jerk_dec = (uint32_t)(v_jdec_start * t_jdec + 0.5f * A * t_jdec * t_jdec 
                                        - J * t_jdec * t_jdec * t_jdec / 6.0f);
    } 
    else 
    {
        /*------------------------------------------------------------
         * 情况B：达不到最大加速度（纯S曲线加速）
         * 
         * 速度曲线：
         *        ^
         *        |   /\        
         *        |  /  \       
         *        | /    \      
         *        |/      \    
         *        +----------->
         * 
         * 只有两个对称的加加速和减加速段
         *------------------------------------------------------------*/
        
        /* 半段加速时间（从vs加速到vt的一半所需时间） */
        float dt_acc = sqrtf((vt - vs) / J);
        
        /* 计算纯S曲线的步数（使用三次多项式积分） */
        h->steps.jerk_inc = (uint32_t)(vs * dt_acc + J * dt_acc * dt_acc * dt_acc / 6.0f);
        h->steps.accel_const = 0;  /* 无匀加速段 */
        h->steps.jerk_dec = (uint32_t)((vs + vt) * dt_acc - h->steps.jerk_inc);
    }
    
    /* 计算加速阶段总步数 */
    h->steps.accel_total = h->steps.jerk_inc + h->steps.accel_const + h->steps.jerk_dec;
    
    /**
     * ================================================================
     *                      减速阶段步数计算
     * ================================================================
     * 
     * 减速段与加速段类似，采用对称结构
     * 区别：起始速度是vt，结束速度是ve（通常为0）
     */
    
    float D = cfg->decel;      /* 最大减速度 */
    float ve = cfg->v_end;      /* 结束速度 */
    
    /* 减速判断条件 */
    if ((vt - ve) > (D * D) / J) 
    {
        /*------------------------------------------------------------
         * 能达到最大减速度：减速过程有三个子段
         * 
         * 速度曲线：
         *        ^
         *        |   /\        
         *        |  /  \       
         *        | /    \      
         *        |/      \    
         *        +----------->
         * 
         * 顺序：加减速段(jerk_dec_2) → 匀减速段(decel_const) → 减减速段(jerk_inc_2)
         *------------------------------------------------------------*/
        
        float t_jinc2 = D / J;  /* 加减速段时间 */
        float t_jdec2 = D / J;  /* 减减速段时间 */
        
        /* 计算加减速段结束时的速度 */
        float v_jdec2_end = vt - 0.5f * J * t_jinc2 * t_jinc2;
        /* 计算减减速段开始时的速度 */
        float v_jinc2_start = ve + 0.5f * J * t_jdec2 * t_jdec2;
        /* 计算匀减速段时间 */
        float t_decel_const = (v_jdec2_end - v_jinc2_start) / D;
        
        /* 计算各子段步数（使用梯形积分法） */
        h->steps.jerk_dec_2 = (uint32_t)((vt + v_jdec2_end) * 0.5f * t_jinc2);
        h->steps.decel_const = (uint32_t)((v_jdec2_end + v_jinc2_start) * 0.5f * t_decel_const);
        h->steps.jerk_inc_2 = (uint32_t)((v_jinc2_start + ve) * 0.5f * t_jdec2);
    } 
    else 
    {
        /* 纯S曲线减速 */
        float dt_dec = sqrtf((vt - ve) / J);
        h->steps.jerk_dec_2 = (uint32_t)(vt * dt_dec - J * dt_dec * dt_dec * dt_dec / 6.0f);
        h->steps.decel_const = 0;
        h->steps.jerk_inc_2 = (uint32_t)((vt + ve) * dt_dec - h->steps.jerk_dec_2);
    }
    
    /* 计算减速阶段总步数 */
    h->steps.decel_total = h->steps.jerk_dec_2 + h->steps.decel_const + h->steps.jerk_inc_2;
}

/**
 * @brief       启动S曲线运动
 * @details     根据给定的总步数和方向启动S曲线运动：
 *              1. 设置总步数和方向
 *              2. 初始化速度和加速度
 *              3. 计算匀速段步数
 *              4. 处理短行程情况（行程不够时的处理）
 *              5. 启动状态机，进入加加速段
 * 
 * @param[in]   h           S曲线句柄指针
 * @param[in]   total_steps 总运动步数
 * @param[in]   dir         运动方向：0=反向，1=正向
 * @return      无
 */
void SCurve_Start(SCurve_Handle_t *h, uint32_t total_steps, uint8_t dir) 
{
    /* 保存总步数和方向 */
    h->total_steps = total_steps;
    h->steps_done = 0;
    h->dir = dir;
    
    /* 初始化运动状态 */
    h->v_current = h->cfg.v_start;  /* 从起始速度开始 */
    h->a_current = 0.0f;            /* 初始加速度为0 */
    h->pos_accum = 0.0f;            /* 位置累加器清零 */
    
    /**
     * 计算匀速段步数
     * 
     * 情况1：行程足够
     *        总步数 = 加速步数 + 匀速步数 + 减速步数
     *        匀速步数 = 总步数 - 加速步数 - 减速步数
     * 
     * 情况2：行程不够（短行程）
     *        匀速步数 = 0
     *        需要按比例缩小加速和减速步数
     */
    if (total_steps > h->steps.accel_total + h->steps.decel_total) 
    {
        /* 情况1：行程足够，有匀速段 */
        h->steps.const_total = total_steps - h->steps.accel_total - h->steps.decel_total;
    } 
    else 
    {
        /* 情况2：行程不够，无匀速段 */
        h->steps.const_total = 0;
        
        /* 按比例缩小加速和减速步数 */
        float ratio = (float)total_steps / (h->steps.accel_total + h->steps.decel_total);
        h->steps.accel_total = (uint32_t)(h->steps.accel_total * ratio);
        h->steps.decel_total = total_steps - h->steps.accel_total;
        /* 注意：子阶段的缩放在此简化处理 */
    }
    
    /* 启动状态机 */
    h->state = STATE_JERK_INC;              /* 进入加加速段 */
    h->phase_step_target = h->steps.jerk_inc;  /* 设置当前阶段目标步数 */
    h->phase_step_cnt = 0;                  /* 阶段内步数清零 */
    h->running = true;                     /* 设置运行标志 */
}

/**
 * @brief       紧急停止
 * @details     立即以最大减速度减速停车
 *              用于安全保护或急停按钮触发
 *              - 跳过所有过渡段
 *              - 直接进入匀减速状态
 *              - 设置很大的减速步数确保完全停止
 * 
 * @param[in]   h   S曲线句柄指针
 * @return      无
 */
void SCurve_EmergencyStop(SCurve_Handle_t *h) 
{
    /* 如果未运行，直接返回 */
    if (!h->running) return;
    
    /* 直接跳到匀减速段 */
    h->state = STATE_DECEL_CONST;
    h->a_current = -h->cfg.decel;  /* 设置最大减速度（负值） */
    h->steps.decel_total = 999999; /* 设置很大的值确保能减速到0 */
    h->steps.const_total = 0;      /* 取消匀速段 */
}

/**
 * @brief       更新S曲线状态
 * @details     每个定时器周期调用一次，根据当前状态更新：
 *              - 加速度（基于jerk和状态）
 *              - 速度（加速度积分）
 *              - 状态切换判断
 * 
 *              状态机流程：
 *              加速过程：JERK_INC → ACCEL_CONST → JERK_DEC → CONST_SPEED
 *              减速过程：JERK_DEC_2 → DECEL_CONST → JERK_INC_2 → IDLE
 * 
 * @param[in]   h   S曲线句柄指针
 * @param[in]   dt  时间片长度，单位：秒
 * @return      当前计算得到的速度值，单位：step/s
 */
float SCurve_Update(SCurve_Handle_t *h, float dt) 
{
    /* 未运行直接返回0 */
    if (!h->running) return 0.0f;
    
    /* 读取配置参数 */
    float J = h->cfg.jerk;        /* 加加速度 */
    float A = h->cfg.accl;         /* 最大加速度 */
    float D = h->cfg.decel;        /* 最大减速度 */
    float v_target = h->cfg.v_target;  /* 目标速度 */
    
    /* 根据当前状态进行状态机切换 */
    switch (h->state) 
    {
        /*------------------------------------------------------------
         * 加加速段
         * 状态：加速度从0线性增加到最大值A
         * 条件：加速度达到A时切换
         *------------------------------------------------------------*/
        case STATE_JERK_INC:
            h->a_current += J * dt;  /* 加速度线性增加 */
            if (h->a_current >= A) 
            {
                h->a_current = A;  /* 限幅 */
                
                /* 根据是否有匀加速段决定下一个状态 */
                if (h->steps.accel_const > 0) 
                {
                    h->state = STATE_ACCEL_CONST;
                    h->phase_step_target = h->steps.accel_const;
                } 
                else 
                {
                    h->state = STATE_JERK_DEC;
                    h->phase_step_target = h->steps.jerk_dec;
                }
                h->phase_step_cnt = 0;  /* 重置阶段计数器 */
            }
            break;
            
        /*------------------------------------------------------------
         * 匀加速段
         * 状态：加速度保持最大值A
         * 条件：步数达到目标时切换
         *------------------------------------------------------------*/
        case STATE_ACCEL_CONST:
            h->a_current = A;  /* 保持最大加速度 */
            
            /* 通过步数判断切换 */
            if (h->phase_step_target == 0 || h->phase_step_cnt >= h->phase_step_target) 
            {
                h->state = STATE_JERK_DEC;
                h->phase_step_target = h->steps.jerk_dec;
                h->phase_step_cnt = 0;
            }
            break;
            
        /*------------------------------------------------------------
         * 减加速段
         * 状态：加速度从A线性减小到0
         * 条件：加速度降到0时切换
         *------------------------------------------------------------*/
        case STATE_JERK_DEC:
            h->a_current -= J * dt;  /* 加速度线性减小 */
            if (h->a_current <= 0.0f) 
            {
                h->a_current = 0.0f;
                
                /* 进入匀速或直接减速 */
                if (h->steps.const_total > 0) 
                {
                    h->state = STATE_CONST_SPEED;
                    h->phase_step_target = h->steps.const_total;
                } 
                else 
                {
                    h->state = STATE_JERK_DEC_2;
                    h->phase_step_target = h->steps.jerk_dec_2;
                }
                h->phase_step_cnt = 0;
            }
            break;
            
        /*------------------------------------------------------------
         * 匀速段
         * 状态：速度和加速度都为0
         * 条件：剩余步数接近减速需求时切换
         *------------------------------------------------------------*/
        case STATE_CONST_SPEED:
            h->a_current = 0.0f;  /* 无加速度 */
            
            /* 前瞻判断：检查是否需要开始减速 */
            {
                uint32_t decel_needed = SCurve_CalcDecelSteps(h, h->v_current, h->cfg.v_end);
                if (h->total_steps - h->steps_done <= decel_needed) 
                {
                    h->state = STATE_JERK_DEC_2;
                    h->phase_step_target = h->steps.jerk_dec_2;
                    h->phase_step_cnt = 0;
                }
            }
            break;
            
        /*------------------------------------------------------------
         * 加减速段（第二减加速段）
         * 状态：减速度从0线性增加到最大值D（加速度为负）
         * 条件：减速度达到D时切换
         *------------------------------------------------------------*/
        case STATE_JERK_DEC_2:
            h->a_current -= J * dt;  /* 加速度继续减小（变为负值） */
            if (h->a_current <= -D) 
            {
                h->a_current = -D;  /* 限幅 */
                
                /* 根据是否有匀减速段决定下一个状态 */
                if (h->steps.decel_const > 0) 
                {
                    h->state = STATE_DECEL_CONST;
                    h->phase_step_target = h->steps.decel_const;
                } 
                else 
                {
                    h->state = STATE_JERK_INC_2;
                    h->phase_step_target = h->steps.jerk_inc_2;
                }
                h->phase_step_cnt = 0;
            }
            break;
            
        /*------------------------------------------------------------
         * 匀减速段
         * 状态：减速度保持最大值D（加速度为-D）
         * 条件：步数达到目标时切换
         *------------------------------------------------------------*/
        case STATE_DECEL_CONST:
            h->a_current = -D;  /* 保持最大减速度 */
            if (h->phase_step_cnt >= h->phase_step_target) 
            {
                h->state = STATE_JERK_INC_2;
                h->phase_step_target = h->steps.jerk_inc_2;
                h->phase_step_cnt = 0;
            }
            break;
            
        /*------------------------------------------------------------
         * 减减速段（第二减加速段）
         * 状态：减速度从D线性减小到0（加速度从-D向0变化）
         * 条件：加速度回到0时，运动完成
         *------------------------------------------------------------*/
        case STATE_JERK_INC_2:
            h->a_current += J * dt;  /* 减速度减小（加速度向0变化） */
            if (h->a_current >= 0.0f) 
            {
                h->a_current = 0.0f;
                h->v_current = h->cfg.v_end;  /* 设置最终速度 */
                h->running = false;           /* 标记运行结束 */
                h->state = STATE_IDLE;         /* 进入空闲状态 */
                return h->v_current;           /* 返回最终速度 */
            }
            break;
            
        /*------------------------------------------------------------
         * 默认情况
         *------------------------------------------------------------*/
        default:
            return 0.0f;
    }
    
    /* 更新速度：v = v + a * dt */
    h->v_current += h->a_current * dt;
    
    /* 速度限幅保护 */
    if (h->v_current > v_target) h->v_current = v_target;  /* 不超过目标速度 */
    if (h->v_current < 0.0f) h->v_current = 0.0f;          /* 不低于0 */
    
    return h->v_current;
}

/**
 * @brief       计算减速所需步数
 * @details     根据当前速度和目标速度，计算以S曲线方式减速到目标速度所需的步数
 *              这是实现匀速段"前瞻"功能的关键函数
 * 
 *              算法原理：
 *              - 情况1（速度增量小）：无法达到最大减速度D，只有两个对称段
 *              - 情况2（速度增量大）：可以达到最大减速度D，有三个段
 * 
 * @param[in]   h       S曲线句柄指针
 * @param[in]   v_from  起始速度，单位：step/s
 * @param[in]   v_to    目标速度，单位：step/s
 * @return      减速所需的步数
 */
uint32_t SCurve_CalcDecelSteps(SCurve_Handle_t *h, float v_from, float v_to) 
{
    /* 如果起始速度小于等于目标速度，不需要减速 */
    if (v_from <= v_to) return 0;
    
    /* 读取参数 */
    float J = h->cfg.jerk;    /* 加加速度 */
    float D = h->cfg.decel;    /* 最大减速度 */
    float dv = v_from - v_to;  /* 速度变化量 */
    
    /* 判断条件：dv <= D²/J */
    if (dv <= (D * D) / J) 
    {
        /*------------------------------------------------------------
         * 情况1：速度增量较小，无法达到最大减速度（纯S曲线减速）
         * 
         * 时间：t = 2 * sqrt(dv / J)
         * 步数：s = (v_from + v_to) / 2 * t
         *------------------------------------------------------------*/
        float t_half = sqrtf(dv / J);
        /* 步数 = 平均速度 × 总时间 */
        return (uint32_t)((v_from + v_to) * 0.5f * 2.0f * t_half);
    } 
    else 
    {
        /*------------------------------------------------------------
         * 情况2：速度增量较大，可以达到最大减速度（三段式减速）
         * 
         * 三段时间：
         * - t_inc: 加减速段时间 = D/J
         * - t_const: 匀减速段时间 = (dv - D²/J) / D
         * - t_dec: 减减速段时间 = D/J
         *------------------------------------------------------------*/
        float t_inc = D / J;                  /* 加减速段时间 */
        float t_const = (dv - D*D/J) / D;     /* 匀减速段时间 */
        float v_dec = v_to + D*D/(2.0f*J);    /* 匀减速段起始速度 */
        
        /* 计算三段的步数 */
        float s_inc = (v_from + v_dec) * 0.5f * t_inc;
        float s_const = (v_dec - D*D/(2.0f*J) + v_to + D*D/(2.0f*J)) * 0.5f * t_const;
        float s_dec = (v_to + v_to + D*D/(2.0f*J)) * 0.5f * t_inc;
        
        return (uint32_t)(s_inc + s_const + s_dec);
    }
}

/**
 * @brief       处理脉冲输出
 * @details     综合S曲线计算和脉冲生成逻辑：
 *              1. 调用SCurve_Update更新运动状态
 *              2. 累加位置积分
 *              3. 判断是否需要输出脉冲
 *              4. 更新步数计数器和状态
 * 
 *              位置积分原理：
 *              每隔dt时间，速度v对应的位移为 v*dt
 *              当累加位移达到1时，表示需要输出一个脉冲
 * 
 * @param[in]   h           S曲线句柄指针
 * @param[in]   dt          时间片长度，单位：秒
 * @param[out]  need_pulse  输出参数，是否需要输出一个脉冲
 * @return      true=运动仍在进行，false=运动已完成或未运行
 */
bool SCurve_ProcessPulse(SCurve_Handle_t *h, float dt, bool *need_pulse) 
{
    /* 初始化输出参数 */
    *need_pulse = false;
    
    /* 如果未运行，直接返回false */
    if (!h->running) return false;
    
    /* 更新S曲线状态，获取当前速度 */
    float v = SCurve_Update(h, dt);
    
    /* 如果速度降到0，运动结束 */
    if (v <= 0.0f) {
        h->running = false;
        return false;
    }
    
    /**
     * 位置累加
     * 
     * 原理：v = ds/dt  →  ds = v * dt
     * 每隔dt时间，位移增加 v*dt
     * 当位移累加到1时，表示电机应该走一步
     */
    h->pos_accum += v * dt;
    
    /**
     * 判断是否需要输出脉冲
     * 
     * 当位置累加器超过1时：
     * - 设置need_pulse为true（需要输出脉冲）
     * - 减去1（保留余数用于下次累加）
     * - 步数计数器加1
     */
    if (h->pos_accum >= 1.0f) 
    {
        *need_pulse = true;           /* 需要输出脉冲 */
        h->pos_accum -= 1.0f;         /* 减去一个单位，保留余数 */
        h->steps_done++;              /* 已完成步数加1 */
        h->phase_step_cnt++;         /* 当前阶段步数加1 */
    }
    
    return h->running;
}