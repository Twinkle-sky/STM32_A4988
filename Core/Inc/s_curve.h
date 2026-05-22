/**
 *******************************************************************************
 * @file    s_curve.h
 * @brief   S曲线加减速算法头文件
 * 
 * @details 
 * S曲线是一种平滑的速度曲线，相比传统的梯形加减速，具有以下优点：
 * - 速度变化更平滑，无突变
 * - 机械冲击更小，延长设备寿命
 * - 运动过程更加平稳
 * 
 * 本实现采用七段式S曲线模型：
 * 1. 加加速段（jerk > 0） - 加速度从0线性增加到最大值
 * 2. 匀加速段（jerk = 0） - 加速度保持最大值
 * 3. 减加速段（jerk < 0） - 加速度从最大值线性减小到0
 * 4. 匀速段（jerk = 0, a = 0） - 速度保持恒定
 * 5. 加减速段（jerk < 0） - 减速度从0线性增加到最大值
 * 6. 匀减速段（jerk = 0） - 减速度保持最大值
 * 7. 减减速段（jerk > 0） - 减速度从最大值线性减小到0
 * 
 * @version V1.0
 * @author  Twinkle-sky
 * @date    2026-05-12
 *******************************************************************************
 */

#ifndef __S_CURVE_H
#define __S_CURVE_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

/**
 * @brief 运动状态枚举
 * @details 描述S曲线运动过程中所处的不同阶段
 */
typedef enum {
    STATE_IDLE = 0,           /**< 空闲状态 - 电机未运行或已停止 */
    STATE_JERK_INC,           /**< 加加速段 - 加速度逐渐增加（jerk为正） */
    STATE_ACCEL_CONST,        /**< 匀加速段 - 加速度保持恒定最大值 */
    STATE_JERK_DEC,           /**< 减加速段 - 加速度逐渐减小（jerk为负） */
    STATE_CONST_SPEED,        /**< 匀速段 - 速度保持恒定 */
    STATE_JERK_DEC_2,         /**< 加减速段 - 减速度逐渐增加 */
    STATE_DECEL_CONST,        /**< 匀减速段 - 减速度保持恒定 */
    STATE_JERK_INC_2          /**< 减减速段 - 减速度逐渐减小到零 */
} MotionState_t;

/**
 * @brief S曲线运动参数配置结构体
 * @details 用户需要配置的所有S曲线运动参数
 */
typedef struct {
    float v_start;             /**< 起始速度，单位：step/s（脉冲/秒） */
    float v_target;           /**< 目标速度（最大运行速度），单位：step/s */
    float v_end;              /**< 结束速度，单位：step/s（通常为0表示停止） */
    float v_max;              /**< 最大限制速度，单位：step/s（用于速度限制保护） */
    float accl;               /**< 最大加速度，单位：step/s²（正数） */
    float decel;              /**< 最大减速度，单位：step/s²（正数） */
    float jerk;               /**< 加加速度（加速度变化率），单位：step/s³ */
    float jerk_homing;        /**< 归零时的加加速度，单位：step/s³（用于原点回归） */
} SCurveConfig_t;

/**
 * @brief S曲线实时运动状态句柄
 * @details 包含S曲线运动控制所需的所有运行时状态和参数
 */
typedef struct {
    /* 参数副本 - 保存初始化时的配置参数 */
    SCurveConfig_t cfg;
    
    /* 实时状态变量 */
    MotionState_t state;      /**< 当前运动状态 */
    float v_current;          /**< 当前速度，单位：step/s */
    float a_current;          /**< 当前加速度，单位：step/s² */
    float pos_accum;          /**< 位置累加器 - 用于精确的微步计算 */
    
    /* 各阶段步数预计算结果 - 初始化时计算，运行时不改变 */
    struct {
        uint32_t accel_total;     /**< 加速阶段总步数（包含加加速+匀加速+减加速） */
        uint32_t decel_total;     /**< 减速阶段总步数（包含加减速+匀减速+减减速） */
        uint32_t const_total;     /**< 匀速阶段总步数 */
        uint32_t jerk_inc;        /**< 加加速段步数 - 速度增加，加速度从0到最大值 */
        uint32_t accel_const;     /**< 匀加速段步数 - 速度继续增加，加速度保持最大值 */
        uint32_t jerk_dec;        /**< 减加速段步数 - 速度继续增加，加速度从最大值到0 */
        uint32_t jerk_dec_2;      /**< 加减速段步数 - 速度开始减小，减速度从0到最大值 */
        uint32_t decel_const;     /**< 匀减速段步数 - 速度继续减小，减速度保持最大值 */
        uint32_t jerk_inc_2;      /**< 减减速段步数 - 速度继续减小，减速度从最大值到0 */
    } steps;
    
    /* 阶段内步数计数器 - 用于状态切换判断 */
    uint32_t phase_step_cnt;      /**< 当前阶段已执行的步数 */
    uint32_t phase_step_target;    /**< 当前阶段的目标步数（到达后切换状态） */
    
    /* 总步数管理 */
    uint32_t total_steps;         /**< 本次运动的总目标步数 */
    uint32_t steps_done;           /**< 已完成的步数计数 */
    uint8_t  dir;                  /**< 运动方向：0=反向，1=正向 */
    
    bool     running;              /**< 运动运行标志：true=正在运动，false=已停止 */
} SCurve_Handle_t;

/*============================================================================*
 *                          API 函数声明
 *============================================================================*/

/**
 * @brief       初始化S曲线句柄
 * @details     根据给定的配置参数初始化S曲线控制器，包括：
 *              - 保存配置参数
 *              - 预计算加速阶段各子段的步数
 *              - 预计算减速阶段各子段的步数
 *              - 初始化状态为IDLE
 * 
 * @param[in]   h   S曲线句柄指针（需预先分配内存）
 * @param[in]   cfg S曲线配置参数指针
 * @return      无
 * 
 * @note        此函数应在电机启动前调用一次
 */
void SCurve_Init(SCurve_Handle_t *h, SCurveConfig_t *cfg);

/**
 * @brief       启动S曲线运动
 * @details     根据指定的总步数和方向启动S曲线运动
 *              - 计算匀速段步数（总步数 - 加速步数 - 减速步数）
 *              - 如果行程不够，将按比例调整各段步数
 *              - 初始化状态机，进入加加速段
 * 
 * @param[in]   h           S曲线句柄指针
 * @param[in]   total_steps 总运动步数
 * @param[in]   dir         运动方向：0=反向，1=正向
 * @return      无
 * 
 * @note        确保在调用此函数前已完成初始化
 */
void SCurve_Start(SCurve_Handle_t *h, uint32_t total_steps, uint8_t dir);

/**
 * @brief       紧急停止
 * @details     立即以最大减速度减速停车
 *              - 跳过匀速段
 *              - 直接进入匀减速段
 *              - 用于安全保护或急停按钮
 * 
 * @param[in]   h   S曲线句柄指针
 * @return      无
 */
void SCurve_EmergencyStop(SCurve_Handle_t *h);

/**
 * @brief       更新S曲线状态
 * @details     每个定时器周期调用一次，根据当前状态计算：
 *              - 当前加速度（基于jerk和状态）
 *              - 当前速度（加速度积分）
 *              - 状态切换判断
 * 
 * @param[in]   h   S曲线句柄指针
 * @param[in]   dt  时间片长度，单位：秒
 *                  例如：50微秒 = 0.00005f
 * @return      当前计算得到的速度值，单位：step/s
 * 
 * @note        通常在定时器中断中调用，dt应与中断周期一致
 */
float SCurve_Update(SCurve_Handle_t *h, float dt);

/**
 * @brief       计算减速所需步数
 * @details     根据当前速度和目标速度，计算以S曲线方式减速所需的步数
 *              用于匀速段的前瞻判断，提前开始减速
 * 
 * @param[in]   h       S曲线句柄指针
 * @param[in]   v_from  起始速度，单位：step/s
 * @param[in]   v_to    目标速度，单位：step/s
 * @return      减速所需的步数
 * 
 * @note        内部自动处理能否达到最大减速度两种情况
 */
uint32_t SCurve_CalcDecelSteps(SCurve_Handle_t *h, float v_from, float v_to);
    
/**
 * @brief       处理脉冲输出
 * @details     综合S曲线计算和脉冲生成逻辑：
 *              - 调用SCurve_Update更新运动状态
 *              - 累加位置积分
 *              - 判断是否需要输出脉冲
 *              - 更新步数计数器和状态
 * 
 * @param[in]   h           S曲线句柄指针
 * @param[in]   dt          时间片长度，单位：秒
 * @param[out]  need_pulse  输出参数，是否需要输出一个脉冲
 * @return      true=运动仍在进行，false=运动已完成或未运行
 * 
 * @note        通常在定时器中断中调用
 */
bool SCurve_ProcessPulse(SCurve_Handle_t *h, float dt, bool *need_pulse);

/*============================================================================*
 *                          宏定义
 *============================================================================*/

/* 预留用于调试的宏定义 */
/* #define S_CURVE_DEBUG_ENABLED */

#endif /* __S_CURVE_H */