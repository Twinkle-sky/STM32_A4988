/**
 *******************************************************************************
 * @file    stepper_motor.h
 * @brief   步进电机驱动头文件
 * 
 * @details 
 * 本模块负责步进电机的底层驱动控制，包括：
 * - GPIO引脚配置（STEP脉冲、DIR方向、EN使能）
 * - 定时器配置（用于生成精确的脉冲时序）
 * - 运动指令接口
 * 
 * 硬件连接说明：
 * - STEP_PIN: 脉冲引脚，每个脉冲驱动电机走一步
 * - DIR_PIN: 方向引脚，输出高低电平控制转动方向
 * - EN_PIN: 使能引脚，低电平使能电机，高电平禁用
 * 
 * @version V1.0
 * @author  Twinkle-sky
 * @date    2026-05-12
 *******************************************************************************
 */

#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "s_curve.h"
#include <stdbool.h>

/*============================================================================*
 *                          GPIO操作接口定义
 *============================================================================*/

/**
 * @brief GPIO引脚状态类型
 * @details 用于表示GPIO引脚的高低电平状态
 */
typedef uint8_t Stepper_PinState_t;

/**
 * @brief GPIO操作接口结构体
 * @details 定义步进电机驱动所需的GPIO操作接口，实现与具体硬件平台的解耦
 *          使用函数指针将GPIO操作抽象出来，调用者需要提供具体实现
 */
typedef struct {
    /**
     * @brief 写STEP脉冲引脚
     * @param state 引脚状态：0=低电平，1=高电平
     */
    void (*write_step)(Stepper_PinState_t state);
    
    /**
     * @brief 写DIR方向引脚
     * @param state 引脚状态：0=反向，1=正向
     */
    void (*write_dir)(Stepper_PinState_t state);
    
    /**
     * @brief 写EN使能引脚
     * @param state 引脚状态：0=使能电机，1=禁用电机
     */
    void (*write_en)(Stepper_PinState_t state);
    
    /**
     * @brief 读STEP脉冲引脚状态
     * @return 当前引脚状态
     */
    Stepper_PinState_t (*read_step)(void);
} Stepper_GPIO_Interface_t;

/*============================================================================*
 *                          函数声明
 *============================================================================*/

/**
 * @brief       初始化步进电机驱动
 * @details     配置步进电机驱动：
 *              - 保存S曲线句柄和GPIO操作接口
 *              - 后续定时器配置由调用者负责在外部完成
 * 
 * @param[in]   h           S曲线句柄指针，用于关联运动控制器
 * @param[in]   gpio_if     GPIO操作接口指针，提供平台相关的GPIO操作
 * @return      无
 * 
 * @note        此函数应在系统初始化时调用一次
 * @note        GPIO引脚和定时器需要在外部预先初始化完成
 */
void Stepper_Init(SCurve_Handle_t *h, const Stepper_GPIO_Interface_t *gpio_if);

/**
 * @brief       启动步进电机运动
 * @details     根据给定的步数和方向启动电机运动：
 *              - 如果电机正在运动，先停止当前运动
 *              - 设置方向引脚
 *              - 使能电机驱动器
 *              - 启动S曲线运动
 * 
 * @param[in]   steps  目标运动步数
 * @param[in]   dir    运动方向：0=反向，1=正向
 * @return      无
 */
void Stepper_Move(uint32_t steps, uint8_t dir);

/**
 * @brief       停止步进电机
 * @details     立即以最大减速度停止电机运动
 *              内部调用S曲线的紧急停止功能
 * 
 * @param[in]   无
 * @return      无
 */
void Stepper_Stop(void);

/**
 * @brief       步进电机定时器中断处理
 * @details     在HAL_TIM_PeriodElapsedCallback中调用
 *              处理S曲线并生成STEP脉冲
 * 
 * @param[in]   无
 * @return      无
 */
void Stepper_TIM_PeriodElapsed(void);

#endif /* __STEPPER_MOTOR_H */