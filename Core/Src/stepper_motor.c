/**
 *******************************************************************************
 * @file    stepper_motor.c
 * @brief   步进电机驱动实现
 * 
 * @details 
 * 
 * @version V1.0
 * @author  Twinkle-sky
 * @date    2026-05-12
 *******************************************************************************
 */

#include "stepper_motor.h"
/*============================================================================*
 *                          全局变量
 *============================================================================*/

/**
 * @brief 全局S曲线运动句柄指针
 * @details 保存初始化时传入的运动句柄，供中断服务程序使用
 */
static SCurve_Handle_t *g_motion;

/**
 * @brief GPIO操作接口指针
 * @details 保存初始化时传入的GPIO操作接口，供中断服务程序使用
 */
static const Stepper_GPIO_Interface_t *g_gpio_if;

/**
 * @brief 系统tick计数器
 * @details 用于记录系统运行时间，可用于调试和性能分析
 */
static volatile uint32_t g_tick_count = 0;

/*============================================================================*
 *                          宏定义
 *============================================================================*/

/** @brief 定时器中断周期（微秒）
 *  @note 100us周期对应10kHz中断频率
 */
#define TIM_PERIOD_US  100

/** @brief 时间片长度（秒）
 *  @note DT_SEC = 100 / 1000000 = 0.0001秒 = 100微秒
 *  @note S曲线算法中使用此值进行积分计算
 */
#define DT_SEC         (TIM_PERIOD_US / 1000000.0f)

/*============================================================================*
 *                          函数实现
 *============================================================================*/

/**
 * @brief       初始化步进电机驱动
 * @details     保存S曲线句柄指针
 * 
 * @param[in]   h   S曲线句柄指针
 * @return      无
 */
void Stepper_Init(SCurve_Handle_t *h, const Stepper_GPIO_Interface_t *gpio_if) 
{
    /* 保存运动句柄 */
    g_motion = h;
    
    /* 保存GPIO操作接口 */
    g_gpio_if = gpio_if;
}

/**
 * @brief       启动步进电机运动
 * @details     根据给定的步数和方向启动电机运动：
 *              1. 如果电机正在运动，先停止
 *              2. 设置方向引脚
 *              3. 使能电机驱动器
 *              4. 启动S曲线运动
 * 
 * @param[in]   steps  目标运动步数
 * @param[in]   dir    运动方向：0=反向，1=正向
 * @return      无
 */
void Stepper_Move(uint32_t steps, uint8_t dir) 
{
    /* 如果电机正在运行，先停止 */
    if (g_motion->running) {
        Stepper_Stop();
    }
    
    /* 检查GPIO接口是否有效 */
    if (!g_gpio_if) return;
    
    /* 设置方向引脚 */
    g_gpio_if->write_dir(dir ? 1 : 0);
    
    /* 使能电机驱动器（EN引脚拉低） */
    g_gpio_if->write_en(0);
    
    /* 启动S曲线运动 */
    SCurve_Start(g_motion, steps, dir);
}

/**
 * @brief       停止步进电机
 * @details     立即以最大减速度停止电机
 *              内部调用S曲线的紧急停止功能
 * 
 * @param[in]   无
 * @return      无
 */
void Stepper_Stop(void) 
{
    /* 调用S曲线的紧急停止 */
    SCurve_EmergencyStop(g_motion);
}

/**
 * @brief       步进电机定时器中断处理
 * @details     在HAL_TIM_PeriodElapsedCallback中调用
 *              处理S曲线并生成STEP脉冲
 * 
 * @param[in]   无
 * @return      无
 */
void Stepper_TIM_PeriodElapsed(void)
{
    /* 如果未初始化或未运行，直接返回 */
    if (!g_motion || !g_motion->running) return;
    
    /* 检查GPIO接口是否有效 */
    if (!g_gpio_if) return;
    
    /* 定义脉冲输出标志 */
    bool need_pulse;
    
    /* 调用S曲线处理函数，获取脉冲需求和运行状态 */
    bool still_running = SCurve_ProcessPulse(g_motion, DT_SEC, &need_pulse);
    
    /**
     * 生成STEP脉冲
     * 使用翻转方式：每次need_pulse为true时翻转一次
     * 两次翻转形成一个完整的脉冲（高电平+低电平）
     */
    if (need_pulse) 
    {
        /* 读取当前STEP引脚状态并翻转 */
        Stepper_PinState_t current_state = g_gpio_if->read_step();
        g_gpio_if->write_step(current_state ? 0 : 1);
    }
    
    /**
     * 运动结束时禁用电机
     * 当still_running为false时，表示运动已完成
     */
    if (!still_running) 
    {
        /* 禁用电机驱动器 */
        g_gpio_if->write_en(1);
    }
}