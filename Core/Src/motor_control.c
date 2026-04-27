/*============================================================================
 * 文件名称: motor_control.c
 * 功能描述: 步进电机梯形加减速控制模块
 * 作者: Twinkle-sky
 * 版本: V1.0
 * 日期: 2026-03-24
 * 修改: 
 *============================================================================*/

#include "motor_control.h"
#include "stm32f1xx_hal.h"
#include "math.h"

/*============================================================================
 * 静态函数声明
 *============================================================================*/

static void Motor_DefaultGpioWrite(void *port, uint16_t pin, uint8_t state);
static void Motor_DefaultDelayUs(uint32_t us);
static uint32_t Motor_SpeedToPeriod(uint32_t speed);
static uint32_t Motor_PeriodToSpeed(uint32_t period_us);
static void Motor_StepOutput(Motor_HandleTypeDef *motor, uint32_t period_us);
static uint32_t Motor_TrapezoidalControl(Motor_HandleTypeDef *motor);

/*============================================================================
 * 全局变量
 *============================================================================*/

static Motor_HalTypeDef g_motor_hal = {
    .gpio_write = Motor_DefaultGpioWrite,
    .delay_us = Motor_DefaultDelayUs
};

/*============================================================================
 * 函数实现
 *============================================================================*/

 /**
 * @brief 默认GPIO写入函数
 * @param port GPIO端口
 * @param pin GPIO引脚
 * @param state 电平状态
 */
static void Motor_DefaultGpioWrite(void *port, uint16_t pin, uint8_t state)
{
    if (port == NULL)
    {
        return;
    }
    
    GPIO_PinState pin_state = (state == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, pin_state);
}

/**
 * @brief 默认微秒级延时函数
 * @param us 延时微秒数
 * @note 根据系统时钟调整循环系数，用示波器观察，简单定了一个
 */
static void Motor_DefaultDelayUs(uint32_t us)
{
    const uint32_t loop_coefficient = 2;
    
    for (uint32_t i = 0; i < us * loop_coefficient; i++)
    {
        __NOP();
    }
}

/**
 * @brief 设置硬件抽象层接口
 * @param hal 硬件抽象层接口指针
 * @return 错误代码
 * @note 移植的时候调用一次这个函数
 */
Motor_Error_t Motor_SetHalInterface(Motor_HalTypeDef *hal)
{
    if (hal == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    if (hal->gpio_write == NULL || hal->delay_us == NULL)
    {
        return MOTOR_ERROR_INVALID_PARAM;
    }
    
    g_motor_hal.gpio_write = hal->gpio_write;
    g_motor_hal.delay_us = hal->delay_us;
    
    return MOTOR_OK;
}

/**
 * @brief 速度转换为脉冲周期
 * @param speed 速度(步/秒)
 * @return 脉冲周期(微秒)
 */
static uint32_t Motor_SpeedToPeriod(uint32_t speed)
{
    uint32_t period;
    
    if (speed == 0)
    {
        return 0xFFFFFFFFU;
    }
    
    period = 1000000U / speed;
    
    /* 限制最小周期，防止脉冲频率过高 */
    if (period < MOTOR_MIN_PERIOD_US)
    {
        period = MOTOR_MIN_PERIOD_US;
    }
    
    return period;
}

/**
 * @brief 脉冲周期转换为速度
 * @param period_us 脉冲周期(微秒)
 * @return 速度(步/秒)
 */
static uint32_t Motor_PeriodToSpeed(uint32_t period_us)
{
    if (period_us == 0)
    {
        return 0;
    }
    
    return 1000000U / period_us;
}

/**
 * @brief 输出单个脉冲并更新步数
 * @param motor 电机句柄指针
 * @param period_us 脉冲周期(微秒)
 */
static void Motor_StepOutput(Motor_HandleTypeDef *motor, uint32_t period_us)
{
    if (period_us < MOTOR_PULSE_HIGH_US + 1)
    {
        period_us = MOTOR_PULSE_HIGH_US + 1;
    }
    
    if (motor->step_port != NULL)
    {
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_HIGH_LEVEL);
        g_motor_hal.delay_us(MOTOR_PULSE_HIGH_US);
        
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_LOW_LEVEL);
    }
    
    motor->current_step++;
    
    if (period_us > MOTOR_PULSE_HIGH_US)
    {
        g_motor_hal.delay_us(period_us - MOTOR_PULSE_HIGH_US);
    }
}

/**
 * @brief 梯形加减速控制算法
 * @param motor 电机句柄指针
 */
static uint32_t Motor_TrapezoidalControl(Motor_HandleTypeDef *motor)
{
    /* 加速段 */
    if (motor->current_step < motor->accel_steps)
    {
        motor->n++;

        motor->cn = motor->cn -
            (2.0f * motor->cn) / (4.0f * motor->n + 1.0f);

        if (motor->cn < motor->min_interval)
        {
            motor->cn = motor->min_interval;
        }
    }
    /* 减速段 */
    else if (motor->current_step >= motor->decel_start)
    {
        if (motor->n > 1)
        {
            motor->n--;

            motor->cn = motor->cn +
                (2.0f * motor->cn) / (4.0f * motor->n + 1.0f);
        }
    }
    /* 匀速段 */
    else
    {
        motor->cn = motor->min_interval;
    }

    motor->current_speed =
        Motor_PeriodToSpeed((uint32_t)motor->cn);

    return (uint32_t)motor->cn;
}

/**
 * @brief 电机初始化
 * @param motor 电机句柄指针
 * @param config 配置参数指针
 * @return 错误代码
 */
Motor_Error_t Motor_Init(Motor_HandleTypeDef *motor, Motor_Config_t *config)
{
    if (motor == NULL || config == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    motor->max_speed = (config->max_speed > 0) ? config->max_speed : MOTOR_DEFAULT_MAX_SPEED;
    motor->min_speed = (config->min_speed > 0) ? config->min_speed : MOTOR_DEFAULT_MIN_SPEED;
    motor->accel = (config->accel > 0) ? config->accel : MOTOR_DEFAULT_ACCEL;

    motor->step_port = config->step_port;
    motor->step_pin = config->step_pin;
    motor->dir_port = config->dir_port;
    motor->dir_pin = config->dir_pin;
    motor->en_port = config->en_port;
    motor->en_pin = config->en_pin;
    
    if (motor->max_speed < motor->min_speed)
    {
        motor->max_speed = motor->min_speed;
    }
    
    
    motor->total_steps = 0;
    motor->current_step = 0;
    motor->decel_start = 0;
    motor->current_speed = 0;
    
    motor->direction = MOTOR_DIR_CW;
    motor->state = MOTOR_STATE_IDLE;
    
    if (motor->en_port != NULL)
    {
        g_motor_hal.gpio_write(motor->en_port, motor->en_pin, MOTOR_ENABLE_LEVEL);
    }
    
    if (motor->step_port != NULL)
    {
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_LOW_LEVEL);
    }
    
    return MOTOR_OK;
}

/**
 * @brief 电机反初始化
 * @param motor 电机句柄指针
 * @return 错误代码
 */
Motor_Error_t Motor_DeInit(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    motor->state = MOTOR_STATE_UNINIT;

    if (motor->en_port != NULL)
    {
        g_motor_hal.gpio_write(motor->en_port, motor->en_pin, MOTOR_DISABLE_LEVEL);
    }

    if (motor->step_port != NULL)
    {
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_LOW_LEVEL);
    }
    
    return MOTOR_OK;
}

/**
 * @brief 启动电机运动
 * @param motor 电机句柄指针
 * @param steps 总步数
 * @param direction 运动方向
 * @return 错误代码
 */
Motor_Error_t Motor_Start(Motor_HandleTypeDef *motor, uint32_t steps, Motor_Direction_t direction)
{
    if (motor == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    if (steps == 0)
    {
        return MOTOR_ERROR_INVALID_PARAM;
    }
    
    if (motor->state != MOTOR_STATE_IDLE)
    {
        return MOTOR_ERROR_BUSY;
    }
    
    if (motor->dir_port != NULL)
    {
        if (direction == MOTOR_DIR_CW)
        {
            g_motor_hal.gpio_write(motor->dir_port, motor->dir_pin, MOTOR_CW_LEVEL);
        }
        else
        {
            g_motor_hal.gpio_write(motor->dir_port, motor->dir_pin, MOTOR_CCW_LEVEL);
        }
    }
    
    motor->total_steps   = steps;
    motor->current_step  = 0;
    motor->n             = 1;
    motor->state         = MOTOR_STATE_RUNNING;

    /* 根据真实加速度计算理论加速步数 */
    motor->accel_steps =
        (motor->max_speed * motor->max_speed -
        motor->min_speed * motor->min_speed)
        / (2 * motor->accel);

    /* 短距离自动压缩梯形 */
    if (motor->accel_steps * 2 > steps)
    {
        motor->accel_steps = steps / 2;
    }

    motor->decel_start = steps - motor->accel_steps;

    /* 初始脉冲间隔 c0 */
    motor->c0 = 0.676f * sqrtf(2.0f / motor->accel) * 1000000.0f ;

    /* 从 min_speed 起步则限制初值 */
    uint32_t min_speed_period = Motor_SpeedToPeriod(motor->min_speed);

    if (motor->c0 > min_speed_period)
    {
        motor->c0 = min_speed_period;
    }

    motor->cn = motor->c0;

    /* 最大速度对应最小周期 */
    motor->min_interval = Motor_SpeedToPeriod(motor->max_speed);

    // motor->current_speed = Motor_PeriodToSpeed((uint32_t)motor->cn);
    

    while ((motor->current_step < steps) && (motor->state == MOTOR_STATE_RUNNING))
    {
        Motor_TrapezoidalControl(motor);
        
        // uint32_t period = Motor_SpeedToPeriod(motor->current_speed);
        Motor_StepOutput(motor, motor->cn);
    }

    motor->state = MOTOR_STATE_IDLE;
    
    return MOTOR_OK;
}

/**
 * @brief 停止电机运动
 * @param motor 电机句柄指针
 * @return 错误代码
 */
Motor_Error_t Motor_Stop(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    if (motor->state != MOTOR_STATE_RUNNING)
    {
        return MOTOR_ERROR_NOT_INIT;
    }
    
    motor->state = MOTOR_STATE_STOPPING;
    
    if (motor->step_port != NULL)
    {
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_LOW_LEVEL);
    }
    
    motor->state = MOTOR_STATE_IDLE;
    
    return MOTOR_OK;
}

/**
 * @brief 紧急停止(立即停止并失能)
 * @param motor 电机句柄指针
 * @return 错误代码
 */
Motor_Error_t Motor_EmergencyStop(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return MOTOR_ERROR_NULL_PTR;
    }
    
    motor->state = MOTOR_STATE_UNINIT;

    if (motor->step_port != NULL)
    {
        g_motor_hal.gpio_write(motor->step_port, motor->step_pin, MOTOR_STEP_LOW_LEVEL);
    }
    
    if (motor->en_port != NULL)
    {
        g_motor_hal.gpio_write(motor->en_port, motor->en_pin, MOTOR_DISABLE_LEVEL);
    }
    
    return MOTOR_OK;
}

/**
 * @brief 获取电机状态
 * @param motor 电机句柄指针
 * @return 电机状态
 */
Motor_State_t Motor_GetState(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return MOTOR_STATE_UNINIT;
    }
    
    return motor->state;
}

/**
 * @brief 获取当前速度
 * @param motor 电机句柄指针
 * @return 当前速度(步/秒)
 */
uint32_t Motor_GetCurrentSpeed(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return 0;
    }
    
    return motor->current_speed;
}

/**
 * @brief 获取当前步数
 * @param motor 电机句柄指针
 * @return 当前步数
 */
uint32_t Motor_GetCurrentStep(Motor_HandleTypeDef *motor)
{
    if (motor == NULL)
    {
        return 0;
    }
    
    return motor->current_step;
}
