#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stdint.h"

/*============================================================================
 * 宏定义
 *============================================================================*/
#define MOTOR_PULSE_HIGH_US         2U      /* 脉冲高电平持续时间(us) */
#define MOTOR_MIN_PERIOD_US         10U    /* 最小脉冲周期(us)，对应最大速度10000步/秒 */
#define MOTOR_DEFAULT_MAX_SPEED     4000U   /* 默认最大速度(步/秒) */
#define MOTOR_DEFAULT_MIN_SPEED     400U    /* 默认最小速度(步/秒) */
#define MOTOR_DEFAULT_ACCEL         800U    /* 默认加速度(步/秒^2) */

/* 通用电平定义 */
#define MOTOR_PIN_RESET              0U     /* 低电平 */
#define MOTOR_PIN_SET                1U     /* 高电平 */

/* 电平定义 */
#define MOTOR_ENABLE_LEVEL           MOTOR_PIN_RESET  /* 使能电平(低电平使能) */
#define MOTOR_DISABLE_LEVEL          MOTOR_PIN_SET    /* 失能电平 */
#define MOTOR_CW_LEVEL               MOTOR_PIN_SET    /* 顺时针方向电平 */
#define MOTOR_CCW_LEVEL              MOTOR_PIN_RESET  /* 逆时针方向电平 */
#define MOTOR_STEP_HIGH_LEVEL        MOTOR_PIN_SET    /* STEP引脚高电平 */
#define MOTOR_STEP_LOW_LEVEL         MOTOR_PIN_RESET  /* STEP引脚低电平 */

/*============================================================================
 * 硬件抽象层接口
 *============================================================================*/

typedef void (*Motor_GpioWriteFunc)(void *port, uint16_t pin, uint8_t state);
typedef void (*Motor_DelayUsFunc)(uint32_t us);

typedef struct {
    Motor_GpioWriteFunc gpio_write;    /* GPIO写入函数指针 */
    Motor_DelayUsFunc delay_us;         /* 微秒延时函数指针 */
} Motor_HalTypeDef;

/*============================================================================
 * 枚举定义
 *============================================================================*/

typedef enum {
    MOTOR_DIR_CW = 1,       /* 顺时针(正转) */
    MOTOR_DIR_CCW = -1      /* 逆时针(反转) */
} Motor_Direction_t;

typedef enum {
    MOTOR_STATE_UNINIT = 0, /* 未初始化状态 */
    MOTOR_STATE_IDLE,       /* 空闲状态 */
    MOTOR_STATE_RUNNING,    /* 运行状态 */
    MOTOR_STATE_STOPPING    /* 停止状态 */
} Motor_State_t;

typedef enum {
    MOTOR_OK = 0,               /* 操作成功 */
    MOTOR_ERROR_NULL_PTR,       /* 空指针错误 */
    MOTOR_ERROR_INVALID_PARAM,  /* 无效参数 */
    MOTOR_ERROR_NOT_INIT,       /* 未初始化 */
    MOTOR_ERROR_BUSY,           /* 设备忙 */
    MOTOR_ERROR_HAL             /* 硬件抽象层错误 */
} Motor_Error_t;

/*============================================================================
 * 结构体定义
 *============================================================================*/

typedef struct {
    /* GPIO配置 */
    void *step_port;            /* STEP引脚端口 */
    uint16_t step_pin;          /* STEP引脚编号 */
    void *dir_port;             /* DIR引脚端口 */
    uint16_t dir_pin;           /* DIR引脚编号 */
    void *en_port;              /* EN引脚端口 */
    uint16_t en_pin;            /* EN引脚编号 */
    
    /* 运动参数 */
    uint32_t max_speed;         /* 最大速度(步/秒) */
    uint32_t min_speed;         /* 最小速度(步/秒) */
    uint32_t accel;             /* 加速度(步/秒^2) */
} Motor_Config_t;

typedef struct {
    /* 配置参数 */
    uint32_t max_speed;         /* 最大速度(步/秒) */
    uint32_t min_speed;         /* 最小速度(步/秒) */
    uint32_t accel;             /* 加速度(步/秒^2) */
    
    /* GPIO配置 */
    void *step_port;            /* STEP引脚端口 */
    uint16_t step_pin;          /* STEP引脚编号 */
    void *dir_port;             /* DIR引脚端口 */
    uint16_t dir_pin;           /* DIR引脚编号 */
    void *en_port;              /* EN引脚端口 */
    uint16_t en_pin;            /* EN引脚编号 */
    
    /* 运动状态 */
    uint32_t total_steps;       /* 总步数 */
    uint32_t current_step;      /* 当前步数 */

    uint32_t accel_steps;       /* 加速步数 */
    uint32_t decel_start;       /* 减速起始步数 */

    float c0;                  /* 速度系数0 */
    float cn;                  /* 速度系数n */
    int32_t n;                 /* 速度系数n */

    uint32_t min_interval;      /* 最小间隔时间(微秒) */
    uint32_t current_speed;     /* 当前速度(步/秒) */
    
    /* 控制标志 */
    Motor_Direction_t direction; /* 运动方向 */
    Motor_State_t state;         /* 电机状态 */
    
} Motor_HandleTypeDef;

/*============================================================================
 * 函数声明
 *============================================================================*/

/* 硬件抽象层配置 */
Motor_Error_t Motor_SetHalInterface(Motor_HalTypeDef *hal);

/* 初始化 */
Motor_Error_t Motor_Init(Motor_HandleTypeDef *motor, Motor_Config_t *config);
Motor_Error_t Motor_DeInit(Motor_HandleTypeDef *motor);

/* 运动控制 */
Motor_Error_t Motor_Start(Motor_HandleTypeDef *motor, uint32_t steps, Motor_Direction_t direction);
Motor_Error_t Motor_Stop(Motor_HandleTypeDef *motor);
Motor_Error_t Motor_EmergencyStop(Motor_HandleTypeDef *motor);

/* 状态查询 */
Motor_State_t Motor_GetState(Motor_HandleTypeDef *motor);
uint32_t Motor_GetCurrentSpeed(Motor_HandleTypeDef *motor);
uint32_t Motor_GetCurrentStep(Motor_HandleTypeDef *motor);

#endif /* __MOTOR_CONTROL_H */