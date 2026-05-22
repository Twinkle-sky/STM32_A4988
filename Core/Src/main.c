/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "s_curve.h"       /* S曲线算法头文件 */
#include "stepper_motor.h" /* 步进电机驱动头文件 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SCurve_Handle_t motion;

/* GPIO引脚定义 - 步进电机控制 */
#define STEP_PIN    GPIO_PIN_5
#define DIR_PIN     GPIO_PIN_4
#define EN_PIN      GPIO_PIN_6
#define STEP_PORT   GPIOA
#define DIR_PORT    GPIOA
#define EN_PORT     GPIOA

/* GPIO操作函数实现 */
static void stepper_write_step(Stepper_PinState_t state) {
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_write_dir(Stepper_PinState_t state) {
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_write_en(Stepper_PinState_t state) {
    HAL_GPIO_WritePin(EN_PORT, EN_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static Stepper_PinState_t stepper_read_step(void) {
    return HAL_GPIO_ReadPin(STEP_PORT, STEP_PIN);
}

/* GPIO操作接口实例 */
const Stepper_GPIO_Interface_t stepper_gpio_if = {
    .write_step = stepper_write_step,
    .write_dir  = stepper_write_dir,
    .write_en   = stepper_write_en,
    .read_step  = stepper_read_step
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    SCurveConfig_t cfg = {
        .v_start  = 200.0f,   /**< 起始速度: 200 step/s (约12 RPM) */
        .v_target = 4000.0f,  /**< 目标速度: 4000 step/s */
        .v_end    = 0.0f,     /**< 结束速度: 0 (完全停止) */
        .v_max    = 4000.0f,  /**< 最大限制速度: 4000 step/s */
        .accl     = 1000.0f,  /**< 最大加速度: 1000 step/s^2 */
        .decel    = 1000.0f,  /**< 最大减速度: 1000 step/s^2 */
        .jerk     = 8000.0f   /**< 加加速度: 8000 step/s^3 */
    };

    /* 初始化S曲线控制器 */
    SCurve_Init(&motion, &cfg);

    /* 初始化步进电机驱动 */
    Stepper_Init(&motion, &stepper_gpio_if);

    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        Stepper_Move(6400, 1);
        while (motion.running);
        HAL_Delay(1000);

        Stepper_Move(6400, 0);
        while (motion.running);
        HAL_Delay(1000);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  TIM周期中断回调函数
 * @param  htim: TIM句柄
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Stepper_TIM_PeriodElapsed();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
