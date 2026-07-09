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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include "control.h"
#include "rc.h"
#include "SPI_rc.h"

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

volatile uint8_t i2c_busy = 0;
volatile uint8_t control_flag = 0;
volatile uint32_t last_ACK_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (!i2c_busy) {
            control_flag = 1;
        }
    }
}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_initialize();
  DMP_Init();
  HAL_Delay(2000);

  Motor_Init();
  Control_PID_Init();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(50);

  RC_App_Init();

  // ==================== 对频连接循环 ====================
  // 接收端: NF04_Init 已进入 RX 模式并预装 ACK 载荷
  // 此循环持续重装 ACK 载荷, 等待发送端首次连接成功
  {
      int i;
      RC_Data_Ready_Flag = 0;

      while (1)
      {
          if (RC_Data_Ready_Flag == 1)
          {
              RC_Data_Ready_Flag = 0;

              /* 连接成功: LED 快闪 3 次 */
              for (i = 0; i < 3; i++)
              {
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                  HAL_Delay(300);
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                  HAL_Delay(300);
              }
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
              break;
          }

          /* 重装 ACK 载荷保持配对握手活跃 */
          uint8_t bind_ack[8] = {0xBB, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
          NF04_Write_Ack_Payload(bind_ack);
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          HAL_Delay(150);
      }
  }
  // ======================================================

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (control_flag) {
          control_flag = 0;
          i2c_busy = 1;
          MPU_Check_And_Read();
          Control_Flight_Loop(Target_Roll, Target_Pitch, Target_Yaw, Final_Throttle,
                              Roll, Pitch, Yaw, Gyro_X, Gyro_Y, Gyro_Z);
          i2c_busy = 0;
      }

      if (RC_Data_Ready_Flag == 1)
      {
          RC_Data_Ready_Flag = 0;
          RC_Resolve_Control_Logic();
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }
	  
      if ((HAL_GetTick() - last_ACK_tick) > 50)
      {
          last_ACK_tick = HAL_GetTick();

          uint8_t ack[8];
          ack[0] = 0xBB;
          ack[1] = (int8_t)Roll;
          ack[2] = (int8_t)Pitch;
          ack[3] = (int8_t)Yaw;
          ack[4] = Final_Throttle & 0xFF;
          ack[5] = (Final_Throttle >> 8) & 0xFF;
          ack[6] = 0;
          ack[7] = 0;

          if (NF04_Send_Packet(ack) == TX_DS) {
              HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          }
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
