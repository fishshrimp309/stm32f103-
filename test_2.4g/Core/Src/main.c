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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SPI_tx.h"

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

uint8_t Rx_Byte;
uint8_t UART_Rx_Buffer[8];
uint8_t UART_Rx_Counter = 0;
volatile uint8_t Transmit_Flag = 0;
volatile uint32_t last_byte_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  NF04_TX_Init();

  // ==================== 对频连接循环 ====================
  // 发送端: 持续发送配对请求包, 等待接收端 ACK 载荷回复
  {
      uint8_t ping_buf[8] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      int i;

      while (1)
      {
          /* 发送配对包: 内部切到 TX 模式 → 发送 → 等待 ACK → 切回 RX */
          NF04_Transmit_Packet(ping_buf);

          /* 判断是否收到接收端的 ACK 载荷回复 (0xBB 开头) */
          if (ACK_Rx_Buffer[0] == 0xBB)
          {
              /* 连接成功: LED 快闪 3 次 */
              for (i = 0; i < 3; i++)
              {
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                  HAL_Delay(300);
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                  HAL_Delay(300);
              }
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
              break;
          }

          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
          HAL_Delay(150);
      }
  }
  // ======================================================

  /* 连接成功后启动 UART 接收中断, 准备接收上位机按键数据 */
  HAL_UART_Receive_IT(&huart1, &Rx_Byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t rx_buf[8];

      /* UART 收到 8 字节按键数据 → 通过 NRF 发送 */
      if (Transmit_Flag == 1)
      {
          Transmit_Flag = 0;
          NF04_Transmit_Packet(UART_Rx_Buffer);
      }

      /* 轮询接收 ACK 载荷中的遥测数据 */
      if (NF04_Poll_Receive(rx_buf) == 0)
      {
          if (rx_buf[0] == 0xBB)
          {
              HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
              for (int i = 0; i < 8; i++) ACK_Rx_Buffer[i] = rx_buf[i];
              HAL_UART_Transmit(&huart1, rx_buf, 8, 10);
          }
      }

    /* USER CODE END WHILE */
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 帧同步: 若距上一字节超过 5ms, 认为上一帧已断, 复位从头开始 */
        uint32_t now = HAL_GetTick();
        if ((UART_Rx_Counter > 0) && ((now - last_byte_tick) > 5))
        {
            UART_Rx_Counter = 0;
        }
        last_byte_tick = now;

        UART_Rx_Buffer[UART_Rx_Counter++] = Rx_Byte;

        if (UART_Rx_Counter >= 8)
        {
            UART_Rx_Counter = 0;
            Transmit_Flag = 1;
        }

        /* 重新启用下一次接收中断 */
        HAL_UART_Receive_IT(&huart1, &Rx_Byte, 1);
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
