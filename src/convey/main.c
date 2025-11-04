/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2025 STMicroelectronics.

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

#include <stdio.h>

#include <string.h>

#include <stdarg.h>



/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */



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

TIM_HandleTypeDef htim2;



UART_HandleTypeDef huart2;



/* USER CODE BEGIN PV */



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

void UART_Print(const char *fmt, ...);

/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */



uint8_t rx;                      // UART 수신용 변수

int32_t enc_start[26];          // 알파벳별 시작 엔코더 값 ('A' ~ 'Z')

int32_t enc_target[26];         // 알파벳별 목표 엔코더 값 (시작 + 13900)

uint8_t active[26];             // 해당 알파벳 동작 진행 여부 (1: 동작 중)

int move_flag = 0;



#define ENC_MODULO 65536          // 엔코더 범위 (16비트 타이머 기준)

#define MOVE_DISTANCE 7500   // 이동할 거리 (카운트 수)



int32_t prev_enc = 0;

int32_t curr_enc = 0;

uint32_t last_check_time = 0;



#define ENC_THRESHOLD   5       // 움직임으로 판단할 최소 차이

#define CHECK_INTERVAL  100     // 100ms 간격 체크



int conveyor_running = 0;       // 0: 정지, 1: 동작



volatile uint8_t pause_flag = 0;



/* USER CODE END 0 */



/*

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

  MX_USART2_UART_Init();

  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */



  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

   __HAL_TIM_SET_COUNTER(&htim2, 0);



  /* USER CODE END 2 */



  /* Infinite loop */

  /* USER CODE BEGIN WHILE */



   while (1)

   {
       // 1. 알파벳 수신 (폴링 방식)
       if (HAL_UART_Receive(&huart2, &rx, 1, 1) == HAL_OK)
       {
           if (rx >= 'A' && rx <= 'Z')
           {
               uint8_t idx = rx - 'A';
               int32_t current = __HAL_TIM_GET_COUNTER(&htim2);
               int32_t enc_now = (ENC_MODULO - current) % ENC_MODULO;

               // 🔁 무조건 최신 값으로 갱신

               enc_start[idx]  = enc_now;
               enc_target[idx] = (enc_now + MOVE_DISTANCE) % ENC_MODULO;
               active[idx]     = 1;
           }
       }
       // 2. 각 알파벳별 거리 비교 및 정지 처리
       int32_t current = __HAL_TIM_GET_COUNTER(&htim2);
       int32_t enc_now = (ENC_MODULO - current) % ENC_MODULO;
       for (uint8_t i = 0; i < 26; i++)
       {
           if (active[i])
           {
               int32_t diff = (enc_now - enc_start[i] + ENC_MODULO) % ENC_MODULO;
               if (diff >= MOVE_DISTANCE - 5 && diff <= MOVE_DISTANCE + 5)
               {
                   active[i] = 0;
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // 정지
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  // F429 알림
                   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);

                   while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
                   {
                       HAL_Delay(10);
                   }
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
               }
           }
       }

       if (HAL_GetTick() - last_check_time > CHECK_INTERVAL)
       {
           last_check_time = HAL_GetTick();
           curr_enc = __HAL_TIM_GET_COUNTER(&htim2);

           // 차이 계산 (오버플로우 보정 포함)
           int32_t diff = (curr_enc - prev_enc + ENC_MODULO) % ENC_MODULO;
           if (diff > ENC_THRESHOLD)
           {
               conveyor_running = 1; // 움직임 있음 → 동작 중
           }

           else
           {
               conveyor_running = 0; // 거의 변화 없음 → 정지 상태
           }
           prev_enc = curr_enc;
       }
       if (conveyor_running)
       {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
       }
       else
       {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
       }
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;

  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;

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



/**

  * @brief TIM2 Initialization Function

  * @param None

  * @retval None

  */

static void MX_TIM2_Init(void)

{



  /* USER CODE BEGIN TIM2_Init 0 */



  /* USER CODE END TIM2_Init 0 */



  TIM_Encoder_InitTypeDef sConfig = {0};

  TIM_MasterConfigTypeDef sMasterConfig = {0};



  /* USER CODE BEGIN TIM2_Init 1 */



  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;

  htim2.Init.Prescaler = 0;

  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  htim2.Init.Period = 65535;

  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;

  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;

  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;

  sConfig.IC1Filter = 0;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;

  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;

  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;

  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)

  {

    Error_Handler();

  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN TIM2_Init 2 */



  /* USER CODE END TIM2_Init 2 */



}



/**

  * @brief USART2 Initialization Function

  * @param None

  * @retval None

  */

static void MX_USART2_UART_Init(void)

{



  /* USER CODE BEGIN USART2_Init 0 */



  /* USER CODE END USART2_Init 0 */



  /* USER CODE BEGIN USART2_Init 1 */



  /* USER CODE END USART2_Init 1 */

  huart2.Instance = USART2;

  huart2.Init.BaudRate = 9600;

  huart2.Init.WordLength = UART_WORDLENGTH_8B;

  huart2.Init.StopBits = UART_STOPBITS_1;

  huart2.Init.Parity = UART_PARITY_NONE;

  huart2.Init.Mode = UART_MODE_TX_RX;

  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)

  {

    Error_Handler();

  }

  /* USER CODE BEGIN USART2_Init 2 */





  /* USER CODE END USART2_Init 2 */



}



/**

  * @brief GPIO Initialization Function

  * @param None

  * @retval None

  */

static void MX_GPIO_Init(void)

{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */



  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();



  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);



  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);



  /*Configure GPIO pin : B1_Pin */

  GPIO_InitStruct.Pin = B1_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);



  /*Configure GPIO pin : PC0 */

  GPIO_InitStruct.Pin = GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



  /*Configure GPIO pins : LD2_Pin PA6 PA7 */

  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  /*Configure GPIO pins : PB8 */

  GPIO_InitStruct.Pin = GPIO_PIN_8;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /*Configure GPIO pin : PB0 */

  GPIO_InitStruct.Pin = GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /*Configure GPIO pin : PB6 */

  GPIO_InitStruct.Pin = GPIO_PIN_6;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /*Configure GPIO pin : PB9 */

  GPIO_InitStruct.Pin = GPIO_PIN_9;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;

  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* EXTI interrupt init*/

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);



  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);

  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);



/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */

}



/* USER CODE BEGIN 4 */

void UART_Print(const char *fmt, ...)

 {

    char uart_buf[100];

     va_list args;

     va_start(args, fmt);

     vsnprintf(uart_buf, sizeof(uart_buf), fmt, args);

     va_end(args);

     HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

 }



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{

    if (GPIO_Pin == GPIO_PIN_9)  // MOVE 버튼 (재시작)

    {

       if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == SET)

          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // 동작 출력

       else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == RESET)

          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

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

  /* User can add his own implementation to report the file name and line number,

     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */

}

#endif /* USE_FULL_ASSERT */
