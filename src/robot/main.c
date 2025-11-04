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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h>   // atof()
#include <string.h>   // strtok()
#include <math.h>

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

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

#define PI 3.141592   // 원주율 상수
#define MOTOR_MAX 180000  // 모터 최대 스텝 수
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
void wait_motor_done();
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes')
     calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    // huart1은 UART 핸들입니다. (예: UART1)
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 이물 좌표 저장용 변수 (zone 별 최대 3개 좌표, 각 좌표는 x/y/z)
double latest_coords[26][3][3];   // A~Z 구역별 좌표 저장
int coord_counts[26] = {0};       // 각 구역별 좌표 수
uint8_t zone_pending[26] = {0};   // 구역별 대기 여부 // zone 별로 아직 처리되지 않은 플래그
char current_zone = 0;            // 현재 처리 중인 zone 문자 ('A' ~ 'Z')
uint8_t zone_in_progress = 0;     // 로봇 동작 중인지 여부

// RS485 통신 송수신을 위한 함수
void RS485_Transmit(uint8_t *data, uint16_t size);
void RS485_Receive();
void ID_Check(uint8_t ID);
void Relay_ON(uint8_t ID);
void angle_move(uint8_t ID, uint32_t angle);
void status(uint8_t ID);
void inverse_kinematics(double x, double y, double z); // 역기구학 함수
void move_linear_auto(double x1, double y1, double x2, double y2, double z); // 거리 보간 함수
uint8_t buffer[13];  // RS485 수신 버퍼
uint8_t motor[13];   // RS485 송신용 버퍼

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);   // RS485 수신 모드
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);   // 파기 OFF 상태
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char uart_buffer[100]; // 수신된 문자열 저장용
    uint8_t rx_byte;       // 1바이트 수신용
    uint8_t idx = 0;       // 수신 버퍼 인덱스
    double current_x = 0.0, current_y = 0.0;  // 현재 로봇 위치 저장 (X/Y)


    while (1)
    {
    	// UART3 수신 처리 (1바이트씩)
        if (HAL_UART_Receive(&huart3, &rx_byte, 1, 10) == HAL_OK) {
            if (rx_byte == '\n') {               // 줄바꿈 문자가 오면 전체 패킷 수신 완료
                uart_buffer[idx] = '\0';         // 문자열 종료 문자 추가
                if (uart_buffer[1] == ';' && uart_buffer[0] >= 'A' && uart_buffer[0] <= 'Z') {
                    char zone = uart_buffer[0];   // 구역 문자 추출 ('A'~'Z')
                    int zid = zone - 'A';         // 배열 인덱스로 변환
                    int count = 0;                // 좌표 개수
                    char *p = strchr(uart_buffer, ';');   // 세미콜론 다음부터 좌표 시작
                    if (p) {
                        p++;
                        while (count < 3 && *p) {
                            if (*p == 'n') break;   // 'n,n' 예외 처리
                            char *next_sep = strchr(p, '|');
                            if (!next_sep) next_sep = p + strlen(p);
                            char coord_pair[32] = {0};
                            strncpy(coord_pair, p, next_sep - p);
                            char *comma = strchr(coord_pair, ',');
                            if (!comma) break;
                            *comma = '\0';
                            latest_coords[zid][count][0] = atof(coord_pair);  // x
                            latest_coords[zid][count][1] = atof(comma + 1);   // y
                            latest_coords[zid][count][2] = -0.275;            // z 고정값
                            count++;
                            if (*next_sep == '\0') break;
                            p = next_sep + 1;
                        }
                        coord_counts[zid] = count;   // 좌표 수 저장
                        if (!zone_in_progress || current_zone != zone)
                            zone_pending[zid] = 1;   // 현재 작업 중이 아니면 처리 대기 등록
                    }
                }
                idx = 0;  // 수신 버퍼 초기화
            } else if (idx < sizeof(uart_buffer) - 1) {
                uart_buffer[idx++] = rx_byte;  // 버퍼에 누적 저장
            }
        }
        // 로봇 유휴 상태 + 컨베이어 정지 상태이면 작업 시작
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET && !zone_in_progress) {
            for (int zid = 0; zid < 26; zid++) {
                if (zone_pending[zid]) {
                    current_zone = 'A' + zid;
                    zone_pending[zid] = 0;
                    zone_in_progress = 1;

                    HAL_UART_Transmit(&huart3, (uint8_t*)"1\r\n", 3, HAL_MAX_DELAY); // 로봇 동작 시작 알림

                    for (int i = 0; i < coord_counts[zid]; i++) {
                        double x = latest_coords[zid][i][0];
                        double y = latest_coords[zid][i][1];
                        double z_pick = latest_coords[zid][i][2];
                        double z_safe = -0.23; // 안전 높이

                        move_linear_auto(current_x, current_y, x, y, z_safe);  // 안전 높이로 이물 위치까지 보간 이동
                        inverse_kinematics(x, y, z_pick);                      // z_pick으로 내려감
                        wait_motor_done();                                     // 도달 대기

                        // 흡착 ON
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
                        HAL_Delay(100);

                        // 마지막 좌표에서 컨베이어 재시작 요청
                        if (i == coord_counts[zid] - 1) {
                             HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
                             HAL_Delay(200);
                             HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
                        }

                        inverse_kinematics(x, y, z_safe);  // 다시 안전 높이로 상승
                        wait_motor_done();

                        // 파기 위치 결정: 중심 기준 좌우로 나눔
                        double drop_y = (y >= -0.20 && y <= 0.20) ? ((y > 0.0) ? 0.13 : -0.13) : -0.13;
                        move_linear_auto(x, y, 0.0, drop_y, z_safe);  // 파기 위치까지 보간 이동
                        inverse_kinematics(0.0, drop_y, -0.26);       // 파기 위치로 하강
                        wait_motor_done();

                        // 흡착 OFF, 파기 ON
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
                        HAL_Delay(100);
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

                        // 다시 안전 높이로 상승
                        inverse_kinematics(0.0, drop_y, z_safe);
                        wait_motor_done();

                        // 현재 위치 갱신
                        current_x = 0.0;
                        current_y = drop_y;
                    }//for

                    zone_in_progress = 0;  // ✅ 수신 루프 가능
                    char done_signal[] = "0\r\n";
                    HAL_UART_Transmit(&huart3, (uint8_t*)done_signal, strlen(done_signal), HAL_MAX_DELAY);

                    break;  // 하나의 구역만 처리
                }//if
            }//for
        }//if
    }//while
 }//main



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  huart2.Init.BaudRate = 19200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE6 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


char Checksum(char *msg, int len)
{
   char cs = 0;

   for(int i=2; i<=len; ++i){
      cs += msg[i];
   }

   return cs;
}

void RS485_Transmit(uint8_t *data, uint16_t size)
{

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);  // 송신 모드 전환
    HAL_UART_Transmit(&huart2, data, size, HAL_MAX_DELAY);  // 데이터 송신
    //HAL_UART_Transmit(&huart2, data, size, HAL_MAX_DELAY);  // 데이터 송신 확인
    //HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // 줄바꿈 추가
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // 수신 모드 전환

}

void RS485_Receive()
{
    HAL_UART_Receive(&huart2, buffer, 13, 1000);  // 데이터 수신
    HAL_Delay(10);
    //HAL_UART_Transmit(&huart3, buffer, 13, HAL_MAX_DELAY);   // 데이터 출력
    //HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);  // 줄바꿈 추가
}

void inverse_kinematics(double x, double y, double z) {
   double Sb = 0.5196;   // 베이스 삼각형 변
   double Sp = 0.0866;    // 플랫폼 삼각형 변
   double L = 0.175;    // 상부 링크 길이
   double l = 0.350;   // 하부 링크 길이

   // Wb, Ub, Wp, Up 계산
   double Wb = (sqrt(3) / 6) * Sb;
   double Ub = (sqrt(3) / 3) * Sb;
   double Wp = (sqrt(3) / 6) * Sp;
   double Up = (sqrt(3) / 3) * Sp;

    double a = Wb - Up;
    double b = (Sp * 0.5) - ((sqrt(3) * 0.5) * Wb);
    double c = Wp - (0.5 * Wb);

    // E, F, G 계산
    double E1 = 2 * L * (y + a);
    double F1 = 2 * z * L;
    double G1 = x * x + y * y + z * z + a * a + L * L + (2 * y * a) - (l * l);

    double E2 = -L * ((sqrt(3) * (x + b)) + y + c);
    double F2 = 2 * z * L;
    double G2 = x * x + y * y + z * z + b * b + c * c + L * L + (2 * (x * b + y * c)) - (l * l);

    double E3 = L * ((sqrt(3) * (x - b)) - y - c);
    double F3 = 2 * z * L;
    double G3 = x * x + y * y + z * z + b * b + c * c + L * L + (2 * (-x * b + y * c)) - (l * l);

    // EFG 값 계산
    double EFG1 = E1 * E1 + F1 * F1 - G1 * G1;
    double EFG2 = E2 * E2 + F2 * F2 - G2 * G2;
    double EFG3 = E3 * E3 + F3 * F3 - G3 * G3;

    double sqrt_EFG1 = (EFG1 < 0) ? -sqrt(-EFG1) : sqrt(EFG1);
    double sqrt_EFG2 = (EFG2 < 0) ? -sqrt(-EFG2) : sqrt(EFG2);
    double sqrt_EFG3 = (EFG3 < 0) ? -sqrt(-EFG3) : sqrt(EFG3);

    // 역기구학 각도 계산 (Solution 1)
    double theta1 = 2 * atan((-F1 - sqrt_EFG1) / (G1 - E1)) * (180 / PI);
    double theta2 = 2 * atan((-F2 - sqrt_EFG2) / (G2 - E2)) * (180 / PI);
    double theta3 = 2 * atan((-F3 - sqrt_EFG3) / (G3 - E3)) * (180 / PI);

    // 각도를 모터 스텝 값으로 변환
    int motor1_steps = (int)(((-theta1) / 360.0) * MOTOR_MAX);
    int motor2_steps = (int)(((-theta2) / 360.0) * MOTOR_MAX);
    int motor3_steps = (int)(((-theta3) / 360.0) * MOTOR_MAX);

   // printf("motor1_steps = %d\r\n", motor1_steps);
    //printf("motor2_steps = %d\r\n", motor2_steps);
    //printf("motor3_steps = %d\r\n", motor3_steps);
    // 모터 이동 실행
   angle_move(0x01, motor1_steps);
   HAL_Delay(10);
   angle_move(0x02, motor2_steps);
   HAL_Delay(10);
   angle_move(0x03, motor3_steps);
   HAL_Delay(10);
}

void ID_Check(uint8_t ID)
{
   motor[0] = 0x02;          // 1st byte : STX(0x02)
   motor[1] = 0x09;          // 2nd byte : Length
   motor[2] = ID;            // 3rd byte : Device ID
   motor[3] = 0x38;          // 4th byte : Command
   motor[4] = 0x11;          // 5th byte : Index
   motor[5] = 0x20;          // 6th byte : Index
   motor[6] = 0x00;          // 7th byte : Sub-index
   motor[7] = 0x00;          // 8th byte : Value
   motor[8] = 0x00;          // 9th byte : Value
   motor[9] = 0x00;          // 10th byte : Value
   motor[10] = 0x00;         // 11th byte : Value
   motor[11] = Checksum(motor, 10);  // 12th byte : Checksum
   motor[12] = 0x03;                 // 13th byte : ETX(0x03)

   RS485_Transmit(motor, 13);
   RS485_Receive();
}
// 릴레이 ON 함수
void Relay_ON(uint8_t ID)
{
   motor[0] = 0x02;   // Address
   motor[1] = 0x09;   // Function
   motor[2] = ID;   // Data
   motor[3] = 0x38;      // Data(ID)
   motor[4] = 0x31;   // Data
   motor[5] = 0x21;   // Data
   motor[6] = 0x00;
   motor[7] = 0x00;
   motor[8] = 0x00;
   motor[9] = 0x00;
   motor[10] = 0x00;
   motor[11] = 0x8B;
   motor[12] = 0x03;

   RS485_Transmit(motor, 13);   // 송신
   RS485_Receive();
}
void angle_move(uint8_t ID, uint32_t angle)
{
   motor[0] = 0x02;            // 1st byte : STX(0x02)
   motor[1] = 0x09;            // 2nd byte : Length
   motor[2] = ID;              // 3rd byte : Device ID
   motor[3] = 0x18;            // 4th byte : Command
   motor[4] = 0x11;            // 5th byte : Index
   motor[5] = 0x21;            // 6th byte : Index
   motor[6] = 0x00;                   // 7th byte : Sub-index
   motor[7] = angle & 0xFF;           // 8th byte : Value
   motor[8] = (angle >> 8) & 0xFF;    // 9th byte : Value
   motor[9] = (angle >> 16) & 0xFF;   // 10th byte : Value
   motor[10] = (angle >> 24) & 0xFF;  // 11th byte : Value
   motor[11] = Checksum(motor, 10);   // 12th byte : Checksum
   motor[12] = 0x03;                  // 13th byte : ETX(0x03)

    RS485_Transmit(motor, 13);
  //RS485_Receive();
}

//motor 상태 확인
void status(uint8_t ID)
{
   motor[0] = 0x02;                   // 1st byte : STX(0x02)
   motor[1] = 0x09;                   // 2nd byte : Length
   motor[2] = ID;                     // 3rd byte : Device ID
   motor[3] = 0x38;                   // 4th byte : Command
   motor[4] = 0x02;                   // 5th byte : Index
   motor[5] = 0x21;                   // 6th byte : Index
   motor[6] = 0x00;                   // 7th byte : Sub-index
   motor[7] = 0x00;                   // 8th byte : Value
   motor[8] = 0x00;                   // 9th byte : Value
   motor[9] = 0x00;                   // 10th byte : Value
   motor[10] = 0x00;                  // 11th byte : Value
   motor[11] = Checksum(motor, 10);   // 12th byte : Checksum
   motor[12] = 0x03;                  // 13th byte : ETX(0x03)

   RS485_Transmit(motor, 13);
   RS485_Receive(); // 데이터 수신
}

/**
 * @brief 세 모터(1, 2, 3)의 동작 완료 신호를 모두 수신할 때까지 대기
 * @details
 * - 각 모터에 대해 status(0x01 ~ 0x03)를 순차적으로 요청하여 응답 확인
 * - 응답된 데이터(buffer[9]) 값이 0x04이면 해당 모터 동작 완료로 판단
 * - 세 모터가 모두 완료될 때까지 반복 대기
 */
void wait_motor_done()
{
    uint8_t done1 = 0, done2 = 0, done3 = 0;   // 각 모터의 완료 여부 플래그 (0: 미완료, 1: 완료)
    while (!(done1 && done2 && done3))         // 세 모터 모두 완료될 때까지 반복
    {
        if (!done1) { status(0x01); if (buffer[9] == 0x04) done1 = 1; }
        if (!done2) { status(0x02); if (buffer[9] == 0x04) done2 = 1; }
        if (!done3) { status(0x03); if (buffer[9] == 0x04) done3 = 1; }
        HAL_Delay(100);
    }
}

/**
 * @brief 시작점(x1, y1)에서 목표점(x2, y2)까지 z높이를 유지하며 보간 직선 이동
 * @details
 * - 거리 기준으로 보간 스텝 수 결정:
 *     - 거리 <= 0.05m → 2스텝 (짧은 거리, 빠른 이동)
 *     - 거리 <  0.10m → 6스텝 (중간 거리, 적절한 보간)
 *     - 거리 >= 0.10m → 9스텝 (긴 거리, 부드럽고 정직한 이동)
 * - 마지막 지점에서만 wait_motor_done() 호출하여 정확한 도달 보장
 *
 * @param x1 시작점 X좌표 (단위: m)
 * @param y1 시작점 Y좌표 (단위: m)
 * @param x2 목표점 X좌표 (단위: m)
 * @param y2 목표점 Y좌표 (단위: m)
 * @param z  고정된 Z좌표 (단위: m) — 전체 이동 동안 유지됨
 */
void move_linear_auto(double x1, double y1, double x2, double y2, double z) {
	// 목표점까지의 X, Y 방향 거리 차이 계산
    double dx = x2 - x1;  // X축 차이
    double dy = y2 - y1;  // Y축 차이
    double dist = sqrt(dx * dx + dy * dy); // 시작점과 목표점 사이의 2차원 유클리드 거리 계산

    int steps;
    if (dist <= 0.05)      // 5cm 이하
        steps = 2;         // 짧은 거리일 땐 2스텝만 사용
    else if (dist < 0.10)  // 5cm 초과 ~ 10cm 미만
        steps = 6;         // 중간 거리일 땐 6스텝
    else                   // 10cm 이상
        steps = 9;         // 긴 거리일 땐 9스텝으로 부드럽게 이동

    // 보간된 각 스텝 위치로 반복 이동
    for (int i = 1; i <= steps; i++) {
    	// 현재 스텝 위치 계산 (선형 보간)
        double ix = x1 + dx * i / steps; // 현재 보간 X좌표
        double iy = y1 + dy * i / steps; // 현재 보간 Y좌표
        inverse_kinematics(ix, iy, z);   // 해당 보간 위치로 로봇 이동 (z는 고정된 높이)
        if (i == steps) wait_motor_done();   // 마지막 위치에 도달했을 때 모터 동작 완료 대기

    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
