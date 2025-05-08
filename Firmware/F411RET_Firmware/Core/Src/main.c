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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "calculations.hpp"
#include "sunCalc.hpp"
#include "motorFuncs.hpp"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <time.h>
#define RX_BUFFER_SIZE 50
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#pragma pack(push, 1)
typedef struct {
    double latitude;   // 8 bytes
    double longitude;  // 8 bytes
    int32_t timestamp; // 4 bytes
} PacketData;
#pragma pack(pop)
static PacketData rxPacket;
static uint8_t rxBuffer[sizeof(PacketData)];
static uint8_t tempByte;
volatile bool rxPacketReady = false;
// Our "2 states"
typedef enum {
    WAIT_HEADER,
    WAIT_PAYLOAD
} RxState;
static RxState rxState = WAIT_HEADER;
// Indices for partial progress
static uint8_t headerIndex = 0;     // 0..2 (2 means header matched)
static uint8_t payloadIndex = 0;
uint32_t timeAtSync;
uint32_t unixNow;
bool gotData = false;

char buffer[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void SetRTC_FromUnixTime(uint32_t unixTime)
{
    // Convert epoch to calendar time (UTC).
    // If you want local time, use localtime(&unixTime) instead,
    // but typically RTC is set to UTC.
    struct tm *t = gmtime((time_t*)&unixTime);

    // Prepare the time/date structs for the HAL RTC
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // The STM32 HAL RTC expects Hours/Minutes/Seconds in 0–23 / 0–59 / 0–59
    sTime.Hours   = t->tm_hour + 3;
    sTime.Minutes = t->tm_min;
    sTime.Seconds = t->tm_sec;
    sTime.TimeFormat = RTC_HOURFORMAT_24;
    // Optionally set SubSeconds, DayLightSaving, StoreOperation fields if needed

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        // Error handling
    }

    // The HAL expects year 00–99 to represent 2000–2099 by default
    // so we subtract 100 from tm_year (which is years since 1900).
    // e.g. if tm_year=123 => year 2023.
    // STM Year = 23 => means 2023 in RTC
    sDate.Year    = (t->tm_year + 1900) - 2000;
    sDate.Month   = t->tm_mon + 1;  // tm_mon is 0–11
    sDate.Date    = t->tm_mday;
    sDate.WeekDay = t->tm_wday ? t->tm_wday : 7;
    // HAL_RTC week day is 1=Monday, 7=Sunday;
    // Standard tm_wday is 0=Sunday, so you might adjust logic as needed.

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        // Error handling
    	printf("Error in setting time");
    }
}

time_t GetUnixTimeFromRTC(void)
{
    RTC_TimeTypeDef sTimeTemp;
    RTC_DateTypeDef sDateTemp;
    struct tm t = {0};

    // 1) Read RTC time/date
    HAL_RTC_GetTime(&hrtc, &sTimeTemp, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDateTemp, RTC_FORMAT_BIN);

    // 2) Convert to 'struct tm'
    uint16_t fullYear = sDateTemp.Year + 2000;     // e.g. if year=23 => 2023
    t.tm_year = fullYear - 1900;               // years since 1900
    t.tm_mon  = sDateTemp.Month - 1;               // tm_mon is 0–11
    t.tm_mday = sDateTemp.Date;                    // day of month (1–31)
    t.tm_hour = sTimeTemp.Hours;                   // 0–23
    t.tm_min  = sTimeTemp.Minutes;                 // 0–59
    t.tm_sec  = sTimeTemp.Seconds;                 // 0–59

    // 3) Convert 'tm' to Unix time
    //    Note: by default, mktime() treats this as local time
    //    If your RTC is in UTC, treat it as local offset=0
    time_t unixTime = mktime(&t);

    return unixTime;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int counter = 0;

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3 );
  HAL_TIM_Base_Start(&htim1);
  setServoToInit();
  // Start receiving 1 byte in interrupt mode
//    HAL_UART_Receive_IT(&huart1, &tempByte, 1);
  uint8_t dummy;
  bool didClean = false;
//  while (HAL_UART_Receive(&huart1, &dummy, 1, 0) == HAL_OK) {
//      // just drain leftover data
//	  printf("CLEANING\r\n");
//	  didClean = true;
//	  __NOP();
//  }
  HAL_UART_Receive_IT(&huart1, &tempByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("--------------Main loop begin--------------\r\n");
  HAL_Delay(2000);
  while (1)
  {
	  printf("\n####_%d_LOOP_%d_####\r\n",didClean,counter);
	  counter++;

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(500);
      if (rxPacketReady)
      {
          rxPacketReady = false;
          // Safe to print or handle data here
          printf(">>>>>>>>>>>Got data: lat=%.6f, lon=%.6f, time: %d\r\n",rxPacket.latitude, rxPacket.longitude, rxPacket.timestamp);
          SetRTC_FromUnixTime(rxPacket.timestamp);
          timeAtSync = HAL_GetTick();
          gotData = true;
//          testFunc();
      }

      if (!gotData) continue;

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d, Date: %02d:%02d:%02d\r\n",
	  	  	  sTime.Hours,sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year + 2000);
	  printf(buffer);

	  unixNow = ((HAL_GetTick() - timeAtSync) / 1000) + rxPacket.timestamp;
	  sunPos test = sunTest(sDate.Year + 2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, rxPacket.latitude, rxPacket.longitude);
	  printf("azimuth: %.2f, altitude: %.2f\r\n",test.azimuth, test.altitude);

	  motorAngles angles = anglesCalculation(test.azimuth, test.altitude);
	  printf("angles: %d | %d | %d | %d\r\n", angles.val1, angles.val2, angles.val3, angles.val4);

	  int anglesSend[4];
	  anglesSend[0] = angles.val1;
	  anglesSend[1] = angles.val2;
	  anglesSend[2] = angles.val3;
	  anglesSend[3] = angles.val4;
	  moveMotors(anglesSend);


	  HAL_Delay(2000);

	  __NOP();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 17;
  sTime.Minutes = 15;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 4;
  sDate.Year = 25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8400-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (rxState)
        {
        case WAIT_HEADER:
            // We want a sequence [0xAA, 0x55].
            // headerIndex==0 => next byte must be 0xAA
            // headerIndex==1 => next byte must be 0x55
            // If we reach headerIndex==2 => done, move to WAIT_PAYLOAD

            if (headerIndex == 0)
            {
                if (tempByte == 0xAA)
                {
                    headerIndex = 1; // found first header byte
                }
                else
                {
                    // remain at 0, keep waiting for 0xAA
                	printf("BAD DATA\r\n");
                }
            }
            else if (headerIndex == 1)
            {
                if (tempByte == 0x55)
                {
                    // valid header
                    headerIndex = 2;
                    rxState = WAIT_PAYLOAD;
                    payloadIndex = 0;   // reset for payload
                }
                else
                {
                    // not 0x55
                    // if it's 0xAA, stay at 1 (like partial overlap)
                	printf("BAD DATA\r\n");
                    if (tempByte == 0xAA)
                        headerIndex = 1;
                    else
                        headerIndex = 0;
                }
            }
            break;

        case WAIT_PAYLOAD:
            // Store each incoming byte
            rxBuffer[payloadIndex++] = tempByte;

            if (payloadIndex >= sizeof(PacketData))
            {
                // We have the entire payload
                memcpy(&rxPacket, rxBuffer, sizeof(rxPacket));
                rxPacketReady = true;

                // Return to WAIT_HEADER to look for next packet
                rxState = WAIT_HEADER;
                headerIndex = 0;
                payloadIndex = 0;
            }
            break;
        }

        // Re-initiate 1-byte interrupt receive
        HAL_UART_Receive_IT(&huart1, &tempByte, 1);
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
