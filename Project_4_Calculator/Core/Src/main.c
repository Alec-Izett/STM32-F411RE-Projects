/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "float.h"
#include "stdio.h"
#include "memory.h"
#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0x08008000
#define EEPROM_lastCounterInt (EEPROM_ADDR)
#define FLASH_PROGRAM_BYTE	((uint32_t)0x00)
#define FLASH_PROGRAM_HWORD	((uint32_t)0x01)
#define FLASH_PROGRAM_WORD	((uint32_t)0x02)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char keyChar = 0;
uint16_t key = 0;
uint32_t resultInt;

int blueButton = 0;

int step;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void screenClear(void);
uint16_t keyPadScan(void);
void writeEEPROM(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char allBuffer[8];
char resultString[8];
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	uint8_t clearInstruction[]={0x7C, 0x2D};
	HAL_UART_Transmit(&huart1, (uint8_t *) clearInstruction, sizeof(clearInstruction), HAL_MAX_DELAY);

	uint8_t screenSettings[]={0x7C, 0xBB};
	HAL_UART_Transmit(&huart1, (uint8_t *) screenSettings, sizeof(screenSettings), HAL_MAX_DELAY);

  char sendMessage[]="Launching Izett_Calc\n";
  char sendMessage2[]="Available functions\n";
  char sendMessage3[]="add, subtract\n\r";
  char sendMessage4[]="multiply, and divide\n";
  char sendMessage5[]="Press '#' to cont...\r\n";
  char sendMessage6[]="Enter first number\r\n";
  char sendMessage7[]="Enter operator\r\n";
  char sendMessage8[]="Enter second number\r\n";
  char sendMessage9[]="Result:\r\n";
  char sendMessage10[]="Undefined";

//  char newLine[]="\r\n";

  screenClear();

  int step;

  float number1 = 0;
  int operation = 0;
  float number2 = 0;
  float result = 0;


  int i = 0;

  char tempNumber = 0;

  step = 0;

  int y = 0;
  int x[10] = {0};
  int n = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
if(step == 0){
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage, strlen(sendMessage), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage2, strlen(sendMessage2), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage3, strlen(sendMessage3), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage4, strlen(sendMessage4), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  screenClear();
	  HAL_Delay(100);
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage5, strlen(sendMessage5), HAL_MAX_DELAY);
	  step = 1;
}else if(step == 1){

	  if(keyChar == 35){
	  screenClear();
	  HAL_Delay(100);
	  keyChar = 0;
	  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage6, strlen(sendMessage6), HAL_MAX_DELAY);
	  step = 2;
	  }
	  }else if(step == 2){
	 // key = keyPadScan();
	  if(keyChar != 0){
		  if(keyChar == 35){
			  screenClear();
			  HAL_Delay(100);
			  keyChar = 0;
			  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage7, strlen(sendMessage7), HAL_MAX_DELAY);


			  number1 = atof(allBuffer);

			  memset(allBuffer, 0, sizeof(allBuffer));
			  keyChar = 0;
			  tempNumber = 0;
			  i=0;

			  step = 3;


		  }else{
		  tempNumber = keyChar;
		  char type[2];
		  sprintf(type,"%c", keyChar);
		  HAL_UART_Transmit(&huart1, (uint8_t *) type, strlen(type), HAL_MAX_DELAY);
		  HAL_Delay(100);

		  allBuffer[i] = tempNumber;
		  i++;

		  keyChar = 0;
		  }
	  }else if(blueButton == 1){
		  memcpy(&resultInt, (void *) EEPROM_lastCounterInt, sizeof(resultInt));
	  	  HAL_Delay(1000);
    	  number1 = (float)resultInt;
		  HAL_Delay(1000);

		  blueButton = 0;

		  screenClear();
		  HAL_Delay(100);
		  keyChar = 0;
		  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage7, strlen(sendMessage7), HAL_MAX_DELAY);
		  memset(allBuffer, 0, sizeof(allBuffer));
		  keyChar = 0;
		  tempNumber = 0;
		  i=0;

		  step = 3;
	  }
}else if(step == 3){
	 // key = keyPadScan();
	  if(keyChar != 0){
		  if(keyChar == 35){
			  screenClear();
			  HAL_Delay(100);
			  keyChar = 0;
			  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage8, strlen(sendMessage8), HAL_MAX_DELAY);

			  n = 0;
			  y = 0;

			  step = 4;
		  }else{
		  tempNumber = keyChar;
		  char type[2];
		  sprintf(type,"%c", keyChar);
		  HAL_UART_Transmit(&huart1, (uint8_t *) type, strlen(type), HAL_MAX_DELAY);
		  HAL_Delay(100);

		  y = (int)(keyChar);
		  x[n] = y;
		  n++;
		  operation = x[0];

		  keyChar = 0;
		  }
	  }
}else if(step == 4){
	 // key = keyPadScan();
	  if(keyChar != 0){
		  if(keyChar == 35){
			  screenClear();
			  HAL_Delay(100);
			  keyChar = 0;
			  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage9, strlen(sendMessage9), HAL_MAX_DELAY);

			  number2 = atof(allBuffer);

			  memset(allBuffer, 0, sizeof(allBuffer));
			  keyChar = 0;
			  tempNumber = 0;
			  i=0;
			  step = 5;
		  }else{
			  tempNumber = keyChar;
			  char type[2];
			  sprintf(type,"%c", keyChar);
			  HAL_UART_Transmit(&huart1, (uint8_t *) type, strlen(type), HAL_MAX_DELAY);
			  HAL_Delay(100);

			  allBuffer[i] = tempNumber;
			  i++;

			  keyChar = 0;
			  }
	  }else if(blueButton == 1){
		  memcpy(&resultInt, (void *) EEPROM_lastCounterInt, sizeof(resultInt));
	  	  HAL_Delay(1000);
    	  number2 = (float)resultInt;
		  HAL_Delay(1000);

		  blueButton = 0;

		  screenClear();
		  HAL_Delay(100);
		  keyChar = 0;
		  HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage8, strlen(sendMessage8), HAL_MAX_DELAY);
		  memset(allBuffer, 0, sizeof(allBuffer));
		  keyChar = 0;
		  tempNumber = 0;
		  i=0;

		  step = 5;
	  }
}else if(step == 5){
	  if(keyChar == 35){

		  resultInt = result;
		  HAL_Delay(200);
		  writeEEPROM();
		  HAL_Delay(1000);

		  resultInt = 0;
		  screenClear();
		  HAL_Delay(200);
		  result = 0;
		  *resultString = '\0';
		  number1 = 0;
		  number2 = 0;
		  operation = 0;
		  step = 1;
		  tempNumber = 0;
		  i = 0;
		  HAL_Delay(100);
	  }
	    if((operation == 68) & (number2 == 0)){
	    	HAL_UART_Transmit(&huart1, (uint8_t *) sendMessage10, strlen(sendMessage10), HAL_MAX_DELAY);
	    	HAL_Delay(100);
	    	operation = 0;
	    }else if(operation == 65){
	        result = number1 + number2;
	        sprintf(resultString, "%f", result);
	    	HAL_UART_Transmit(&huart1, (uint8_t *) resultString, strlen(resultString), HAL_MAX_DELAY);
	    	HAL_Delay(100);
	    	operation = 0;
	    }else if(operation == 66){
	        result = number1 - number2;
	        sprintf(resultString, "%f", result);
	    	HAL_UART_Transmit(&huart1, (uint8_t *) resultString, strlen(resultString), HAL_MAX_DELAY);
	    	HAL_Delay(100);
	    	operation = 0;
	    }else if(operation == 67){
	        result = number1 * number2;
	        sprintf(resultString, "%f", result);
	    	HAL_UART_Transmit(&huart1, (uint8_t *) resultString, strlen(resultString), HAL_MAX_DELAY);
	    	HAL_Delay(100);
	    	operation = 0;
	    }else if(operation == 68){
	        result = number1 / number2;
	        sprintf(resultString, "%f", result);
	    	HAL_UART_Transmit(&huart1, (uint8_t *) resultString, strlen(resultString), HAL_MAX_DELAY);
	    	HAL_Delay(100);
	    	operation = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void screenClear(void){
	uint8_t clearInstruction[]={0x7C, 0x2D};
	HAL_UART_Transmit(&huart1, (uint8_t *) clearInstruction, sizeof(clearInstruction), HAL_MAX_DELAY);

}

uint16_t keyPadScan(void){

	return 0;


}

void writeEEPROM(void){

	  HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

	  FLASH_Erase_Sector(FLASH_SECTOR_2, FLASH_VOLTAGE_RANGE_1);

	  HAL_FLASH_Program(TYPEPROGRAM_WORD, EEPROM_lastCounterInt, *(uint32_t *)&resultInt);
	  HAL_Delay(1000);
	  HAL_FLASH_Lock();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
