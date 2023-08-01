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
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define MAX_SECTOR	12
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Base address of the Flash sectors */
uint32_t sector[12] = {
ADDR_FLASH_SECTOR_0, /* Base @ of Sector 0, 16 Kbytes */
ADDR_FLASH_SECTOR_1, /* Base @ of Sector 1, 16 Kbytes */
ADDR_FLASH_SECTOR_2, /* Base @ of Sector 2, 16 Kbytes */
ADDR_FLASH_SECTOR_3, /* Base @ of Sector 3, 16 Kbytes */
ADDR_FLASH_SECTOR_4, /* Base @ of Sector 4, 64 Kbytes */
ADDR_FLASH_SECTOR_5, /* Base @ of Sector 5, 128 Kbytes */
ADDR_FLASH_SECTOR_6, /* Base @ of Sector 6, 128 Kbytes */
ADDR_FLASH_SECTOR_7, /* Base @ of Sector 7, 128 Kbytes */
ADDR_FLASH_SECTOR_8, /* Base @ of Sector 8, 128 Kbytes */
ADDR_FLASH_SECTOR_9, /* Base @ of Sector 9, 128 Kbytes */
ADDR_FLASH_SECTOR_10, /* Base @ of Sector 10, 128 Kbytes */
ADDR_FLASH_SECTOR_11/* Base @ of Sector 11, 128 Kbytes */

};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer[100];
uint8_t *str;
uint8_t rxd;
uint8_t rx_complete = 0;

void PutString(char *str)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
}

void DisplayDump(uint32_t *data, uint32_t addr, uint32_t size)
{
	PutString("\r\n-----------+-------------------------+------------------------");
	PutString("\r\n  Address  | 00 01 02 03 04 05 06 07 | 08 09 0A 0B 0C 0D 0E 0F");
	PutString("\r\n-----------+-------------------------+------------------------");

	for(uint16_t i=0; i<size; i++){
	  if((i % 4) == 0 || i == 0){
		  if(i != 0) {
			  PutString((char*)buffer);
			  buffer[0] = NULL;
		  }
		  sprintf((char*)buffer,"\r\n  %08X   ", addr + i * 4);
	  }

	  for(uint8_t j=0; j<4; j++){
		  char temp[10];
		  if((i % 4) != 0 && i % 2 == 0 && j == 0)
			  sprintf(temp, "  %02lX ", (*(data + i) >> (8 * j)) & 0xFF);
		  else
			  sprintf(temp, "%02lX ", (*(data + i) >> (8 * j)) & 0xFF);
		  strcat(buffer, temp);
	  }
	}

	if(*buffer != NULL) PutString((char*)buffer);
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
  MX_DMA_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rxd, 1);
  str = buffer;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char cmd[10];
  uint32_t addr, len, sec;

  PutString("\r\n\nFLASH Memory Dump\r\n");
  PutString(">");

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(rx_complete == 1){
		  rx_complete = 0;
		  sscanf((char*)buffer, "%s%lx%ld", cmd, &addr, &len);
		  uint32_t size = len / 4 + ((len % 4) ? 1 : 0);

		  if(strcmp(cmd, "flashdump") == 0){
			  uint32_t *p = (uint32_t *) malloc(sizeof(uint32_t) * size);
			  if(p != NULL){
				  for(uint16_t i=0; i<size; i++)
					  *(p + i) = *(uint32_t *)(addr + i * 4);

				  DisplayDump(p, addr, size);
				  free(p);
			  }
		  }
		  else if(strcmp(cmd,"flashsdump") == 0){
			  uint32_t *p = (uint32_t *) malloc(sizeof(uint32_t) * size);

			  if(p != NULL){
			  	  sscanf((char*)buffer, "%s%ld%ld", cmd, &sec, &len);
				  addr = sector[sec];
				  if(sec < MAX_SECTOR){
					  for(uint16_t i=0; i<size; i++)
						  *(p + i) = *(uint32_t *)(addr + i * 4);

					  DisplayDump(p, addr, size);
				  }
				  free(p);
			  }
		  }
		  else if(buffer[0] != '\n' && buffer[0] != '\r' ){
			  PutString("\r\nInvalid Command!!");

		  }
		  memset(buffer, 0, sizeof(buffer));
		  PutString("\r\n\n>");
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
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
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3){
		*(str++) = rxd;
		HAL_UART_Transmit(&huart3, &rxd, 1, 1000);
		if(rxd == 0x0d || rxd == 0x0a){
			*str = 0;
			//HAL_UART_Transmit_IT(&huart3, buffer, strlen((char*)buffer));
			rx_complete = 1;
			str = buffer;
		}
		HAL_UART_Receive_IT(&huart3, &rxd, 1);
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
