/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initI2C(void);
//I2C_start zal de start conditie op de bus plaatsen en kan ook gebruikt worden voor een "repeated start".
//Een start bit is als men de data lijn laag maakt als de kloklijn hoog is.
void I2C_start(void);

//I2C_stop zal de stop conditie op de bus plaatsen.
//Een stop bit is als men de data lijn hoog maakt als de kloklijn hoog is.
void I2C_stop(void);

//I2C_write zal 1 byte op de I2C bus plaatsen. Deze functie kan ook gebruikt worden om een slaveadres op de bus te plaatsen.
//De functie zal ook de ack binnen lezen en teruggeven (bij geldige ack op de bus geeft deze functie 0 terug)
uint8_t I2C_write(uint8_t data);

//I2C_read zal 1 byte van de I2C bus lezen en al dan niet een ack of een nack terug sturen
//afhankelijk van het argument dat je meegeeft. (ack is 0, nack is 1)
uint8_t I2C_read(uint8_t ack);

//I2C_readMem zal eerst de slave adreseren dan het register adres op de bus schrijven en na een repeated start
//het register lezen. De functie returned het aantal gelezen bytes.
uint8_t I2C_readMem(uint8_t slaveAdres, uint8_t memAdres, uint8_t * buffer, uint8_t byteCount);

uint32_t delay1us = 10000; //20
void SysTickDelayCount(uint32_t);
uint8_t reverse(uint8_t);

#include <stdio.h>

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
int _write(int file, char *ptr, int len) {
	HAL_StatusTypeDef xStatus;
	switch (file) {
	case STDOUT_FILENO: /*stdout*/
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
		break;
	case STDERR_FILENO: /* stderr */
		xStatus = HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
		if (xStatus != HAL_OK) {
			errno = EIO;
			return -1;
		}
		break;
	default:
		errno = EBADF;
		return -1;
	}
	return len;
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	uint8_t	slaveAdres = 0b00111010;
	uint8_t	memAdres = 0x0D;
	uint8_t	byteCount = 4;
	uint8_t *buffer;

	uint8_t input = 0x00;

	initI2C();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		input = I2C_readMem(slaveAdres, memAdres, buffer, byteCount);

		printf("%d\n\r", input);

		HAL_Delay(1);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, I2C_SCL_Pin|I2C_SDA_Pin|SPI_MOSI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_SCK_GPIO_Port, SPI_SCK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : I2C_SCL_Pin I2C_SDA_Pin */
	GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : INT_RTC_Pin SPI_MISO_Pin */
	GPIO_InitStruct.Pin = INT_RTC_Pin|SPI_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : INT_TEMP_Pin */
	GPIO_InitStruct.Pin = INT_TEMP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT_TEMP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_SCK_Pin */
	GPIO_InitStruct.Pin = SPI_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_SCK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : INT_MMA1_Pin */
	GPIO_InitStruct.Pin = INT_MMA1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT_MMA1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_Pin */
	GPIO_InitStruct.Pin = SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : INT_MMA2_Pin */
	GPIO_InitStruct.Pin = INT_MMA2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT_MMA2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_MOSI_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void initI2C(void){
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 1);
	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 1);
}


//I2C_start zal de start conditie op de bus plaatsen en kan ook gebruikt worden voor een "repeated start".
//Een start bit is als men de data lijn laag maakt als de kloklijn hoog is.
void I2C_start(void){
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 0);
	SysTickDelayCount(delay1us);
	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 0);
	SysTickDelayCount(delay1us);
}

//I2C_stop zal de stop conditie op de bus plaatsen.
//Een stop bit is als men de data lijn hoog maakt als de kloklijn hoog is.
void I2C_stop(void){
	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 1);
	SysTickDelayCount(delay1us);
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 1);
	SysTickDelayCount(delay1us);
}

//I2C_write zal 1 byte op de I2C bus plaatsen. Deze functie kan ook gebruikt worden om een slaveadres op de bus te plaatsen.
//De functie zal ook de ack binnen lezen en teruggeven (bij geldige ack op de bus geeft deze functie 0 terug)
uint8_t I2C_write(uint8_t data){
	uint8_t shift = 0x00;
	uint8_t mask = 0b00000001;
	uint8_t result = 0x00;
	uint8_t ack = 0x00;

	//send input
	shift = reverse(data);
	for(uint8_t index = 0; index < 8; index++){
		result = shift & mask;
		if(result == 0x01){
			HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 1);
		}
		else{
			HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 0);
		}

		HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 1);
		SysTickDelayCount(delay1us);
		HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 0);
		SysTickDelayCount(delay1us);

		shift = (shift >> 1);	//shift right by one
	}
	//ACK
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, 1);

	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 1);
	SysTickDelayCount(delay1us);

	ack = HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin);

	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 0);
	SysTickDelayCount(delay1us);

	return ack;
}

//I2C_read zal 1 byte van de I2C bus lezen en al dan niet een ack of een nack terug sturen
//afhankelijk van het argument dat je meegeeft. (ack is 0, nack is 1)
uint8_t I2C_read(uint8_t ack){
	for(uint8_t index = 0; index < 8; index++){
		HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 1);
		SysTickDelayCount(delay1us);

		//read bit and shift

		HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, 0);
		SysTickDelayCount(delay1us);

	}
	return 0x00;
}

//I2C_readMem zal eerst de slave adreseren dan het register adres op de bus schrijven en na een repeated start
//het register lezen. De functie returned het aantal gelezen bytes.
uint8_t I2C_readMem(uint8_t slaveAdres, uint8_t memAdres, uint8_t * buffer, uint8_t byteCount){
	uint8_t input = 0x00;
	uint8_t ack = 0x00;

	I2C_start();

	ack = I2C_write(slaveAdres);
	SysTickDelayCount(delay1us);
	ack = I2C_write(memAdres);
	SysTickDelayCount(delay1us);
	ack = I2C_write(slaveAdres+1);
	SysTickDelayCount(delay1us);

	input = I2C_read(0x00);

	I2C_stop();

	return input;
}

void SysTickDelayCount(uint32_t ulCount){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	while(DWT->CYCCNT < ulCount);
}


uint8_t reverse(uint8_t b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}



/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
