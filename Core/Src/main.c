/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Buff_Size 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flashing = 1;
uint8_t Receive_buff[Buff_Size];
uint8_t UART_Message = 0;

ADC_ChannelConfTypeDef sConfig =
{ 0 };

extern DMA_HandleTypeDef hdma_usart2_rx;
extern ADC_HandleTypeDef hadc1;

struct bf
{
	unsigned int uart :1;
	unsigned int flashing :1;
} bit_field;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_IRQHandler(UART_HandleTypeDef *huart);
void UART_IDLECallback(UART_HandleTypeDef *huart);
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	bit_field.uart = 1;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2, Receive_buff, Buff_Size - 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		if (bit_field.uart == 1)
		{
			bit_field.uart = 0;
			if (UART_Message)
			{
				uint8_t str_len = Buff_Size
						- __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
				HAL_UART_Receive_DMA(&huart2, Receive_buff, Buff_Size - 1);
				memset(Receive_buff + str_len - 1, 0, Buff_Size - str_len);
				UART_Message = 0;
				if ((Receive_buff[0] == 't' || Receive_buff[0] == 'T')
						&& Receive_buff[1] == ' '
						&& (Receive_buff[2] == 'm' || Receive_buff[2] == 'M')
						&& (Receive_buff[3] == 'c' || Receive_buff[3] == 'C')
						&& (Receive_buff[4] == 'u' || Receive_buff[4] == 'U')
						&& Receive_buff[5] == '?')
				{

					sConfig.Channel = ADC_CHANNEL_16;
					sConfig.Rank = 1;
					sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
					HAL_ADC_ConfigChannel(&hadc1, &sConfig);

					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
					uint16_t val = HAL_ADC_GetValue(&hadc1);
					float temp;
					temp = (float) val / 4096 * 3.3f;
					temp = (1.43f - temp) / 4.3f + 25;
					HAL_ADC_Stop(&hadc1);
					char responseTemp[10];
					sprintf(responseTemp, "T MCU=%dC\r", (int) temp);
					responseTemp[9] = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) responseTemp, 10,
							0xFFFFFFFF);
				}
				else if ((Receive_buff[0] == 'v' || Receive_buff[0] == 'V')
						&& Receive_buff[1] == ' '
						&& (Receive_buff[2] == 'r' || Receive_buff[2] == 'R')
						&& (Receive_buff[3] == 'e' || Receive_buff[3] == 'E')
						&& (Receive_buff[4] == 'f' || Receive_buff[4] == 'F')
						&& Receive_buff[5] == '?')
				{
					sConfig.Channel = ADC_CHANNEL_VREFINT;
					sConfig.Rank = 1;
					sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
					HAL_ADC_ConfigChannel(&hadc1, &sConfig);

					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
					uint16_t val = HAL_ADC_GetValue(&hadc1);
					float vref = 1.2f * 4096.0f / val;
					HAL_ADC_Stop(&hadc1);
					char responseV[13];
					responseV[12] = 0;
					sprintf(responseV, "V REF=%.2f V\r", vref);
					HAL_UART_Transmit(&huart2, (uint8_t*) responseV, 13,
							0xFFFFFFFF);
				}
				else if ((Receive_buff[0] == 'a' || Receive_buff[0] == 'A')
						&& (Receive_buff[1] == 'l' || Receive_buff[1] == 'L')
						&& (Receive_buff[2] == 'l' || Receive_buff[2] == 'L')
						&& Receive_buff[3] == ' '
						&& (Receive_buff[4] == 's' || Receive_buff[4] == 'S')
						&& (Receive_buff[5] == 'e' || Receive_buff[5] == 'E')
						&& (Receive_buff[6] == 'n' || Receive_buff[6] == 'N')
						&& (Receive_buff[7] == 's' || Receive_buff[7] == 'S')
						&& (Receive_buff[8] == 'e' || Receive_buff[8] == 'E')
						&& Receive_buff[9] == '?')
				{
					sConfig.Channel = ADC_CHANNEL_16;
					sConfig.Rank = 1;
					sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
					HAL_ADC_ConfigChannel(&hadc1, &sConfig);

					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
					uint16_t val = HAL_ADC_GetValue(&hadc1);
					float temp;
					temp = (float) val / 4096 * 3.3f;
					temp = (1.43f - temp) / 4.3f + 25;
					HAL_ADC_Stop(&hadc1);
					char responseTemp[10];
					sprintf(responseTemp, "T MCU=%dC\r", (int) temp);
					responseTemp[9] = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*) responseTemp, 10,
							0xFFFFFFFF);

					sConfig.Channel = ADC_CHANNEL_VREFINT;
					sConfig.Rank = 1;
					sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
					HAL_ADC_ConfigChannel(&hadc1, &sConfig);

					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
					uint16_t vVal = HAL_ADC_GetValue(&hadc1);
					float vref = 1.2f * 4096.0f / vVal;
					HAL_ADC_Stop(&hadc1);
					char responseV[13];
					responseV[12] = 0;
					sprintf(responseV, "V REF=%.2f V\r", vref);
					HAL_UART_Transmit(&huart2, (uint8_t*) responseV, 13,
							0xFFFFFFFF);
				}
				else
				{
					HAL_UART_Transmit(&huart2, (uint8_t*) "Wrong command",
							strlen("Wrong command\r"), 0xFFFFFFFF);
				}
			}
			bit_field.flashing = 1;
		}
		/* USER CODE BEGIN 3 */

		if (bit_field.flashing == 1)
		{
			bit_field.flashing = 0;
			if (flashing)
			{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				HAL_Delay(250);
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
				HAL_Delay(250);
			}
			bit_field.uart = 1;
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		UART_IDLECallback(huart);
	}
}

void UART_IDLECallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(huart);
	UART_Message = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
