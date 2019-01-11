/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//****************************************************************************** DEFINICJE
#define BLISKO_OBIEKT_LEWY 492 // 6 cm od ściany, wartość im bliżej tym rośnie
#define BLISKO_OBIEKT_PRAWY 0 //hgw
#define KONIEC_PLANSZY 0	//TODO: dodać wartości!!!!!!!!!!
#define POCZATEK_PLANSZY 0

enum
{
	JEDZ_PROSTO, PRZECIWNIK, ZAWROT_LEWO, ZAWROT_PRAWO
};
//****************************************************************************** FLAGI
uint8_t flaga_proxim_znalazl = 0;
uint8_t flaga_stanu = 0;
uint8_t stan = JEDZ_PROSTO;
//********************************************************************* STAŁE DO MYSZY
int message[] = { 0, 0, 1, 0, 1, 1, 1, 1, 0 };
int messOffset = 0;
int package1 = 0;
int package2 = 0;
int package3 = 0;
int response[40];
int wysylanie = 0;
int i = 0;
int j = 0;
int mouseReset = 0;
//***************************************************************** STAŁE DO WSPÓŁRZĘDNYCH
int absX = 0;
int absY = 0;
int dx = 0;
int dy = 0;
int lastX = 0;
int lastY = 0;
int remX = 0;
//************************************************************************ DO SILNIKÓW
int punch = 0;
int pwm1 = 0;
int pwm2 = 0;
int moc = 120;
int remMoc = 110;
int counterPWM = 0;
int motorWork = 0;
//****************************************************************************** ZMIENNE
uint16_t odczyt_proxim_lewy = 0, odczyt_proxim_prawy = 0;
int iterator = 0;
char data_to_send[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void send_string(char* s)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), 1000);
}
char convertBitsToInt(int byte[])
{
	char i = 0;
	char result = 0;
	char ones = 0;

	for(i = 0; i < 8; i++)
	{
		if(byte[i] == 1)
		{
			result = result + pow(2, i);
			ones++;
		}
	}
	if(ones % 2 == byte[8])
	{
		result = -1;
	}
	return result;
}
int adc_read(uint32_t channel)	// odczytuje wartość napięcia danego channelu
{
	ADC_ChannelConfTypeDef adc_ch;
	adc_ch.Channel = channel;
	adc_ch.Rank = ADC_REGULAR_RANK_1;
	adc_ch.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &adc_ch);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	return HAL_ADC_GetValue(&hadc1);
}
//TODO: ogarnąć funkcje jeżdżenia
void zawrot_w_lewo()
{
}
void zawrot_w_prawo()
{
}
void zatrzymaj_sie()
{
}
void sortuj_pucki()
{
} //TODO: sortowanie też ogarnąć

//********************************************************************** USTAWIENIA MYSZY
void ustaw_pin()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = mouseData_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(mouseData_GPIO_Port, &GPIO_InitStruct);
}
void pobierz_dane()
{
	messOffset = 0;
	j = 9 - motorWork;
	int responseArray[9] = { response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8], response[9] };
	package1 = convertBitsToInt(responseArray);
	int responseArray2[9] = { response[12], response[13], response[14], response[15], response[16], response[17], response[18], response[19],
			response[20] };
	package2 = convertBitsToInt(responseArray2);
	int responseArray3[9] = { response[23], response[24], response[25], response[26], response[27], response[28], response[29], response[30],
			response[31] };
	package3 = convertBitsToInt(responseArray3);
}
void wylicz_wspolrzedne()
{
	dy = (package3 - ((package1 << 3) & 0x100)) - lastY;
	dx = (package2 - ((package1 << 4) & 0x100)) - lastX;
	absX = absX + (package2 - ((package1 << 4) & 0x100));
	absY = absY + (package3 - ((package1 << 3) & 0x100));
	lastX = (package2 - ((package1 << 4) & 0x100));
	lastY = (package3 - ((package1 << 3) & 0x100));
}
void wylicz_moc()
{
//	if(dy < 30 && dy > -30)
//	{
//		if(dy < 0)
//			moc = 200 + dy * 2;
//		else
//			moc = 200 - dy * 2;	//motorWork=1;
//	}
//	else
//	{
//		moc = remMoc;
//	}

}
void jedz_do_przodu()
{
	HAL_GPIO_WritePin(H_Input1_GPIO_Port, H_Input1_Pin, 0);
	HAL_GPIO_WritePin(H_Input2_GPIO_Port, H_Input2_Pin, 1);
	HAL_GPIO_WritePin(H_Input3_GPIO_Port, H_Input3_Pin, 0);
	HAL_GPIO_WritePin(H_Input4_GPIO_Port, H_Input4_Pin, 1);
}
//*************************************************************************************
uint16_t proxim_blisko_obiekt()
{
	return (odczyt_proxim_lewy > BLISKO_OBIEKT_LEWY || odczyt_proxim_prawy > BLISKO_OBIEKT_PRAWY) ? 1 : 0;
}
uint8_t is_kraniec_planszy()
{
	if(absY >= KONIEC_PLANSZY)
		return ZAWROT_LEWO;
	else if(absY <= POCZATEK_PLANSZY)
		return ZAWROT_PRAWO;
	else
		return PRZECIWNIK;
}
//**************************************************************************************************
//*****************************		PRZERWANIA OD PRZYRZĄDÓW 	************************************
//**************************************************************************************************
//TODO: pamiętać o zerowaniu że proxim znalazł
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	odczyt_proxim_lewy = adc_read(ADC_CHANNEL_12); // Pobranie zmierzonej wartosci
	odczyt_proxim_prawy = adc_read(ADC_CHANNEL_13);	// z danego kanału

	sprintf(data_to_send, "%d", odczyt_proxim_lewy);
	send_string("\n\r");
	send_string(data_to_send);
	return; // TODO: tylko do celów sprawdzenia odczytów!

	if(proxim_blisko_obiekt() && !flaga_proxim_znalazl)
	{
		flaga_proxim_znalazl = 1;
		zatrzymaj_sie();

		switch(is_kraniec_planszy())
		{
		case PRZECIWNIK:
			//TODO: dodać co ma zrobić jak zobaczy przeciwnika
			break;
		case ZAWROT_LEWO:
			zawrot_w_lewo();
			break;
		case ZAWROT_PRAWO:
			zawrot_w_prawo();
			break;
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == mouseClock_Pin)
	{
		if(wysylanie == 1)
		{
			if(j >= 9)
			{
				if(j == 9)
				{
					ustaw_pin();
				}
				response[j - 9 - messOffset] = HAL_GPIO_ReadPin(
				mouseData_GPIO_Port, mouseData_Pin);
				if(response[0] == 1)
				{
					messOffset = j - 8;
				}
				j++;

				if(j >= 42 - motorWork)
				{
					pobierz_dane();
					if(package1 == -1 || package2 == -1 || package3 == -1)
					{
						return;
					}
//					if (package1 == 255 && package2 == 255 && package3 == 255) {
//						//motorWork = 1;
//					}

					wylicz_wspolrzedne();
					wylicz_moc();
//**************************************************************************************************
//**************************		WIKTORIA TU SIĘ ROZGASZCZA		********************************
//**************************************************************************************************
					if(stan == JEDZ_PROSTO)
					{
						if(!flaga_stanu)
						{
							flaga_stanu = 1;
							jedz_do_przodu();
						}
						pwm1 = moc;
						pwm2 = moc;

						TIM2->CCR1 = pwm1;
						TIM2->CCR2 = pwm2;
					}
//					} else if (remX == 0 && absY > 15000) {
//						if (remX == 0) {
//							//remX = absX;
//						}
//						if (dx < -10 && dx > 10) {
//							pwm1 = 200;	// - dx * 3;
//							pwm2 = 200;	// - dx * 3;
//
//						} else {
//							pwm1 = 200;
//							pwm2 = 200;
//						}
//						HAL_GPIO_WritePin(H_Input1_GPIO_Port, H_Input1_Pin, 1);
//						HAL_GPIO_WritePin(H_Input2_GPIO_Port, H_Input2_Pin, 0);
//						HAL_GPIO_WritePin(H_Input3_GPIO_Port, H_Input3_Pin, 1);
//						HAL_GPIO_WritePin(H_Input4_GPIO_Port, H_Input4_Pin, 0);
//					}
				}
			}
			else
			{
				HAL_GPIO_WritePin(mouseData_GPIO_Port, mouseData_Pin, message[j]);
				j++;
			}
		}
	}

	if(GPIO_Pin == PWM_counter_Pin)
	{
		if(punch == 1)
		{
			TIM2->CCR1 = 0;
			TIM2->CCR2 = 0;
			counterPWM++;
			if(counterPWM < 3)
			{
				TIM3->CCR2 = 21;
			}
			else if(counterPWM < 8)
			{
				TIM3->CCR2 = 10;
			}
			else if(counterPWM >= 8)
			{
				TIM3->CCR2 = 0;
				counterPWM = 0;
				punch = 0;
				TIM2->CCR1 = pwm1;
				TIM2->CCR2 = pwm2;
			}
		}
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(left_proxim_ster_GPIO_Port, left_proxim_ster_Pin, 1);
	HAL_ADC_Start_IT(&hadc1); // tryb interrupt przetworka
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while(1)
	{

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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if(HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
	sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
	sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
	sClockSourceConfig.ClockFilter = 0;
	if(HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 319;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 199;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 6399;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 199;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if(HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 6399;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 39;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&huart2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	colSen_s3_Pin | left_proxim_ster_Pin | colSen_OE_Pin | colSen_s0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | colSen_s2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | H_Input1_Pin | H_Input2_Pin | H_Input3_Pin | H_Input4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(colSen_s1_GPIO_Port, colSen_s1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : colSen_s3_Pin left_proxim_ster_Pin colSen_OE_Pin colSen_s0_Pin */
	GPIO_InitStruct.Pin = colSen_s3_Pin | left_proxim_ster_Pin | colSen_OE_Pin | colSen_s0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : mouseClock_Pin clock_Pin PWM_counter_Pin */
	GPIO_InitStruct.Pin = mouseClock_Pin | clock_Pin | PWM_counter_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : mouseData_Pin PC9 */
	GPIO_InitStruct.Pin = mouseData_Pin | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin colSen_s2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | colSen_s2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : mClock_Pin */
	GPIO_InitStruct.Pin = mClock_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(mClock_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB4
	 H_Input1_Pin H_Input2_Pin H_Input3_Pin H_Input4_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | H_Input1_Pin | H_Input2_Pin | H_Input3_Pin | H_Input4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : mData_Pin */
	GPIO_InitStruct.Pin = mData_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(mData_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : colSen_s1_Pin */
	GPIO_InitStruct.Pin = colSen_s1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(colSen_s1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
