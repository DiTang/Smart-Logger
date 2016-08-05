/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char nl,chr;
float temp,curr;

typedef struct{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t sub;
	uint8_t interval;
}Timer;
Timer myTimer;

#define SENDMSG(__R__) HAL_UART_Transmit(&huart1,(uint8_t*)__R__, COUNTOF(__R__) ,500);
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Private variables ---------------------------------------------------------*/
char RBUF;
uint16_t i;
uint32_t abuf[3];
uint32_t tempT;

/*State machine variable*/
typedef enum{
	welcome = 0U,
	logging,
	monitor,
	setting,
}osState;

typedef struct{
	osState state;
	volatile uint8_t mflg;
	volatile uint8_t deci_sec_flg;
	volatile uint8_t sec_flg;
	volatile uint8_t timer_flg;
}StateMachine;
StateMachine mySM;

typedef struct{
	volatile float TcalA[5];
	uint8_t latch;
	uint8_t empty;
}SettingData;
SettingData myDat;

typedef struct{
	char info[50];
	char result[50];
	char error[50];
	char names[50];
}MSG;
MSG myMSG;


volatile osState state = welcome;

#define FLASH_USER_START_ADDR   0x8008000   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     0x800f100   /* End @ of user Flash area */
uint32_t FirstPage = 0, NbOfPages = 0, Address = 0;
uint32_t PageError = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM21_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
osState transState(osState,uint8_t);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
	state = transState(state,RBUF);
	HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
	for(i=0;i<999;i++);
	HAL_TIM_PWM_Stop(&htim21,TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&RBUF, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*if (sec==1)
		HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
	else if (sec==2)
		HAL_TIM_PWM_Stop(&htim21,TIM_CHANNEL_1);
	else{
		if (!mflg){
		HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
		HAL_Delay(2);
		HAL_TIM_PWM_Stop(&htim21,TIM_CHANNEL_1);
		}
	}*/

	myTimer.sub++;	
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	
	if (myTimer.sub % 50 == 0) mySM.deci_sec_flg = 1;
	
	if(myTimer.sub == 100){
		myTimer.sub = 0;
		myTimer.sec++;
		mySM.sec_flg = 1;
		if(myTimer.sec == 60){
			myTimer.sec = 0;
			myTimer.min++;
			if(myTimer.min == 60){
				myTimer.min = 0;
				myTimer.hour++;
			}
		}
	}
		//HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*HAL_TIM_Base_Stop_IT(&htim6);
	HAL_TIM_Base_Stop(&htim6);
	HAL_ADC_Stop_DMA(&hadc);
	state = welcome;
	*/
	mySM.mflg = 1 - mySM.mflg;
	HAL_FLASH_Unlock();
	NbOfPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR + 1) >> 7;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages = NbOfPages;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	myDat.TcalA[0] = .1;
	myDat.TcalA[1] = .2;
	myDat.TcalA[2] = .3;
	myDat.TcalA[3] = pow(4,2.2);
	myDat.TcalA[4] = sin(3.1416);
	memcpy((SettingData*)FLASH_USER_START_ADDR, &myDat, sizeof(SettingData));
	memcpy(&myDat, (SettingData*)FLASH_USER_START_ADDR, sizeof(SettingData));
	HAL_FLASH_Lock();
}

void num2str(uint16_t num)
{
  uint16_t i, j;
  uint8_t	res;
  j = 1000;
	for(i = 4; i > 0; i--){
    res = (num / j) % 10 + '0';
		HAL_UART_Transmit(&huart1, &res, 1 ,100);
		j = j / 10;
  }
}

osState transState(osState state,uint8_t cmd){
	osState res = welcome;
 switch(state){
		case welcome:
			switch(cmd){
				case '1':{
						res = logging;
						break;
					}
				case '2':{
						res = monitor;
						break;
					}
				case '3':{
						res = setting;
						break;
					}
				default: break;
			}
			break;
		case logging:
			switch(cmd){
				case '0':{
						res = welcome;
						break;
					}
				default: break;
			}
			break;
		case monitor:
			switch(cmd){
				case '0':{
						res = welcome;
						break;
					}
				default: break;
			}
			break;
		case setting:
			switch(cmd){
				case '0':{
						res = welcome;
						break;
					}
				default: break;
			}
			break;
		default: break;
	}
	/*switch(cmd){
		case '1':{PRINTS("start logging",newline)};break;
		case '2':{PRINTS("monitoring",newline)};break;
		case '3':{PRINTS("setting",newline)};break;
		case '4':{PRINTS("calibrating",newline)};break;
		default: break;
	}*/
	return res;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM21_Init();

  /* USER CODE BEGIN 2 */
	sprintf(myMSG.info,"\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r");
	SENDMSG(myMSG.info)
	sprintf(myMSG.info,"***********************************************\n\r");
	SENDMSG(myMSG.info)
	sprintf(myMSG.info,"Welcome to the EYEQ4 chip validation logger!\n\r");
	SENDMSG(myMSG.info)

	
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
	HAL_TIM_Base_Start(&htim21);
	//HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
	
	
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(state == welcome){
			myTimer.hour = 0;
			myTimer.min = 0;
			myTimer.sec = 0;
			myTimer.sub = 0;
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim6);
			HAL_ADC_Stop_DMA(&hadc);
			sprintf(myMSG.info,"1.Logger\n\r2.Meter\n\r3.Settings\n\r            ");
			SENDMSG(myMSG.info)
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&RBUF, 1);
			while(state == welcome){
			}
		}
		if(state == logging){
			HAL_TIM_Base_Start(&htim6);
			HAL_TIM_Base_Start_IT(&htim6);
			while(state == logging){				
				if (mySM.sec_flg){
					mySM.sec_flg = 0;
					HAL_ADC_Start_DMA(&hadc,abuf,3);
					tempT = HAL_GetTick();
					while(tempT + 50> HAL_GetTick());
					HAL_ADC_Stop_DMA(&hadc);
					temp = ((float)abuf[0] / 32 + 50);
					curr = ((float)(abuf[2]-(float)abuf[1])/20);
					sprintf(myMSG.result,"\r%2d:%2d:%2d, %3.3f, %3.3f, %3.3f         ", myTimer.hour, myTimer.min, myTimer.sec, curr, temp, pow(temp,1.25));
					SENDMSG(myMSG.result)
					sprintf(myMSG.names,"\r\n  time     current     temperature");
					SENDMSG(myMSG.names)
					if (!mySM.mflg){
						HAL_TIM_PWM_Start(&htim21,TIM_CHANNEL_1);
						HAL_Delay(2);
						HAL_TIM_PWM_Stop(&htim21,TIM_CHANNEL_1);
					}
				}
			}
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim6);
		}
		if(state == monitor){
			sprintf(myMSG.info,"I1   I2   T1    T1               \n");
			SENDMSG(myMSG.info)
			HAL_TIM_Base_Start(&htim6);
			HAL_TIM_Base_Start_IT(&htim6);
			HAL_UART_Receive_IT(&huart1, (uint8_t *)&RBUF, 1);
			while(state == monitor){
				if (mySM.deci_sec_flg){
					mySM.deci_sec_flg = 0;
					HAL_ADC_Start_DMA(&hadc,abuf,3);
					tempT = HAL_GetTick();
					while(tempT + 50> HAL_GetTick());
					HAL_ADC_Stop_DMA(&hadc);
					temp = ((float)abuf[0] / 32 + 50);
					curr = ((float)(abuf[2]-(float)abuf[1])/20);
					sprintf(myMSG.result,"\r%3.3f, %3.3f, %3.3f                     ", curr, temp, pow(temp,1.25));
					SENDMSG(myMSG.result)
				}
			}
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2097 / 100;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 21;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 100;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim21);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
