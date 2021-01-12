#include "main.h"

/* Variabili esportate*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;

int DatoADCpronto;
int DatiADC[MAX_ADC_CH+1];
int errors=0;
uint8_t mode=0;
int ritardo=ERRORS_ADC;
uint8_t calibrated=0;


// Counter decresaed each time SysTick_Handler(void) is called
uint16_t warmup_time=CALIBRATION_TIME;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

void OZONATOR_Init();
uint16_t getMode(void);
float average(uint16_t *array,uint16_t t);
void SENSOR_Warmup(uint16_t short_n);
void VENTOLA_run(uint32_t delay);
float ADC_AVERAGE_Value(uint32_t delta_sampling,uint8_t average_sample);
void OZONE_Pulse(uint32_t pulse_time);


/* Enter in the active mode
 * 1) Change air in the measure chamber
 * 2) Wait the sensor stabilization
 * 3) Sample Ozone concentration (averaging)
 * 4) Generate a ozone pulse that depends on the selected status
 * 5) repeat from 1)
 */
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_TIM3_Init();
	OZONATOR_Init();

	SENSOR_Warmup(SHORT_N);

	while(1){
		VENTOLA_run(VENTOLA_TIME); // Avvia la ventola della camera di misura per VENTOLA_TIME ms

		HAL_Delay(STABILIZATION_TIME); // Attendi STABILIZATION_TIME ms prima di leggere

		if(ADC_AVERAGE_Value(DELTA_SAMPLING,AVERAGE_SAMPLE) < OZONE_THR && mode > 0) { // se non c'e' sufficiente Ozono
			OZONE_Pulse(getMode()); // attiva generatore per un tempo determinato dallo stato del sistema
		}
	}
}

void OZONATOR_Init() {
	HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,0);
	HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,0);
	HAL_GPIO_WritePin(STATUS_LED2_GPIO_Port,STATUS_LED2_Pin,0);
	HAL_ADC_Start_IT(&hadc);
}


uint16_t getMode() {
	switch(mode) {
	case 0:
		return(0);
	case 1:
		return(500);
	case 2:
		return(1000);
	case 3:
		return(2000);
	case 4:
			return(4000);
	default:
		return(0);
	}
}

float average(uint16_t *array,uint16_t t) {
	int i;
	int sum=0;
	for(i=0;i<t;i++)
		sum += array[i];
	return sum/t;
}

void SENSOR_Warmup(uint16_t short_n) {

	uint16_t average_short[short_n];
	uint32_t short_time=0;
	float shortAvg,prevAvg;
	prevAvg=0;

	while (!calibrated) {
		//	  vref= *((uint16_t*)VREFINT_CAL_ADDR);

		if(DatoADCpronto) {
			DatoADCpronto=0;

			(short_time >= short_n) ? short_time=0 : short_time++ ;
			average_short[short_time]=DatiADC[0];
			shortAvg=average(average_short,short_n);

			//voltage1=DatiADC[0]*(1.23/DatiADC[1]); //Vrefind table datasheet 6.3.4

			if (DatiADC[0]>4090) {
				HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,1);
				errors++;
			} else if(shortAvg>prevAvg+DELTA) {
				HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,1);
				prevAvg=shortAvg;
				errors++;
			} else if(shortAvg<prevAvg-DELTA ){
				HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,1);
				prevAvg=shortAvg;
				errors++;
			} else {
				HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,0);
			}
			HAL_ADC_Start_IT(&hadc);
		}
	}
	HAL_ADC_Stop_IT(&hadc);
	HAL_GPIO_WritePin(STATUS_LED2_GPIO_Port,STATUS_LED2_Pin,1);
}


void VENTOLA_run(uint32_t delay){
	HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,1);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,0);
}

float ADC_AVERAGE_Value(uint32_t delta_sampling,uint8_t average_sample) {
	uint8_t i=0;
	uint16_t adc_value[average_sample];
	float avg;
	HAL_ADC_Start_IT(&hadc);
	HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,1);
	while(i<average_sample) {
		if(DatoADCpronto) {
			DatoADCpronto=0;
			adc_value[i++]=DatiADC[0];
			HAL_Delay(delta_sampling);
			HAL_ADC_Start_IT(&hadc);
		}
	}
	avg=average(adc_value,average_sample);
	HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port,STATUS_LED1_Pin,0);
	return(avg);
}

void OZONE_Pulse(uint32_t pulse_time) {
	HAL_GPIO_WritePin(OZONATORE_GPIO_Port,OZONATORE_Pin,1);
	HAL_Delay(pulse_time);
	HAL_GPIO_WritePin(OZONATORE_GPIO_Port,OZONATORE_Pin,0);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	ADC_ChannelConfTypeDef sConfig = {0};
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
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
	TIM_OC_InitTypeDef sConfigOC = {0};


	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 8000-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2000-1;
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
	HAL_TIM_MspPostInit(&htim3);

}


/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, STATUS_LED1_Pin|STATUS_LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(VENTOLA_GPIO_Port, VENTOLA_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(OZONATORE_GPIO_Port, OZONATORE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : STATUS_LED1_Pin STAUS_LED2_Pin */
	GPIO_InitStruct.Pin = STATUS_LED1_Pin|STATUS_LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : VENTOLA_Pin */
	GPIO_InitStruct.Pin = VENTOLA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VENTOLA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA7 */
	GPIO_InitStruct.Pin = OZONATORE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OZONATORE_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

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
