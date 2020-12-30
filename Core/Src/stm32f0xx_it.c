/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include <adc.h>
#include "main.h"
#include "stm32f0xx_it.h"
/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;

extern int stato;
extern int ritardo;
extern int accensioni;
extern uint16_t warmup_time;
extern uint8_t calibrated;


int GO_BOTTONE=1;

enum STATI_LED{
	ON_LED=1,
	OFF_LED=0
};

enum STATI_RELE{
	ON=0,
	OFF=1
};

enum O3_VALUES{
	O3_LOW=100,
	O3_MED=300,
	O3_HI=1000
};
enum STATI_RELE ventola,ozono;
enum STATI_LED led0;

uint32_t o3;
int counting=0;
uint8_t mode;

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  if (warmup_time) { // wait first CALIBRATION_TIME seconds before entering in calibration mode
	  warmup_time--;
  } else if(!--ritardo) { // no errors time is NO_ERRORS_ADC sec
	  if (!errors)
		  calibrated=1;
	  else {
		  errors=0;			// reset error counter
	      ritardo=NO_ERRORS_ADC; //
	  }
  }
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles ADC interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);
  if (GPIO_Pin==BUTTON_Pin) {
	  if(mode>=3) mode=0;
	  else mode++;
	  if(mode>0) {
		  	  HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,1);
		  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  		  //led1=ON_LED;
	  		  //ozono=ON;
	  		  //HAL_GPIO_WritePin(OzoneCtrl_GPIO_Port,OzoneCtrl_Pin,led1);
	  		  if(mode==1) {

	  			htim3.Instance -> CCR1 = O3_LOW;
//	  			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//	  			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,O3_LOW);
//	  			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  			counting=1;
	  		  } else if(mode==2) {
	  			  htim3.Instance -> CCR1 = O3_MED;
//	  			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,O3_MED);
//	  		      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  			  counting=1;
	  		  } else if(mode==3) {
	  			  htim3.Instance -> CCR1 = O3_HI;
//	  			  HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
//	  			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,O3_HI);
//	  		      HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

	  			  counting=1;
	  		  }
	  } else {
		  	  HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,0);
		  	  htim3.Instance -> CCR1 = 0;
	  		  //led1=OFF_LED;
	  		  //ozono=OFF;
	  		  //HAL_GPIO_WritePin(OzoneCtrl_GPIO_Port,OzoneCtrl_Pin,led1);
	  		  //HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
	  		  counting=0;
	  		  mode=0;
	  }
  }
  /*if (GPIO_Pin==BUTTON_Pin) {
	  GO_BOTTONE=0;
	  if (ventola==OFF) {
		ventola=ON;
		led0=ON_LED;
		ozono=OFF;
		led1=OFF_LED;
		counting=0;
		mode=0;
	    HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,led0);
	    HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,ventola);
	  }
	  else {
		ventola=OFF;
		ozono=OFF;
		led0=OFF_LED;
		led1=OFF_LED;
		counting=0;
		mode=0;
		HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,led0);
	    HAL_GPIO_WritePin(VENTOLA_GPIO_Port,VENTOLA_Pin,ventola);
	    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
	  }
	  HAL_GPIO_WritePin(OzoneCtrl_GPIO_Port,OzoneCtrl_Pin,led1);
  } else if ((GPIO_Pin==MODE_Pin)&&(ventola==ON)&&(GO_BOTTONE)) {
	  GO_BOTTONE=0;
	  if(mode>=3) mode=0;
	  else mode++;
	  if(mode>0) {
		  led1=ON_LED;
		  ozono=ON;
		  HAL_GPIO_WritePin(OzoneCtrl_GPIO_Port,OzoneCtrl_Pin,led1);
		  if(mode==1) {
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,O3_LOW);
			HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

			  counting=1;
		  } else if(mode==2) {
			  HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
		      HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,O3_MED);
			  counting=1;
		  } else if(mode==3) {
			  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
		      HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
			  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,O3_HI);
			  counting=1;
		  }
	  } else {
		  led1=OFF_LED;
		  ozono=OFF;
		  HAL_GPIO_WritePin(OzoneCtrl_GPIO_Port,OzoneCtrl_Pin,led1);
		  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
		  counting=0;
		  mode=0;
	  }
  }*/
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
