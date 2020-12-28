/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

/* GPIO definitions*/
#define STATUS_LED1_Pin GPIO_PIN_0
#define STATUS_LED1_GPIO_Port GPIOF
#define STAUS_LED2_Pin GPIO_PIN_1
#define STAUS_LED2_GPIO_Port GPIOF
#define VENTOLA_Pin GPIO_PIN_0
#define VENTOLA_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOA
#define OZONATORE_Pin GPIO_PIN_6
#define OZONATORE_GPIO_Port GPIOA
#define CTRL_LED_Pin GPIO_PIN_7
#define CTRL_LED_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* ADC definitions*/
#define CALIBRATION_TIME 180000
#define MAX_ADC_CH 1
#define NO_ERRORS_ADC 6000
#define DELTA 3
#define VREFINT_CAL_ADDR                0x1FFFF7BA  /* datasheet p. 19 */
#define VREFINT_CAL_ADDR_B                0x1FFFF7BB  /* datasheet p. 19 */
#define VREFINT_CAL ((uint16_t*) VREFINT_CAL_ADDR)
#define VREFINT_CAL_B ((uint16_t*) VREFINT_CAL_ADDR_B)
#define VREFINT 1.2

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
