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

