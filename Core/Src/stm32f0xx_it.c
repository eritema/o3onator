#include "main.h"
#include "stm32f0xx_it.h"

extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim3;

extern int stato;
extern int ritardo;
extern int accensioni;
extern uint16_t warmup_time;
extern uint8_t calibrated;
extern uint8_t mode;

extern int DatiADC[];
extern int DatoADCpronto;
extern int errors;

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
	O3_LOW=500,
	O3_MED=1000,
	O3_HI=2000
};

enum STATI_RELE ventola,ozono;
enum STATI_LED led0;

uint32_t o3;
int counting=0;



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
	      ritardo=ERRORS_ADC; //
	  }
  }

}

void EXTI2_3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
  * @brief This function handles ADC interrupt.
  */
void ADC1_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc);
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin==BUTTON_Pin && calibrated) {
	  if(mode>=3) mode=0;
	  else mode++;
	  if(mode>0) {
		  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  		  if(mode==1) {
	  			htim3.Instance -> CCR1 = O3_LOW;
	  			counting=1;
	  		  } else if(mode==2) {
	  			  htim3.Instance -> CCR1 = O3_MED;
	  			  counting=1;
	  		  } else if(mode==3) {
	  			  htim3.Instance -> CCR1 = O3_HI;
	  			  counting=1;
	  		  }
	  } else {
		  	  htim3.Instance -> CCR1 = 0;
	  		  counting=0;
	  		  mode=0;
	  }
  }
}
