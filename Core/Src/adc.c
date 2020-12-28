/*
 * adc.c
 *
 *  Created on: Nov 14, 2020
 *      Author: raf
 */


#include "adc.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"
#include "stdint.h"



uint8_t  indiceConversioniADC=0;

//inserire i canali  connessi da convertire;  MAX_INDEX_ADC_CH è l'indice massimo valido
uint32_t CanaliADC[MAX_ADC_CH+1]={ADC_CHANNEL_1,ADC_CHANNEL_VREFINT};
//nella prima posizione mettere il primo canale da convertire

ADC_ChannelConfTypeDef sConfigApp = {ADC_CHANNEL_1,1,ADC_SAMPLETIME_239CYCLES_5};

//call back di fine conversione canale
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	DatiADC[indiceConversioniADC]=HAL_ADC_GetValue(hadc);
/*    if (indiceConversioniADC==0) {
    	sConfigApp.Channel = CanaliADC[1];
    	indiceConversioniADC++;
    } else {
    	sConfigApp.Channel = CanaliADC[0];
    	indiceConversioniADC=0;
    }

//	sConfigApp.Channel = CanaliADC[indiceConversioniADC];

	if (HAL_ADC_ConfigChannel(hadc, &sConfigApp) != HAL_OK)
		Error_Handler();

	if(indiceConversioniADC) {
		HAL_ADC_Start_IT(hadc);

	} else {
		DatoADCpronto=1;
	}*/
	//DatiADC[0]=HAL_ADC_GetValue(hadc);
	DatoADCpronto=1;
}
