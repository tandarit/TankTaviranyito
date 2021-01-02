#ifndef __ADCDMA_H
#define __ADCDMA_H


#include "main.h"

#define ADC1_DR_Address    ((uint32_t)0x4001204C)
#define ADC_JOY_X_PIN		GPIO_Pin_0
#define ADC_JOY_Y_PIN		GPIO_Pin_1
#define ADC_LIPOL_PIN		GPIO_Pin_2
#define ADC_USB_PIN			GPIO_Pin_3
#define ADC_PORT	GPIOA

#define ANALOG_X_AXE					0
#define ANALOG_Y_AXE					1
#define	ANALOG_LIPO_BATTERY		2
#define ANALOG_CHARGING				3

extern __IO uint16_t ADCConvertedValue[4];

void ADCDMA_Init(void);

#endif 
