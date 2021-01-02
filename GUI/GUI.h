#ifndef __GUI_H
#define __GUI_H

#include "main.h"

#define JOYBUTTON_PORT						GPIOB
#define JOYBUTTON_PIN							GPIO_Pin_8
#define JOYBUTTON_RCC							RCC_APB2Periph_SYSCFG
#define JOYBUTTON_EXTI_PORT 			EXTI_PortSourceGPIOB
#define JOYBUTTON_EXTI_PINSOURCE	EXTI_PinSource8
#define JOYBUTTON_EXTI_LINE				EXTI_Line8

typedef struct {
	uint8_t NRFConnect:1;  
	uint8_t NRFNotConnected:1;
	uint8_t ChargingInfo:1;
	uint8_t NOTChargingInfo:1;
	uint8_t LowPowerInfo:1;
	uint8_t Battery20:1;
	uint8_t Battery40:1;
	uint8_t Battery60:1;
	uint8_t Battery80:1;
	uint8_t Battery100:1;
	uint8_t BatteryPercent:1;
	uint8_t TextControlData:1;
	uint8_t TankFoundText:1;
	uint8_t TankNotFoundText:1;
	
} StatusDisplay_t;


typedef struct {
	
	uint8_t TankFoundMessage:1;
	uint8_t TankNotFoundMessage:1;
} MessageDisplay_t;



void InitJoyButton(void);
void StatusBarInit(void);
void StatusBarControl(void);
void WorkPanelInit(void);
void WorkPanelControl(void);
void BatteryStatePrint(void);
void IntToStringPercent(uint8_t number, char* percentString);
#endif 
