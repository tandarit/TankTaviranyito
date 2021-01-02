#include "GUI.h"

StatusDisplay_t sd;
MessageDisplay_t md;
extern NRF24L01_Transmit_Status_t transmissionStatus;
extern bool receiveTimeout;

uint8_t uiBatteryPercent, uiOldBatteryPercent;
uint8_t uiBatteryDifferent;
uint16_t uiBatteryVoltage;
bool batteryStatusTimeout;
char cPercentString[5];


void InitJoyButton() {
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(JOYBUTTON_RCC, ENABLE);    /* EXTI vonal kiválasztás */    
	
	GPIO_InitStructure.GPIO_Pin = JOYBUTTON_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(JOYBUTTON_PORT, &GPIO_InitStructure);
	
		
	SYSCFG_EXTILineConfig(JOYBUTTON_EXTI_PORT, JOYBUTTON_EXTI_PINSOURCE);    /* EXTI vonal beállítás */    
	EXTI_InitStructure.EXTI_Line = JOYBUTTON_EXTI_LINE;    
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;    
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    
	EXTI_Init(&EXTI_InitStructure); 

}


void StatusBarInit() {
	uiBatteryDifferent=0;
	
	sd.NRFNotConnected			=	0;
	sd.NRFConnect						=	0;
	sd.Battery100						=	0;
	sd.Battery80						=	0;
	sd.Battery60						=	0;
	sd.Battery40						=	0;
	sd.Battery20						=	0;
	sd.ChargingInfo					=	0;
	sd.NOTChargingInfo			=	0;
	sd.LowPowerInfo					=	0;
	
	uiBatteryVoltage=ADCConvertedValue[ANALOG_LIPO_BATTERY]-0x911;
	uiBatteryPercent=(uint8_t)(((double)uiBatteryVoltage)/5.21);

	IntToStringPercent(uiBatteryPercent, cPercentString);
	ILI9341_Puts(154, 0, cPercentString, &Font_11x18,ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	uiOldBatteryPercent = uiBatteryPercent;

	ILI9341_DrawLine(0, 20, 239, 20, ILI9341_COLOR_WHITE);
}

void StatusBarControl() {	
	//NRF24 Connection icon
	//NRF24L01_Transmit_Status_Ok;
	if((transmissionStatus==NRF24L01_Transmit_Status_Ok) && (receiveTimeout==false) && (sd.NRFConnect==0)) {
		sd.NRFConnect=1;
		sd.NRFNotConnected=0;
		//ILI9341_DrawImage(0,0, ILI9341_NRFConnect);
	}
	if((transmissionStatus==NRF24L01_Transmit_Status_Lost) || ((receiveTimeout==true) && (sd.NRFNotConnected==0))) {
		sd.NRFConnect=0;
		sd.NRFNotConnected=1;
		//ILI9341_DrawImage(0,0, ILI9341_NRFNotConnected);
	}
	
	
	//Charging felirat
	if((ADCConvertedValue[ANALOG_CHARGING]>0x0800) && (sd.ChargingInfo==0)) {
			sd.ChargingInfo=1;
			sd.NOTChargingInfo=0;
			sd.LowPowerInfo=0;
			ILI9341_Puts(33,0, "Charging...", &Font_11x18,ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}
	if((ADCConvertedValue[ANALOG_CHARGING]<0x0800) && (sd.NOTChargingInfo==0)) {
		sd.ChargingInfo=0;
		sd.NOTChargingInfo=1;
		sd.LowPowerInfo=0;
		ILI9341_Puts(33,0, "           ", &Font_11x18,ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
	}	
	//Low power felirat
	if((ADCConvertedValue[ANALOG_LIPO_BATTERY]<0x0999) && (sd.LowPowerInfo==0)) {
		sd.LowPowerInfo=1;
		sd.NOTChargingInfo=0;
		sd.ChargingInfo=0;
		ILI9341_Puts(33,0, "Low Power", &Font_11x18,ILI9341_COLOR_RED, ILI9341_COLOR_BLACK);
	}
	
	BatteryStatePrint();
}



void WorkPanelInit() {	
	md.TankFoundMessage			=	0;
	md.TankNotFoundMessage	=	0;
	
	ILI9341_Puts(32,25, "Tank Remote", &Font_16x26, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
	ILI9341_Puts(65,52, "Control", &Font_16x26, ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK);
	
}

void WorkPanelControl() {
		
	if((md.TankFoundMessage==1) && (sd.TankFoundText==0)) {
		ILI9341_INT_Fill(0,100,239,224, ILI9341_COLOR_BLACK);
		ILI9341_Puts(0,120, "  Tank found", &Font_16x26, ILI9341_COLOR_GREEN2, ILI9341_COLOR_BLACK);
		sd.TankFoundText=1;
		sd.TankNotFoundText=0;
	}
	
	if((md.TankNotFoundMessage==1) && (sd.TankNotFoundText==0)) {
		ILI9341_INT_Fill(0,100,239,224, ILI9341_COLOR_BLACK);
		ILI9341_Puts(0,120, "   Tank not        found", &Font_16x26, ILI9341_COLOR_GREEN2, ILI9341_COLOR_BLACK);
		sd.TankNotFoundText=1;
		sd.TankFoundText=0;
	}
	
}

void IntToStringPercent(uint8_t number, char* percentString) {
	if(number == 100)
			percentString = "100%";
	else	{
			percentString[0] = (char)((number / 10) + 0x30);
			percentString[1] = (char)((number % 10) + 0x30);
			percentString[2] = '%';
			percentString[3] = 0;
	}

}

void BatteryStatePrint() {
			if(batteryStatusTimeout == true) {
				batteryStatusTimeout = false;

				uiBatteryVoltage=ADCConvertedValue[ANALOG_LIPO_BATTERY]-0x911;
				uiBatteryPercent=(uint8_t)(((double)uiBatteryVoltage)/5.21);

				if(uiBatteryPercent < uiOldBatteryPercent) {
					IntToStringPercent(uiBatteryPercent, cPercentString);
					ILI9341_Puts(154, 0, cPercentString, &Font_11x18,ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
				}
				uiOldBatteryPercent = uiBatteryPercent;
			}
			/*
			if(uiBatteryVoltage>416) {
				ILI9341_DrawImage(199,0, ILI9341_Battery100);
			}
			else if((uiBatteryVoltage<400) && (uiBatteryVoltage>312)) {
				ILI9341_DrawImage(199,0, ILI9341_Battery80);
			}
			else if((uiBatteryVoltage<300) && (uiBatteryVoltage>208)) {
				ILI9341_DrawImage(199,0, ILI9341_Battery60);
			}
			else if((uiBatteryVoltage<200) && (uiBatteryVoltage>104)) {
				ILI9341_DrawImage(199,0, ILI9341_Battery40);
			}
			else if((uiBatteryVoltage<100)) {
				ILI9341_DrawImage(199,0, ILI9341_Battery20);
			}
*/
}
