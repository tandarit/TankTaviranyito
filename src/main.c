#include "main.h"

uint8_t MyAddress[] = {
    0xE7,
    0xE7,
    0xE7,
    0xE7,
    0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
    0x7E,
    0x7E,
    0x7E,
    0x7E,
    0x7E
};
bool receiveTimeout;
uint8_t dataOut[4], dataIn[4], retransmittedPackages;
NRF24L01_Transmit_Status_t transmissionStatus;
__IO uint16_t ADCConvertedValue[4];
uint8_t transmisionCounter;

extern bool batteryStatusTimeout;
extern StatusDisplay_t sd;
extern MessageDisplay_t md;

int main() {
	receiveTimeout = false;
	batteryStatusTimeout = true;

	if (SysTick_Config(SystemCoreClock / 1000000))
	{
		while (1);
	}
	
	ADCDMA_Init();

	XPT2046_Init();

	ILI9341_InitLCD();
	ILI9341_Blacklight_Init();
	ILI9341_SetBlacklight(10);
	ILI9341_Rotate(ILI9341_Orientation_Portrait_1);
  	ILI9341_Fill(ILI9341_COLOR_BLACK);
	ILI9341_Puts(35,309, "Tandari Elektronika GmbH.", &Font_7x10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);

	NRF24L01_HWInit();
	NRF24L01_Init(100, 4);
	NRF24L01_SetRF(NRF24L01_DataRate_1M, NRF24L01_OutputPower_0dBm);
	NRF24L01_SetMyAddress(MyAddress);
	NRF24L01_SetTxAddress(TxAddress);

	InitJoyButton();

	NVIC_Setting();

	TIM4_Init();

	TIM5_Init();

	StatusBarInit();

	WorkPanelInit();


	while(1) {

				StatusBarControl();

				dataOut[0]=(uint8_t)(ADCConvertedValue[0]>>4);
				dataOut[1]=(uint8_t)(ADCConvertedValue[1]>>4);

				receiveTimeout=false;

				NRF24L01_Transmit(dataOut);
				while ((GPIO_ReadInputDataBit(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN)==1));
				transmissionStatus=NRF24L01_GetTransmissionStatus();
				//retransmittedPackages=NRF24L01_GetRetransmissionsCount();

				if(NRF24L01_GetTransmissionStatus() == NRF24L01_Transmit_Status_Ok) {

					md.TankFoundMessage=1;
					md.TankNotFoundMessage=0;

				}
				else {
					md.TankFoundMessage=0;
					md.TankNotFoundMessage=1;

				}
				

				WorkPanelControl();
	}
}

void TIM4_Init(void) {
	//350ms idozito a kommunikacios ciklusokhoz
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=13999;
	TIM_TimeBaseStructure.TIM_Prescaler=999;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

//Altalanos idozito (batteryStatus????)
void TIM5_Init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=221999;
	TIM_TimeBaseStructure.TIM_Prescaler=999;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}


void NVIC_Setting(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Timer4 IT konfigur�l�sa */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	/* Timer5 IT konfigur�l�sa */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	//JOYBUTTON konfiguralas
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}

void TIM4_IRQHandler(void) {
	if ( TIM_GetITStatus(TIM4, TIM_IT_Update)) {

		receiveTimeout = true;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}


void TIM5_IRQHandler(void) {
	if ( TIM_GetITStatus(TIM5, TIM_IT_Update)) {

		 batteryStatusTimeout = true;
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

//Joystic button interrupt
void EXTI9_5_IRQHandler(void) {



		EXTI_ClearITPendingBit(EXTI_Line8);

}

