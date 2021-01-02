#ifndef __MAIN_H
#define __MAIN_H


#include "stm32f4xx_conf.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "ADCDMA.h"
#include "ILI9341.h"
#include "nrf24l01.h"
#include "XPT2046.h"
#include "GUI.h"

extern tImage ILI9341_Battery100;
extern tImage ILI9341_Battery80;
extern tImage ILI9341_Battery60;
extern tImage ILI9341_Battery40;
extern tImage ILI9341_Battery20;
extern tImage ILI9341_NRFConnect;
extern tImage ILI9341_NRFNotConnected;

extern uint8_t statusBarTimeout;


void TIM4_Init(void);
void TIM5_Init(void);
void NVIC_Setting(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void EXTI9_5_IRQHandler(void);

#endif 
