#ifndef TT_XPT2046_H
#define TT_XPT2046_H 133

#include "main.h"


#define XPT2046_CS_PORT       	GPIOB
#define XPT2046_CS_PIN        	GPIO_Pin_6
#define XPT2046_IRQ_PORT       	GPIOB
#define XPT2046_IRQ_PIN        	GPIO_Pin_7


#define XPT2046_CS_SET				XPT2046_CS_PORT->BSRRL=XPT2046_CS_PIN
#define XPT2046_CS_RESET			XPT2046_CS_PORT->BSRRH=XPT2046_CS_PIN
#define XPT2046_IRQ_READ			GPIO_ReadInputDataBit(XPT2046_IRQ_PORT, XPT2046_IRQ_PIN)

#define XPT2046_SPI						SPI1

void XPT2046_Init(void);
void XPT2046_GetCordinate(uint8_t *pX , uint8_t *pY);


#endif

