#include "XPT2046.h"


void XPT2046_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	//T_CS init
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = XPT2046_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(XPT2046_CS_PORT, &GPIO_InitStructure);	
	
	XPT2046_CS_SET;
	
	//T_IRQ init
	GPIO_InitStructure.GPIO_Pin = XPT2046_IRQ_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(XPT2046_IRQ_PORT, &GPIO_InitStructure);	
}




void XPT2046_GetCordinate(uint8_t *pX , uint8_t *pY) {
	uint8_t i;
	
	XPT2046_CS_RESET;
	
	XPT2046_SPI->DR=0x91;
	while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (XPT2046_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)XPT2046_SPI->DR;
	
	for (i = 0; i < 2; i++) {
		XPT2046_SPI->DR = 0x00;
		while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (XPT2046_SPI->SR & SPI_I2S_FLAG_BSY);
    pX[i] = XPT2046_SPI->DR;
	}
	
	XPT2046_SPI->DR=0xD1;
	while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (XPT2046_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)XPT2046_SPI->DR;
	
	for (i = 0; i < 2; i++) {
		XPT2046_SPI->DR = 0x00;
		while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(XPT2046_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (XPT2046_SPI->SR & SPI_I2S_FLAG_BSY);
    pY[i] = XPT2046_SPI->DR;
	}
	
	XPT2046_CS_SET;

}


	/*		
int XPT2046_GetAverageCoordinates( int * pX , int * pY , int nSamples )
{
	uint8_t nRead = 0;
	uint8_t xAcc = 0 , yAcc = 0;
	int x , y;

	while ( nRead < nSamples ) {
		if ( !xpt2046GetCoordinates( &x , &y ) ) {
			break;
		}
		xAcc += x;
		yAcc += y;
		nRead ++;
	}

	if ( nRead == 0 ) {
		return 0;
	}
	*pX = xAcc / nRead;
	*pY = yAcc / nRead;
	return 1;
}




uint8_t TT_XPT2046_Read(uint16_t* xy) {
	uint16_t z1, z2, tmpH, tmpL;
	
  XPT2046_CS_RESET;
 
  //Check if touch screen is pressed.
	TT_SPI_Send(XPT2046_SPI, 0xB0);      // Z1
  tmpH = (TT_SPI_Send(XPT2046_SPI, 0x00) << 5);
  tmpL = (TT_SPI_Send(XPT2046_SPI, 0x00) >> 3);
  z1 = tmpH | tmpL;
 
  TT_SPI_Send(XPT2046_SPI, 0xC0); // Z2
  tmpH = (TT_SPI_Send(XPT2046_SPI, 0x00) << 5);
  tmpL = (TT_SPI_Send(XPT2046_SPI, 0x00) >> 3);
  z2 = tmpH | tmpL;
 
  if((z2 - z1) < 3500){ //If the touch screen is pressed, read the X,Y  coordinates from XPT2046.
    TT_SPI_Send(XPT2046_SPI, 0xD0); // X
    tmpH = (TT_SPI_Send(XPT2046_SPI, 0x00) << 5);
		tmpL = (TT_SPI_Send(XPT2046_SPI, 0x00) >> 3);
    xy[0] =  tmpH | tmpL;
   
    TT_SPI_Send(XPT2046_SPI, 0x90); // Y
    tmpH = (TT_SPI_Send(XPT2046_SPI, 0x00) << 5);
		tmpL = (TT_SPI_Send(XPT2046_SPI, 0x00) >> 3);
    xy[1] =  tmpH | tmpL;
		
    XPT2046_CS_SET;
    return 0;
  }
  XPT2046_CS_SET;
	return 1;
}
*/

