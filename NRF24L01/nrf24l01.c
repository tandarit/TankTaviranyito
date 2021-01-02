#include "nrf24l01.h"

static NRF24L01_t NRF24L01_Struct;

void NRF24L01_HWInit(void) {	
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	//EXTI_InitTypeDef   EXTI_InitStructure;
	
	#ifdef  STM32F10X_MD
	RCC_APB2PeriphClockCmd(NRF24L01_RCC_GPIO | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN | NRF24L01_CSN_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF24L01_PORT, &GPIO_InitStructure);	
	
	//IRQ setting active low in case of pin receive 
	GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStructure);
	
	NRF24L01_RCC_SPI_ENABLE;
		
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_SCK | NRF24L01_GPIO_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF24L01_PORT, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(NRF24L01_PORT, &GPIO_InitStructure);
		
	SPI_InitStructure.SPI_Mode        			= SPI_Mode_Master;
	SPI_InitStructure.SPI_BaudRatePrescaler		= SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_Direction   			= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize 				= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 					= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 					= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 					= SPI_NSS_Soft;    
	SPI_InitStructure.SPI_FirstBit  	        = SPI_FirstBit_MSB;
	SPI_Init(NRF24L01_SPI, &SPI_InitStructure);
	SPI_Cmd(NRF24L01_SPI, ENABLE);	
	#endif
	
	#ifdef  STM32L1XX_MD
	RCC_AHBPeriphClockCmd(NRF24L01_RCC_GPIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN | NRF24L01_CSN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	NRF24L01_RCC_SPI_ENABLE;
	
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_SCK, NRF24L01_GPIO_AF);   // SPI1_CLK
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_MISO, NRF24L01_GPIO_AF);   // SPI1_MISO
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_MOSI, NRF24L01_GPIO_AF);   // SPI1_MOSI
	
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	
	
	//SPI SCK
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_SCK;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	//SPI MOSI
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_MOSI;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	//SPI MISO
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_MISO;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	SPI_I2S_DeInit(NRF24L01_SPI);
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Mode        			= SPI_Mode_Master;
	SPI_InitStructure.SPI_BaudRatePrescaler		= SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_Direction   			= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize 				= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 					= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 					= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 					= SPI_NSS_Soft;    
	SPI_InitStructure.SPI_FirstBit  	        = SPI_FirstBit_MSB;
	SPI_Init(NRF24L01_SPI, &SPI_InitStructure);
	SPI_Cmd(NRF24L01_SPI, ENABLE);		
	#endif
	
	#ifdef STM32F40_41xxx
	RCC_AHB1PeriphClockCmd(NRF24L01_RCC_GPIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN | NRF24L01_CSN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStructure);

	NRF24L01_RCC_SPI_ENABLE;
	
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_SCK, NRF24L01_GPIO_AF);   // SPI1_CLK
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_MISO, NRF24L01_GPIO_AF);   // SPI1_MISO
	GPIO_PinAFConfig(NRF24L01_AF_PORT, NRF24L01_SPI_MOSI, NRF24L01_GPIO_AF);   // SPI1_MOSI
	
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
	
	//SPI SCK
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_SCK;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	//SPI MOSI
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_MOSI;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	//SPI MISO
	GPIO_InitStructure.GPIO_Pin = NRF24L01_GPIO_MISO;
	GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);
	
	SPI_I2S_DeInit(NRF24L01_SPI);
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Mode        				= SPI_Mode_Master;
	SPI_InitStructure.SPI_BaudRatePrescaler		= SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_Direction   				= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize 						= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 								= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 								= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 								= SPI_NSS_Soft;    
	SPI_InitStructure.SPI_FirstBit  	        = SPI_FirstBit_MSB;
	SPI_Init(NRF24L01_SPI, &SPI_InitStructure);
	SPI_Cmd(NRF24L01_SPI, ENABLE);
	
	#endif
	
	/* CSN high = disable SPI */
	NRF24L01_CSN_HIGH;
	
	/* CE low = disable TX/RX */
	NRF24L01_CE_LOW;
	
}

uint8_t NRF24L01_Init(uint8_t channel, uint8_t payload_size) {	
	/* Max payload is 32bytes */
	if (payload_size > 32) {
		payload_size = 32;
	}
	
	/* Fill structure */
	NRF24L01_Struct.Channel = !channel; /* Set channel to some different value for TM_NRF24L01_SetChannel() function */
	NRF24L01_Struct.PayloadSize = payload_size;
	NRF24L01_Struct.OutPwr = NRF24L01_OutputPower_0dBm;
	NRF24L01_Struct.DataRate = NRF24L01_DataRate_1M;
	
	/* Reset nRF24L01+ to power on registers values */
	NRF24L01_SoftwareReset();
	
	/* Channel select */
	NRF24L01_SetChannel(channel);
	
	/* Set pipeline to max possible 32 bytes */
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, NRF24L01_Struct.PayloadSize); // Auto-ACK pipe
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, NRF24L01_Struct.PayloadSize); // Data payload pipe
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, NRF24L01_Struct.PayloadSize);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, NRF24L01_Struct.PayloadSize);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, NRF24L01_Struct.PayloadSize);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, NRF24L01_Struct.PayloadSize);
	
	/* Set RF settings (1mbps, output power) */
	NRF24L01_SetRF(NRF24L01_Struct.DataRate, NRF24L01_Struct.OutPwr);
	
	/* Config register */
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG);
	
	/* Enable auto-acknowledgment for all pipes */
	NRF24L01_WriteRegister(NRF24L01_REG_EN_AA, 0x3F);
	
	/* Enable RX addresses */
	NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 0x3F);

	/* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, 0x4F);
	
	/* Dynamic length configurations: No dynamic length */
	NRF24L01_WriteRegister(NRF24L01_REG_DYNPD, (0 << NRF24L01_DPL_P0) | (0 << NRF24L01_DPL_P1) | (0 << NRF24L01_DPL_P2) | (0 << NRF24L01_DPL_P3) | (0 << NRF24L01_DPL_P4) | (0 << NRF24L01_DPL_P5));
	
	/* Clear FIFOs */
	NRF24L01_FLUSH_TX();
	NRF24L01_FLUSH_RX();
	
	/* Clear interrupts */
	NRF24L01_CLEAR_INTERRUPTS;
	
	/* Go to RX mode */
	NRF24L01_PowerUpRx();
	
	/* Return OK */
	return 1;
}

void NRF24L01_FLUSH_TX(void) {
	//uint8_t valueMISOFLUSH;
	NRF24L01_CSN_LOW; 
	NRF24L01_SPI->DR=NRF24L01_FLUSH_TX_MASK;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	NRF24L01_CSN_HIGH;
}

void NRF24L01_FLUSH_RX(void) {
	//uint8_t valueMISOFLUSH;
	NRF24L01_CSN_LOW; 
	NRF24L01_SPI->DR=NRF24L01_FLUSH_RX_MASK;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	NRF24L01_CSN_HIGH;
}


void NRF24L01_SetMyAddress(uint8_t *adr) {
	NRF24L01_CE_LOW;
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, adr, 5);
	NRF24L01_CE_HIGH;
}

void NRF24L01_SetTxAddress(uint8_t *adr) {
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, adr, 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, adr, 5);
}

void NRF24L01_WriteBit(uint8_t reg, uint8_t bit, uint8_t value) {
	uint8_t tmp;
	/* Read register */
	tmp = NRF24L01_ReadRegister(reg);
	/* Make operation */
	if (value) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}
	/* Write back */
	NRF24L01_WriteRegister(reg, tmp);
}

uint8_t TM_NRF24L01_ReadBit(uint8_t reg, uint8_t bit) {
	uint8_t tmp;
	tmp = NRF24L01_ReadRegister(reg);
	if (!NRF24L01_CHECK_BIT(tmp, bit)) {
		return 0;
	}
	return 1;
}

uint8_t NRF24L01_ReadRegister(uint8_t reg) {
	uint8_t value;
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_READ_REGISTER_MASK(reg);
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	value = NRF24L01_SPI->DR;
	NRF24L01_SPI->DR=0xFF;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	value = NRF24L01_SPI->DR;
	NRF24L01_CSN_HIGH;	
	return value;
}

void NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t count) {
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_READ_REGISTER_MASK(reg);
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	SPI_ReadMulti(NRF24L01_SPI, data, NRF24L01_NOP_MASK, count);
	NRF24L01_CSN_HIGH;
}

void NRF24L01_WriteRegister(uint8_t reg, uint8_t value) {
	//uint8_t valueMISO;
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_WRITE_REGISTER_MASK(reg);
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	NRF24L01_SPI->DR=value;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	NRF24L01_CSN_HIGH;
}

void NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count) {
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_WRITE_REGISTER_MASK(reg);
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	SPI_WriteMulti(NRF24L01_SPI, data, count);
	NRF24L01_CSN_HIGH;
}

void NRF24L01_PowerUpTx(void) {
	NRF24L01_CLEAR_INTERRUPTS;
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | (0 << NRF24L01_PRIM_RX) | (1 << NRF24L01_PWR_UP));
}

void NRF24L01_PowerUpRx(void) {
	/* Disable RX/TX mode */
	NRF24L01_CE_LOW;
	/* Clear RX buffer */
	NRF24L01_FLUSH_RX();
	/* Clear interrupts */
	NRF24L01_CLEAR_INTERRUPTS;
	/* Setup RX mode */
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
	/* Start listening */
	NRF24L01_CE_HIGH;
}

void NRF24L01_PowerDown(void) {
	NRF24L01_CE_LOW;
	NRF24L01_WriteBit(NRF24L01_REG_CONFIG, NRF24L01_PWR_UP, Bit_RESET);
}

void NRF24L01_Transmit(uint8_t *data) {
	uint8_t i;
	NRF24L01_CE_LOW;
	NRF24L01_PowerUpTx();
	NRF24L01_FLUSH_TX();
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_W_TX_PAYLOAD_MASK;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
	while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	for (i = 0; i < NRF24L01_Struct.PayloadSize; i++) {
		NRF24L01_SPI->DR = data[i];
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
		(void)NRF24L01_SPI->DR;
	}
	NRF24L01_CSN_HIGH;	
	NRF24L01_CE_HIGH;	
	Delay(15);
	NRF24L01_CE_LOW;
}

void NRF24L01_GetData(uint8_t* data) {
	uint8_t i;
	
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_R_RX_PAYLOAD_MASK;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	(void)NRF24L01_SPI->DR;
	for (i = 0; i < NRF24L01_Struct.PayloadSize; i++) {
		NRF24L01_SPI->DR = 0xFF;
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
    data[i] = NRF24L01_SPI->DR;
	}
	NRF24L01_CSN_HIGH;
		
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

uint8_t NRF24L01_DataReady(void) {
	uint8_t status = NRF24L01_GetStatus();
	
	if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR)) {
		return 1;
	}
	return !NRF24L01_RxFifoEmpty();
}

uint8_t NRF24L01_RxFifoEmpty(void) {
	uint8_t reg = NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS);
	return NRF24L01_CHECK_BIT(reg, NRF24L01_RX_EMPTY);
}

uint8_t NRF24L01_GetStatus(void) {
	uint8_t status;	
	NRF24L01_CSN_LOW;
	NRF24L01_SPI->DR=NRF24L01_NOP_MASK;
	while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
	status=NRF24L01_SPI->DR;
	NRF24L01_CSN_HIGH;	
	return status;
}

NRF24L01_Transmit_Status_t NRF24L01_GetTransmissionStatus(void) {
	uint8_t status = NRF24L01_GetStatus();
	if (NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS)) {
		return NRF24L01_Transmit_Status_Ok;
	} else if (NRF24L01_CHECK_BIT(status, NRF24L01_MAX_RT)) {
		return NRF24L01_Transmit_Status_Lost;
	}
	return NRF24L01_Transmit_Status_Sending;
}

void NRF24L01_SoftwareReset(void) {
	uint8_t data[5];
	
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, 		NRF24L01_REG_DEFAULT_VAL_CONFIG);
	NRF24L01_WriteRegister(NRF24L01_REG_EN_AA,		NRF24L01_REG_DEFAULT_VAL_EN_AA);
	NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, 	NRF24L01_REG_DEFAULT_VAL_EN_RXADDR);
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_AW, 	NRF24L01_REG_DEFAULT_VAL_SETUP_AW);
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, 	NRF24L01_REG_DEFAULT_VAL_SETUP_RETR);
	NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, 		NRF24L01_REG_DEFAULT_VAL_RF_CH);
	NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, 	NRF24L01_REG_DEFAULT_VAL_RF_SETUP);
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, 		NRF24L01_REG_DEFAULT_VAL_STATUS);
	NRF24L01_WriteRegister(NRF24L01_REG_OBSERVE_TX, 	NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX);
	NRF24L01_WriteRegister(NRF24L01_REG_RPD, 		NRF24L01_REG_DEFAULT_VAL_RPD);
	
	//P0
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4;
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, data, 5);
	
	//P1
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4;
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, data, 5);
	
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P2, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P3, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P4, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P5, 	NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5);
	
	//TX
	data[0] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4;
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, data, 5);
	
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P0);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P1);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P2);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P3);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P4);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, 	NRF24L01_REG_DEFAULT_VAL_RX_PW_P5);
	NRF24L01_WriteRegister(NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS);
	NRF24L01_WriteRegister(NRF24L01_REG_DYNPD, 		NRF24L01_REG_DEFAULT_VAL_DYNPD);
	NRF24L01_WriteRegister(NRF24L01_REG_FEATURE, 	NRF24L01_REG_DEFAULT_VAL_FEATURE);
}

uint8_t NRF24L01_GetRetransmissionsCount(void) {
	/* Low 4 bits */
	return NRF24L01_ReadRegister(NRF24L01_REG_OBSERVE_TX) & 0x0F;
}

void NRF24L01_SetChannel(uint8_t channel) {
	if (channel <= 125 && channel != NRF24L01_Struct.Channel) {
		/* Store new channel setting */
		NRF24L01_Struct.Channel = channel;
		/* Write channel */
		NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, channel);
	}
}

void NRF24L01_SetRF(NRF24L01_DataRate_t DataRate, NRF24L01_OutputPower_t OutPwr) {
	uint8_t tmp = 0;
	NRF24L01_Struct.DataRate = DataRate;
	NRF24L01_Struct.OutPwr = OutPwr;
	
	if (DataRate == NRF24L01_DataRate_2M) {
		tmp |= 1 << NRF24L01_RF_DR_HIGH;
	} else if (DataRate == NRF24L01_DataRate_250k) {
		tmp |= 1 << NRF24L01_RF_DR_LOW;
	}
	/* If 1Mbps, all bits set to 0 */
	
	if (OutPwr == NRF24L01_OutputPower_0dBm) {
		tmp |= 3 << NRF24L01_RF_PWR;
	} else if (OutPwr == NRF24L01_OutputPower_M6dBm) {
		tmp |= 2 << NRF24L01_RF_PWR;
	} else if (OutPwr == NRF24L01_OutputPower_M12dBm) {
		tmp |= 1 << NRF24L01_RF_PWR;
	}
	
	NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, tmp);
}

void SPI_WriteMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint32_t count) {
	uint32_t i;	
	//uint8_t valueMISO;
	NRF24L01_CSN_LOW;
	for (i = 0; i < count; i++) {
		NRF24L01_SPI->DR=dataOut[i];
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
  while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
  while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
		(void)NRF24L01_SPI->DR;	
	}
	NRF24L01_CSN_HIGH;
}

void SPI_ReadMulti(SPI_TypeDef* SPIx, uint8_t* dataIn, uint8_t dummy, uint32_t count) {
	uint32_t i;
	
	NRF24L01_CSN_LOW;
	for (i = 0; i < count; i++) {
		NRF24L01_SPI->DR=dummy;
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
		dataIn[i] = NRF24L01_SPI->DR;		
	}
	NRF24L01_CSN_HIGH;	
}


void SPI_SendMulti(SPI_TypeDef* SPIx, uint8_t* dataOut, uint8_t* dataIn, uint32_t count) {
	uint32_t i;
	
	for (i = 0; i < count; i++) {
		NRF24L01_SPI->DR = dataOut[i];
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_TXE));
		while (!(NRF24L01_SPI->SR & SPI_I2S_FLAG_RXNE));
		while (NRF24L01_SPI->SR & SPI_I2S_FLAG_BSY);
    dataIn[i] = NRF24L01_SPI->DR;
	}
}




