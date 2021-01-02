#include "ILI9341_HAL.h"

#define GET_STREAM_NUMBER_DMA1(stream)    (((uint32_t)(stream) - (uint32_t)DMA1_Stream0) / (0x18))
#define GET_STREAM_NUMBER_DMA2(stream)    (((uint32_t)(stream) - (uint32_t)DMA2_Stream0) / (0x18))


typedef struct {
	uint32_t TX_Channel;
	DMA_Stream_TypeDef* TX_Stream;
	uint32_t RX_Channel;
	DMA_Stream_TypeDef* RX_Stream;
	uint32_t Dummy32;
	uint16_t Dummy16;
} SPI_DMA_INT_t;

/* Offsets for bits */
const static uint8_t DMA_Flags_Bit_Pos[4] = {
	0, 6, 16, 22
};

/* Private DMA structure */
static DMA_InitTypeDef DMA_InitStruct;
static SPI_DMA_INT_t SPI2_DMA_INT = {ILI9341_DMA_TX_CHANNEL, ILI9341_DMA_TX_STREAM, ILI9341_DMA_RX_CHANNEL, ILI9341_DMA_RX_STREAM};

void ILI9341_HWInit(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	//DMA_InitTypeDef DMA_InitStruct;

	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	RCC_AHB1PeriphClockCmd(ILI9341_RCC_GPIO, ENABLE);
	RCC_APB1PeriphClockCmd(ILI9341_RCC_SPI, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);


	GPIO_InitStruct.GPIO_Mode 	= 	GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType 	= 	GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd	=	GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed 	= 	GPIO_Speed_100MHz;

	//RS pin init
	GPIO_InitStruct.GPIO_Pin 	= 	ILI9341_RS_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	//CS pin init
	GPIO_InitStruct.GPIO_Pin 	= 	ILI9341_CS_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	//RST pin init
	GPIO_InitStruct.GPIO_Pin 	= 	ILI9341_RESET_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	//CS high
	ILI9341_CS_SET;

	//Init SPI2 for ILI9341
	GPIO_PinAFConfig(ILI9341_PORT, ILI9341_AF_SCK_PIN, ILI9341_GPIO_AF);   // SPI1_CLK
	GPIO_PinAFConfig(ILI9341_PORT, ILI9341_AF_MISO_PIN, ILI9341_GPIO_AF);   // SPI1_MISO
	GPIO_PinAFConfig(ILI9341_PORT, ILI9341_AF_MOSI_PIN, ILI9341_GPIO_AF);   // SPI1_MOSI

	GPIO_InitStruct.GPIO_Mode 	= 	GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType 	= 	GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  	= 	GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed 	= 	GPIO_Speed_100MHz;

	//SPI SCK
	GPIO_InitStruct.GPIO_Pin = ILI9341_SCK_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	//SPI MOSI
	GPIO_InitStruct.GPIO_Pin = ILI9341_MOSI_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	//SPI MISO
	GPIO_InitStruct.GPIO_Pin = ILI9341_MISO_PIN;
	GPIO_Init(ILI9341_PORT, &GPIO_InitStruct);

	SPI_I2S_DeInit(ILI9341_SPI);
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Mode        				= 	SPI_Mode_Master;
	SPI_InitStruct.SPI_BaudRatePrescaler		= 	SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction   				= 	SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize 				= 	SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL 					= 	SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA 					= 	SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS 						= 	SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit  	        	= 	SPI_FirstBit_MSB;
	SPI_Init(ILI9341_SPI, &SPI_InitStruct);
	SPI_Cmd(ILI9341_SPI, ENABLE);


	//Init DMA for SPI2
	/* Set DMA options for TX stream */
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}

void SPI_DMA_InitWithStreamAndChannel(DMA_Stream_TypeDef* TX_Stream, uint32_t TX_Channel, DMA_Stream_TypeDef* RX_Stream, uint32_t RX_Channel) {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Set values */
	Settings->RX_Channel = RX_Channel;
	Settings->RX_Stream = RX_Stream;
	Settings->TX_Channel = TX_Channel;
	Settings->TX_Stream = TX_Stream;

	/* Init SPI */
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}

void SPI_DMA_Deinit() {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Deinit DMA Streams */
	DMA_DeInit(Settings->TX_Stream);
	DMA_DeInit(Settings->RX_Stream);
}

uint8_t SPI_DMA_Transmit(uint8_t* TX_Buffer, uint8_t* RX_Buffer, uint16_t count) {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Check if DMA available */
	if (
		Settings->RX_Stream->NDTR ||
		Settings->TX_Stream->NDTR ||
		(TX_Buffer == NULL && RX_Buffer == NULL)
	) {
		return 0;
	}

	/* Set dummy memory to default */
	Settings->Dummy16 = 0x00;

	/* Set memory size */
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	/* Set DMA peripheral address and count */
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI2->DR;
	DMA_InitStruct.DMA_BufferSize = count;

	/* Configure TX DMA */
	DMA_InitStruct.DMA_Channel = Settings->TX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	if (TX_Buffer != NULL) {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) TX_Buffer;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	} else {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &Settings->Dummy32;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
	}

	/* Deinit first TX stream */
	DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);

	/* Init TX stream */
	DMA_Init(Settings->TX_Stream, &DMA_InitStruct);

	/* Configure RX DMA */
	DMA_InitStruct.DMA_Channel = Settings->RX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;

	if (RX_Buffer != NULL) {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) RX_Buffer;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	} else {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &Settings->Dummy32;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
	}

	/* Deinit first RX stream */
	DMA_ClearFlag(Settings->RX_Stream, DMA_FLAG_ALL);

	/* Init RX stream */
	DMA_Init(Settings->RX_Stream, &DMA_InitStruct);

	/* Enable RX stream */
	Settings->RX_Stream->CR |= DMA_SxCR_EN;

	/* Enable TX stream */
	Settings->TX_Stream->CR |= DMA_SxCR_EN;

	/* Enable SPI RX & TX DMA */
	SPI2->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

	/* Return OK */
	return 1;
}

uint8_t SPI_DMA_TransmitHalfWord(uint16_t* TX_Buffer, uint16_t* RX_Buffer, uint32_t count) {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Check if DMA available */
	if (
		Settings->RX_Stream->NDTR ||
		Settings->TX_Stream->NDTR ||
		(TX_Buffer == NULL && RX_Buffer == NULL)
	) {
		return 0;
	}

	/* Set dummy memory to default */
	Settings->Dummy16 = 0x00;

	/* Set memory size */
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	/* Set DMA peripheral address and count */
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI2->DR;
	DMA_InitStruct.DMA_BufferSize = count;

	/* Configure TX DMA */
	DMA_InitStruct.DMA_Channel = Settings->TX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	if (TX_Buffer != NULL) {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) TX_Buffer;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	} else {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &Settings->Dummy32;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
	}

	/* Deinit first TX stream */
	DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);

	/* Init TX stream */
	DMA_Init(Settings->TX_Stream, &DMA_InitStruct);

	/* Configure RX DMA */
	DMA_InitStruct.DMA_Channel = Settings->RX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;

	if (RX_Buffer != NULL) {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) RX_Buffer;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	} else {
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &Settings->Dummy32;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
	}

	/* Deinit first RX stream */
	DMA_ClearFlag(Settings->RX_Stream, DMA_FLAG_ALL);

	/* Init RX stream */
	DMA_Init(Settings->RX_Stream, &DMA_InitStruct);

	/* Enable RX stream */
	Settings->RX_Stream->CR |= DMA_SxCR_EN;

	/* Enable TX stream */
	Settings->TX_Stream->CR |= DMA_SxCR_EN;

	/* Enable SPI RX & TX DMA */
	SPI2->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

	/* Return OK */
	return 1;
}

uint8_t SPI_DMA_SendByte(uint8_t value, uint16_t count) {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Check if DMA available */
	if (Settings->TX_Stream->NDTR) {
		return 0;
	}

	/* Set dummy memory to value we specify */
	Settings->Dummy32 = value;

	/* Set DMA peripheral address, number of bytes and disable memory increase pointer */
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI2->DR;
	DMA_InitStruct.DMA_BufferSize = count;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;

	/* Configure TX DMA */
	DMA_InitStruct.DMA_Channel = Settings->TX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	/* Set memory size */
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	/* Deinit first TX stream */
	DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);

	/* Init TX stream */
	DMA_Init(Settings->TX_Stream, &DMA_InitStruct);

	/* Enable TX stream */
	Settings->TX_Stream->CR |= DMA_SxCR_EN;

	/* Enable SPI TX DMA */
	SPI2->CR2 |= SPI_CR2_TXDMAEN;

	/* Return OK */
	return 1;
}

uint8_t SPI_DMA_SendHalfWord(uint16_t value, uint16_t count) {
	/* Get USART settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Check if DMA available */
	if (Settings->TX_Stream->NDTR) {
		return 0;
	}

	/* Set dummy memory to value we specify */
	Settings->Dummy16 = value;

	/* Set DMA peripheral address, number of bytes and disable memory increase pointer */
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI2->DR;
	DMA_InitStruct.DMA_BufferSize = count;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &Settings->Dummy16;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;

	/* Configure TX DMA */
	DMA_InitStruct.DMA_Channel = Settings->TX_Channel;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	/* Set memory size */
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	/* Deinit first TX stream */
	TT_DMA_ClearFlag(Settings->TX_Stream, DMA_FLAG_ALL);

	/* Init TX stream */
	DMA_Init(Settings->TX_Stream, &DMA_InitStruct);

	/* Enable TX stream */
	Settings->TX_Stream->CR |= DMA_SxCR_EN;

	/* Enable SPI TX DMA */
	SPI2->CR2 |= SPI_CR2_TXDMAEN;

	/* Return OK */
	return 1;
}

uint8_t SPI_DMA_Working() {
	/* Get SPI settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Check if TX or RX DMA are working */
	return (
		Settings->RX_Stream->NDTR || /*!< RX is working */
		Settings->TX_Stream->NDTR || /*!< TX is working */
		SPI_IS_BUSY(SPI2)            /*!< SPI is busy */
	);
}

DMA_Stream_TypeDef* SPI_DMA_GetStreamTX() {
	/* Return pointer to TX stream */
	return ILI9341_DMA_TX_STREAM;
}

DMA_Stream_TypeDef* SPI_DMA_GetStreamRX() {
	/* Return pointer to TX stream */
	return ILI9341_DMA_RX_STREAM;
}

void SPI_DMA_EnableInterrupts() {
	/* Get SPI settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Enable interrupts for TX and RX streams */
	TT_DMA_EnableInterrupts(Settings->TX_Stream);
	TT_DMA_EnableInterrupts(Settings->RX_Stream);
}

void SPI_DMA_DisableInterrupts() {
	/* Get SPI settings */
	SPI_DMA_INT_t* Settings = &SPI2_DMA_INT;

	/* Enable interrupts for TX and RX streams */
	DMA_DisableInterrupts(Settings->TX_Stream);
	DMA_DisableInterrupts(Settings->RX_Stream);
}


void TT_DMA_ClearFlags(DMA_Stream_TypeDef* DMA_Stream) {
	/* Clear all flags */
	TT_DMA_ClearFlag(DMA_Stream, DMA_FLAG_ALL);
}

void TT_DMA_ClearFlag(DMA_Stream_TypeDef* DMA_Stream, uint32_t flag) {
	uint32_t location;
	uint32_t stream_number;

	/* Check stream value */
	if (DMA_Stream < DMA2_Stream0) {
		location = (uint32_t)&DMA1->LIFCR;
		stream_number = GET_STREAM_NUMBER_DMA1(DMA_Stream);
	} else {
		location = (uint32_t)&DMA2->LIFCR;
		stream_number = GET_STREAM_NUMBER_DMA2(DMA_Stream);
	}

	/* Get register offset */
	if (stream_number >= 4) {
		/* High registers for DMA clear */
		location += 4;

		/* Do offset for high DMA registers */
		stream_number -= 4;
	}

	/* Clear flags */
	*(__IO uint32_t *)location = (flag & DMA_FLAG_ALL) << DMA_Flags_Bit_Pos[stream_number];
}

SPI_DataSize_t SPI_SetDataSize(SPI_TypeDef* SPIx, SPI_DataSize_t DataSize) {
	SPI_DataSize_t status = (SPIx->CR1 & SPI_CR1_DFF) ? SPI_DataSize_16bit : SPI_DataSize_8bit;

	/* Disable SPI first */
	SPIx->CR1 &= ~SPI_CR1_SPE;

	/* Set proper value */
	if (DataSize == SPI_DataSize_16bit) {
		/* Set bit for frame */
		SPIx->CR1 |= SPI_CR1_DFF;
	} else {
		/* Clear bit for frame */
		SPIx->CR1 &= ~SPI_CR1_DFF;
	}

	/* Enable SPI back */
	SPIx->CR1 |= SPI_CR1_SPE;

	/* Return status */
	return status;
}

