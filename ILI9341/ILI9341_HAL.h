#ifndef ILI9341_HAL_H
#define ILI9341_HAL 130

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

#define NULL ((void *)0)

#define ILI9341_SPI			SPI2
#define	ILI9341_PORT		GPIOB
#define ILI9341_GPIO_AF		GPIO_AF_SPI2
#define ILI9341_RS_PIN		GPIO_Pin_10
#define ILI9341_RESET_PIN	GPIO_Pin_11
#define ILI9341_CS_PIN		GPIO_Pin_12
#define ILI9341_SCK_PIN		GPIO_Pin_13
#define ILI9341_MISO_PIN	GPIO_Pin_14
#define ILI9341_MOSI_PIN	GPIO_Pin_15
#define ILI9341_AF_SCK_PIN	GPIO_PinSource13
#define ILI9341_AF_MISO_PIN	GPIO_PinSource14
#define ILI9341_AF_MOSI_PIN	GPIO_PinSource15

#define ILI9341_RCC_GPIO	RCC_AHB1Periph_GPIOB
#define ILI9341_RCC_SPI		RCC_APB1Periph_SPI2


#define ILI9341_RST_SET				GPIO_SetBits(ILI9341_PORT, ILI9341_RESET_PIN)
#define ILI9341_RST_RESET			GPIO_ResetBits(ILI9341_PORT, ILI9341_RESET_PIN)
#define ILI9341_CS_SET				GPIO_SetBits(ILI9341_PORT, ILI9341_CS_PIN)
#define ILI9341_CS_RESET			GPIO_ResetBits(ILI9341_PORT, ILI9341_CS_PIN)
#define ILI9341_RS_SET				GPIO_SetBits(ILI9341_PORT, ILI9341_RS_PIN)
#define ILI9341_RS_RESET			GPIO_ResetBits(ILI9341_PORT, ILI9341_RS_PIN)


#define ILI9341_DMA_TX_STREAM    DMA1_Stream4
#define ILI9341_DMA_TX_CHANNEL   DMA_Channel_0

#define ILI9341_DMA_RX_STREAM    DMA1_Stream3
#define ILI9341_DMA_RX_CHANNEL   DMA_Channel_0


typedef enum {
	SPI_DataSize_8bit	=	SPI_DataSize_8b, /*!< SPI in 8-bits mode */
	SPI_DataSize_16bit	=	SPI_DataSize_16b /*!< SPI in 16-bits mode */
} SPI_DataSize_t;
/**
 * @brief  DMA macros for interrupt flags
 */
#define DMA_FLAG_TCIF    ((uint32_t)0x00000020) /*!< DMA stream transfer complete */
#define DMA_FLAG_HTIF    ((uint32_t)0x00000010) /*!< DMA stream half transfer complete */
#define DMA_FLAG_TEIF    ((uint32_t)0x00000008) /*!< DMA stream transfer error */
#define DMA_FLAG_DMEIF   ((uint32_t)0x00000004) /*!< DMA stream direct mode error */
#define DMA_FLAG_FEIF    ((uint32_t)0x00000001) /*!< DMA stream FIFO error */
#define DMA_FLAG_ALL     ((uint32_t)0x0000003D) /*!< DMA stream all flags */

/* DMA1 preemption priority */
#define DMA1_NVIC_PREEMPTION_PRIORITY   0x01
#define DMA2_NVIC_PREEMPTION_PRIORITY   0x01

#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))
#define SPI_CHECK_ENABLED(SPIx)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}
#define SPI_CHECK_ENABLED_RESP(SPIx, val)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return (val);}

static __INLINE uint8_t SPI_Send(uint8_t data) {
	/* Check if SPI is enabled */
	SPI_CHECK_ENABLED_RESP(ILI9341_SPI, 0);

	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT(ILI9341_SPI);

	/* Fill output buffer with data */
	ILI9341_SPI->DR = data;

	/* Wait for transmission to complete */
	SPI_WAIT(ILI9341_SPI);

	/* Return data from buffer */
	return ILI9341_SPI->DR;
}


void ILI9341_HWInit(void);
void SPI_DMA_InitWithStreamAndChannel(DMA_Stream_TypeDef* TX_Stream, uint32_t TX_Channel, DMA_Stream_TypeDef* RX_Stream, uint32_t RX_Channel);
void SPI_DMA_Deinit();
uint8_t SPI_DMA_Transmit(uint8_t* TX_Buffer, uint8_t* RX_Buffer, uint16_t count);
uint8_t SPI_DMA_TransmitHalfWord(uint16_t* TX_Buffer, uint16_t* RX_Buffer, uint32_t count);
uint8_t SPI_DMA_SendByte(uint8_t value, uint16_t count);
uint8_t SPI_DMA_SendHalfWord(uint16_t value, uint16_t count);
uint8_t SPI_DMA_Working();
DMA_Stream_TypeDef* SPI_DMA_GetStreamTX();
DMA_Stream_TypeDef* SPI_DMA_GetStreamRX();
void SPI_DMA_EnableInterrupts();
void SPI_DMA_DisableInterrupts();

SPI_DataSize_t SPI_SetDataSize(SPI_TypeDef* SPIx, SPI_DataSize_t DataSize);



#endif
