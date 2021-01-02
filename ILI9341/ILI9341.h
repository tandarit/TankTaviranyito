#ifndef ILI9341_H
#define ILI9341 140


#include "ILI9341_HAL.h"
#include "ILI9341_Fonts.h"

/* Colors */
#define ILI9341_COLOR_WHITE			0xFFFF
#define ILI9341_COLOR_BLACK			0x0000
#define ILI9341_COLOR_RED       	0xF800
#define ILI9341_COLOR_GREEN			0x07E0
#define ILI9341_COLOR_GREEN2		0xB723
#define ILI9341_COLOR_BLUE			0x001F
#define ILI9341_COLOR_BLUE2			0x051D
#define ILI9341_COLOR_YELLOW		0xFFE0
#define ILI9341_COLOR_ORANGE		0xFBE4
#define ILI9341_COLOR_CYAN			0x07FF
#define ILI9341_COLOR_MAGENTA		0xA254
#define ILI9341_COLOR_GRAY			0x7BEF
#define ILI9341_COLOR_BROWN			0xBBCA

/* Transparent background, only for strings and chars */
#define ILI9341_TRANSPARENT			0x80000000

/**
 * @brief  Possible orientations for LCD
 */

typedef enum {
	ILI9341_Landscape,
	ILI9341_Portrait
} ILI9341_Orientation;

/**
 * @brief  LCD options
 * @note   Used private
 */
typedef struct {
	uint16_t width;
	uint16_t height;
	ILI9341_Orientation orientation; // 1 = portrait; 0 = landscape
} ILI931_Options_t;

typedef enum {
	ILI9341_Orientation_Portrait_1,  /*!< Portrait orientation mode 1 */
	ILI9341_Orientation_Portrait_2,  /*!< Portrait orientation mode 2 */
	ILI9341_Orientation_Landscape_1, /*!< Landscape orientation mode 1 */
	ILI9341_Orientation_Landscape_2  /*!< Landscape orientation mode 2 */
} ILI9341_Orientation_t;

typedef struct {
     uint16_t *data;
     uint16_t width;
     uint16_t height;
} tImage;

#define ILI9341_WIDTH        240
#define ILI9341_HEIGHT       320
#define ILI9341_PIXEL        76800

#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC					0x36
#define ILI9341_PIXEL_FORMAT		0x3A
#define ILI9341_WDB					0x51
#define ILI9341_WCD					0x53
#define ILI9341_RGB_INTERFACE		0xB0
#define ILI9341_FRC					0xB1
#define ILI9341_BPC					0xB5
#define ILI9341_DFC					0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC					0xF7

/* Pin functions */
uint16_t ILI9341_x;
uint16_t ILI9341_y;
ILI931_Options_t ILI9341_Opts;

void ILI9341_InitLCD();
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint32_t color);
void ILI9341_Fill(uint32_t color);
void ILI9341_INT_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ILI9341_Rotate(ILI9341_Orientation_t orientation);
void ILI9341_Putc(uint16_t x, uint16_t y, char c, FontDef_t* font, uint32_t foreground, uint32_t background);
void ILI9341_Puts(uint16_t x, uint16_t y, char* str, FontDef_t *font, uint32_t foreground, uint32_t background);
void ILI9341_GetStringSize(char* str, FontDef_t* font, uint16_t* width, uint16_t* height);
void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);
void ILI9341_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);
void ILI9341_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);
void ILI9341_DisplayOn(void);
void ILI9341_DrawImage(uint16_t x, uint16_t y, tImage img);
void ILI9341_DisplayOff(void);
void ILI9341_Blacklight_Init(void);
void ILI9341_SetBlacklight(uint8_t dutycycle);

#endif
