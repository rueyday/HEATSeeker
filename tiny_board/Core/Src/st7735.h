#ifndef __ST7735_H__
#define __ST7735_H__

#include "main.h"
#include "fonts.h"
//#include "stm32f4xx_hal.h"
//----------------------FIX----------------------------
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 200

#define ST7735_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define ST7735_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define ST7735_DC_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define ST7735_DC_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define ST7735_RST_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define ST7735_RST_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)

// Colors
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
} ST7735_TypeDef;

void ST7735_Init(SPI_HandleTypeDef *hspi);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, const uint8_t color[2]);
void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7735_DrawBlock(int x, int y, int size, uint16_t color);
//void ST7735_DrawFullGrid(void);
char ST7735_WriteChar(char ch, FontDef Font, uint16_t color);
char ST7735_WriteString(const char* str, FontDef Font, uint16_t color);
void ST7735_SetCursor(uint8_t x, uint8_t y);
#endif
