#include "st7735.h"
#include "fonts.h"
//#include "stm32l4xx_hal.h"

static SPI_HandleTypeDef *ST7735_hspi;
static ST7735_TypeDef screen;

static void ST7735_WriteCommand(uint8_t cmd)
{
    ST7735_DC_LOW();
    ST7735_CS_LOW();
    HAL_SPI_Transmit(ST7735_hspi, &cmd, 1, HAL_MAX_DELAY);
    ST7735_CS_HIGH();
}

static void ST7735_WriteData(uint8_t data)
{
    ST7735_DC_HIGH();
    ST7735_CS_LOW();
    HAL_SPI_Transmit(ST7735_hspi, &data, 1, HAL_MAX_DELAY);
    ST7735_CS_HIGH();
}

static void ST7735_Reset(void)
{
    ST7735_RST_LOW();
    HAL_Delay(50);
    ST7735_RST_HIGH();
    HAL_Delay(50);
}

void ST7735_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    ST7735_WriteCommand(0x2A); // Column addr
    ST7735_WriteData(0x00);
    ST7735_WriteData(x0);
    ST7735_WriteData(0x00);
    ST7735_WriteData(x1);

    ST7735_WriteCommand(0x2B); // Row addr
    ST7735_WriteData(0x00);
    ST7735_WriteData(y0);
    ST7735_WriteData(0x00);
    ST7735_WriteData(y1);

    ST7735_WriteCommand(0x2C); // Write RAM
}

void ST7735_Init(SPI_HandleTypeDef *hspi)
{
    ST7735_hspi = hspi;

    ST7735_Reset();

    // Sleep Out
    ST7735_WriteCommand(0x11);
    HAL_Delay(120);

    // Color Mode = 16-bit
    ST7735_WriteCommand(0x3A);
    ST7735_WriteData(0x05);

    // Display ON
    ST7735_WriteCommand(0x29);
    HAL_Delay(20);

    ST7735_FillScreen(BLACK);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, const uint8_t color[2])
{
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT)
        return;

    ST7735_SetAddrWindow(x, y, x+1, y+1);

    ST7735_DC_HIGH();
    ST7735_CS_LOW();
    HAL_SPI_Transmit(ST7735_hspi, (uint8_t*)color, 2, HAL_MAX_DELAY);
    ST7735_CS_HIGH();
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_SetAddrWindow(0, 0, ST7735_WIDTH-1, ST7735_HEIGHT-1);

    uint8_t data[] = { color >> 8, color & 0xFF };

    ST7735_DC_HIGH();
    ST7735_CS_LOW();

    for (uint32_t i = 0; i < ST7735_WIDTH * ST7735_HEIGHT; i++)
        HAL_SPI_Transmit(ST7735_hspi, data, 2, HAL_MAX_DELAY);

    ST7735_CS_HIGH();
}

void ST7735_DrawBlock(int x, int y, int size, uint16_t color) {
	for (int j = 0; j < size; j++) {
	  for (int i = 0; i < size; i++) {
		  ST7735_DrawPixel(x + i, y + j, color);
	  }
	}
}



//Font Def is the size
char ST7735_WriteChar(char ch, FontDef Font, uint16_t color) {
//	ST7735_SetAddrWindow(95, 0, 127, 63);
	//un-hardcode this
    uint32_t i, b, j;
    uint8_t data[] = { color >> 8, color & 0xFF};
    // Check remaining space on current line
    if (ST7735_WIDTH <= (screen.CurrentX + Font.width) ||
    		ST7735_HEIGHT <= (screen.CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }
    for (i = 0; i < Font.height; i++)
    {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++)
        {
            if ((b << j) & 0x8000)
            {
            	ST7735_DrawPixel(screen.CurrentX + j, (screen.CurrentY + i), data);
            }
            else
            {
            	ST7735_DrawPixel(screen.CurrentX + j, (screen.CurrentY + i), BLACK);
            }
        }
    }
    screen.CurrentX += Font.width;

    // Return written char for validation
    return ch;
}

char ST7735_WriteString(const char* str, FontDef Font, uint16_t color)
{
    // Write until null-byte
    while (*str)
    {
        if (ST7735_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        str++;
    }
    return *str;
}

void ST7735_SetCursor(uint8_t x, uint8_t y)
{
    screen.CurrentX = x;
    screen.CurrentY = y;
}
