#include "oled_driver.h"


// Screenbuffer
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// Screen object
static OLED_t Screen;


//
//  Send a byte to the command register
//
static uint8_t OLED_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    return HAL_I2C_Mem_Write(hi2c, OLED_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

//
//  Initialize the oled screen
//
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c)
{
    // Wait for the screen to boot
    HAL_Delay(100);
    int status = 0;

    // Init LCD
    printf("Initializing OLED Screen...\r\n");
    status += OLED_WriteCommand(hi2c, 0xAE);   // Display off
    status += OLED_WriteCommand(hi2c, 0x20);   // Set Memory Addressing Mode
    status += OLED_WriteCommand(hi2c, 0x10);   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    status += OLED_WriteCommand(hi2c, 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7
    status += OLED_WriteCommand(hi2c, 0xC8);   // Set COM Output Scan Direction
    status += OLED_WriteCommand(hi2c, 0x00);   // Set low column address
    status += OLED_WriteCommand(hi2c, 0x10);   // Set high column address
    status += OLED_WriteCommand(hi2c, 0x40);   // Set start line address
    status += OLED_WriteCommand(hi2c, 0x81);   // set contrast control register
    status += OLED_WriteCommand(hi2c, 0xFF);
    status += OLED_WriteCommand(hi2c, 0xA1);   // Set segment re-map 0 to 127
    status += OLED_WriteCommand(hi2c, 0xA6);   // Set normal display

    status += OLED_WriteCommand(hi2c, 0xA8);   // Set multiplex ratio(1 to 64)
    status += OLED_WriteCommand(hi2c, OLED_HEIGHT - 1);

    status += OLED_WriteCommand(hi2c, 0xA4);   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    status += OLED_WriteCommand(hi2c, 0xD3);   // Set display offset
    status += OLED_WriteCommand(hi2c, 0x00);   // No offset

    status += OLED_WriteCommand(hi2c, 0xD5);   // Set display clock divide ratio/oscillator frequency
    status += OLED_WriteCommand(hi2c, 0xF0);   // Set divide ratio
    status += OLED_WriteCommand(hi2c, 0xD9);   // Set pre-charge period
    status += OLED_WriteCommand(hi2c, 0x22);

    status += OLED_WriteCommand(hi2c, 0xDA);   // Set com pins hardware configuration
    status += OLED_WriteCommand(hi2c, OLED_COM_LR_REMAP << 5 | OLED_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02);

    status += OLED_WriteCommand(hi2c, 0xDB);   // Set vcomh
    status += OLED_WriteCommand(hi2c, 0x20);   // 0x20,0.77xVcc
    status += OLED_WriteCommand(hi2c, 0x8D);   // Set DC-DC enable
    status += OLED_WriteCommand(hi2c, 0x14);   //

    status += OLED_WriteCommand(hi2c, 0xAF);   // Turn on OLED panel
    if (status != 0) {
        return 1;
    }

    // Clear screen
    OLED_Fill(Black);

    // Flush buffer to screen
    OLED_UpdateScreen(hi2c);
    // Set default values for screen object
    Screen.CurrentX = 0;
    Screen.CurrentY = 0;

    Screen.Initialized = 1;
    return 0;
}

//
//  Fill the whole screen with the given color
//
void OLED_Fill(OLED_COLOR color)
{
    // Fill screenbuffer with a constant value (color)
    uint32_t i;

    for(i = 0; i < sizeof(OLED_Buffer); i++)
    {
        OLED_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

//
//  Write the screenbuffer with changed to the screen
//
void OLED_UpdateScreen(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        OLED_WriteCommand(hi2c, 0xB0 + i);
        OLED_WriteCommand(hi2c, 0x00);
        OLED_WriteCommand(hi2c, 0x10);

        HAL_I2C_Mem_Write(hi2c, OLED_I2C_ADDR, 0x40, 1, &OLED_Buffer[OLED_WIDTH * i], OLED_WIDTH, 100);
    }
}

//
//  Draw one pixel in the screenbuffer
//  X => X Coordinate
//  Y => Y Coordinate
//  color => Pixel color
//
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if (Screen.Inverted)
    {
        color = (OLED_COLOR)!color;
    }

    // Draw in the correct color
    if (color == White)
    {
    	OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    }
    else
    {
    	OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}


//
//  Draw 1 char to the screen buffer
//  ch      => Character to write
//  Font    => Font to use
//  color   => Black or White
//
char OLED_WriteChar(char ch, FontDef Font, OLED_COLOR color)
{
    uint32_t i, b, j;

    // Check remaining space on current line
    if (OLED_WIDTH <= (Screen.CurrentX + Font.width) ||
    		OLED_HEIGHT <= (Screen.CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }

    // Translate font to screenbuffer
    for (i = 0; i < Font.height; i++)
    {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++)
        {
            if ((b << j) & 0x8000)
            {
            	OLED_DrawPixel(Screen.CurrentX + j, (Screen.CurrentY + i), (OLED_COLOR) color);
            }
            else
            {
            	OLED_DrawPixel(Screen.CurrentX + j, (Screen.CurrentY + i), (OLED_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    Screen.CurrentX += Font.width;

    // Return written char for validation
    return ch;
}

//
//  Write full string to screenbuffer
//
char OLED_WriteString(const char* str, FontDef Font, OLED_COLOR color)
{
    // Write until null-byte
    while (*str)
    {
        if (OLED_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        // Next char
        str++;
    }
    // Everything ok
    return *str;
}

//
//  Invert background/foreground colors
//
void OLED_InvertColors(void)
{
    Screen.Inverted = !Screen.Inverted;
}

//
//  Set cursor position
//
void OLED_SetCursor(uint8_t x, uint8_t y)
{
    Screen.CurrentX = x;
    Screen.CurrentY = y;
}
