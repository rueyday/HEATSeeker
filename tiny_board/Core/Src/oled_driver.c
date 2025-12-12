/*
 * oled_driver.c
 *
 *  Created on: Nov 22, 2025
 */

#include "oled_driver.h"

static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static OLED_t Screen;

uint8_t OLED_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t command) {
    return HAL_I2C_Mem_Write(hi2c, OLED_I2C_ADDR, 0x00, 1, &command, 1, 10);
}

uint8_t OLED_Init(I2C_HandleTypeDef *hi2c)
{

    HAL_Delay(100);

    printf("Initializing OLED Screen...\r\n");

    OLED_WriteCommand(hi2c, 0xAE);   // Display off

    //initial settings configuration
    OLED_WriteCommand(hi2c, 0x20);   //Memory Addressing Mode
    OLED_WriteCommand(hi2c, 0x10);   //Vertical Addressing
    OLED_WriteCommand(hi2c, 0xB0);   //Page Start Address for Page Addressing Mode

    OLED_WriteCommand(hi2c, 0x00);   //low column address
    OLED_WriteCommand(hi2c, 0x10);   //high column address

    OLED_WriteCommand(hi2c, 0xD5);   //display clock divide ratio/oscillator frequency
    OLED_WriteCommand(hi2c, 0xF0);   //divide ratio

    OLED_WriteCommand(hi2c, 0xA8);   //multiplex ratio
    OLED_WriteCommand(hi2c, OLED_HEIGHT - 1); //reset

    OLED_WriteCommand(hi2c, 0xD3);   // display offset
    OLED_WriteCommand(hi2c, 0x00);   // No offset
    OLED_WriteCommand(hi2c, 0x40);   //start line address

    OLED_WriteCommand(hi2c, 0xA1);   // segment re-map 0 to 127
    OLED_WriteCommand(hi2c, 0xC8);   // COM Output Scan Direction
    OLED_WriteCommand(hi2c, 0xDA);   //com pins hardware configuration
    OLED_WriteCommand(hi2c, OLED_COM_LR_REMAP << 5 | OLED_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02);

    OLED_WriteCommand(hi2c, 0x81);   // set contrast control register
    OLED_WriteCommand(hi2c, 0xFF);	//next command, dummy byte
    OLED_WriteCommand(hi2c, 0xD9);   //pre-charge period
    OLED_WriteCommand(hi2c, 0x22);	//page start and end address ?????

    sOLED_WriteCommand(hi2c, 0xDB);   // Set vcomh deselect level
    OLED_WriteCommand(hi2c, 0x20);

    //set charge pump
    OLED_WriteCommand(hi2c, 0x8D);
    OLED_WriteCommand(hi2c, 0x14);   // enable charge pump

    //set entire display on / off
    OLED_WriteCommand(hi2c, 0xA4);   //output for RAM
    OLED_WriteCommand(hi2c, 0xA6);

    //clear screen
    OLED_Fill(Black);
    OLED_WriteCommand(hi2c, 0xAF);   // Turn on OLED panel

    // Flush buffer to screen
    OLED_UpdateScreen(hi2c);

    // Set default values for screen object
    Screen.CurrentX = 0;
    Screen.CurrentY = 0;

    Screen.Initialized = 1;
    return 0;
}

void OLED_Fill(OLED_COLOR color) {
    uint32_t i;

    for(i = 0; i < sizeof(OLED_Buffer); i++) {
        OLED_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

void OLED_UpdateScreen(I2C_HandleTypeDef *hi2c) {
    uint8_t i;

    for (i = 0; i < 8; i++) {
        OLED_WriteCommand(hi2c, 0xB0 + i);
        OLED_WriteCommand(hi2c, 0x00);
        OLED_WriteCommand(hi2c, 0x10);

        HAL_I2C_Mem_Write(hi2c, OLED_I2C_ADDR, 0x40, 1, &OLED_Buffer[OLED_WIDTH * i], OLED_WIDTH, 100);
    }
}

void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) { // Don't write outside the buffer
        return;
    }

    if (color == White) {
    	OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    }
    else {
    	OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

char OLED_WriteChar(char ch, FontDef Font, OLED_COLOR color) {
    uint32_t i, b, j;

    // Check remaining space on current line
    if (OLED_WIDTH <= (Screen.CurrentX + Font.width) ||
    		OLED_HEIGHT <= (Screen.CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }

    // Translate font to screen buffer
    for (i = 0; i < Font.height; i++) {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++) {
            if ((b << j) & 0x8000) {
            	OLED_DrawPixel(Screen.CurrentX + j, (Screen.CurrentY + i), (OLED_COLOR) color);
            }
            else {
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
char OLED_WriteString(const char* str, FontDef Font, OLED_COLOR color) {
    // Write until null-byte
    while (*str) {
        if (OLED_WriteChar(*str, Font, color) != *str) { // Char could not be written
            return *str;
        }
        // Next char
        str++;
    }
    // Everything ok
    return *str;
}

