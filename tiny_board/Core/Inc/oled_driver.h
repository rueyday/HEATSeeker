/*
 * oled_driver.h
 *
 *  Created on: Nov 22, 2025
 *      Author: yklim
 */

#ifndef INC_OLED_DRIVER_H_
#define INC_OLED_DRIVER_H_

#include "stm32l4xx_hal.h"
#include "fonts.h"

// Screen I2c address
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR        0x3C<<1
#endif // OLED_I2C_ADDR

// Screen width in pixels
#ifndef OLED_WIDTH
#define OLED_WIDTH           128
#endif // OLED_WIDTH

// Screen LCD height in pixels
#ifndef OLED_HEIGHT
#define OLED_HEIGHT          64
#endif // OLED_HEIGHT

#ifndef OLED_COM_LR_REMAP
#define OLED_COM_LR_REMAP    0
#endif // OLED_COM_LR_REMAP

#ifndef OLED_COM_ALTERNATIVE_PIN_CONFIG
#define OLED_COM_ALTERNATIVE_PIN_CONFIG    1
#endif // OLED_COM_ALTERNATIVE_PIN_CONFIG


//
//  Enumeration for screen colors
//
typedef enum {
    Black = 0x00,   // Black color, no pixel
    White = 0x01,   // Pixel is set. Color depends on LCD
} OLED_COLOR;

//
//  Struct to store transformations
//
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} OLED_t;

//
//  Function definitions
//

uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_UpdateScreen(I2C_HandleTypeDef *hi2c);
void OLED_Fill(OLED_COLOR color);
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color);
char OLED_WriteChar(char ch, FontDef Font, OLED_COLOR color);
char OLED_WriteString(const char* str, FontDef Font, OLED_COLOR color);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_InvertColors(void);

#endif /* INC_OLED_DRIVER_H_ */
