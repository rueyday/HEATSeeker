/*
 * oled_driver.h
 *
 *  Created on: Nov 22, 2025
 */

#ifndef INC_OLED_DRIVER_H_
#define INC_OLED_DRIVER_H_

#include "stm32l4xx_hal.h"
#include "fonts.h"

// Screen I2c address
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR        0x3C<<1
#endif

#ifndef OLED_WIDTH
#define OLED_WIDTH           128
#endif

#ifndef OLED_HEIGHT
#define OLED_HEIGHT          64
#endif

#ifndef OLED_COM_LR_REMAP
#define OLED_COM_LR_REMAP    0
#endif

#ifndef OLED_COM_ALTERNATIVE_PIN_CONFIG
#define OLED_COM_ALTERNATIVE_PIN_CONFIG    1
#endif

typedef enum {
    Black = 0x00,   // no pixel
    White = 0x01,
} OLED_COLOR;

typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
} OLED_t;

uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_UpdateScreen(I2C_HandleTypeDef *hi2c);
void OLED_Fill(OLED_COLOR color);
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color);
char OLED_WriteChar(char ch, FontDef Font, OLED_COLOR color);
char OLED_WriteString(const char* str, FontDef Font, OLED_COLOR color);
void OLED_SetCursor(uint8_t x, uint8_t y);

#endif /* INC_OLED_DRIVER_H_ */
