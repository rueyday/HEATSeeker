/*
 * led.h
 *
 *  Created on: Nov 26, 2025
 *      Author: yklim
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stdint.h"
#include "stm32l4xx_hal.h"
//#include "main.h"

#define led_count 30
#define startFrame 0x00000000
#define startFrame_Size 4
#define endFrame 0xFFFFFFFF
#define endFrame_Size 4

typedef struct {
	uint8_t illum; //brightness; 0x00 to 0x1F
	uint8_t red; //colour; 0x00 to 0xFF
	uint8_t green;
	uint8_t blue;
} ledFrame;

void led_Update(SPI_HandleTypeDef *hspi);
void led_Init(SPI_HandleTypeDef *hspi);
void led_setColor(SPI_HandleTypeDef *hspi, uint8_t red, uint8_t green, uint8_t blue);
void led_SetIllum(SPI_HandleTypeDef *hspi, uint8_t illum);
//void led_SetColor(uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
//void led_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
//void led_SetIllumination(uint8_t led, uint8_t illum);

#endif /* INC_LED_H_ */
