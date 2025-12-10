/*
 * led.c
 *
 *  Created on: Nov 26, 2025
 *      Author: yklim
 */

#include "led.h"
#include "stdint.h"

SPI_HandleTypeDef *hspi;
static ledFrame leds;
uint8_t spiFrame[startFrame_Size + 4 * led_count + endFrame_Size];

void led_Update(SPI_HandleTypeDef *hspi) {
	for(uint8_t i = 0; i < startFrame_Size; i++) {
		spiFrame[i] = 0x00;
	}
	for(uint8_t i = sizeof(spiFrame) - 1; i > sizeof(spiFrame) - 5; i--) {
		spiFrame[i] = 0xFF;
	}
	for(uint8_t i = 0; i < led_count * 4; i += 4) {
		spiFrame[startFrame_Size + i] = 0xE0 | leds.illum; //MSB 3 bits is 111
		spiFrame[startFrame_Size + i + 1] = leds.red;
		spiFrame[startFrame_Size + i + 2] = leds.green;
		spiFrame[startFrame_Size + i + 3] = leds.blue;

	}
	HAL_SPI_Transmit(hspi, spiFrame, sizeof(spiFrame), 10);
}

void led_Init(SPI_HandleTypeDef *hspi) {
	leds.illum = 0x00;
	leds.red = 0x00;
	leds.green = 0x00;
	leds.blue = 0x00;
	led_Update(hspi);
}

void led_setColor(SPI_HandleTypeDef *hspi, uint8_t red, uint8_t green, uint8_t blue) {
	leds.red = red;
	leds.green = green;
	leds.blue = blue;
	led_Update(hspi);
}

void led_SetIllum(SPI_HandleTypeDef *hspi, uint8_t illum) {
	leds.illum = illum;
	led_Update(hspi);
}
