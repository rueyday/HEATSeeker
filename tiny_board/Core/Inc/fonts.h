/*
 * fonts.h
 *
 *  Created on: Nov 22, 2025
 *      Author: yklim
 */

#ifndef INC_FONTS_H_
#define INC_FONTS_H_

// fonts.h

#include <stdint.h>

typedef struct {
    const uint8_t width;    // Character width in pixels
    const uint8_t height;   // Character height in pixels
    const uint16_t *data;   // Pointer to font data array
} FontDef;

// Available fonts
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

// Function prototypes
uint16_t GetFontWidth(FontDef* font, const char* text);
uint16_t GetFontHeight(FontDef* font);


#endif /* INC_FONTS_H_ */
