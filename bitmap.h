#ifndef CODE_BITMAP_H_
#define CODE_BITMAP_H_

#include <string.h>
#include "type.h"

typedef struct color
{
    uint8 b;
    uint8 g;
    uint8 r;
} COLOR;

static COLOR RED = {0, 0, 255};
static COLOR GREEN = {0, 255, 0};
static COLOR BLUE = {255, 0, 0};

extern void bitmapBit8to24(uint8 imageBit8[], uint8 imageBit24[], int width, int height);
extern void drawPoint(uint8 imageBit24[], int width, int height, int16 x, int16 y, COLOR color);
extern void saveBit8Bitmap(char *fileName, uint8 image[], int width, int height);
extern void saveBit24Bitmap(char *fileName, uint8 image[], int width, int height);

#endif