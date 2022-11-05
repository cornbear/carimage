#ifndef CODE_BITMAP_H_
#define CODE_BITMAP_H_
#include "type.h"

typedef struct color
{
    uint8 r;
    uint8 g;
    uint8 b;
} COLOR;

static COLOR RED = {255, 0, 0};
static COLOR GREEN = {0, 255, 0};
static COLOR BLUE = {0, 0, 255};

extern void bitmapBit8to24(uint8 imageBit8[], uint8 imageBit24[], int width, int height);
extern void drawPoint(uint8 imageBit24[], int16 x, int16 y, COLOR color);
extern void saveBit8Bitmap(char *fileName, uint8 image[], int width, int height);
extern void saveBit24Bitmap(char *fileName, uint8 image[], int width, int height);

#endif