#include <stdio.h>
#include "type.h"

void saveBitmap(char *fileName, uint8 image[], int width, int height, uint8 bytesPerPixel)
{
    unsigned char bmpFileHeader[14] = {'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0};
    unsigned char bmpInfoHeader[40] = {40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 8 * bytesPerPixel, 0};
    unsigned char bmppad[3] = {0, 0, 0};
    unsigned char bmpColorPalette[1024];

    int filesize = 54 + width * height * bytesPerPixel;

    bmpFileHeader[2] = (unsigned char)(filesize);
    bmpFileHeader[3] = (unsigned char)(filesize >> 8);
    bmpFileHeader[4] = (unsigned char)(filesize >> 16);
    bmpFileHeader[5] = (unsigned char)(filesize >> 24);

    bmpInfoHeader[4] = (unsigned char)(width);
    bmpInfoHeader[5] = (unsigned char)(width >> 8);
    bmpInfoHeader[6] = (unsigned char)(width >> 16);
    bmpInfoHeader[7] = (unsigned char)(width >> 24);

    bmpInfoHeader[8] = (unsigned char)(height);
    bmpInfoHeader[9] = (unsigned char)(height >> 8);
    bmpInfoHeader[10] = (unsigned char)(height >> 16);
    bmpInfoHeader[11] = (unsigned char)(height >> 24);

    for (int i = 0; i < 256; i++)
    {
        bmpColorPalette[4*i] = i;
        bmpColorPalette[4*i+1] = i;
        bmpColorPalette[4*i+2] = i;
        bmpColorPalette[4*i+3] = i;
    }

    FILE *f = fopen(fileName, "wb");
    fwrite(bmpFileHeader, 1, 14, f);
    fwrite(bmpInfoHeader, 1, 40, f);
    fwrite(bmpColorPalette, 1, 1024, f);

    for (int i = 0; i < height; i++)
    {
        fwrite(image + (width * (height - i - 1) * bytesPerPixel), bytesPerPixel, width, f);
        fwrite(bmppad, 1, (4 - (width * bytesPerPixel) % 4) % 4, f);
    }
    fclose(f);
}
