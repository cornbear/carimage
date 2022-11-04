#ifndef CODE_IMAGEPROCESS_H_
#define CODE_IMAGEPROCESS_H_

#include <stdio.h>
#include <string.h>

#include "type.h"
#include "hardware.h"

#define GrayScale 256
#define ERROR 0

#define B_BLACK 0
#define B_WHITE 255

uint8 FuzzyOstu(uint8 (*Img)[IMGW]);
uint8 GetThreshold(uint8 (*Img)[IMGW]);
void getBinaryImage(uint8 (*InImg)[IMGW], uint8 (*OutImg)[IMGW], float Threshold);
extern void ThBi(uint8 (*InImg)[IMGW], uint8 (*OutImg)[IMGW]);
extern uint8 Multi_Peak_Threshold(uint8 (*Img)[IMGW]);
float ThresholdGet(uint8 (*Img)[IMGW]);
uint8 getThreshold(uint8 (*Img)[IMGW]);

#endif /* CODE_IMAGEPROCESS_H_ */
