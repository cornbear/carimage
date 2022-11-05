#include <stdio.h>

#include "type.h"
#include "hardware.h"
#include "image.h"
#include "imageprocess.h"
#include "mock.h"
#include "bitmap.h"

TRACK_BORDER_INFO g_Border;
TRACK_TYPE_INFO g_TrackType; /*赛道类型*/

int main()
{
    uint8 mt9v03x_image[IMGH][IMGW];
    uint8 imageBit8[IMGH][IMGW];
    uint8 imageBit24[IMGH][IMGW * 3];

    for (int i = 0; i < IMGH * IMGW; i++)
    {
        mt9v03x_image[i / IMGW][i % IMGW] = mockImage[i];
    }

    g_TrackType.m_u8CarBarnState = 0;
    g_TrackType.m_u8ThreeRoadsDir = 1;
    g_TrackType.m_u8RunTurns = 0;

    uint8 threshold = getThreshold(mt9v03x_image); //获取图像阈值
    // printf("Threshold: %d\n", threshold);

    getBinaryImage(mt9v03x_image, imageBit8, threshold); //二值化图像
    // saveBit8Bitmap("gray.bmp", (uint8 *)imageBit8, IMGW, IMGH);

    getOptimumColumn(imageBit8, &g_Border, &g_TrackType); //获取最优点
    // printf("Optimum Point: (%d,%d)\n", g_Border.m_OptimalPoint.m_i16x, g_Border.m_OptimalPoint.m_i16y);

    bitmapBit8to24((uint8 *)imageBit8, (uint8 *)imageBit24, IMGW, IMGH);

    drawPoint((uint8 *)imageBit24, IMGW, IMGH, g_Border.m_OptimalPoint.m_i16x, g_Border.m_OptimalPoint.m_i16y, RED);
    drawPoint((uint8 *)imageBit24, IMGW, IMGH, g_Border.m_CenterLinePoint.m_i16x, g_Border.m_CenterLinePoint.m_i16y, GREEN);

    // saveBit24Bitmap("point.bmp", (uint8 *)imageBit24, IMGW, IMGH);

    getBorder(imageBit8, &g_Border); //获取边线

    for (int i = 0; i < IMGH; i++)
    {
        INT_POINT_INFO lp = g_Border.m_LPnt[i];
        INT_POINT_INFO rp = g_Border.m_RPnt[i];
        drawPoint((uint8 *)imageBit24, IMGW, IMGH, lp.m_i16x, lp.m_i16y, BLUE);
        drawPoint((uint8 *)imageBit24, IMGW, IMGH, rp.m_i16x, rp.m_i16y, BLUE);
    }

    saveBit24Bitmap("border.bmp", (uint8 *)imageBit24, IMGW, IMGH);

    Perspective_Change(&g_Border); //相机畸变矫正

    straightaway_curve(&g_Border, 0);

    // state_judgement(image_new, &g_Border, &g_TrackType); ///??????
    // Out_Protect(image_new);
    // if (g_TrackType.m_u8CrossFlag != 4)
    //     CenterlineGet(image_new, &g_Border);
    // err_Cal(&g_Border);

    // if (g_TrackType.m_u8CarRunningState != 2 && g_TrackType.m_u8CarRunningState != 4)
    // {
    //     // Speed_plan();
    // }

    // if (!g_TrackType.m_u8CarBarnState && g_TrackType.m_u8CarRunningState != 2 && g_TrackType.m_u8CarRunningState != 4)
    // {
    //     // angel_Control();
    // }

    // g_TrackType.m_u8LAllLostFlag = 0;
    // g_TrackType.m_u8RAllLostFlag = 0;
}