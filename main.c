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
    uint8 image_new[IMGH][IMGW];

    for (int i = 0; i < IMGH * IMGW; i++)
    {
        mt9v03x_image[i / IMGW][i % IMGW] = mockImage[i];
    }

    g_TrackType.m_u8CarBarnState = 0;
    g_TrackType.m_u8ThreeRoadsDir = 1;
    g_TrackType.m_u8RunTurns = 0;

    g_Border.Threshold = getThreshold(mt9v03x_image); //获取图像阈值
    printf("Threshold:%f\n", g_Border.Threshold);

    getBinaryImage(mt9v03x_image, image_new, g_Border.Threshold); //二值化图像

    saveBitmap("gray.bmp", (uint8 *)image_new, IMGW, IMGH, 1);

    INT_POINT_INFO optimumPoint = getOptimumColumn(image_new, &g_Border, &g_TrackType); //获取最优点
    printf("Optimum Point: (%d,%d)\n", optimumPoint.m_i16x, optimumPoint.m_i16y);

    getBorder(image_new, &g_Border); //获取边线

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