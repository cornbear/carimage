#include <stdio.h>

#include "type.h"
#include "hardware.h"
#include "image.h"
#include "imageprocess.h"
#include "mock.h"
#include "bitmap.h"

TRACK_BORDER_INFO g_Border;
TRACK_TYPE_INFO g_TrackType; /*赛道类型*/

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
uint8 image_new[MT9V03X_H][MT9V03X_W];

int main()
{
    for (int i = 0; i < MT9V03X_H * MT9V03X_W; i++)
    {
        mt9v03x_image[i / MT9V03X_W][i % MT9V03X_W] = mockImage[i];
    }

    // for (int i = 0; i < MT9V03X_H; i++)
    // {
    //     for (int j = 0; j < MT9V03X_W; j++)
    //     {
    //         printf("%x ", mt9v03x_image[i][j]);
    //     }
    //     printf("\n"); //换行
    // }

    // Angle_offset = 0;

    g_TrackType.m_u8CarBarnState = 0;
    g_TrackType.m_u8ThreeRoadsDir = 1;
    g_TrackType.m_u8RunTurns = 0;

    g_Border.Threshold = 128; //ThresholdGet(mt9v03x_image); //获取图像阈值
    printf("Threshold:%f\n", g_Border.Threshold);

    GetBinaryImage(mt9v03x_image, image_new, g_Border.Threshold); //二值化图像

    saveBitmap("gray.bmp", (uint8*)image_new, IMGW, IMGH, 1);

    // GetOptimumColumn(image_new, &g_Border, &g_TrackType);
    // GetBorder(image_new, &g_Border); //获取边线
    // Perspective_Change(&g_Border);
    // straightaway_curve(&g_Border, 0);
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