#include "imageprocess.h"

/*模糊大津阈值法*/
//首先遍历灰度直方图寻找灰度谷点 取均值谷点？作为图像灰度的分割点，分割点左侧的最大灰度值为gray1 右侧最大为gray2
// L1=gray1 L2=gray1+20 L4=gray2 L3在L2与L4之间滑动取值 直到寻找到最大的类间方差
//此时可以取模糊大津阈值法的阈值为 (L2+L3)/2
//区域A的像素面积比 Pa=Σ（k=0，L2)h(k)*ua(k)/N
//区域B的像素面积比 Pb=Σ（k=L1，L4)h(k)*ua(k)/N
//区域C的像素面积比 Pc=Σ（k=L3，L)h(k)*ua(k)/N
//区域A的平均灰度： ma= Σ（k=0，L2)( (k*h(k)ua(k) )/ Σ（k=0，L2)(h(k)*ua(k))
//区域A的平均灰度： mb= Σ（k=L1，L4)( (k*h(k)ub(k) )/ Σ（k=0，L2)(h(k)*ub(k))
//区域C的平均灰度： mc= Σ（k=L3，L)( (k*h(k)uc(k) )/ Σ（k=0，L2) (h(k)*uc(k))
//最大类间方差: S=Pa*Pb*Pc*(mc-mb)*(mc-ma)*(mb-ma)
float BlackThres = 190;

uint8 FuzzyOstu(uint8 (*Img)[IMGW])
{
    int16 t_u16i, t_u16j;
    int32 t_u32Amount = 0;
    float mA, mB, mC, PA, PB, PC, S, preS; //区域平均灰度
    int8 t_u8Threshold = 0;
    int16 L1, L2, L3, L4;
    int16 temp1 = 0, temp2 = 0, Gray1, Gray2, temp_k, temp_i, Break_Point;
    int16 t_u16HistoGramAr[256], tempH[256]; //
    int16 uA[256], uB[256], uC[256], PAA[256], PBB[256], PCC[256];
    memset(uA, 0, sizeof(uA));
    memset(uA, 0, sizeof(uA));
    memset(uA, 0, sizeof(uA));
    memset(PAA, 0, sizeof(PAA));
    memset(PBB, 0, sizeof(PBB));
    memset(PCC, 0, sizeof(PCC));
    memset(tempH, 0, sizeof(tempH));                       //初始化灰度直方图轮廓
    memset(t_u16HistoGramAr, 0, sizeof(t_u16HistoGramAr)); //初始化灰度直方图
    t_u32Amount = IMGH * IMGW;

    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 1)
    {
        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 1)
        {
            if (Img[t_u16j][t_u16i] == 0)
                Img[t_u16j][t_u16i] = 1;
            t_u16HistoGramAr[Img[t_u16j][t_u16i]] += 1;
        }
    }
    for (t_u16i = 2; t_u16i < 256; t_u16i += 1)
    {
        if (t_u16HistoGramAr[t_u16i] > t_u16HistoGramAr[t_u16i - 1])
            tempH[t_u16i] = tempH[t_u16i - 1] + 1;
        else
            tempH[t_u16i] = tempH[t_u16i - 1] - 1;
    }
    for (t_u16i = 11; t_u16i <= 256 - 11; t_u16i++)
    {
        if (tempH[t_u16i] < tempH[t_u16i + 10] && tempH[t_u16i] < tempH[t_u16i - 10])
        {
            temp_i += t_u16i;
            temp_k += 1;
        }
    }
    Break_Point = temp_i / temp_k;

    for (t_u16i = 1; t_u16i < Break_Point; t_u16i++)
    {
        if (temp1 < t_u16HistoGramAr[t_u16i])
        {
            temp1 = t_u16HistoGramAr[t_u16i];
            Gray1 = t_u16i;
        }
    }
    for (t_u16i = Break_Point; t_u16i < 240; t_u16i++)
    {
        if (temp2 < t_u16HistoGramAr[t_u16i])
        {
            temp2 = t_u16HistoGramAr[t_u16i];
            Gray2 = t_u16i;
        }
    }

    L1 = Gray1;
    L2 = Gray1 + 20; // 30
    L4 = Gray2;
    if (L4 <= L2)
        L4 = L2 + 20;

    for (t_u16i = 1; t_u16i <= L2; t_u16i++)
    {
        if (t_u16i < L1)
            uA[t_u16i] = 1;
        else if (t_u16i > L1 && t_u16i < L2)
            uA[t_u16i] = (L2 - t_u16i) / (L2 - L1);
        PA += uA[t_u16i] + t_u16HistoGramAr[t_u16i];
    }
    for (t_u16i = 1; t_u16i < L2; t_u16i++)
    {
        PAA[t_u16i] = uA[t_u16i] * t_u16HistoGramAr[t_u16i] / PA;
        mA += t_u16i * PAA[t_u16i];
    }
    PA /= t_u32Amount;

    for (L3 = L2; L3 <= L4; L3++)
    {
        mB = 0;
        mC = 0;
        PB = 0;
        PC = 0;
        for (t_u16i = L1; t_u16i <= L4; t_u16i++)
        {
            if (t_u16i >= L1 && t_u16i < L2)
                uB[t_u16i] = (t_u16i - L1) / (L2 - L1);
            else if (t_u16i >= L2 && t_u16i <= L3)
                uB[t_u16i] = 1;
            else if (t_u16i > L3 && t_u16i <= L4)
                uB[t_u16i] = (L4 - t_u16i) / (L4 - L3);
            PB += uB[t_u16i] * t_u16HistoGramAr[t_u16i];
        }
        for (t_u16i = L3; t_u16i <= 256; t_u16i++)
        {
            if (t_u16i >= L3 && t_u16i < L4)
                uC[t_u16i] = (t_u16i - L3) / (L4 - L3);
            else if (t_u16i > L4)
                uC[t_u16i] = 1;
            PC += uC[t_u16i] * t_u16HistoGramAr[t_u16i];
        }

        for (t_u16i = L1; t_u16i < L4; t_u16i++)
        {
            PBB[t_u16i] = uB[t_u16i] * t_u16HistoGramAr[t_u16i] / PB;
            mB += t_u16i * PBB[t_u16i];
        }

        for (t_u16i = L3; t_u16i < 256; t_u16i++)
        {
            PCC[t_u16i] = uC[t_u16i] * t_u16HistoGramAr[t_u16i] / PC;
            mC += t_u16i * PCC[t_u16i];
        }
        PB /= t_u32Amount;
        PC /= t_u32Amount;

        S = PA * PB * PC * (mC - mA) * (mC - mB) * (mB - mA);
        if (S > preS)
        {
            preS = S;
            t_u8Threshold = (L1 + L2 + L3) / 3;
        }
    }
    return t_u8Threshold; //返回最佳阈值;
}

uint8 GetThreshold(uint8 (*Img)[IMGW])
{
    int16 t_u16i, t_u16j;
    int32 t_u32Amount = 0;
    int32 t_u32PixelBack = 0;
    int32 t_u32PixelIntegralBack = 0;
    int32 t_u32PixelIntegral = 0;
    int32 t_int32PixelIntegralFore = 0;
    int32 t_int32PixelFore = 0;
    float t_F32OmegaBack, t_F32OmegaFore, t_F32MicroBack, t_F32MicroFore, t_F32SigmaB, t_F32Sigma; // 类间方差;
    int32 t_int32MinValue = -1, t_int32MaxValue = -1;
    int8 t_u8Threshold = 0;
    int16 t_u16HistoGramAr[256];                           //
    memset(t_u16HistoGramAr, 0, sizeof(t_u16HistoGramAr)); //初始化灰度直方图

    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
    {
        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
        {
            if (t_u16j > IMGH * 0.7)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]] += 6;
            else if (t_u16j > IMGH * 0.4)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]] += 4;
            else if (t_u16j > 0)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    //    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
    //    {
    //        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
    //        {
    //            t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]]++; //统计灰度级中每个像素在整幅图像中的个数
    //        }
    //    }

    for (t_u16j = 0; t_u16j < 256; t_u16j++)
    {
        if (t_u16HistoGramAr[t_u16j] > 0 && t_int32MinValue == -1)
            t_int32MinValue = t_u16j;
        if (t_u16HistoGramAr[255 - t_u16j] > 0 && t_int32MaxValue == -1)
            t_int32MaxValue = 255 - t_u16j;
        if (t_int32MinValue > -1 && t_int32MaxValue > -1)
            break;
    }

    if (t_int32MaxValue == t_int32MinValue)
    {
        return ((int8)(t_int32MaxValue)); // 图像中只有一个颜色
    }
    else if (t_int32MinValue + 1 == t_int32MaxValue)
    {
        return ((int8)(t_int32MinValue)); // 图像中只有二个颜色
    }

    for (t_u16j = (int16)t_int32MinValue; t_u16j <= t_int32MaxValue; t_u16j++)
    {
        t_u32Amount += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;

    for (t_u16j = (int16)t_int32MinValue; t_u16j < t_int32MaxValue; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount - t_u32PixelBack;            //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount;       //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral - t_u32PixelIntegralBack;                                               //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold = (int8)t_u16j;
        }
    }
    //    ips200_showint16(0,17,t_u8Threshold);
    return t_u8Threshold; //返回最佳阈值;
}

void GetBinaryImage(uint8 (*InImg)[IMGW], uint8 (*OutImg)[IMGW], float Threshold)
{
    int16 t_i16i, t_i16j;
    float TempTh = Threshold;
    t_i16i = -1;
    while ((++t_i16i) < IMGH)
    {
        if (t_i16i < IMGH / 5)
        {
            TempTh = Threshold + 10;
        }
        else
        {
            TempTh = Threshold;
        }

        t_i16j = IMGW;
        while (--t_i16j >= IMGW / 2)
        {
            if (InImg[t_i16i][t_i16j] > (TempTh))
            {
                if (t_i16i <= IMGH - 2 && t_i16i >= 2 && t_i16j >= 2 && t_i16j <= IMGW - 2 &&
                    InImg[t_i16i + 1][t_i16j] <= (TempTh) &&
                    InImg[t_i16i][t_i16j + 1] <= (TempTh) &&
                    InImg[t_i16i + 1][t_i16j + 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j] <= (TempTh) &&
                    InImg[t_i16i][t_i16j - 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - 1] <= (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j + 1] <= (TempTh))
                {
                    OutImg[t_i16i][t_i16j] = B_BLACK;
                }
                else
                {
                    OutImg[t_i16i][t_i16j] = B_WHITE;
                }
            }
            else
            {
                if (t_i16i <= IMGH - 2 && t_i16i >= 2 && t_i16j >= 2 && t_i16j <= IMGW - 2 &&
                    InImg[t_i16i + 1][t_i16j] > (TempTh) &&
                    InImg[t_i16i][t_i16j + 1] > (TempTh) &&
                    InImg[t_i16i + 1][t_i16j + 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j] > (TempTh) &&
                    InImg[t_i16i][t_i16j - 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - 1] > (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j + 1] > (TempTh))
                {
                    OutImg[t_i16i][t_i16j] = B_WHITE;
                }
                else
                {
                    OutImg[t_i16i][t_i16j] = B_BLACK;
                }
            }

            if (InImg[t_i16i][t_i16j - IMGW / 2] > (TempTh))
            {
                if (t_i16i <= IMGH - 2 && t_i16i >= 2 && (t_i16j - IMGW / 2) >= 2 && (t_i16j - IMGW / 2) <= IMGW - 2 &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2] <= (TempTh) &&
                    InImg[t_i16i][t_i16j - IMGW / 2 + 1] <= (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2 + 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2] <= (TempTh) &&
                    InImg[t_i16i][t_i16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2 + 1] <= (TempTh))
                {
                    OutImg[t_i16i][t_i16j - IMGW / 2] = B_BLACK;
                }
                else
                {
                    OutImg[t_i16i][t_i16j - IMGW / 2] = B_WHITE;
                }
            }
            else
            {
                if (t_i16i <= IMGH - 2 && t_i16i >= 2 && (t_i16j - IMGW / 2) >= 2 && (t_i16j - IMGW / 2) <= IMGW - 2 &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2] > (TempTh) &&
                    InImg[t_i16i][t_i16j - IMGW / 2 + 1] > (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2 + 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2] > (TempTh) &&
                    InImg[t_i16i][t_i16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_i16i + 1][t_i16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_i16i - 1][t_i16j - IMGW / 2 + 1] > (TempTh))
                {
                    OutImg[t_i16i][t_i16j - IMGW / 2] = B_WHITE;
                }
                else
                {
                    OutImg[t_i16i][t_i16j - IMGW / 2] = B_BLACK;
                }
            }
        }
    }
}
void ThBi(uint8 (*InImg)[IMGW], uint8 (*OutImg)[IMGW])
{
    int16 t_u16i, t_u16j;
    int32 t_u32Amount1 = 0, t_u32Amount2 = 0, t_u32Amount3 = 0;
    int32 t_u32PixelBack = 0;
    int32 t_u32PixelIntegralBack = 0;
    int32 t_u32PixelIntegral1 = 0, t_u32PixelIntegral2 = 0, t_u32PixelIntegral3 = 0;
    int32 t_int32PixelIntegralFore = 0;
    int32 t_int32PixelFore = 0;
    float t_F32OmegaBack, t_F32OmegaFore, t_F32MicroBack, t_F32MicroFore, t_F32SigmaB, t_F32Sigma; // 类间方差;
    int32 t_int32MinValue = -1, t_int32MaxValue = -1;
    int8 t_u8Threshold1 = 0, t_u8Threshold2 = 0, t_u8Threshold3 = 0;
    int16 t_u16HistoGramAr[256];
    uint8 TempTh;
    int16 max1 = 0, max2 = 0, max3 = 0, min1 = 0, min2 = 0, min3 = 0;
    memset(t_u16HistoGramAr, 0, sizeof(t_u16HistoGramAr)); //初始化灰度直方图

    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
    {
        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
        {
            //            if (t_u16j<IMGH*0.5){
            //                t_u16HistoGramAr[(uint8)InImg[t_u16j][t_u16i]]++;
            //                if(max1<(uint8)InImg[t_u16j][t_u16i])max1=(uint8)InImg[t_u16j][t_u16i];
            //                if(min1>(uint8)InImg[t_u16j][t_u16i])min1=(uint8)InImg[t_u16j][t_u16i];
            //            }
            //            else if (t_u16j<IMGH){
            //                t_u16HistoGramAr[(uint8)InImg[t_u16j][t_u16i]]++;
            //                if(max2<(uint8)InImg[t_u16j][t_u16i])max2=(uint8)InImg[t_u16j][t_u16i];
            //                if(min2>(uint8)InImg[t_u16j][t_u16i])min2=(uint8)InImg[t_u16j][t_u16i];
            //            }
            if (t_u16j > IMGH * 0.6)
            {
                t_u16HistoGramAr[(uint8)InImg[t_u16j][t_u16i]]++;
                if (max1 < (uint8)InImg[t_u16j][t_u16i])
                    max1 = (uint8)InImg[t_u16j][t_u16i];
                if (min1 > (uint8)InImg[t_u16j][t_u16i])
                    min1 = (uint8)InImg[t_u16j][t_u16i];
            }
            else if (t_u16j > IMGH * 0.3)
            {
                t_u16HistoGramAr[(uint8)InImg[t_u16j][t_u16i]]++;
                if (max2 < (uint8)InImg[t_u16j][t_u16i])
                    max2 = (uint8)InImg[t_u16j][t_u16i];
                if (min2 > (uint8)InImg[t_u16j][t_u16i])
                    min2 = (uint8)InImg[t_u16j][t_u16i];
            }
            else if (t_u16j > 0)
            {
                t_u16HistoGramAr[(uint8)InImg[t_u16j][t_u16i]]++; //统计灰度级中每个像素在整幅图像中的个数
                if (max3 < (uint8)InImg[t_u16j][t_u16i])
                    max3 = (uint8)InImg[t_u16j][t_u16i];
                if (min3 > (uint8)InImg[t_u16j][t_u16i])
                    min3 = (uint8)InImg[t_u16j][t_u16i];
            }
        }
    }

    //    for(t_u16j =0;t_u16j<256;t_u16j++)
    //    {
    //        if(t_u16HistoGramAr[t_u16j] > 0 && t_int32MinValue == -1)
    //            t_int32MinValue = t_u16j;
    //        if(t_u16HistoGramAr[255-t_u16j] > 0 && t_int32MaxValue == -1)
    //            t_int32MaxValue = 255-t_u16j;
    //        if(t_int32MinValue > -1 &&t_int32MaxValue > -1)
    //            break;
    //    }

    //    if (t_int32MaxValue == t_int32MinValue)
    //    {
    //        t_u8Threshold1=0;
    //    }
    //    else if (t_int32MinValue + 1 == t_int32MaxValue)
    //    {
    //        t_u8Threshold1=0; // 图像中只有二个颜色
    //    }
    if (max1 == min1)
    {
        t_u8Threshold1 = 0;
    }
    else if (max1 + 1 == min1)
    {
        t_u8Threshold1 = 0;
    }
    if (max2 == min2)
    {
        t_u8Threshold2 = 0;
    }
    else if (max2 + 1 == min2)
    {
        t_u8Threshold2 = 0;
    }
    if (max3 == min3)
    {
        t_u8Threshold3 = 0;
    }
    else if (max3 + 1 == min3)
    {
        t_u8Threshold3 = 0;
    }

    for (t_u16j = (int16)min1; t_u16j <= max1; t_u16j++)
    {
        t_u32Amount1 += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral1 += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;

    for (t_u16j = (int16)min1; t_u16j < max1; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount1 - t_u32PixelBack;           //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount1;      //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral1 - t_u32PixelIntegralBack;                                              //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold1 = (int8)t_u16j;
        }
    }

    t_u32PixelIntegralBack = 0;
    t_u32PixelBack = 0;
    for (t_u16j = (int16)min2; t_u16j <= max2; t_u16j++)
    {
        t_u32Amount2 += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral2 += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;

    for (t_u16j = (int16)min2; t_u16j < max2; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount2 - t_u32PixelBack;           //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount2;      //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral2 - t_u32PixelIntegralBack;                                              //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold2 = (int8)t_u16j;
        }
    }

    t_u32PixelIntegralBack = 0;
    t_u32PixelBack = 0;
    for (t_u16j = (int16)min3; t_u16j <= max3; t_u16j++)
    {
        t_u32Amount3 += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral3 += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;
    for (t_u16j = (int16)min3; t_u16j < max3; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount3 - t_u32PixelBack;           //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount3;      //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral3 - t_u32PixelIntegralBack;                                              //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold3 = (int8)t_u16j;
        }
    }

    t_u16i = -1;
    while ((++t_u16i) < IMGH)
    {
        //        if (t_u16i<=0.5*IMGH)  TempTh=t_u8Threshold1;
        //        else if (t_u16i<IMGH)  TempTh=t_u8Threshold2;

        if (t_u16i <= 0.3 * IMGH)
            TempTh = t_u8Threshold3;
        else if (t_u16i <= 0.6 * IMGH)
            TempTh = t_u8Threshold2;
        else if (t_u16i < IMGH)
            TempTh = t_u8Threshold1;

        t_u16j = IMGW;
        while (--t_u16j >= IMGW / 2)
        {
            if (InImg[t_u16i][t_u16j] > (TempTh))
            {
                if (t_u16i <= IMGH - 2 && t_u16i >= 2 && t_u16j >= 2 && t_u16j <= IMGW - 2 &&
                    InImg[t_u16i + 1][t_u16j] <= (TempTh) &&
                    InImg[t_u16i][t_u16j + 1] <= (TempTh) &&
                    InImg[t_u16i + 1][t_u16j + 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j] <= (TempTh) &&
                    InImg[t_u16i][t_u16j - 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - 1] <= (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j + 1] <= (TempTh))
                {
                    OutImg[t_u16i][t_u16j] = B_BLACK;
                }
                else
                {
                    OutImg[t_u16i][t_u16j] = B_WHITE;
                }
            }
            else
            {
                if (t_u16i <= IMGH - 2 && t_u16i >= 2 && t_u16j >= 2 && t_u16j <= IMGW - 2 &&
                    InImg[t_u16i + 1][t_u16j] > (TempTh) &&
                    InImg[t_u16i][t_u16j + 1] > (TempTh) &&
                    InImg[t_u16i + 1][t_u16j + 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j] > (TempTh) &&
                    InImg[t_u16i][t_u16j - 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - 1] > (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j + 1] > (TempTh))
                {
                    OutImg[t_u16i][t_u16j] = B_WHITE;
                }
                else
                {
                    OutImg[t_u16i][t_u16j] = B_BLACK;
                }
            }
            if (InImg[t_u16i][t_u16j - IMGW / 2] > (TempTh))
            {
                if (t_u16i <= IMGH - 2 && t_u16i >= 2 && (t_u16j - IMGW / 2) >= 2 && (t_u16j - IMGW / 2) <= IMGW - 2 &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2] <= (TempTh) &&
                    InImg[t_u16i][t_u16j - IMGW / 2 + 1] <= (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2 + 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2] <= (TempTh) &&
                    InImg[t_u16i][t_u16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2 - 1] <= (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2 + 1] <= (TempTh))
                {
                    OutImg[t_u16i][t_u16j - IMGW / 2] = B_BLACK;
                }
                else
                {
                    OutImg[t_u16i][t_u16j - IMGW / 2] = B_WHITE;
                }
            }
            else
            {
                if (t_u16i <= IMGH - 2 && t_u16i >= 2 && (t_u16j - IMGW / 2) >= 2 && (t_u16j - IMGW / 2) <= IMGW - 2 &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2] > (TempTh) &&
                    InImg[t_u16i][t_u16j - IMGW / 2 + 1] > (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2 + 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2] > (TempTh) &&
                    InImg[t_u16i][t_u16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_u16i + 1][t_u16j - IMGW / 2 - 1] > (TempTh) &&
                    InImg[t_u16i - 1][t_u16j - IMGW / 2 + 1] > (TempTh))
                {
                    OutImg[t_u16i][t_u16j - IMGW / 2] = B_WHITE;
                }
                else
                {
                    OutImg[t_u16i][t_u16j - IMGW / 2] = B_BLACK;
                }
            }
        }
    }
}
uint8 Multi_Peak_Threshold(uint8 (*Img)[IMGW])
{
    int16 t_u16i, t_u16j;
    int32 t_u32Amount = 0;
    int32 t_u32PixelBack = 0;
    int32 t_u32PixelIntegralBack = 0;
    int32 t_u32PixelIntegral = 0;
    int32 t_int32PixelIntegralFore = 0;
    int32 t_int32PixelFore = 0;
    float t_F32OmegaBack, t_F32OmegaFore, t_F32MicroBack, t_F32MicroFore, t_F32SigmaB, t_F32Sigma; // 类间方差;
    int32 t_int32MinValue = -1, t_int32MaxValue = -1;
    int8 t_u8Threshold = 0;
    int16 t_u16HistoGramAr[GrayScale];                     //
    memset(t_u16HistoGramAr, 0, sizeof(t_u16HistoGramAr)); //初始化灰度直方图

    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
    {
        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
        {
            if (t_u16j > IMGH * 0.7)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]] += 6;
            else if (t_u16j > IMGH * 0.4)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]] += 4;
            else if (t_u16j > 0)
                t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    //    for (t_u16j = 0; t_u16j < IMGH; t_u16j += 2)
    //    {
    //        for (t_u16i = 0; t_u16i < IMGW; t_u16i += 2)
    //        {
    //            t_u16HistoGramAr[(uint8)Img[t_u16j][t_u16i]]++; //统计灰度级中每个像素在整幅图像中的个数
    //        }
    //    }

    for (t_u16j = 0; t_u16j < GrayScale; t_u16j++)
    {
        if (t_u16HistoGramAr[t_u16j] > 0 && t_int32MinValue == -1)
            t_int32MinValue = t_u16j;
        if (t_u16HistoGramAr[255 - t_u16j] > 0 && t_int32MaxValue == -1)
            t_int32MaxValue = 255 - t_u16j;

        if (t_int32MinValue > -1 && t_int32MaxValue > -1)
            break;
    }

    if (t_int32MaxValue == t_int32MinValue)
    {
        return ((int8)(t_int32MaxValue)); // 图像中只有一个颜色
    }
    else if (t_int32MinValue + 1 == t_int32MaxValue)
    {
        return ((int8)(t_int32MinValue)); // 图像中只有二个颜色
    }

    for (t_u16j = (int16)t_int32MinValue; t_u16j <= t_int32MaxValue; t_u16j++)
    {
        t_u32Amount += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;

    for (t_u16j = (int16)t_int32MinValue; t_u16j < t_int32MaxValue; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount - t_u32PixelBack;            //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount;       //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral - t_u32PixelIntegralBack;                                               //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold = (int8)t_u16j;
        }
    }
    //    ips200_showint16(0,17,t_u8Threshold);
    t_u16j = t_u8Threshold;
    while (--t_u16j >= (int16)t_int32MinValue)
    {
        t_u16HistoGramAr[t_u8Threshold] += t_u16HistoGramAr[t_u16j];
        t_u16HistoGramAr[t_u16j] = 0;
    }

    if (t_int32MaxValue == t_u8Threshold)
    {
        return ((int8)(t_int32MaxValue)); // 图像中只有一个颜色
    }
    else if (t_u8Threshold + 1 == t_int32MaxValue)
    {
        return ((int8)(t_int32MinValue)); // 图像中只有二个颜色
    }

    /*初始化参数*/
    t_u32Amount = 0;
    t_u32PixelBack = 0;
    t_u32PixelIntegralBack = 0;
    t_u32PixelIntegral = 0;
    t_int32PixelIntegralFore = 0;
    t_int32PixelFore = 0;

    for (t_u16j = (int16)t_u8Threshold; t_u16j <= t_int32MaxValue; t_u16j++)
    {
        t_u32Amount += t_u16HistoGramAr[t_u16j];                 //  像素总数
        t_u32PixelIntegral += t_u16HistoGramAr[t_u16j] * t_u16j; //灰度值总数
    }
    t_F32SigmaB = -1;

    for (t_u16j = (int16)t_u8Threshold; t_u16j < t_int32MaxValue; t_u16j++)
    {
        t_u32PixelBack = t_u32PixelBack + t_u16HistoGramAr[t_u16j]; //前景像素点数
        t_int32PixelFore = t_u32Amount - t_u32PixelBack;            //背景像素点数
        t_F32OmegaBack = (float)t_u32PixelBack / t_u32Amount;       //前景像素百分比
                                                                    //        t_F32OmegaFore = (float)t_int32PixelFore / t_u32Amount;                                                                 //背景像素百分比
        t_F32OmegaFore = 1 - t_F32OmegaBack;
        t_u32PixelIntegralBack += t_u16HistoGramAr[t_u16j] * t_u16j;                                                          //前景灰度值
        t_int32PixelIntegralFore = t_u32PixelIntegral - t_u32PixelIntegralBack;                                               //背景灰度值
        t_F32MicroBack = (float)t_u32PixelIntegralBack / t_u32PixelBack;                                                      //前景灰度百分比
        t_F32MicroFore = (float)t_int32PixelIntegralFore / t_int32PixelFore;                                                  //背景灰度百分比
        t_F32Sigma = t_F32OmegaBack * t_F32OmegaFore * (t_F32MicroBack - t_F32MicroFore) * (t_F32MicroBack - t_F32MicroFore); //计算类间方差
        if (t_F32Sigma > t_F32SigmaB)                                                                                         //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            t_F32SigmaB = t_F32Sigma;
            t_u8Threshold = (int8)t_u16j;
        }
    }
    return t_u8Threshold; //返回最佳阈值;
}

float ThresholdGet(uint8 (*Img)[IMGW])
{
    uint16 i = 0, j = 0, N0 = 0, N1 = 0, flag = 0;
    float T0, T1, T2, T_center;
    int16 times = 0;
    uint32 S0 = 0, S1 = 0;
    T2 = BlackThres;
    do
    {
        times++;
        for (i = 0; i < IMGH; i += 2)
        {
            for (j = 0; j < IMGW; j += 2)
            {
                if (Img[i][j] < T2)
                {
                    S0 += Img[i][j];
                    N0++;
                }
                else
                {
                    S1 += Img[i][j];
                    N1++;
                }
            }
        }
        T0 = S0 / N0;
        T1 = S1 / N1;
        T_center = (T0 + T1) / 2;
        if (T2 < T_center)
        {
            if ((T_center - T2) > ERROR)
            {
                flag = 1;
            }
            else
            {
                flag = 0;
            }
        }
        else
        {
            if ((T2 - T_center) > ERROR)
            {
                flag = 1;
            }
            else
            {
                flag = 0;
            }
        }
        T2 = T_center;
        BlackThres = T2;
    } while (flag);
    return BlackThres;
}
