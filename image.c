#include "perspective.h"
#include "image.h"

extern TRACK_BORDER_INFO g_Border;
extern TRACK_TYPE_INFO g_TrackType; /*赛道类型*/

/*图像处理全局变量存放*/
int16 Threeroads_Last_Line, Pflag_Last_Line, Roundabout_Last_Line_Out;                            // Roundabout_Last_Line_In
const uint8 ThreeRoads_End = 0.1 * IMGH, Pflag_End = 0.3 * IMGH, Roundabout_Out_End = 0.3 * IMGH; // Roundabout_In_End= 0.3*IMGH
uint8 Normal_Prospect_Front;
uint8 Normal_Prospect_Later;

uint8 Center;

//前瞻
int16 prospect_front;
int16 prospect_later;

//图像前瞻
float rate = 0.2;
int16 Prospect_Image;
int16 Gap;
uint8 beep_flag;
float res;
float Variance; //方差
float parameterB,parameterA;
float last_err;     //中线上次误差

uint8 RRoundabout_TimeToTurn;
uint8 LRoundabout_TimeToTurn;

//直道行数
uint8 straight_lines = 24;
uint8 last_straight_lines;
uint8 last_last_straight_lines;
uint8 Single_missing=0;

float Angle_offset; //环岛计算角度

int16 Left_Max_angle;
int16 Right_Max_angle;

uint8 Carbarn_Left_Angle;
uint8 Carbarn_Right_Angle;

uint8 Zebra_Switch_Flag;
uint8 Error_Reserve_Flag;

float Angle; //陀螺仪计算角度

uint8 Ramp_Refuse_Flag;
float ThreeRoads_Finsh_Flag = 0;

uint8 Roundabout_Wire_Flag;
uint8 Roundabout_To_Straight_Flag;

float Perspective_Normal_Width;

int16 LastData_X;
int16 LastData_Y;

//环岛画线
float LPflag_TimeToTurn;
float RPflag_TimeToTurn;
uint8 RRoundabout_TimeToTurn;
uint8 LRoundabout_TimeToTurn;


uint8 Garage_Left_Angle;
uint8 Garage_Right_Angle;
float Garage_Left_Times;
float Garage_Right_Times;


uint8 Gear;

float err_duty;
int16 Pturn = 0;


Err_S err_speed;
PID_S pid_speed;

Err_D err_angel;
PID_D pid_angel;


INT_POINT_INFO GetOptimumColumn(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_stBorder, TRACK_TYPE_INFO *p_TrackType)
{
    INT_POINT_INFO t_stReturnPoint = {0, 0};
    int16 t_i16LoopY; /*用来循环行*/
    int16 t_i16LoopX;

    /*用来循环列*/
    INT_POINT_INFO t_stTemp_LX = {0, 0}; /*用来临时保存从左往右找的最优点*/
    INT_POINT_INFO t_stTemp_RX = {0, 0}; /*用来临时保存从右往左找的最优点*/

    p_stBorder->m_OptimalPoint.m_i16x = 0; //全局最优解的点
    p_stBorder->m_OptimalPoint.m_i16y = 0;

    p_stBorder->m_LOptimalPoint.m_i16x = 0; //左边最优解的点
    p_stBorder->m_LOptimalPoint.m_i16y = 0;

    p_stBorder->m_ROptimalPoint.m_i16x = 0; //右边最优解的点
    p_stBorder->m_ROptimalPoint.m_i16y = 0;

    p_stBorder->m_CenterLinePoint.m_i16x = 0; //中线对应的的白行的最大高度
    p_stBorder->m_CenterLinePoint.m_i16y = 0;
    p_stBorder->m_u32LAllArea = 0;
    p_stBorder->m_u32RAllArea = 0;
    memset(p_stBorder->m_u16LineBAr, 0, sizeof(p_stBorder->m_u16LineBAr));
    memset(p_stBorder->m_u16MyLineBAr, 0, sizeof(p_stBorder->m_u16MyLineBAr));
    //整个库的所有while循环开头都做了越界保护和对抗第一次判断
    t_i16LoopX = IMGW;
    while ((--t_i16LoopX) + 1) /*+1是为了不漏掉第0列*/
    {
        t_i16LoopY = IMGH - 2;
        while (t_i16LoopY > (IMGH / 2) && ((B_WHITE == InImg[t_i16LoopY][t_i16LoopX]) || (B_WHITE == InImg[t_i16LoopY + 1][t_i16LoopX]))) /*循环列,通时检测白点*/
        {
            (p_stBorder->m_u16MyLineBAr[t_i16LoopX])++;
            (p_stBorder->m_u16LineBAr[t_i16LoopX])++; //如果是白点,那么对应列的白点数+1;
            t_i16LoopY--;
        }
        while (t_i16LoopY && (B_WHITE == InImg[t_i16LoopY][t_i16LoopX])) /*循环列,通时检测白点*/
        {
            if (t_i16LoopY > rate * IMGH)
                (p_stBorder->m_u16MyLineBAr[t_i16LoopX])++;
            (p_stBorder->m_u16LineBAr[t_i16LoopX])++; //如果时白点,那么对应列的白点数+1;
            t_i16LoopY--;
        }
    }

    /*最优列在入环时存在反打角的问题 希望通过人为处理的方式进行解决*/
    if (g_TrackType.m_u8LRoundaboutFlag == 3) //|| g_TrackType.m_u8LRoundaboutFlag==4 || g_TrackType.m_u8LRoundaboutFlag==5
    {
        for (t_i16LoopX = IMGW - 1; t_i16LoopX >= IMGW / 2; t_i16LoopX--)
        {
            if (p_stBorder->m_u16LineBAr[t_i16LoopX] > 50)
                p_stBorder->m_u16LineBAr[t_i16LoopX] = 0;
            else if (t_i16LoopX < IMGW - 20)
                break;
        }
    }
    if (g_TrackType.m_u8RRoundaboutFlag == 3) //|| g_TrackType.m_u8RRoundaboutFlag==4 || g_TrackType.m_u8RRoundaboutFlag==5
    {
        for (t_i16LoopX = 0; t_i16LoopX <= IMGW / 2; t_i16LoopX++)
        {
            if (p_stBorder->m_u16LineBAr[t_i16LoopX] > 50)
                p_stBorder->m_u16LineBAr[t_i16LoopX] = 0;
            else if (t_i16LoopX > 20)
                break;
        }
    }
    if (g_TrackType.m_u8CrossFlag == 4)
    {
        for (t_i16LoopX = 0; t_i16LoopX <= 30; t_i16LoopX++)
            p_stBorder->m_u16LineBAr[t_i16LoopX] = 0;
        for (t_i16LoopX = IMGW - 1; t_i16LoopX >= IMGW - 31; t_i16LoopX--)
            p_stBorder->m_u16LineBAr[t_i16LoopX] = 0;
    }

    //筛选出左右白点最大的点作为左右的最优解
    t_i16LoopX = IMGW - 1;
    while (t_i16LoopX--)
    {
        if (p_stBorder->m_u16LineBAr[t_i16LoopX] >= t_stTemp_RX.m_i16y)
        {
            t_stTemp_RX.m_i16y = p_stBorder->m_u16LineBAr[t_i16LoopX];
            t_stTemp_RX.m_i16x = t_i16LoopX;
        }
        if (t_i16LoopX < IMGW / 2 && p_stBorder->m_u16LineBAr[t_i16LoopX] >= p_stBorder->m_LOptimalPoint.m_i16y)
        {
            p_stBorder->m_LOptimalPoint.m_i16y = p_stBorder->m_u16LineBAr[t_i16LoopX];
            p_stBorder->m_LOptimalPoint.m_i16x = t_i16LoopX;
        }
        if (t_i16LoopX < IMGW / 2)
            p_stBorder->m_u32LAllArea += p_stBorder->m_u16LineBAr[t_i16LoopX];
    }
    t_i16LoopX = 0;
    while ((++t_i16LoopX) < IMGW - 1)
    {
        if (p_stBorder->m_u16LineBAr[t_i16LoopX - 1] >= t_stTemp_LX.m_i16y)
        {
            t_stTemp_LX.m_i16y = p_stBorder->m_u16LineBAr[t_i16LoopX - 1];
            t_stTemp_LX.m_i16x = t_i16LoopX;
        }
        if (t_i16LoopX > IMGW / 2 && p_stBorder->m_u16LineBAr[t_i16LoopX] >= p_stBorder->m_ROptimalPoint.m_i16y)
        {
            p_stBorder->m_ROptimalPoint.m_i16y = p_stBorder->m_u16LineBAr[t_i16LoopX];
            p_stBorder->m_ROptimalPoint.m_i16x = t_i16LoopX;
        }
        if (t_i16LoopX > IMGW / 2)
            p_stBorder->m_u32RAllArea += p_stBorder->m_u16LineBAr[t_i16LoopX];
    }

    //如果左右都找到了,更新全局解解,同时更新返回值的点
    if (t_stTemp_LX.m_i16x != 0 && t_stTemp_RX.m_i16x != 0)
    {
        t_stReturnPoint.m_i16x = (int16)((t_stTemp_RX.m_i16x + t_stTemp_LX.m_i16x) * 0.5f);
        t_stReturnPoint.m_i16y = (int16)(IMGH - p_stBorder->m_u16LineBAr[t_stReturnPoint.m_i16x]);
    }
    if (myabs(t_stTemp_RX.m_i16x - t_stTemp_LX.m_i16x) > IMGW * 0.5) // IMGH*2
    {
        if (p_stBorder->m_u32LAllArea < p_stBorder->m_u32RAllArea) //选左边
        {
            t_stReturnPoint.m_i16x = (int16)((t_stReturnPoint.m_i16x + t_stTemp_LX.m_i16x) * 0.5f);
            t_stReturnPoint.m_i16y = (int16)(IMGH - p_stBorder->m_u16LineBAr[t_stReturnPoint.m_i16x]);
        }
        if (p_stBorder->m_u32LAllArea > p_stBorder->m_u32RAllArea) //选右边
        {
            t_stReturnPoint.m_i16x = (int16)((t_stTemp_RX.m_i16x + t_stReturnPoint.m_i16x) * 0.5f);
            t_stReturnPoint.m_i16y = (int16)(IMGH - p_stBorder->m_u16LineBAr[t_stReturnPoint.m_i16x]);
        }
    }

    p_stBorder->m_OptimalPoint = t_stReturnPoint; //将最优点返回直接赋值到p_stBorder结构体中,实际函数也返回了这个变量,在调用的地方使用函数有一个ST_2D_INT_POINT_INFO类型的变量来接也可以
                                                  //    if(p_TrackType->m_u8LRoundaboutFlag==3 || p_TrackType->m_u8RRoundaboutFlag ==3) p_stBorder->m_OptimalPoint.m_i16x = IMGW/2;
    p_stBorder->m_CenterLinePoint.m_i16x = IMGW >> 1;
    p_stBorder->m_CenterLinePoint.m_i16y = IMGH - p_stBorder->m_u16LineBAr[p_stBorder->m_CenterLinePoint.m_i16x]; //从下往上
    return t_stReturnPoint;                                                                                       //返回实际计算出的最优点
}

void GetBorder(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    int16 i;
    const uint8 t_u8OptPntSetoffX = 2;
    //    int16 i,k=0;
    /*每行找线,左边或者右边都会确定一个范围*/
    int16 t_i16LStartX = 0;
    int16 t_i16RStartX = 0;
    int16 t_i16LEndX = 0;
    int16 t_i16REndX = 0;
    /*遍历X坐标,遍历Y坐标*/
    // int16 t_i16LoopX = 0; //循环Y
    int16 t_i16LoopY = 0; //循环X
    /*左右找到标志位*/
    uint8 t_u8LFindFlag = NO;
    uint8 t_u8RFindFlag = NO;
    /*上一次的找线状态*/
    uint8 t_u8LFindFlagLast = NO;
    uint8 t_u8RFindFlagLast = NO;
    /*左右完成找线标志位*/
    uint8 t_u8LFinishedFlag = NO;
    uint8 t_u8RFinishedFlag = NO;
    //临时用来判断结束的点
    INT_POINT_INFO t_LInPoint = {0, 0};
    INT_POINT_INFO t_RInPoint = {0, 0};
    //左边线最大点
    p_Border->m_LMaxPoint.m_i16x = 0;
    p_Border->m_LMaxPoint.m_i16y = 0;
    //右边线最小点
    p_Border->m_RMinPoint.m_i16x = IMGW;
    p_Border->m_RMinPoint.m_i16y = 0;
    int16 t_i16Step = 7; //搜索范围
    //清空左右边线中的数据
    memset(p_Border->m_LPnt, 0, sizeof(p_Border->m_LPnt));
    memset(p_Border->m_RPnt, 0, sizeof(p_Border->m_RPnt));
    memset(p_Border->m_i8LMonotonicity, 0, sizeof(p_Border->m_i8LMonotonicity));
    memset(p_Border->m_i8RMonotonicity, 0, sizeof(p_Border->m_i8RMonotonicity));
    //清空左右边线的数量
    p_Border->m_i16LPointCnt = 0;
    p_Border->m_i16RPointCnt = 0;
    p_Border->m_i16LLostPointCnt = 0;
    p_Border->m_i16RLostPointCnt = 0;
    /*通过最优点来计算图像下半部分的边线*/
    t_i16LoopY = IMGH;   //此处在while循环之前是做了防越界的
    while (--t_i16LoopY) //开始循环图像行
    {
        /*确定搜索范围*/
        /*找线步进大小*/
        if (t_i16LoopY > IMGH * 0.8)
            t_i16Step = 4;
        else if (t_i16LoopY > IMGH * 0.2)
            t_i16Step = 8;
        else
            t_i16Step = 10;
        if (myabs(p_Border->m_OptimalPoint.m_i16x - IMGW / 2) < 3 || t_i16LoopY > 10) //如果当前最优列偏离中心线不多
        {
            t_i16LStartX = p_Border->m_OptimalPoint.m_i16x + t_u8OptPntSetoffX;
            t_i16LEndX = 1;

            t_i16RStartX = p_Border->m_OptimalPoint.m_i16x - t_u8OptPntSetoffX;
            t_i16REndX = IMGW - 2;
        }
        else //偏离太多属于弯道或者其他情况了
        {
            //如果之前的三个点都找到了,那么就用这三个点来推算范围
            if (p_Border->m_LPnt[t_i16LoopY + 1].m_i16x != 1 && p_Border->m_LPnt[t_i16LoopY + 2].m_i16x != 1 && p_Border->m_LPnt[t_i16LoopY + 3].m_i16x != 1)
            {
                t_i16LStartX = (p_Border->m_LPnt[t_i16LoopY + 1].m_i16x + p_Border->m_LPnt[t_i16LoopY + 2].m_i16x + p_Border->m_LPnt[t_i16LoopY + 3].m_i16x) / 3 + t_i16Step;
                t_i16LEndX = t_i16LStartX - (t_i16Step << 1); //左移1位是乘以2
            }
            else //如果三个点中少了一个点,那么还是按照最优解来计算范围
            {
                t_i16LStartX = p_Border->m_OptimalPoint.m_i16x + t_u8OptPntSetoffX;
                t_i16LEndX = 1;
            }
            //如果右边之前的三个点找到了
            if (p_Border->m_RPnt[t_i16LoopY + 1].m_i16x != IMGW - 2 && p_Border->m_RPnt[t_i16LoopY + 2].m_i16x != IMGW - 2 && p_Border->m_RPnt[t_i16LoopY + 3].m_i16x != IMGW - 2)
            {
                t_i16RStartX = (p_Border->m_RPnt[t_i16LoopY + 1].m_i16x + p_Border->m_RPnt[t_i16LoopY + 2].m_i16x + p_Border->m_RPnt[t_i16LoopY + 3].m_i16x) / 3 - t_i16Step;
                t_i16REndX = t_i16RStartX + (t_i16Step << 1);
            }
            else //如果右边之前的三个连续的点缺少了,那么从最优列开始计算找线范围
            {
                t_i16RStartX = p_Border->m_OptimalPoint.m_i16x - t_u8OptPntSetoffX;
                t_i16REndX = IMGW - 2;
            }
        }
        /*范围合理性检测*/
        if (t_i16LEndX < 1)
            t_i16LEndX = 1;
        if (t_i16LStartX > IMGW - 1)
            t_i16LStartX = IMGW - 1;
        if (t_i16REndX > IMGW - 2)
            t_i16REndX = IMGW - 2;
        if (t_i16RStartX < 1)
            t_i16RStartX = 1;
        if (t_i16LEndX > t_i16LStartX)
        {
            int16 temp = t_i16LEndX;
            t_i16LEndX = t_i16LStartX;
            t_i16LStartX = temp;
        }
        if (t_i16REndX < t_i16RStartX)
        {
            int16 temp = t_i16REndX;
            t_i16REndX = t_i16RStartX;
            t_i16RStartX = temp;
        }

        //进入的点可以用来判断找线时候应该结束
        t_RInPoint.m_i16x = t_i16RStartX; //赋值进入的点
        t_RInPoint.m_i16y = t_i16LoopY;
        t_LInPoint.m_i16x = t_i16LStartX; //记录当前进入时候的点
        t_LInPoint.m_i16y = t_i16LoopY;

        //先判断一波是否可以结束找线
        if (NO == t_u8LFinishedFlag) //如果左边找线没有结束
        {
            int16 LBlackPointNum = 0;
            t_i16LStartX++;                       //此处++是为了进while循环时抵消减减
            t_u8LFindFlag = NO;                   //置位左边找线
            while ((--t_i16LStartX) > t_i16LEndX) //循环,直到超出找线范围
            {
                if (B_BLACK == InImg[t_i16LoopY][t_i16LStartX] &&
                    B_WHITE == InImg[t_i16LoopY][t_i16LStartX + 1] &&
                    B_BLACK == InImg[t_i16LoopY][t_i16LStartX - 1]) //遇到白白黑跳变,那么黑色的那个点为边界点
                {
                    p_Border->m_LPnt[t_i16LoopY].m_i16x = t_i16LStartX; //搜索到边界,赋值X坐标
                    p_Border->m_LPnt[t_i16LoopY].m_i16y = t_i16LoopY;   //搜索到边界,赋值Y坐标
                    t_u8LFindFlag = YES;                                //开启当前已经找到边线
                    break;                                              //跳出找线循环
                }
                if (t_i16LoopY < IMGH / 2 && t_i16LStartX > (IMGW * 0.55) && InImg[t_i16LoopY][t_i16LStartX] == B_BLACK) //图像结束部分了
                {
                    ++LBlackPointNum;        //记录黑色点
                    if (LBlackPointNum >= 3) //左右都结束找线
                    {
                        int16 tmpY = t_i16LoopY;
                        if (p_Border->m_LPnt[t_i16LoopY + 1].m_i16x < IMGW * 0.9f)
                        {
                            int cut = (p_Border->m_LPnt[t_i16LoopY + 3].m_i16x > 1) && (p_Border->m_LPnt[t_i16LoopY + 2].m_i16x > 1) ? p_Border->m_LPnt[t_i16LoopY + 2].m_i16x - p_Border->m_LPnt[t_i16LoopY + 3].m_i16x : 5;
                            if (cut < 0)
                                cut = -cut;
                            if (cut > 5)
                                cut = 5;
                            for (; tmpY > 0; --tmpY)
                            {
                                p_Border->m_LPnt[tmpY].m_i16x = p_Border->m_LPnt[tmpY + 1].m_i16x + cut;
                                p_Border->m_LPnt[tmpY].m_i16y = tmpY;
                                // p_Border->m_i16LPointCnt++;
                                if (p_Border->m_LPnt[tmpY].m_i16x >= IMGW - 1)
                                {
                                    p_Border->m_LPnt[tmpY].m_i16x = IMGW - 1;
                                }
                                else if (p_Border->m_LPnt[tmpY].m_i16x < 1)
                                {
                                    p_Border->m_LPnt[tmpY].m_i16x = 1;
                                }
                            }
                            LBlackPointNum = 10;
                        }

                        break;
                    }
                }
            }
            if (LBlackPointNum == 10)
                break;
            if (NO == t_u8LFindFlag && (B_BLACK == InImg[t_LInPoint.m_i16y][t_LInPoint.m_i16x])) //如果第一轮找线没有找到
            {
                t_i16LEndX += 2 * t_i16Step; //扩大搜索范围,因为前一轮找线并没有破坏t_i16LEndX的数据,所以本轮还是用前面的数据重新推测范围
                t_i16LStartX = t_i16LEndX + 2 * t_i16Step;
                //                t_LInPoint.m_i16x = t_i16LStartX; //赋值左边进入点
                //                t_LInPoint.m_i16y = t_i16LoopY;
                //限幅保护
                if (t_i16LEndX < 1)
                    t_i16LEndX = 1;
                if (t_i16LStartX < 1)
                    t_i16LStartX = 1;
                if (t_i16LEndX > IMGW - 1)
                    t_i16LEndX = IMGW - 1;
                if (t_i16LStartX > IMGW - 1)
                    t_i16LStartX = IMGW - 1;
                if (t_i16LEndX > t_i16LStartX)
                {
                    int16 temp = t_i16LEndX;
                    t_i16LEndX = t_i16LStartX;
                    t_i16LStartX = temp;
                }
                t_i16LStartX++;
                while ((--t_i16LStartX) > t_i16LEndX) //循环,直到超出找线范围
                {
                    if (B_BLACK == InImg[t_i16LoopY][t_i16LStartX] &&
                        B_WHITE == InImg[t_i16LoopY][t_i16LStartX + 1] &&
                        B_BLACK == InImg[t_i16LoopY][t_i16LStartX - 1]) //遇到白白黑跳变,那么黑色的那个点为边界点
                    {
                        p_Border->m_LPnt[t_i16LoopY].m_i16x = t_i16LStartX; //搜索到边界,赋值X坐标
                        p_Border->m_LPnt[t_i16LoopY].m_i16y = t_i16LoopY;   //搜索到边界,赋值Y坐标
                        t_u8LFindFlag = YES;                                //开启当前已经找到边线
                        break;                                              //跳出找线循环
                    }
                    if (t_i16LoopY < IMGH / 2 && t_i16LStartX > (IMGW * 0.55) && InImg[t_i16LoopY][t_i16LStartX] == B_BLACK) //图像结束部分了
                    {
                        ++LBlackPointNum;        //记录黑色点
                        if (LBlackPointNum >= 3) //左右都结束找线
                        {
                            int16 tmpY = t_i16LoopY;
                            if (p_Border->m_LPnt[t_i16LoopY + 1].m_i16x < IMGW * 0.9f)
                            {
                                int cut = (p_Border->m_LPnt[t_i16LoopY + 3].m_i16x > 1) && (p_Border->m_LPnt[t_i16LoopY + 2].m_i16x > 1) ? p_Border->m_LPnt[t_i16LoopY + 2].m_i16x - p_Border->m_LPnt[t_i16LoopY + 3].m_i16x : 5;
                                if (cut < 0)
                                    cut = -cut;
                                if (cut > 5)
                                    cut = 5;
                                for (; tmpY > 0; --tmpY)
                                {
                                    p_Border->m_LPnt[tmpY].m_i16x = p_Border->m_LPnt[tmpY + 1].m_i16x + cut;
                                    p_Border->m_LPnt[tmpY].m_i16y = tmpY;
                                    // p_Border->m_i16LPointCnt++;
                                    if (p_Border->m_LPnt[tmpY].m_i16x >= IMGW - 1)
                                    {
                                        p_Border->m_LPnt[tmpY].m_i16x = IMGW - 1;
                                    }
                                    else if (p_Border->m_LPnt[tmpY].m_i16x < 1)
                                    {
                                        p_Border->m_LPnt[tmpY].m_i16x = 1;
                                    }
                                }
                                LBlackPointNum = 10;
                            }

                            break;
                        }
                    }
                }
                if (LBlackPointNum == 10)
                    break;
            }
            if (NO == t_u8LFindFlag) //如果本轮还没有找到,那么直接赋值最左边边界(相当于没有找到边界,但是维持了边界的单调性)
            {
                p_Border->m_LPnt[t_i16LoopY].m_i16x = 1;
                p_Border->m_LPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                p_Border->m_i8LMonotonicity[t_i16LoopY] = YES_2; //未找到线需要补线
                p_Border->m_i16LLostPointCnt++;
            }
            else //如果找到了左边边界
            {
                p_Border->m_i16LPointCnt++; //左边边界数量＋1

                //此时进行需要补线的标志位
                if (YES == t_u8LFindFlagLast) //如果上一个点找到
                {
                    //计算单调性是否成立
                    if (p_Border->m_LPnt[t_i16LoopY].m_i16x - p_Border->m_LPnt[t_i16LoopY + 1].m_i16x < 0) //当前行的点减去上一行的点必然大于0或等于0,不能违反边界的单调性一致.如果出现当单调异常,必然出现特殊元素或者是干扰情况
                    {
                        p_Border->m_i8LMonotonicity[t_i16LoopY] = YES; //违反单调性需要补线
                        //异常状况,开启补线
                        //是否应该补线呢,还是在此处标记应该补线,这儿我选择在此处标记补线,标记补线,后面统一区分情况之后再补线,即使找线错误,还是维持当前的找线状态.
                        //此处复用单调性数组,用来记录边线是否应该补线
                        //后面应该会有起奇效
                    }
                    else if (p_Border->m_LPnt[t_i16LoopY].m_i16x - p_Border->m_LPnt[t_i16LoopY + 1].m_i16x == 0 && p_Border->m_i8LMonotonicity[t_i16LoopY + 1] == YES)
                    {
                        p_Border->m_i8LMonotonicity[t_i16LoopY] = YES; //违反单调性需要补线
                    }
                    else //如果单调性正常,那么不考虑补线的情况.
                    {
                        p_Border->m_i8LMonotonicity[t_i16LoopY] = NO;
                    }
                }

                if (t_i16LoopY > 10)
                {
                    if (p_Border->m_LMaxPoint.m_i16x < p_Border->m_LPnt[t_i16LoopY].m_i16x) //更新左边纵坐标最大值的点(后面补线可能需要)
                    {
                        p_Border->m_LMaxPoint.m_i16x = p_Border->m_LPnt[t_i16LoopY].m_i16x;
                        p_Border->m_LMaxPoint.m_i16y = t_i16LoopY;
                    }
                }
            }
        }

        /*搜索右边边线同上*/
        if (NO == t_u8RFinishedFlag) //如果右边全图搜线没有结束
        {
            int RBlackPointNum = 0;
            t_i16RStartX--;                       //对抗while循环进入时判断之前的++
            t_u8RFindFlag = NO;                   //置位右边当行找线完成标志位,在下面的搜索中标记本行找线结束,准备跳出找线循环
            while ((++t_i16RStartX) < t_i16REndX) //开始找线循环.(为什么大量使用while循环,因为测试发现while循环的效率高于for循环30%左右)
            {
                if (B_BLACK == InImg[t_i16LoopY][t_i16RStartX] &&
                    B_WHITE == InImg[t_i16LoopY][t_i16RStartX - 1] &&
                    B_BLACK == InImg[t_i16LoopY][t_i16RStartX + 1]) //判断到白黑黑,中间黑点为边界
                {
                    p_Border->m_RPnt[t_i16LoopY].m_i16x = t_i16RStartX; //赋值边界数据,先赋值的X,再赋值的Y
                    p_Border->m_RPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                    t_u8RFindFlag = YES; //打开此行找线完成标志位
                    break;
                }
                if (t_i16LoopY < IMGH / 2 - 3 && t_i16RStartX < (IMGW * 0.45) && InImg[t_i16LoopY][t_i16RStartX] == B_BLACK) //图像结束部分了
                {
                    ++RBlackPointNum;        //记录黑色点
                    if (RBlackPointNum >= 3) //左右都结束找线
                    {
                        int tmpY = t_i16LoopY;
                        if (p_Border->m_RPnt[t_i16LoopY + 1].m_i16x > IMGW * 0.1f)
                        {
                            int cut = (p_Border->m_RPnt[t_i16LoopY + 3].m_i16x != IMGW - 2) && (p_Border->m_RPnt[t_i16LoopY + 2].m_i16x != IMGW - 2) ? p_Border->m_RPnt[t_i16LoopY + 2].m_i16x - p_Border->m_RPnt[t_i16LoopY + 3].m_i16x : -5;
                            if (cut > 0)
                                cut = -cut;
                            if (cut < -5)
                                cut = -5;
                            for (; tmpY > 0; --tmpY)
                            {
                                p_Border->m_RPnt[tmpY].m_i16x = p_Border->m_RPnt[tmpY + 1].m_i16x + cut;
                                p_Border->m_RPnt[tmpY].m_i16y = (int16)tmpY;
                                // p_Border->m_i16RPointCnt++;
                                if (p_Border->m_RPnt[tmpY].m_i16x < 1)
                                {
                                    p_Border->m_RPnt[tmpY].m_i16x = 1;
                                }
                                else if (p_Border->m_RPnt[tmpY].m_i16x > IMGW - 2)
                                {
                                    p_Border->m_RPnt[tmpY].m_i16x = IMGW - 2;
                                }
                            }
                        }
                        RBlackPointNum = 10;
                        break;
                    }
                }
            }
            if (RBlackPointNum == 10)
                break;
            if (NO == t_u8RFindFlag && (B_BLACK == InImg[t_RInPoint.m_i16y][t_RInPoint.m_i16x])) //如果第一轮找线没有找到
            {
                t_i16REndX -= 2 * t_i16Step; //扩大找线范围再次找线(为什么这样就扩大了,与左边找线一样)
                t_i16RStartX = t_i16REndX - 2 * t_i16Step;
                //                t_RInPoint.m_i16x = t_i16RStartX; //确定当前进入的点
                //                t_RInPoint.m_i16y = t_i16LoopY;
                //限幅保护
                if (t_i16REndX < 1)
                    t_i16REndX = 1;
                if (t_i16RStartX < 1)
                    t_i16RStartX = 1;
                if (t_i16REndX > IMGW - 1)
                    t_i16REndX = IMGW - 1;
                if (t_i16RStartX > IMGW - 1)
                    t_i16RStartX = IMGW - 1;
                if (t_i16REndX < t_i16RStartX)
                {
                    int16 temp = t_i16REndX;
                    t_i16REndX = t_i16RStartX;
                    t_i16RStartX = temp;
                }
                t_i16RStartX--;                       //对抗while
                while ((++t_i16RStartX) < t_i16REndX) //开始找线循环.(为什么大量使用while循环,因为测试发现while循环的效率高于for循环30%左右)
                {
                    if (B_BLACK == InImg[t_i16LoopY][t_i16RStartX] &&
                        B_WHITE == InImg[t_i16LoopY][t_i16RStartX - 1] &&
                        B_BLACK == InImg[t_i16LoopY][t_i16RStartX + 1]) //判断到白黑黑,中间黑点为边界
                    {
                        p_Border->m_RPnt[t_i16LoopY].m_i16x = t_i16RStartX; //赋值边界数据,先赋值的X,再赋值的Y
                        p_Border->m_RPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                        t_u8RFindFlag = YES; //打开此行找线完成标志位
                        break;
                    }
                    if (t_i16LoopY < IMGH / 2 - 3 && t_i16RStartX < (IMGW * 0.45) && InImg[t_i16LoopY][t_i16RStartX] == B_BLACK) //图像结束部分了
                    {
                        ++RBlackPointNum;        //记录黑色点
                        if (RBlackPointNum >= 3) //左右都结束找线
                        {
                            int16 tmpY = t_i16LoopY;
                            if (p_Border->m_RPnt[t_i16LoopY + 1].m_i16x > IMGW * 0.1f)
                            {
                                int cut = (p_Border->m_RPnt[t_i16LoopY + 3].m_i16x != IMGW - 2) && (p_Border->m_RPnt[t_i16LoopY + 2].m_i16x != IMGW - 2) ? p_Border->m_RPnt[t_i16LoopY + 2].m_i16x - p_Border->m_RPnt[t_i16LoopY + 3].m_i16x : -5;
                                if (cut > 0)
                                    cut = -cut;
                                if (cut < -5)
                                    cut = -5;
                                for (; tmpY > 0; --tmpY)
                                {
                                    p_Border->m_RPnt[tmpY].m_i16x = p_Border->m_RPnt[tmpY + 1].m_i16x + cut;
                                    p_Border->m_RPnt[tmpY].m_i16y = tmpY;
                                    // p_Border->m_i16RPointCnt++;
                                    if (p_Border->m_RPnt[tmpY].m_i16x < 1)
                                    {
                                        p_Border->m_RPnt[tmpY].m_i16x = 1;
                                    }
                                    else if (p_Border->m_RPnt[tmpY].m_i16x > IMGW - 2)
                                    {
                                        p_Border->m_RPnt[tmpY].m_i16x = IMGW - 2;
                                    }
                                }
                            }
                            RBlackPointNum = 10;
                            break;
                        }
                    }
                }
                if (RBlackPointNum == 10)
                    break;
            }

            if (NO == t_u8RFindFlag) //如果补救找线还是没有找到线
            {
                p_Border->m_RPnt[t_i16LoopY].m_i16x = IMGW - 2; //赋值最右边边界,默认为没找到线
                p_Border->m_RPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                p_Border->m_i8RMonotonicity[t_i16LoopY] = YES_2;
                p_Border->m_i16RLostPointCnt++; //右边丢失点数加1
            }
            else
            {
                p_Border->m_i16RPointCnt++; //找到边界,右边点数+1

                //此时进行需要补线的标志位
                if (YES == t_u8RFindFlagLast) //如果上一个点找到
                {
                    //计算单调性是否成立
                    if (p_Border->m_RPnt[t_i16LoopY].m_i16x - p_Border->m_RPnt[t_i16LoopY + 1].m_i16x > 0) //当前行的点减去上一行的点必然小于0或等于0,不能违反边界的单调性一致.如果出现当单调异常,必然出现特殊元素或者是干扰情况
                    {
                        p_Border->m_i8RMonotonicity[t_i16LoopY] = YES;
                        //异常状况,开启补线
                        //是否应该补线呢,还是在此处标记应该补线,这儿我选择在此处标记补线,标记补线,后面统一区分情况之后再补线,即使找线错误,还是维持当前的找线状态.
                        //此处复用单调性数组,用来记录边线是否应该补线
                        //后面应该会有起奇效
                    }
                    else if (p_Border->m_RPnt[t_i16LoopY].m_i16x - p_Border->m_RPnt[t_i16LoopY + 1].m_i16x == 0 && p_Border->m_i8RMonotonicity[t_i16LoopY + 1] == YES)
                    {
                        p_Border->m_i8RMonotonicity[t_i16LoopY] = YES; //违反单调性需要补线
                    }
                    else //如果单调性正常,那么不考虑补线的情况.
                    {
                        p_Border->m_i8RMonotonicity[t_i16LoopY] = NO;
                    }
                }
            }
            if (t_i16LoopY > 5) //如果没有到图像顶部,那么更新一下右边的纵坐标点最小的点.可能这个点就是折点,后面补线可能需要用到
            {
                if (p_Border->m_RMinPoint.m_i16x > p_Border->m_RPnt[t_i16LoopY].m_i16x)
                {
                    p_Border->m_RMinPoint.m_i16x = p_Border->m_RPnt[t_i16LoopY].m_i16x;
                    p_Border->m_RMinPoint.m_i16y = t_i16LoopY;
                }
            }
        }
    }
    //检测是否可以结束找线
    if (t_i16LoopY < IMGW / 2)
    {
        if (YES == t_u8LFindFlag && YES == t_u8RFindFlag)
        {
            if (p_Border->m_RPnt[t_i16LoopY].m_i16x - p_Border->m_LPnt[t_i16LoopY].m_i16x < IMGW * 0.05) //如果左右边界都找到了,但是左右边界点相聚很近的时候,考虑结束找线.
            {
                t_u8LFinishedFlag = YES;
                t_u8RFinishedFlag = YES;
                if (p_Border->m_RPnt[t_i16LoopY].m_i16x - p_Border->m_LPnt[t_i16LoopY].m_i16x < 0) //如果是负数,那么这是非常严重的找线错误.
                {
                    //清除当前找到的点,同时回退找到的点数
                    p_Border->m_LPnt[t_i16LoopY].m_i16x = 1;
                    p_Border->m_LPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                    p_Border->m_RPnt[t_i16LoopY].m_i16x = IMGW - 2;
                    p_Border->m_RPnt[t_i16LoopY].m_i16y = t_i16LoopY;
                    p_Border->m_i16LPointCnt--;
                    p_Border->m_i16RPointCnt--;
                }
                return; //直接结束找线
            }
            else //暂时先不处理这种情况
            {
                ;
            }
        }
        if (NO == t_u8LFindFlag) //如果左边找到线了
        {
            // if (p_Border->m_LPnt[t_i16LoopY].m_i16x > IMGW / 2) //当前行边界已经查出了图像中间线(那么非常可能是弯道,准备结束)
            // {
            if (0 <= t_i16LoopY - 2 && t_LInPoint.m_i16y != 0) //这儿可以不限制循环Y的范围,外层已经限制了.然后判断进入点是否真正赋值了
            {
                //如果进入点为黑点,且连续三行向上都是黑点
                if (B_BLACK == InImg[t_LInPoint.m_i16y][t_LInPoint.m_i16x] && B_BLACK == InImg[t_LInPoint.m_i16y - 1][t_LInPoint.m_i16x]) // && B_BLACK == InImg[t_LInPoint.m_i16y - 2][t_LInPoint.m_i16x])
                {
                    t_u8LFinishedFlag = YES; //结束当前的左边全图找线
                }
            }
            // }
        }
        else
        {
            ;
        }

        if (NO == t_u8RFindFlag) //如果右边找到边线了
        {
            // if (p_Border->m_LPnt[t_i16LoopY].m_i16x < IMGW / 2) //判断是否超过图像的一般(注意左右超过的意义是不一样的,这儿只是描述,程序实现手段是正确的)
            // {
            if (0 <= t_i16LoopY - 2 && t_RInPoint.m_i16y != 0) //判断当前图像是否到达图像顶部,同时找线入口点是否正确赋值
            {
                // 已经进入了已经进入黑色区域,且向上三个点都同时满足黑点,那么本边找线结束
                if (B_BLACK == InImg[t_RInPoint.m_i16y][t_RInPoint.m_i16x] && B_BLACK == InImg[t_RInPoint.m_i16y - 1][t_RInPoint.m_i16x]) //&& B_BLACK == InImg[t_RInPoint.m_i16y - 2][t_RInPoint.m_i16x])
                {
                    t_u8RFinishedFlag = YES;
                }
            }
            // }
        }
        else
        {
            ;
        }

        if (NO == t_u8LFindFlag && NO == t_u8RFindFlag)
        {
            //如果左右都没有找到,那么考虑当前是否可以,可能为十字或三叉路口,当前暂时不处理
        }
    }
    //赋值上一次的找线状态
    t_u8LFindFlagLast = t_u8LFindFlag;
    t_u8RFindFlagLast = t_u8RFindFlag;
    if (p_Border->m_LPnt[t_i16LoopY].m_i16x < 1)
    {
        p_Border->m_LPnt[t_i16LoopY].m_i16x = 1;
    }
    else if (p_Border->m_LPnt[t_i16LoopY].m_i16x > IMGW - 2)
    {
        p_Border->m_LPnt[t_i16LoopY].m_i16x = IMGW - 2;
    }
    if (p_Border->m_RPnt[t_i16LoopY].m_i16x < 1)
    {
        p_Border->m_RPnt[t_i16LoopY].m_i16x = 1;
    }
    else if (p_Border->m_RPnt[t_i16LoopY].m_i16x > IMGW - 2)
    {
        p_Border->m_RPnt[t_i16LoopY].m_i16x = IMGW - 2;
    }
}

void Perspective_Change(TRACK_BORDER_INFO *p_Border)
{
    int16 i;
    for (i = IMGH - 1; i >= 5; i--)
    {
        if (p_Border->m_LPnt[i].m_i16x == 0)
            p_Border->m_LPnt[i].m_i16x = 1;
        if (p_Border->m_RPnt[i].m_i16x == 0 || p_Border->m_RPnt[i].m_i16x == 1)
            p_Border->m_RPnt[i].m_i16x = IMGW - 2;
        p_Border->m_PRPnt[i].m_f16x = (X_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
        //        p_Border->m_PRPnt[i].m_f16y=(Y_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
        p_Border->m_PRPnt[i].m_f16y = p_Border->m_RPnt[i].m_i16y;
        p_Border->m_PLPnt[i].m_f16x = (X_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
        //        p_Border->m_PLPnt[i].m_f16y=(Y_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
        p_Border->m_PLPnt[i].m_f16y = p_Border->m_LPnt[i].m_i16y;
    }
}

void state_judgement(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType)
{
    float temp;
    float x1, x2, y1, y2;
    int16 i, j, k, t, X_pos, Y_pos, num, sum;
    int16 times = 0, LeftTimes = 0, RightTimes = 0, left_missingline_times = 0, right_missingline_times = 0, MyLine_Times = 0, Zebra_times;
    int16 block_times = 0, black_blocks = 0;
    int16 Lflag = 0, Rflag = 0, Lureason = 0, Rureason = 0, Lfind = 0, Rfind = 0;
    int16 ThreeRoads_Lpoint[2] = {0}, ThreeRoads_Rpoint[2] = {0}, ThreeRoads_Tpoint[2] = {0};
    int16 Cross_LBpoint[2] = {0}, Cross_RBpoint[2] = {0}, Cross_LTpoint[2] = {0}, Cross_RTpoint[2] = {0};
    int16 Roundabout_LBpoint[2] = {0}, Roundabout_RBpoint[2] = {0}, Roundabout_LTpoint[2] = {0}, Roundabout_RTpoint[2] = {0};
    int16 Garage_Bpoint[2] = {0};
    int16 P_times = 0;
    int16 Zebra_Left_Max = 0, Zebra_Right_Max = 0;
    int16 Zebra_start = 0, Zebra_end = 0, LeftMost, RightMost;
    uint8 Loop_Start, Loop_End;
    uint8 Left_Round_Straight = 0, Right_Round_Straight = 0, Limit_Flag = 0;
    int16 Temp_Round, Temp_ThreeRoads;

    if (!p_TrackType->m_u8ThreeRoadsFlag && !p_TrackType->m_u8LRoundaboutFlag && !p_TrackType->m_u8RRoundaboutFlag &&
        !p_TrackType->m_u8CrossFlag && !p_TrackType->m_u8RampFlag && !p_TrackType->m_u8Pflag && !p_TrackType->m_u8SmallSFlag &&
        !p_TrackType->m_u8ZebraCrossingFlag && !p_TrackType->m_u8CarBarnState && !p_TrackType->m_u8CarRunningState)
    {
        i = IMGH - 5;
        while (i >= Prospect_Image) //环岛封住有长直道 前瞻要拉远 0.7     0.3 太远了 看到别的赛道了  后面加误判校正
        {
            //             if (p_Border->m_i8LMonotonicity[i] && !Lflag)
            //             if (p_Border->m_PLPnt[i].m_f16x-p_Border->m_PLPnt[i-2].m_i16x>5 && p_Border->m_PLPnt[i].m_f16x>p_Border->m_PLPnt[i+2].m_i16x && !Lflag)
            //不是判断尖点 而是违背了单调性的点
            //             if ((p_Border->m_PLPnt[i+3].m_i16x - p_Border->m_PLPnt[i].m_f16x>10 || p_Border->m_PLPnt[i].m_f16x>IMGW-31) && !Lureason) Lureason=1;
            //             /if (p_Border->m_PLPnt[i].m_f16x > p_Border->m_PLPnt[i-3].m_i16x  && p_Border->m_PLPnt[i-3].m_i16x >= p_Border->m_PLPnt[i-6].m_i16x && !Lflag)  //&& !Lureason
            if (!Lflag)
            {
                //                     if (p_Border->m_PLPnt[i-1].m_f16x-p_Border->m_PLPnt[i].m_f16x>2*Gap && !Lureason) Lureason=1;  // ||p_Border->m_LPnt[i].m_i16x>IMGW/2)
                //                     if (!Lureason)
                //                     {
                if (p_Border->m_LPnt[i].m_i16x == 1)
                    left_missingline_times++;
                else if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                         //                                 p_Border->m_LPnt[i-1].m_i16x > p_Border->m_LPnt[i-Gap-1].m_i16x &&
                         p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i + Gap].m_i16x) //
                /*&p_Border->m_PLPnt[i].m_f16x - p_Border->m_PLPnt[i-1].m_f16x > 0 &&
                     p_Border->m_PLPnt[i-1].m_f16x > p_Border->m_PLPnt[i-2].m_f16x &&
                     p_Border->m_PLPnt[i-2].m_f16x > p_Border->m_PLPnt[i-3].m_f16x&*/
                { //难道是黑线不平整导致Lflag的问题   //后面一个需要 "</>="
                    if (left_missingline_times < 40)
                        Lflag = i;
                    else
                        Lflag = 0;
                    if (Lflag)
                    {
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Lflag = k;
                                break;
                            }
                        }
                    }
                }
                //                     }
            }
            //             else if (p_Border->m_LPnt[i].m_f16x==1) left_missingline_times++;
            //            if (p_Border->m_i8RMonotonicity[i] && !Rflag)
            //             if (p_Border->m_PRPnt[i-2].m_i16x-p_Border->m_PRPnt[i].m_f16x>5 && p_Border->m_PRPnt[i].m_f16x<p_Border->m_PRPnt[i+2].m_i16x && !Rflag)
            //             if ((p_Border->m_PRPnt[i+3].m_i16x - p_Border->m_PRPnt[i].m_f16x >10 || p_Border->m_PRPnt[i].m_f16x<30) && !Rureason) Rureason=1;
            //             if (p_Border->m_PRPnt[i-3].m_i16x > p_Border->m_PRPnt[i].m_f16x && p_Border->m_PRPnt[i-3].m_i16x <= p_Border->m_PRPnt[i-6].m_i16x && !Rflag && !Rureason)
            if (!Rflag)
            {
                //                     if (p_Border->m_PRPnt[i].m_f16x-p_Border->m_PRPnt[i-1].m_f16x>2*Gap && !Rureason) Rureason=1;  // ||p_Border->m_RPnt[i].m_i16x <IMGW/2
                //                     if (!Rureason)
                //                     {
                if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                    right_missingline_times++;
                else if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                         //                                 p_Border->m_RPnt[i-1].m_i16x < p_Border->m_RPnt[i-Gap-1].m_i16x &&
                         p_Border->m_RPnt[i].m_i16x <= p_Border->m_RPnt[i + Gap].m_i16x) //
                                                                                         /*p_Border->m_PRPnt[i-1].m_f16x - p_Border->m_PRPnt[i].m_f16x > 0 &&
                                                                                             p_Border->m_PRPnt[i-2].m_f16x > p_Border->m_PRPnt[i-1].m_f16x &&
                                                                                             p_Border->m_PRPnt[i-3].m_f16x > p_Border->m_PRPnt[i-2].m_f16x*/
                {
                    if (right_missingline_times < 40)
                        Rflag = i;
                    else
                        Rflag = 0;
                    if (Rflag)
                    {
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                p_Border->m_RPnt[k].m_i16x <= p_Border->m_RPnt[k + 1].m_i16x)
                            {
                                Rflag = k;
                                break;
                            }
                        }
                    }
                }
                //                     }
            }
            i--;
        }

        /*测试代码*/
        //             if (Lflag)
        //             {
        //                 ips200_showint16(0,7,Lflag);
        //                 ips200_showint16(0,8,p_Border->m_LPnt[Lflag].m_i16x);
        //                 for (j=0;j<=187;j++)
        //                 {
        //                     image[(int16)p_Border->m_LPnt[Lflag].m_i16y][j]=0;
        //                     image[(int16)p_Border->m_LPnt[Lflag-2*Gap].m_i16y][j]=0;
        //                 }
        //                 ips200_showint16(0,9,p_Border->m_LPnt[Lflag].m_i16x - p_Border->m_LPnt[Lflag-2*Gap].m_i16x);
        //             }
        //             if (Rflag)
        //             {
        //                 ips200_showint16(120,7,Rflag);
        //                 ips200_showint16(120,8,p_Border->m_RPnt[Rflag].m_i16x);
        //                 for (j=0;j<=187;j++)
        //                 {
        //                     image[(int16)p_Border->m_LPnt[Rflag].m_i16y][j]=0;
        //                     image[(int16)p_Border->m_LPnt[Rflag-2*Gap].m_i16y][j]=0;
        //                 }
        //                 ips200_showint16(120,9,p_Border->m_RPnt[Rflag-2*Gap].m_i16x - p_Border->m_RPnt[Rflag].m_i16x);
        //             }
        //             if (Lflag || Rflag)  p_TrackType->m_u8RampReadyFlag=0;

        if (!Lflag && !Rflag)
        {
            //
            //                 if (!Ramp_Refuse_Flag)
            //                 {
            //                     if (p_TrackType->m_u8RampReadyFlag)
            //                     {
            //                         if (Angle>8)
            //                         {
            //                             Ramp_Times++;
            //                             p_TrackType->m_u8RampFlag=1;
            //                             p_TrackType->m_u8RampReadyFlag=0;
            //                         }
            //                     }
            //                     else
            //                     {
            //                         if (right_missingline_times>20)
            //                         {
            //                             regression(&g_Border,55,IMGH-1,1);
            ////                             ips200_showfloat(0,10,Variance,3,3);
            ////                             ips200_showfloat(0,11,parameterB,3,3);
            ////                             ips200_showint16(0,12,straight_lines);
            //                             if (Variance < 13 && Fabs(parameterB)<0.8 && straight_lines < 40)
            //                             {
            //                                 for (i=0.8*IMGH;i>=0.3*IMGH;i--)
            //                                 {
            //                                     if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i-Gap].m_i16x &&
            //                                         p_Border->m_RPnt[i].m_i16x <= p_Border->m_RPnt[i+Gap].m_i16x)
            //                                     {
            //                                         Rfind=i;
            //                                         break;
            //                                     }
            //                                 }
            ////                                 ips200_showint16(0,13,Rfind);
            //                                 if (!Ramp_Quit_Flag && !Rfind)
            //                                 {
            //                                     Ramp_Quit_Flag=1;
            //                                     distance();
            ////                                     ips200_showint16(120,14,distances);
            //                                     if (distances<290)
            //                                     {
            //                                         p_TrackType->m_u8RampReadyFlag=1;
            //                                         Angle = 0;
            //                                         beep_flag=1;
            //                                         Prospect_Image=IMGH-30;
            //                                     }
            //                                     else
            //                                     {
            //                                         distance();
            //                                         if(distances<290)
            //                                         {
            //                                             p_TrackType->m_u8RampReadyFlag=1;
            //                                             Angle = 0;
            //                                             beep_flag=1;
            //                                             Prospect_Image=IMGH-30;
            //                                         }
            //                                     }
            //                                 }
            //                             }
            //                         }
            //                         else if (left_missingline_times>20)
            //                         {
            //                             regression(&g_Border,55,IMGH-1,2);
            ////                             ips200_showfloat(120,10,Variance,3,3);
            ////                             ips200_showfloat(120,11,parameterB,3,3);
            ////                             ips200_showint16(120,12,straight_lines);
            //                             if (Variance < 12 && Fabs(parameterB)<0.6 && straight_lines < 40)
            //                             {
            //                                 for (i=0.8*IMGH;i>=0.3*IMGH;i--)
            //                                 {
            //                                     if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i-Gap].m_i16x &&
            //                                         p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i+Gap].m_i16x)
            //                                     {
            //                                         Lfind=i;
            //                                         break;
            //                                     }
            //                                 }
            ////                                 ips200_showint16(120,13,Lfind);
            //                                 if (!Ramp_Quit_Flag && !Lfind)
            //                                 {
            //                                     Ramp_Quit_Flag=1;
            //                                     distance();
            ////                                     ips200_showint16(120,14,distances);
            //                                     if (distances<290)
            //                                     {
            //                                         p_TrackType->m_u8RampReadyFlag=1;
            //                                         Angle = 0;
            //                                         beep_flag=1;
            //                                         Prospect_Image=IMGH-30;
            //                                     }
            //                                     else
            //                                     {
            //                                         distance();
            //                                         if(distances<290)
            //                                         {
            //                                             p_TrackType->m_u8RampReadyFlag=1;
            //                                             Angle = 0;
            //                                             beep_flag=1;
            //                                             Prospect_Image=IMGH-30;
            //                                         }
            //                                     }
            //                                 }
            //                             }
            //                         }
            //                         else if (left_missingline_times<=20 && right_missingline_times<=20)
            //                         {
            //                             if (Roadwidth_Cal(55)>110 && Roadwidth_Cal(60)>115 && Roadwidth_Cal(65)>120 && Roadwidth_Cal(70)>125 && Roadwidth_Cal(75)>130) //&& straight_lines<20
            //                              {
            //                                 if(left_missingline_times < right_missingline_times)  regression(&g_Border,55,IMGH-1,1);
            //                                 else if(left_missingline_times > right_missingline_times)  regression(&g_Border,55,IMGH-1,2);
            //                                 else regression(&g_Border,55,IMGH-1,1);
            ////                                 ips200_showfloat(60,10,Variance,3,3);
            ////                                 ips200_showfloat(60,11,parameterB,3,3);
            //                                 if (Variance < 13 && Fabs(parameterB)<0.7 && !Ramp_Quit_Flag)
            //                                 {
            //                                     distance();
            ////                                     ips200_showint16(60,14,distances);
            //                                     Ramp_Quit_Flag=1;
            //                                     if (distances<290)
            //                                     {
            //                                         p_TrackType->m_u8RampReadyFlag=1;
            //                                         Angle = 0;
            //                                         beep_flag=1;
            //                                         Prospect_Image=IMGH-30;
            //                                     }
            //                                 }
            //                              }
            //                         }
            //                     }
            //                 }
        }
        else if (Lflag && Rflag) //左右都违反了单调性 十字或三岔
        {
            times = 0;
            for (i = IMGH - 1; i >= 0.3 * IMGH; i--)
            {
                if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                    p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2)
                    times++;
                if (times >= 4)
                    break;
            }
            if (times < 4)
            {
                if (Lflag > 0.4 * IMGH && Rflag > 0.4 * IMGH) //双边十字三岔的左右特征点限制行
                {                                             //假如能找到双边的拐点 再满足拐点行横坐标靠边 很难出现正入且上拐点下不丢线的条件
                    if (p_Border->m_RPnt[Rflag].m_i16x > 148)
                    {
                        for (i = Lflag < Rflag ? Lflag : Rflag; i >= IMGH * 0.2; i--)
                        {
                            if (!Lfind && p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 50 &&
                                p_Border->m_LPnt[i - 4 * Gap].m_i16x - p_Border->m_LPnt[i - 3 * Gap].m_i16x < 10 && (i - 2 * Gap) < 0.5 * IMGH)
                                Lfind = i - 2 * Gap;
                            if (!Rfind && p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 15 && //此处>20 不确定 可能会影响三岔
                                p_Border->m_RPnt[i - 3 * Gap].m_i16x - p_Border->m_RPnt[i - 4 * Gap].m_i16x < 10)
                                Rfind = i - 2 * Gap; //测试一下 && p_Border->m_RPnt[i].m_i16x==IMGW-2
                            if (Lfind && Rfind)
                                break;
                        }
                        if (Lfind && Rfind)
                        {
                            MyLine_Times = 0;
                            for (j = p_Border->m_LPnt[Lfind].m_i16x; j <= p_Border->m_RPnt[Rfind].m_i16x; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                            }
                            RightTimes = 0;
                            for (i = Rflag; i > Rfind; i--)
                            {
                                if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                    RightTimes++;
                                else
                                    RightTimes = 0;
                                if (RightTimes > 10)
                                    break;
                            }
                            if (MyLine_Times > 35 && p_Border->m_LPnt[Lflag].m_i16x - p_Border->m_LPnt[Lflag - 2 * Gap].m_i16x > 10 && //>20 //考虑到稍微斜入也应该进做调整 观察车况
                                RightTimes > 10)
                                p_TrackType->m_u8CrossFlag = 1;
                        }
                    }
                    else if (p_Border->m_LPnt[Lflag].m_i16x < 40)
                    {
                        for (i = Lflag < Rflag ? Lflag : Rflag; i >= IMGH * 0.2; i--)
                        {
                            if (!Lfind && p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 15 &&
                                p_Border->m_LPnt[i - 4 * Gap].m_i16x - p_Border->m_LPnt[i - 3 * Gap].m_i16x < 10)
                                Lfind = i - 2 * Gap; // p_Border->m_LPnt[i].m_i16x==1
                            if (!Rfind && p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 50 &&
                                p_Border->m_RPnt[i - 3 * Gap].m_i16x - p_Border->m_RPnt[i - 4 * Gap].m_i16x < 10 &&
                                (i - 2 * Gap) < 0.5 * IMGH)
                                Rfind = i - 2 * Gap; //测试一下
                            if (Lfind && Rfind)
                                break;
                        }

                        if (Lfind && Rfind)
                        {
                            MyLine_Times = 0;
                            for (j = p_Border->m_LPnt[Lfind].m_i16x; j <= p_Border->m_RPnt[Rfind].m_i16x; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                            }
                            LeftTimes = 0;
                            for (i = Lflag; i > Lfind; i--)
                            {
                                if (p_Border->m_LPnt[i].m_i16x == 1)
                                    LeftTimes++;
                                else
                                    LeftTimes = 0;
                                if (LeftTimes > 10)
                                    break;
                            }
                            if (MyLine_Times > 35 && p_Border->m_RPnt[Rflag - 2 * Gap].m_i16x - p_Border->m_RPnt[Rflag].m_i16x > 10 &&
                                LeftTimes > 10)
                                p_TrackType->m_u8CrossFlag = 2;
                        }
                    }
                    else
                    {
                        for (i = Lflag < Rflag ? Lflag : Rflag; i >= IMGH * 0.2; i--)
                        {
                            if (!Lfind && p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 40 && p_Border->m_LPnt[i].m_i16x == 1)
                                Lfind = i - 2 * Gap;
                            if (!Rfind && p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 40 && p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                Rfind = i - 2 * Gap; //测试一下
                            if (Lfind && Rfind)
                                break;
                        }

                        if (Lfind && Rfind)
                        {
                            LeftTimes = 0;
                            for (i = Lflag; i > Lfind; i--)
                            {
                                if (p_Border->m_LPnt[i].m_i16x == 1)
                                    LeftTimes++;
                                else
                                    LeftTimes = 0;
                                if (LeftTimes > 10)
                                    break;
                            }
                            RightTimes = 0;
                            for (i = Rflag; i > Rfind; i--)
                            {
                                if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                    RightTimes++;
                                else
                                    RightTimes = 0;
                                if (RightTimes > 10)
                                    break;
                            }
                            if (LeftTimes > 10 && RightTimes > 10 &&
                                myabs(Lfind - Rfind) < 10 && Lfind < 0.5 * IMGH && Rfind < 0.5 * IMGH)
                            { // p_Border->m_PLPnt[Lflag].m_f16x - p_Border->m_PLPnt[Lflag-2*Gap].m_f16x > 20 && p_Border->m_PRPnt[Rflag-2*Gap].m_f16x - p_Border->m_PRPnt[Rflag].m_f16x > 20
                                p_TrackType->m_u8CrossFlag = 3;
                            }
                        }
                    }
                    if (!Lfind || !Rfind)
                    {
                        Lfind = 0;
                        Rfind = 0;
                        if (p_Border->m_LPnt[Lflag].m_i16x < 10) //双边三岔左拐点信息不足 依据右侧单边三岔判断
                        {
                            k = 100;
                            for (j = p_Border->m_LPnt[Lflag].m_i16x + Gap; j <= p_Border->m_RPnt[Rflag].m_i16x - Gap; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                                else
                                {
                                    //                                     MyLine_Times=0;
                                    if (p_Border->m_u16MyLineBAr[j] < k && p_Border->m_u16MyLineBAr[j] < p_Border->m_u16MyLineBAr[j + Gap] &&
                                        p_Border->m_u16MyLineBAr[j] <= p_Border->m_u16MyLineBAr[j - Gap])
                                    {
                                        k = p_Border->m_u16MyLineBAr[j];
                                        X_pos = j;
                                    }
                                }
                                if (MyLine_Times >= 25)
                                    break;
                            }

                            if (X_pos > 30 && X_pos < 150 && MyLine_Times < 25 && myabs((p_Border->m_LPnt[Lflag].m_i16x + p_Border->m_RPnt[Rflag].m_i16x) / 2 - X_pos) < 10) //左右25的保护量还没测过
                            {
                                i = Lflag < Rflag ? Lflag : Rflag;
                                while (image[i--][X_pos] != 0 && i > 5)
                                    ;
                                i = Lflag < Rflag ? Lflag : Rflag;
                                while (i-- > 5)
                                {
                                    if (image[i][X_pos] == B_BLACK && image[i - 1][X_pos] == B_BLACK && image[i - 2][X_pos] == B_BLACK)
                                        break;
                                }
                                Y_pos = i;
                                times = 0;
                                for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                {
                                    if (p_Border->m_RPnt[i].m_i16x < X_pos)
                                        times++;
                                }

                                LeftTimes = 0;
                                RightTimes = 0;
                                if (times > 2) //三岔Y点尖点往上右边界在左侧
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x == 1)
                                            LeftTimes++;
                                        if (p_Border->m_RPnt[i].m_i16x > X_pos)
                                            RightTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x == 1)
                                            LeftTimes++;
                                    }

                                    if (LeftTimes >= 8 && RightTimes >= 3)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                                else
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                                            RightTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x > X_pos)
                                            LeftTimes++;
                                        if (p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                                            RightTimes++;
                                    }

                                    if (LeftTimes >= 3 && RightTimes >= 5)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                            }
                        }
                        else if (p_Border->m_RPnt[Rflag].m_i16x > IMGW - 11) //双边三岔右拐点信息不足 依据左侧单边三岔判断
                        {

                            k = 100;
                            for (j = p_Border->m_LPnt[Lflag].m_i16x + Gap; j <= p_Border->m_RPnt[Rflag].m_i16x - Gap; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                                else
                                {
                                    //                                     MyLine_Times=0;  //有清零可能会被某一列影响
                                    if (p_Border->m_u16MyLineBAr[j] < k && p_Border->m_u16MyLineBAr[j] <= p_Border->m_u16MyLineBAr[j + Gap] &&
                                        p_Border->m_u16MyLineBAr[j] < p_Border->m_u16MyLineBAr[j - Gap])
                                    {
                                        k = p_Border->m_u16MyLineBAr[j];
                                        X_pos = j;
                                    }
                                }
                                if (MyLine_Times >= 25)
                                    break;
                            }

                            if (X_pos > 30 && X_pos < 150 && MyLine_Times < 25 && myabs((p_Border->m_LPnt[Lflag].m_i16x + p_Border->m_RPnt[Rflag].m_i16x) / 2 - X_pos) < 10)
                            {
                                i = Lflag < Rflag ? Lflag : Rflag;
                                //                                 while (image[i--][X_pos]!=0 && i>5);
                                while (i-- > 5)
                                {
                                    if (image[i][X_pos] == B_BLACK && image[i - 1][X_pos] == B_BLACK && image[i - 2][X_pos] == B_BLACK)
                                        break;
                                }
                                Y_pos = i;
                                times = 0;
                                for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                {
                                    if (p_Border->m_LPnt[i].m_i16x > X_pos)
                                        times++;
                                }
                                if (times >= 3)
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                            RightTimes++;
                                        if (p_Border->m_LPnt[i].m_i16x < X_pos)
                                            LeftTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                            RightTimes++;
                                    }

                                    if (LeftTimes >= 3 && RightTimes >= 8)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                                else
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x != 1)
                                            LeftTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x < X_pos)
                                            RightTimes++;
                                        if (p_Border->m_LPnt[i].m_i16x != 1)
                                            LeftTimes++;
                                    }

                                    if (LeftTimes >= 5 && RightTimes >= 3)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                            }
                        }
                        else if (Angle_judge(0, Lflag + Gap, Lflag, Lflag - Gap) && Angle_judge(1, Rflag + Gap, Rflag, Rflag - Gap) && myabs(Lflag - Rflag) < 30 &&
                                 p_Border->m_RPnt[Rflag - Gap].m_i16x - p_Border->m_RPnt[Rflag].m_i16x > 0 && p_Border->m_PLPnt[Lflag].m_f16x - p_Border->m_PLPnt[Lflag - Gap].m_f16x > 0 && p_Border->m_LPnt[Lflag - Gap].m_i16x != 1 && p_Border->m_RPnt[Rflag - Gap].m_i16x != IMGW - 2)
                        {
                            LeftTimes = 0;
                            RightTimes = 0;
                            for (i = Lflag; i >= 0.4 * IMGH; i--)
                            {
                                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x)
                                    LeftTimes++;
                                else if (p_Border->m_LPnt[i].m_i16x == 1)
                                    LeftTimes++;
                            }
                            for (i = Rflag; i >= 0.4 * IMGH; i--)
                            {
                                if (p_Border->m_RPnt[i - Gap].m_i16x > p_Border->m_RPnt[i].m_i16x)
                                    RightTimes++;
                                else if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                    RightTimes++;
                            }
                            if (LeftTimes + RightTimes > 40)
                            {
                                k = 100;
                                for (j = p_Border->m_LPnt[Lflag].m_i16x + 3; j <= p_Border->m_RPnt[Rflag].m_i16x - 3; j++)
                                {
                                    if (p_Border->m_u16MyLineBAr[j] < k)
                                    {
                                        k = p_Border->m_u16MyLineBAr[j];
                                        X_pos = j;
                                    }
                                }
                                if (X_pos > 30 && X_pos < 150 && myabs((p_Border->m_LPnt[Lflag].m_i16x + p_Border->m_RPnt[Rflag].m_i16x) / 2 - X_pos) < 10)
                                {
                                    beep_flag = 1;
                                    p_TrackType->m_u8ThreeRoadsFlag = 1;
                                    Threeroads_Last_Line = ThreeRoads_End;
                                }
                            }
                        }
                    }
                }
            }
        }
        else if (Lflag || Rflag) //单边违反单调性 车库/环岛
        {
            if (Lflag)
            {
                if (right_missingline_times <= 30) //跳变点  P字两种情况都有可能存在
                {
                    times = 0;
                    for (i = Lflag; i >= 0.2 * IMGH; i--)
                    {
                        if (p_Border->m_RPnt[i].m_i16x < 0.4 * IMGW)
                        {
                            times++;
                        }
                        if (times > 5)
                            break;
                    }
                    if (times > 5)
                    {
                        p_TrackType->m_u8BendFlag = 1;
                    }
                    else
                    {
                        times = 0;
                        for (i = Lflag; i >= 0.3 * IMGH; i--)
                        {
                            if (p_Border->m_LPnt[i].m_i16x == 1)
                                times++;
                            if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 && myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 1].m_i16x) < 12 && p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 && p_Border->m_LPnt[i - 1].m_i16x > 1 && myabs(p_Border->m_RPnt[i - 2].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 && p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 && p_Border->m_LPnt[i - 2].m_i16x > 1)
                            {
                                Limit_Flag = 1;
                                break;
                            }
                        }
                        if (times >= 10 && !Limit_Flag)
                        {
                            regression(&g_Border, Lflag - 2 * Gap, Lflag + 2 * Gap < IMGH ? Lflag + 2 * Gap : IMGH - 1, 1);
                            if (Lflag > 0.6 * IMGH && (Variance == 0 || Variance > 20) &&
                                myabs(p_Border->m_LPnt[Lflag - 2 * Gap].m_i16x - p_Border->m_LPnt[Lflag + 2 * Gap < IMGH - 1 ? Lflag + 2 * Gap : IMGH - 1].m_i16x) > 5)
                            {
                                regression(&g_Border, Lflag - 2 * Gap, Lflag + 2 * Gap < IMGH ? Lflag + 2 * Gap : IMGH - 1, 1);
                                //                                      straightaway_curve(&g_Border,2);
                                if (straight_lines < 35)
                                {
                                    p_TrackType->m_u8LRoundaboutFlag = 1;
                                    //                                           Roundabout_Last_Line_In=Roundabout_In_End;
                                }
                            }
                            else if (Lflag > 0.5 * IMGH && myabs(p_Border->m_LPnt[Lflag - 2 * Gap].m_i16x - p_Border->m_LPnt[Lflag + 2 * Gap < IMGH - 1 ? Lflag + 2 * Gap : IMGH - 1].m_i16x) < 5 && p_Border->m_LPnt[Lflag - 2 * Gap].m_i16x > 1 && p_Border->m_LPnt[Lflag + 2 * Gap < IMGH - 1 ? Lflag + 2 * Gap : IMGH - 1].m_i16x > 1)
                            {
                                times = 0;
                                for (i = Lflag - 10; i >= 0.2 * IMGH; i--)
                                {
                                    if (p_Border->m_LPnt[i].m_i16x == 1)
                                        times++;
                                    if (times > 8 && p_Border->m_LPnt[i].m_i16x - p_Border->m_LPnt[i + Gap].m_i16x > 50 && p_Border->m_LPnt[i + 2 * Gap].m_i16x == 1 && Variance <= 20 && Variance > 0 && straight_lines < 35 && p_Border->m_LPnt[i + 2 * Gap + 1].m_i16x == 1 && p_Border->m_LPnt[i + 2 * Gap + 2].m_i16x == 1 && p_Border->m_LPnt[i - 1].m_i16x > 1 && p_Border->m_LPnt[i - 2].m_i16x > 1)
                                    {
                                        Angle_offset = 0;
                                        p_TrackType->m_u8LRoundaboutFlag = 3;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (Lflag > 0.6 * IMGH && p_Border->m_LPnt[Lflag - Gap].m_i16x != 1) //递减趋势
                {
                    times = 0;
                    for (i = Lflag - 5; i >= 0.3 * IMGH; i--) //其实左斜入十字应该判断左上拐点 但需要看过边线图像
                    {
                        if (p_Border->m_LPnt[i].m_i16x == 1 && !Lfind)
                            times++;
                        if (p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 40 &&
                            p_Border->m_LPnt[i - 2 * Gap - 1].m_i16x - p_Border->m_LPnt[i - 1].m_i16x > 40 &&
                            p_Border->m_LPnt[i - 2 * Gap - 2].m_i16x - p_Border->m_LPnt[i - 2].m_i16x > 40 &&
                            p_Border->m_LPnt[i - 4 * Gap].m_i16x - p_Border->m_LPnt[i - 3 * Gap].m_i16x < 10 &&
                            !Lfind && (i - 2 * Gap) < 0.5 * IMGH)
                            Lfind = i - 2 * Gap;
                        if (p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 5 &&
                            p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_RPnt[i - 2 * Gap - 1].m_i16x > 5 &&
                            p_Border->m_RPnt[i - 2].m_i16x - p_Border->m_RPnt[i - 2 * Gap - 2].m_i16x > 5 &&
                            p_Border->m_RPnt[i + 2 * Gap < IMGH - 1 ? i + 2 * Gap : IMGH - 1].m_i16x == IMGW - 2 && !Rfind)
                            Rfind = i - 2 * Gap;
                        if (Lfind && Rfind)
                            break;
                    }

                    if (Lfind && Rfind)
                    {
                        if (times > 15 && p_Border->m_LPnt[i].m_i16x == 1)
                        {
                            p_TrackType->m_u8CrossFlag = 1;
                        }
                        else
                        {
                            num = 0;
                            for (k = i - 2 * Gap; k >= (i - 3 * Gap > 0 ? i - 3 * Gap : 0); k--)
                            {
                                if (p_Border->m_RPnt[k].m_i16x > p_Border->m_LPnt[i - 2 * Gap].m_i16x && p_Border->m_RPnt[k].m_i16x < IMGW - 2)
                                    num++;
                            }
                            if (num >= 3)
                            {
                                p_TrackType->m_u8CrossFlag = 1;
                            }
                        }
                    }
                    if (!p_TrackType->m_u8CrossFlag && Angle_judge(0, Lflag + 4, Lflag, Lflag - 4) && p_Border->m_LPnt[Lflag].m_i16x < 90) //测出正常左侧单拐点判三岔在95左右
                    {
                        times = 0;
                        for (i = Lflag; i >= 0.2 * IMGH; i--)
                        {
                            if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x)
                                times++;
                            else
                                break;
                        }
                        if (times > 20)
                        {
                            k = 100;
                            for (j = p_Border->m_LPnt[Lflag].m_i16x + Gap; j <= IMGW - Gap - 1; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                                else
                                {
                                    MyLine_Times = 0;
                                    if (p_Border->m_u16MyLineBAr[j] < k && p_Border->m_u16MyLineBAr[j] <= p_Border->m_u16MyLineBAr[j + Gap] &&
                                        p_Border->m_u16MyLineBAr[j] < p_Border->m_u16MyLineBAr[j - Gap])
                                    {
                                        k = p_Border->m_u16MyLineBAr[j];
                                        X_pos = j;
                                    }
                                }
                                if (MyLine_Times >= 25)
                                    break;
                            }

                            if (X_pos > 25 && X_pos < 150 && MyLine_Times < 25)
                            {
                                i = Lflag;
                                //                                  while (image[i--][X_pos]!=0 && i>5);
                                while (i-- > 5)
                                {
                                    if (image[i][X_pos] == B_BLACK && image[i - 1][X_pos] == B_BLACK && image[i - 2][X_pos] == B_BLACK)
                                        break;
                                }
                                Y_pos = i;
                                times = 0;
                                for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                {
                                    if (p_Border->m_LPnt[i].m_i16x > X_pos)
                                        times++;
                                }
                                if (times >= 3)
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                            RightTimes++;
                                        if (p_Border->m_LPnt[i].m_i16x < X_pos)
                                            LeftTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                            RightTimes++;
                                    }

                                    if (LeftTimes >= 3 && RightTimes >= 8)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                                else
                                {
                                    for (i = Y_pos + 2 * Gap; i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x != 1)
                                            LeftTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x < X_pos)
                                            RightTimes++;
                                        if (p_Border->m_LPnt[i].m_i16x != 1)
                                            LeftTimes++;
                                    }

                                    if (LeftTimes >= 5 && RightTimes >= 3)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if (Rflag)
            {
                if (left_missingline_times < 30) //尖点处产生的是跳变 为环岛
                {
                    times = 0;
                    for (i = Rflag; i >= IMGH * 0.2; i--)
                    {
                        if (p_Border->m_LPnt[i].m_i16x > 0.6 * IMGW)
                        {
                            times++;
                        }
                        if (times > 5)
                            break;
                    }
                    if (times > 5)
                    {
                        p_TrackType->m_u8BendFlag = 2;
                    }
                    else
                    {
                        times = 0;
                        for (i = Rflag; i >= 0.3 * IMGH; i--)
                        {
                            if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                times++;
                            if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 && myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 1].m_i16x) < 12 && p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 && p_Border->m_LPnt[i - 1].m_i16x > 1 && myabs(p_Border->m_RPnt[i - 2].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 && p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 && p_Border->m_LPnt[i - 2].m_i16x > 1)
                            {
                                Limit_Flag = 1;
                                break;
                            }
                        }
                        if (!Limit_Flag && times >= 10)
                        {
                            regression(&g_Border, Rflag - 2 * Gap, Rflag + 2 * Gap < IMGH - 1 ? Rflag + 2 * Gap : IMGH - 1, 2);
                            if (Rflag > 0.6 * IMGH && (Variance == 0 || Variance > 20) &&
                                myabs(p_Border->m_RPnt[Rflag - 2 * Gap].m_i16x - p_Border->m_RPnt[Rflag + 2 * Gap < IMGH - 1 ? Rflag + 2 * Gap : IMGH - 1].m_i16x) > 5)
                            {
                                //                                      straightaway_curve(&g_Border,1);
                                if (straight_lines < 35)
                                {
                                    p_TrackType->m_u8RRoundaboutFlag = 1;
                                    //                                          Roundabout_Last_Line_In=Roundabout_In_End;
                                }
                                //                                      times=0;
                                //                                      for (;i>=0.2*IMGH;i--)
                                //                                      {
                                //                                          if(p_Border->m_RPnt[i].m_i16x > p_Border->m_RPnt[i-Gap].m_i16x) times++;
                                //                                          else times = 0;
                                //                                          if(times >= 10){
                                //                                            p_TrackType->m_u8RRoundaboutFlag=1;
                                //                                            break;
                                //                                          }
                                //                                      }
                            }
                            else if (Rflag > 0.5 * IMGH && myabs(p_Border->m_RPnt[Rflag - 2 * Gap].m_i16x - p_Border->m_RPnt[Rflag + 2 * Gap < IMGH - 1 ? Rflag + 2 * Gap : IMGH - 1].m_i16x) < 5 && p_Border->m_RPnt[Rflag - 2 * Gap].m_i16x < IMGW - 2 && p_Border->m_RPnt[Rflag + 2 * Gap < IMGH - 1 ? Rflag + 2 * Gap : IMGH - 1].m_i16x < IMGW - 2)
                            {
                                times = 0;
                                for (i = Rflag - 10; i >= 0.2 * IMGH; i--)
                                {
                                    if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                                        times++;
                                    if (times > 8 && p_Border->m_RPnt[i + Gap].m_i16x - p_Border->m_RPnt[i].m_i16x > 50 && p_Border->m_RPnt[i + 2 * Gap].m_i16x == IMGW - 2 && Variance <= 20 && Variance > 0 && straight_lines < 35 && p_Border->m_RPnt[i + 2 * Gap + 1].m_i16x == IMGW - 2 && p_Border->m_RPnt[i + 2 * Gap + 2].m_i16x == IMGW - 2 && p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 && p_Border->m_RPnt[i - 1].m_i16x > 1 && p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 && p_Border->m_RPnt[i - 2].m_i16x > 1)
                                    {
                                        Angle_offset = 0;
                                        p_TrackType->m_u8RRoundaboutFlag = 3;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (Rflag > 0.6 * IMGH && p_Border->m_RPnt[Rflag - Gap].m_i16x != IMGW - 2) //  <12是测算后的值 虽然前有if和找Rflag的条件 还是强调一下
                {
                    times = 0;
                    for (i = Rflag - 5; i >= 0.3 * IMGH; i--) //右斜入十字应该判断右上拐点 但需要看过边线图像
                    {
                        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2 && !Rfind)
                            times++;
                        if (p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 5 &&
                            p_Border->m_LPnt[i - 2 * Gap - 1].m_i16x - p_Border->m_LPnt[i - 1].m_i16x > 5 &&
                            p_Border->m_LPnt[i - 2 * Gap - 2].m_i16x - p_Border->m_LPnt[i - 2].m_i16x > 5 &&
                            p_Border->m_LPnt[i + 2 * Gap < IMGH - 1 ? i + 2 * Gap : IMGH - 1].m_i16x == 1 && !Lfind)
                            Lfind = i - 2 * Gap;
                        if (p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 40 &&
                            p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_RPnt[i - 2 * Gap - 1].m_i16x > 40 &&
                            p_Border->m_RPnt[i - 2].m_i16x - p_Border->m_RPnt[i - 2 * Gap - 2].m_i16x > 40 && !Rfind && (i - 2 * Gap) < 0.5 * IMGH && p_Border->m_RPnt[i - 3 * Gap].m_i16x - p_Border->m_RPnt[i - 4 * Gap].m_i16x < 10)
                            Rfind = i - 2 * Gap;
                        if (Lfind && Rfind)
                            break;
                    }

                    if (Lfind && Rfind)
                    {
                        if (times > 15 && p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                        {
                            p_TrackType->m_u8CrossFlag = 2;
                        }
                        else
                        {
                            num = 0;
                            for (k = i - 2 * Gap; k >= (i - 3 * Gap > 0 ? i - 3 * Gap : 0); k--)
                            {
                                if (p_Border->m_LPnt[k].m_i16x < p_Border->m_RPnt[i - 2 * Gap].m_i16x && p_Border->m_LPnt[k].m_i16x > 1)
                                    num++;
                            }
                            if (num >= 3)
                            {
                                p_TrackType->m_u8CrossFlag = 2;
                            }
                        }
                    }
                    if (!p_TrackType->m_u8CrossFlag && Angle_judge(1, Rflag + 4, Rflag, Rflag - 4) && p_Border->m_RPnt[Rflag].m_i16x > 80) //正常右侧单拐点进95左右
                    {
                        times = 0;
                        for (i = Rflag; i >= 0.2 * IMGH; i--)
                        {
                            if (p_Border->m_RPnt[i - Gap].m_i16x > p_Border->m_RPnt[i].m_i16x)
                                times++;
                            else
                                break;
                        }
                        if (times > 20)
                        {
                            k = 100;
                            for (j = Gap; j <= p_Border->m_RPnt[Rflag].m_i16x - Gap; j++)
                            {
                                if (p_Border->m_u16MyLineBAr[j] > 75)
                                    MyLine_Times++;
                                else
                                {
                                    MyLine_Times = 0;
                                    if (p_Border->m_u16MyLineBAr[j] < k && p_Border->m_u16MyLineBAr[j] <= p_Border->m_u16MyLineBAr[j + Gap] &&
                                        p_Border->m_u16MyLineBAr[j] < p_Border->m_u16MyLineBAr[j - Gap])
                                    {
                                        k = p_Border->m_u16MyLineBAr[j];
                                        X_pos = j;
                                    }
                                }
                                if (MyLine_Times >= 25)
                                    break;
                            }
                            if (X_pos > 25 && X_pos < 150 && MyLine_Times < 25)
                            {
                                i = Rflag;
                                //                                  while (image[i--][X_pos]!=0 && i>5);
                                while (i-- > 5)
                                {
                                    if (image[i][X_pos] == B_BLACK && image[i - 1][X_pos] == B_BLACK && image[i - 2][X_pos] == B_BLACK)
                                        break;
                                }
                                Y_pos = i;
                                times = 0;
                                for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                {
                                    if (p_Border->m_RPnt[i].m_i16x < X_pos)
                                        times++;
                                }
                                LeftTimes = 0;
                                RightTimes = 0;
                                if (times > 2) //三岔Y点尖点往上右边界在左侧
                                {
                                    for (i = (Y_pos + 2 * Gap < IMGH - 1 ? Y_pos + 2 * Gap : IMGH - 1); i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x == 1)
                                            LeftTimes++;
                                        if (p_Border->m_RPnt[i].m_i16x > X_pos)
                                            RightTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x == 1)
                                            LeftTimes++;
                                    }
                                    if (LeftTimes >= 8 && RightTimes >= 3)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                                else
                                {
                                    for (i = (Y_pos + 2 * Gap < IMGH - 1 ? Y_pos + 2 * Gap : IMGH - 1); i >= Y_pos + Gap; i--)
                                    {
                                        if (p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                                            RightTimes++;
                                    }
                                    for (i = Y_pos - Gap; i >= (Y_pos - 2 * Gap > 0 ? Y_pos - 2 * Gap : 0); i--)
                                    {
                                        if (p_Border->m_LPnt[i].m_i16x > X_pos)
                                            LeftTimes++;
                                        if (p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                                            RightTimes++;
                                    }

                                    if (LeftTimes >= 3 && RightTimes >= 5)
                                    {
                                        beep_flag = 1;
                                        p_TrackType->m_u8ThreeRoadsFlag = 1;
                                        Threeroads_Last_Line = ThreeRoads_End;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!p_TrackType->m_u8ThreeRoadsFlag && !p_TrackType->m_u8LRoundaboutFlag && !p_TrackType->m_u8RRoundaboutFlag &&
            !p_TrackType->m_u8CrossFlag && !p_TrackType->m_u8RampFlag && !p_TrackType->m_u8Pflag && !p_TrackType->m_u8SmallSFlag &&
            !p_TrackType->m_u8ZebraCrossingFlag && !p_TrackType->m_u8CarBarnState && !p_TrackType->m_u8CarRunningState)
        {
            LeftMost = IMGW;
            RightMost = 0;
            Zebra_times = 0;
            for (i = IMGH - 1; i > IMGH * 0.3; i--)
            {
                if (Zebra_start == 0)
                {
                    if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 3].m_i16x - p_Border->m_LPnt[i - 3].m_i16x) < 12 &&
                        p_Border->m_RPnt[i].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 1].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 2].m_i16x > 1)
                    {
                        Zebra_start = i;
                        Zebra_times++;
                        Loop_Start = Zebra_start + 20 < IMGH - 1 ? Zebra_start + 20 : IMGH - 1;
                        Loop_End = Zebra_start - 40 > 5 ? Zebra_start - 40 : 5;
                        for (k = Loop_Start; k >= Loop_End; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x > RightMost && p_Border->m_RPnt[k].m_i16x != IMGW - 2)
                                RightMost = p_Border->m_RPnt[k].m_i16x;
                            if (p_Border->m_LPnt[k].m_i16x < LeftMost && p_Border->m_LPnt[k].m_i16x != 1)
                                LeftMost = p_Border->m_LPnt[k].m_i16x;
                        }
                    }
                }
                else if (Zebra_start != 0)
                {
                    if (p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 &&
                        myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                        Zebra_times++;
                }
            }
            times = 0;
            Loop_Start = Zebra_start - Zebra_times / 2; // 0 <= i < IMGH
            if (Zebra_times >= 3 && Loop_Start > 0.2 * IMGH && Loop_Start < IMGH - 2 && Zebra_start != 0)
            {
                for (k = Loop_Start - 2; k <= Loop_Start + 2; k++)
                {
                    block_times = 0;
                    for (j = p_Border->m_RPnt[k].m_i16x; j < RightMost; j++) // p_Border->m_RPnt[Zebra_start+Gap<IMGH-1?Zebra_start+Gap:IMGH-1].m_i16x
                    {
                        if (image[k][j - 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j++] == 0 && j < IMGW - 2)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    for (j = p_Border->m_LPnt[k].m_i16x; j > LeftMost; j--) // p_Border->m_LPnt[Zebra_start+Gap<IMGH-1?Zebra_start+Gap:IMGH-1].m_i16x
                    {
                        if (image[k][j + 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j--] == 0 && j > 1)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    if (block_times >= 3)
                        times++;
                }
            }

            if (times >= 2)
            {
                i = Loop_Start;
                j = p_Border->m_LPnt[Loop_Start].m_i16x;
                LeftTimes = 0;
                Zebra_Left_Max = 0;
                while (j-- > 5)
                {
                    if (image[i][j] == B_WHITE)
                        LeftTimes++;
                    else
                    {
                        if (Zebra_Left_Max < LeftTimes)
                            Zebra_Left_Max = LeftTimes;
                        LeftTimes = 0;
                    }
                }
                if (Zebra_Left_Max < LeftTimes)
                    Zebra_Left_Max = LeftTimes;
                RightTimes = 0;
                Zebra_Right_Max = 0;
                j = p_Border->m_RPnt[Loop_Start].m_i16x;
                while (j++ < IMGW - 2)
                {
                    if (image[i][j] == B_WHITE)
                        RightTimes++;
                    else
                    {
                        if (Zebra_Right_Max < RightTimes)
                            Zebra_Right_Max = RightTimes;
                        RightTimes = 0;
                    }
                }
                if (Zebra_Right_Max < RightTimes)
                    Zebra_Right_Max = RightTimes;

                p_TrackType->m_u8RunTurns++; //初始值存疑
                if (Zebra_Left_Max > Zebra_Right_Max)
                {
                    if (p_TrackType->m_u8RunTurns != 2)
                    {
                        Zebra_Switch_Flag = 0;
                        beep_flag = 1;
                        p_TrackType->m_u8ZebraCrossingFlag = 1;
                    }
                    else if (p_TrackType->m_u8RunTurns == 2)
                        p_TrackType->m_u8CarRunningState = 1; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                }                                             /*右车库*/
                else if (Zebra_Right_Max > Zebra_Left_Max)
                {
                    if (p_TrackType->m_u8RunTurns != 2)
                    {
                        Zebra_Switch_Flag = 0;
                        p_TrackType->m_u8ZebraCrossingFlag = 3;
                        beep_flag = 1;
                    }
                    else if (p_TrackType->m_u8RunTurns == 2)
                        p_TrackType->m_u8CarRunningState = 3; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                }
            }
            if (!p_TrackType->m_u8CarRunningState && !p_TrackType->m_u8ZebraCrossingFlag) //存在左右拐点却没判进任何状态时进入"准备阶段" 前瞻行不能取到拐点后
            {
                for (i = 0.9 * IMGH; i >= 0.3 * IMGH; i--)
                {
                    if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 && myabs(p_Border->m_RPnt[i - 2].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 && myabs(p_Border->m_RPnt[i - 4].m_i16x - p_Border->m_LPnt[i - 4].m_i16x) < 12 && p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_RPnt[i].m_i16x > 1)
                    {
                        p_TrackType->m_u8ReadyFlag = 1;
                        if (i > Normal_Prospect_Front - 5) //将正常道路前瞻作为一个常量存储
                        {
                            prospect_front = i + 5 < IMGH - 1 ? i + 5 : IMGH - 1;
                            prospect_later = i + 25 < IMGH - 1 ? i + 25 : IMGH - 1;
                        }
                        else
                        {
                            prospect_front = Normal_Prospect_Front; //必须要!
                            prospect_later = Normal_Prospect_Later;
                        }
                        if (prospect_front == prospect_later)
                            Error_Reserve_Flag = 1;
                        break;
                    }
                }
            }
        }
    }

    /*出库状态*/
    if (p_TrackType->m_u8CarBarnState != 0)
    {
        switch (p_TrackType->m_u8CarBarnState) //需要把斑马线滤除吗?!
        {
        case 1:
            if (Fabs(Angle_offset) > Carbarn_Left_Angle)
            {
                beep_flag = 1;
                p_TrackType->m_u8CarBarnState = 0;
                //                    start_motor=0;
            }
            pid_angel.ActualAngel = Left_Max_angle;
            //pwm_duty(ATOM1_CH1_P33_9, (uint32)pid_angel.ActualAngel);
            break;
        case 2:
            if (Fabs(Angle_offset) > Carbarn_Right_Angle)
            {
                beep_flag = 1;
                p_TrackType->m_u8CarBarnState = 0;
            }
            pid_angel.ActualAngel = Right_Max_angle + 30;
            //pwm_duty(ATOM1_CH1_P33_9, (uint32)pid_angel.ActualAngel);
            break;
        }
    }

    if (p_TrackType->m_u8BendFlag != 0)
    {
        switch (p_TrackType->m_u8BendFlag)
        {
        case 1:
            p_TrackType->m_u8LAllLostFlag = 3;
            for (i = IMGH - 1; i >= IMGH * 0.8; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times > 5)
                {
                    p_TrackType->m_u8BendFlag = 0;
                    break;
                }
            }
            break;
        case 2:
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            for (i = IMGH - 1; i >= IMGH * 0.8; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times > 5)
                {
                    p_TrackType->m_u8BendFlag = 0;
                    break;
                }
            }
            break;
        }
    }

    if (p_TrackType->m_u8SmallSFlag != 0)
    {
        i = IMGH - 5;
        while (i-- > 0.3 * IMGH)
        {
            if (!Lflag)
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i + Gap].m_i16x)
                {
                    Lflag = i;
                    if (Lflag)
                    {
                        for (k = i + Gap; k >= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Lflag = k;
                                break;
                            }
                        }
                    }
                }
            }
            if (!Rflag)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    Rflag = i;
                    for (k = i + Gap; k >= i - Gap; k--)
                    {
                        if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                            p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k + 1].m_i16x)
                        {
                            Rflag = k;
                            break;
                        }
                    }
                }
            }
            if (p_Border->m_LPnt[i].m_i16x == 1)
            {
                LeftTimes = 0;
                for (k = i - 1; k >= i - 10; k--)
                {
                    if (p_Border->m_LPnt[k].m_i16x == 1)
                        LeftTimes++;
                }
            }
            else if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
            {
                RightTimes = 0;
                for (k = i - 1; k >= i - 10; k--)
                {
                    if (p_Border->m_RPnt[k].m_i16x == IMGW - 2)
                        RightTimes++;
                }
            }
            if (LeftTimes > 8 || RightTimes > 8)
            {
                p_TrackType->m_u8SmallSFlag = 0;
                break;
            }
        }
        if (!Lflag && !Rflag)
        {
            p_TrackType->m_u8SmallSFlag = 0;
        }
    }

    if (p_TrackType->m_u8RampFlag != 0)
    {
        switch (p_TrackType->m_u8RampFlag)
        {
        case 1:
            if (Angle < -10) //&& Ramp_Quit_Flag
            {
                p_TrackType->m_u8RampFlag = 2;
            }
            break;
        case 2:
            beep_flag = 1;
            if (Fabs(Angle) < 5) //&& Ramp_Quit_Flag
            {
                Ramp_Refuse_Flag = 1;
                Prospect_Image = 0.3 * IMGH;
                p_TrackType->m_u8RampFlag = 0;
            }
        }
    }

    if (p_TrackType->m_u8ThreeRoadsFlag != 0) //检测到进入三岔路口 左转 右侧补线
    {
        switch (p_TrackType->m_u8ThreeRoadsFlag)
        {
        case 1: //检测到三岔路口的三个标志点
            if (ThreeRoads_Finsh_Flag > 6000)
            {
                p_TrackType->m_u8ThreeRoadsFlag = 0;
                ThreeRoads_Finsh_Flag = 0;
                break;
            }
            i = IMGH - 5;
            left_missingline_times = 0;
            right_missingline_times = 0;
            while (i-- > 0.4 * IMGH)
            {
                if (p_Border->m_LPnt[i].m_i16x == 1)
                    left_missingline_times++;
                else if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                         p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i + Gap].m_i16x)
                {
                    if (left_missingline_times < 20)
                    {
                        Lflag = i;
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Lflag = k;
                                break;
                            }
                        }
                    }
                }

                if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                    right_missingline_times++;
                else if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                         p_Border->m_RPnt[i].m_i16x <= p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    if (right_missingline_times < 20)
                    {
                        Rflag = i;
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                p_Border->m_RPnt[k].m_i16x <= p_Border->m_RPnt[k + 1].m_i16x)
                            {
                                Rflag = k;
                                break;
                            }
                        }
                    }
                }
                if (Lflag && Rflag)
                    break;
            }

            if (p_TrackType->m_u8ThreeRoadsDir == 1)
            {
                if (Lflag == 0)
                    Lflag = IMGH - 1;
                if (Rflag == 0)
                {
                    Rflag = IMGH - 1;
                    ThreeRoads_Rpoint[0] = IMGH - 1;
                    ThreeRoads_Rpoint[1] = IMGW - 2;
                }
                else
                {
                    ThreeRoads_Rpoint[0] = Rflag;
                    ThreeRoads_Rpoint[1] = p_Border->m_RPnt[Rflag].m_i16x;
                }
            }
            else if (p_TrackType->m_u8ThreeRoadsDir == 2)
            {
                if (Lflag == 0)
                {
                    Lflag = IMGH - 1;
                    ThreeRoads_Lpoint[0] = IMGH - 1;
                    ThreeRoads_Lpoint[1] = 1;
                }
                else
                {
                    ThreeRoads_Lpoint[0] = Lflag;
                    ThreeRoads_Lpoint[1] = p_Border->m_LPnt[Lflag].m_i16x;
                }
                if (Rflag == 0)
                    Rflag = IMGH - 1;
            }
            Y_pos = Threeroads_Last_Line; //;
            times = 0;
            for (j = p_Border->m_RPnt[Rflag].m_i16x - Gap; j >= p_Border->m_LPnt[Lflag].m_i16x + Gap; j--)
            {
                i = Lflag < Rflag ? Lflag : Rflag;
                while ((image[i][j] == B_WHITE || image[i - 1][j] == B_WHITE || image[i - 2][j] == B_WHITE))
                    i--;
                if (Y_pos <= i)
                {
                    Y_pos = i;
                    X_pos = j;
                    //                        Threeroads_Last_Line = Y_pos;
                }
            }

            if (X_pos != 0)
            {
                ThreeRoads_Tpoint[0] = Y_pos;
                ThreeRoads_Tpoint[1] = X_pos;

                x1 = ThreeRoads_Tpoint[0];
                y1 = ThreeRoads_Tpoint[1];
                if (p_TrackType->m_u8ThreeRoadsDir == 1)
                {
                    x2 = ThreeRoads_Rpoint[0];
                    y2 = ThreeRoads_Rpoint[1];
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_RPnt[j].m_i16x = (int16)t;
                        p_Border->m_PRPnt[j].m_f16x = (X_perspective[p_Border->m_RPnt[j].m_i16y][p_Border->m_RPnt[j].m_i16x]);
                    }

                    Temp_ThreeRoads = p_Border->m_RPnt[ThreeRoads_Tpoint[0]].m_i16x;
                    for (i = x1 - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][Temp_ThreeRoads] == B_BLACK)
                        {
                            for (j = Temp_ThreeRoads; j > 3; j--)
                            {
                                if (image[i][j] == B_BLACK && image[i][j - 1] == B_WHITE)
                                {
                                    p_Border->m_RPnt[i].m_i16x = j;
                                    p_Border->m_PRPnt[i].m_f16x = (X_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
                                    Temp_ThreeRoads = p_Border->m_RPnt[i].m_i16x;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            p_Border->m_RPnt[i].m_i16x = IMGW - 2;
                            p_Border->m_PRPnt[i].m_f16x = (X_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
                        }
                    }
                    p_TrackType->m_u8LAllLostFlag = 3; //补线则沿右边线循线
                    p_TrackType->m_u8RAllLostFlag = 0;
                }
                else if (p_TrackType->m_u8ThreeRoadsDir == 2)
                {
                    x2 = ThreeRoads_Lpoint[0];
                    y2 = ThreeRoads_Lpoint[1];
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_LPnt[j].m_i16x = (int16)t;
                        p_Border->m_PLPnt[j].m_f16x = (X_perspective[p_Border->m_LPnt[j].m_i16y][p_Border->m_LPnt[j].m_i16x]);
                    }

                    Temp_ThreeRoads = p_Border->m_LPnt[ThreeRoads_Tpoint[0]].m_i16x;
                    for (i = x1 - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][Temp_ThreeRoads] == B_BLACK)
                        {
                            for (j = Temp_ThreeRoads; j <= IMGW - 3; j++)
                            {
                                if (image[i][j] == B_BLACK && image[i][j + 1] == B_WHITE)
                                {
                                    p_Border->m_LPnt[i].m_i16x = j;
                                    p_Border->m_PLPnt[i].m_f16x = (X_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
                                    Temp_ThreeRoads = p_Border->m_LPnt[i].m_i16x;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            p_Border->m_LPnt[i].m_i16x = 1;
                            p_Border->m_PLPnt[i].m_f16x = (X_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
                        }
                    }
                    p_TrackType->m_u8RAllLostFlag = 3; //补线则沿右边线循线
                    p_TrackType->m_u8LAllLostFlag = 0;
                }
            }
            //                if (Y_pos>0.7*IMGH) p_TrackType->m_u8ThreeRoadsFlag=0;
            break;
        }
    }

    if (p_TrackType->m_u8CrossFlag != 0) //十字路口补线
    {
        switch (p_TrackType->m_u8CrossFlag)
        {
        case 1:                                        //左斜入十字  斜入上拐点难找还是下面两点连接延长补线的好
            for (i = IMGH * 0.9; i >= 0.6 * IMGH; i--) //左下 右下特征点消失
            {
                if (p_Border->m_LPnt[i].m_i16x == 1 && p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times > 20)
                {
                    p_TrackType->m_u8CrossFlag = 4;
                }
            }
            for (i = IMGH - 5; i >= 0.2 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i + Gap].m_i16x)
                {
                    Lflag = i;
                    if (Lflag)
                    {
                        for (k = i + Gap; k >= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Lflag = k;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if (!Lflag)
                Lflag = IMGH - 1;
            for (i = Lflag - 5; i >= 0.2 * IMGH; i--)
            {
                if ((p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 40 && p_Border->m_LPnt[i].m_i16x == 1) ||
                    p_Border->m_LPnt[i - 2 * Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 60)
                {
                    Cross_LTpoint[0] = i - 2 * Gap;
                    Cross_LTpoint[1] = p_Border->m_LPnt[i - 2 * Gap].m_i16x;
                }
            }
            Cross_LBpoint[0] = Lflag;
            Cross_LBpoint[1] = p_Border->m_LPnt[Lflag].m_i16x;

            if (Cross_LTpoint[0] == 0)
            {
                Error_Reserve_Flag = 1;
                break;
            }
            else
            {
                x1 = Cross_LTpoint[0];
                x2 = Cross_LBpoint[0];
                y1 = Cross_LTpoint[1];
                y2 = Cross_LBpoint[1];
                for (j = x1; j <= x2; j++)
                {
                    t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][t] = 0;
                    p_Border->m_LPnt[j].m_i16x = t;
                    p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                }
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;

        case 2:                                        //右斜入十字
            for (i = IMGH * 0.9; i >= 0.6 * IMGH; i--) //左下 右下特征点消失
            {
                if (p_Border->m_LPnt[i].m_i16x == 1 && p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times > 20)
                {
                    p_TrackType->m_u8CrossFlag = 4;
                }
            }

            for (i = IMGH - 5; i >= 0.2 * IMGH; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    Rflag = i;
                    if (Rflag)
                    {
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                p_Border->m_RPnt[k].m_i16x <= p_Border->m_RPnt[k + 1].m_i16x)
                            {
                                Rflag = k;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if (!Rflag)
                Rflag = IMGH - 1;
            for (i = Rflag - 5; i >= 0.2 * IMGH; i--)
            {
                if ((p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 40 && p_Border->m_RPnt[i].m_i16x == IMGW - 2) ||
                    p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 60)
                {
                    Cross_RTpoint[0] = i - 2 * Gap;
                    Cross_RTpoint[1] = p_Border->m_RPnt[i - 2 * Gap].m_i16x;
                }
            }
            Cross_RBpoint[0] = Rflag;
            Cross_RBpoint[1] = p_Border->m_RPnt[Rflag].m_i16x;
            if (Cross_RTpoint[0] == 0)
            {
                Error_Reserve_Flag = 1;
                break;
            }
            else
            {
                x1 = Cross_RTpoint[0];
                x2 = Cross_RBpoint[0];
                y1 = Cross_RTpoint[1];
                y2 = Cross_RBpoint[1];
                for (j = x1; j <= x2; j++)
                {
                    t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][t] = 0;
                    p_Border->m_RPnt[j].m_i16x = t;
                    p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                }
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;

        case 3:
            for (i = IMGH * 0.9; i >= 0.6 * IMGH; i--) //左下 右下特征点消失
            {
                if (p_Border->m_LPnt[i].m_i16x == 1 && p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times > 20)
                {
                    p_TrackType->m_u8CrossFlag = 4;
                }
            }
            for (i = IMGH - 5; i >= 0.2 * IMGH; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    Rflag = i;
                    if (Rflag)
                    {
                        for (k = i + Gap; k >= i - Gap; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                p_Border->m_RPnt[k].m_i16x <= p_Border->m_RPnt[k + 1].m_i16x)
                            {
                                Rflag = k;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if (!Rflag)
                Rflag = IMGH - 1;
            for (i = Rflag - 5; i >= 0.2 * IMGH; i--)
            {
                if ((p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 40 && p_Border->m_RPnt[i].m_i16x == IMGW - 2) ||
                    p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - 2 * Gap].m_i16x > 60)
                {
                    Cross_RTpoint[0] = i - 2 * Gap;
                    Cross_RTpoint[1] = p_Border->m_RPnt[i - 2 * Gap].m_i16x;
                }
            }
            Cross_RBpoint[0] = Rflag;
            Cross_RBpoint[1] = p_Border->m_RPnt[Rflag].m_i16x;
            if (Cross_RTpoint[0] == 0)
            {
                Error_Reserve_Flag = 1;
                break;
            }
            else
            {
                x1 = Cross_RTpoint[0];
                x2 = Cross_RBpoint[0];
                y1 = Cross_RTpoint[1];
                y2 = Cross_RBpoint[1];
                for (j = x1; j <= x2; j++)
                {
                    t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][t] = 0;
                    p_Border->m_RPnt[j].m_i16x = t;
                    p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                }
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;

            break;

        case 4:
            for (i = IMGH * 0.9; i >= IMGH * 0.6; i--) //把判断语句放最前面 最开始图像非常纯净
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                    times += 1;
                else
                    times = 0;
                if (times >= 25)
                {
                    p_TrackType->m_u8CrossFlag = 0; // 5;
                }
            }
            for (i = IMGH - 30; i >= 0.2 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && Roadwidth_Cal(i) < 110 && p_Border->m_LPnt[i].m_i16x - p_Border->m_LPnt[i + 1].m_i16x < 3 && p_Border->m_RPnt[i + 1].m_i16x - p_Border->m_RPnt[i].m_i16x < 3)
                {
                    y1 = X_perspective[i][(p_Border->m_LPnt[i].m_i16x + p_Border->m_RPnt[i].m_i16x) / 2];
                    x1 = i;
                    break;
                }
            }
            y2 = X_perspective[IMGH - 1][Center];
            x2 = IMGH - 1;
            if (x1 == 0)
            {
                Error_Reserve_Flag = 1;
                break;
            }
            for (j = (int16)x2; j >= (int16)x1; j--)
            {
                t = ((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2);
                p_Border->m_CPnt[j].m_f16x = t;
                p_Border->m_CPnt[j].m_f16y = j;
            }
            while (j-- > 5) //不丢线前延续中线
            {
                if (p_Border->m_LPnt[j].m_i16x != 1 && p_Border->m_RPnt[j].m_i16x != IMGW - 2)
                    break;
                p_Border->m_CPnt[j].m_f16x = t;
                p_Border->m_CPnt[j].m_f16y = j;
            }
            for (; j >= 5; j--)
            {
                p_Border->m_CPnt[j].m_f16x = p_Border->m_PLPnt[j].m_f16x + Perspective_Normal_Width / 2;
                p_Border->m_CPnt[j].m_f16y = j;
            }
            break;
        }
    }

    if (p_TrackType->m_u8LRoundaboutFlag != 0) //左侧环岛补线 相反出库方向及往返赛道
    {
        switch (p_TrackType->m_u8LRoundaboutFlag)
        {
        case 1: //右侧直线 左边界丢线
            i = IMGH;
            while (i-- > 0.3 * IMGH)
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x >= p_Border->m_LPnt[i + Gap].m_i16x)
                {
                    Lflag = i;
                    if (Lflag)
                    {
                        for (k = i + Gap; k <= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Lflag = k;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if (Lflag)
            {
                LeftMost = IMGW;
                RightMost = 0;
                Zebra_times = 0;
                for (i = Lflag - Gap; i > 0.3 * IMGH; i--)
                {
                    if (Zebra_start == 0)
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                            myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 &&
                            myabs(p_Border->m_RPnt[i - 3].m_i16x - p_Border->m_LPnt[i - 3].m_i16x) < 12 &&
                            p_Border->m_RPnt[i].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i].m_i16x > 1 &&
                            p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i - 1].m_i16x > 1 &&
                            p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i - 2].m_i16x > 1)
                        {
                            Zebra_start = i;
                            Zebra_times++;
                            Loop_Start = Zebra_start + 20 < IMGH - 1 ? Zebra_start + 20 : IMGH - 1;
                            Loop_End = Zebra_start - 40 > 5 ? Zebra_start - 40 : 5;
                            for (k = Loop_Start; k >= Loop_End; k--)
                            {
                                if (p_Border->m_LPnt[k].m_i16x < LeftMost && p_Border->m_LPnt[k].m_i16x != 1)
                                    LeftMost = p_Border->m_LPnt[k].m_i16x;
                                if (p_Border->m_RPnt[k].m_i16x > RightMost && p_Border->m_RPnt[k].m_i16x != IMGW - 2)
                                    RightMost = p_Border->m_RPnt[k].m_i16x;
                            }
                        }
                    }
                    else if (Zebra_start != 0)
                    {
                        if (p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 &&
                            myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                            Zebra_times++;
                    }
                }

                times = 0;
                Loop_Start = Zebra_start - Zebra_times / 2; // 0 <= i < IMGH
                if (Zebra_times >= 3 && Loop_Start > 0.2 * IMGH && Loop_Start < IMGH - 2 && Zebra_start != 0)
                {
                    for (k = Loop_Start - 2; k <= Loop_Start + 2; k++)
                    {
                        block_times = 0;
                        for (j = p_Border->m_RPnt[k].m_i16x; j < RightMost; j++)
                        {
                            if (image[k][j - 1] == 255 && image[k][j] == 0)
                            {
                                black_blocks = 0;
                                while (image[k][j++] == 0 && j < IMGW - 2)
                                    black_blocks++;
                                if (black_blocks > 3 && black_blocks < 8)
                                    block_times++;
                            }
                        }
                        for (j = p_Border->m_LPnt[k].m_i16x; j > LeftMost; j--)
                        {
                            if (image[k][j + 1] == 255 && image[k][j] == 0)
                            {
                                black_blocks = 0;
                                while (image[k][j--] == 0 && j > 1)
                                    black_blocks++;
                                if (black_blocks > 3 && black_blocks < 8)
                                    block_times++;
                            }
                        }
                        if (block_times >= 3)
                            times++;
                    }
                }

                if (times >= 2)
                {
                    p_TrackType->m_u8LRoundaboutFlag = 0;
                    p_TrackType->m_u8RunTurns++; //初始值存疑
                    if (p_TrackType->m_u8RunTurns != 2)
                    {
                        p_TrackType->m_u8ZebraCrossingFlag = 1;
                        Zebra_Switch_Flag = 0;
                        beep_flag = 1;
                        break;
                    }
                    else if (p_TrackType->m_u8RunTurns == 2)
                    {
                        /*左车库*/
                        p_TrackType->m_u8CarRunningState = 1; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                        break;
                    }
                }
            }

            if (p_TrackType->m_u8LRoundaboutFlag)
            {
                left_missingline_times = 0;
                for (i = IMGH - 1; i >= 0.85 * IMGH; i--)
                {
                    if (p_Border->m_LPnt[i].m_i16x == 1)
                        left_missingline_times += 1;
                    else
                        left_missingline_times = 0;
                    if (left_missingline_times > 12 && p_Border->m_LPnt[IMGH - 1].m_i16x == 1)
                    {
                        beep_flag = 1;
                        p_TrackType->m_u8LRoundaboutFlag = 2;
                        break;
                    }
                }
                p_TrackType->m_u8LAllLostFlag = 3;
                p_TrackType->m_u8RAllLostFlag = 0;
            }
            break;
        case 2: //环岛右下标志点丢失 固定右下标志点 右下——右中补线
            LeftMost = IMGW;
            RightMost = 0;
            Zebra_times = 0;
            for (i = IMGH - 1; i > IMGH / 2; i--)
            {
                if (Zebra_start == 0)
                {
                    if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 3].m_i16x - p_Border->m_LPnt[i - 3].m_i16x) < 12 &&
                        p_Border->m_RPnt[i].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 1].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 2].m_i16x > 1)
                    {
                        Zebra_start = i;
                        Zebra_times++;
                        Loop_Start = Zebra_start + 20 < IMGH - 1 ? Zebra_start + 20 : IMGH - 1;
                        Loop_End = Zebra_start - 40 > 5 ? Zebra_start - 40 : 5;
                        for (k = Loop_Start; k >= Loop_End; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x < LeftMost && p_Border->m_LPnt[k].m_i16x != 1)
                                LeftMost = p_Border->m_LPnt[k].m_i16x;
                            if (p_Border->m_RPnt[k].m_i16x > RightMost && p_Border->m_RPnt[k].m_i16x != IMGW - 2)
                                RightMost = p_Border->m_RPnt[k].m_i16x;
                        }
                    }
                }
                else if (Zebra_start != 0)
                {
                    if (p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 &&
                        myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                        Zebra_times++;
                }
            }

            times = 0;
            Loop_Start = Zebra_start - Zebra_times / 2; // 0 <= i < IMGH
            if (Zebra_times >= 3 && Loop_Start > 0.2 * IMGH && Loop_Start < IMGH - 2 && Zebra_start != 0)
            {
                for (k = Loop_Start - 2; k <= Loop_Start + 2; k++)
                {
                    block_times = 0;
                    for (j = p_Border->m_RPnt[k].m_i16x; j < RightMost; j++)
                    {
                        if (image[k][j - 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j++] == 0 && j < IMGW - 2)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    for (j = p_Border->m_LPnt[k].m_i16x; j > LeftMost; j--)
                    {
                        if (image[k][j + 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j--] == 0 && j > 1)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    if (block_times >= 3)
                        times++;
                }
            }
            if (times >= 2)
            {
                p_TrackType->m_u8LRoundaboutFlag = 0;
                p_TrackType->m_u8RunTurns++; //初始值存疑
                if (p_TrackType->m_u8RunTurns != 2)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 1;
                    Zebra_Switch_Flag = 0;
                    beep_flag = 1;
                    break;
                }
                else if (p_TrackType->m_u8RunTurns == 2)
                {
                    /*左车库*/
                    p_TrackType->m_u8CarRunningState = 1; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                    break;
                }
            }

            if (p_TrackType->m_u8LRoundaboutFlag)
            {
                times = 0;
                for (i = IMGH - 1; i >= IMGH - 6; i--)
                {
                    if (p_Border->m_LPnt[i].m_i16x > 1)
                        times++;
                    else
                        times = 0;
                    if (times > 5)
                    {
                        beep_flag = 1;
                        Angle_offset = 0;
                        Roundabout_Wire_Flag = 0;
                        p_TrackType->m_u8LRoundaboutFlag = 3;
                    }
                }
                p_TrackType->m_u8LAllLostFlag = 3;
                p_TrackType->m_u8RAllLostFlag = 0;
            }
            break;
        case 3:
            times = 0;
            for (i = IMGH - Gap - 1; i >= 0.25 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i - Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 50 && p_Border->m_LPnt[i - Gap].m_i16x - p_Border->m_LPnt[i].m_i16x < 120)
                {
                    Roundabout_LTpoint[0] = i - Gap;
                    Roundabout_LTpoint[1] = p_Border->m_LPnt[i - Gap].m_i16x;
                    if (!Roundabout_Wire_Flag)
                        Roundabout_Wire_Flag = 1;
                    break;
                }
                if (p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - Gap].m_i16x > 20 && p_Border->m_RPnt[i - Gap].m_i16x != IMGW - 2 && p_Border->m_RPnt[IMGH - 1].m_i16x == IMGW - 2 && p_Border->m_RPnt[IMGH - 2].m_i16x == IMGW - 2 && p_Border->m_RPnt[IMGH - 3].m_i16x == IMGW - 2 && Roundabout_Wire_Flag)
                {
                    Roundabout_LTpoint[0] = i - Gap;
                    Roundabout_LTpoint[1] = p_Border->m_RPnt[i - Gap].m_i16x;
                    break;
                }
            }
            //                  if (Roundabout_LTpoint[0]>=Roundabout_Last_Line_In) Roundabout_Last_Line_In=Roundabout_LTpoint[0];
            //                  else Error_Reserve_Flag=1;

            if (!Roundabout_LTpoint[0] && !Roundabout_Wire_Flag)
            {
                i = IMGH - 5;
                times = 0;
                while (i-- > 0.2 * IMGH)
                {
                    if (p_Border->m_RPnt[i].m_i16x < 0.5 * IMGW)
                        times++;
                    if (times > 5)
                    {
                        p_TrackType->m_u8LRoundaboutFlag = 0;
                        p_TrackType->m_u8Pflag = 2;
                        break;
                    }
                }
            }
            else
            {
                if (Roundabout_LTpoint[0]) // && Roundabout_LTpoint[0]>=Roundabout_Last_Line_In
                {
                    x1 = Roundabout_LTpoint[0];
                    x2 = Roundabout_LTpoint[0] + LRoundabout_TimeToTurn < IMGH - 1 ? Roundabout_LTpoint[0] + LRoundabout_TimeToTurn : IMGH - 1;
                    y1 = Roundabout_LTpoint[1];
                    if (x2 == IMGH - 1)
                        y2 = p_Border->m_RPnt[IMGH - 1].m_i16x;
                    else
                        y2 = p_Border->m_RPnt[Roundabout_LTpoint[0] + LRoundabout_TimeToTurn].m_i16x;
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_RPnt[j].m_i16x = t;
                        p_Border->m_PRPnt[j].m_f16x = (X_perspective[p_Border->m_RPnt[j].m_i16y][p_Border->m_RPnt[j].m_i16x]);
                    }

                    Temp_Round = p_Border->m_RPnt[Roundabout_LTpoint[0]].m_i16x;
                    for (i = x1 - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][Temp_Round] == B_BLACK)
                        {
                            for (j = Temp_Round; j > 3; j--)
                            {
                                if (image[i][j] == B_BLACK && image[i][j - 1] == B_WHITE)
                                {
                                    p_Border->m_RPnt[i].m_i16x = j;
                                    p_Border->m_PRPnt[i].m_f16x = (X_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
                                    Temp_Round = p_Border->m_RPnt[i].m_i16x;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            p_Border->m_RPnt[i].m_i16x = IMGW - 2;
                            p_Border->m_PRPnt[i].m_f16x = (X_perspective[p_Border->m_RPnt[i].m_i16y][p_Border->m_RPnt[i].m_i16x]);
                        }
                    }
                }
                else
                    Error_Reserve_Flag = 1;
            }

            if (Fabs(Angle_offset) > 40)
            {
                beep_flag = 1;
                p_TrackType->m_u8LRoundaboutFlag = 4;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 4:
            for (i = IMGH - 1; i >= IMGH / 2; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x &&
                    ((p_Border->m_RPnt[i - 2 * Gap].m_i16x != IMGW - 2 && p_Border->m_RPnt[i - 2 * Gap].m_i16x - p_Border->m_RPnt[i].m_i16x > 10) ||
                     p_Border->m_RPnt[i - 2 * Gap].m_i16x == IMGW - 2))
                {
                    Roundabout_LBpoint[0] = i; //出环时 右下角拐点实际为圆环左下角标志点
                    Roundabout_LBpoint[1] = p_Border->m_RPnt[i].m_i16x;
                    break;
                }
            }
            //                  if (Fabs(Angle_offset)>223 || (Roundabout_LBpoint[0] && Fabs(Angle_offset)>200))
            if (Fabs(Angle_offset) > 240 && Roundabout_LBpoint[0] > 0.5 * IMGH)
            {
                beep_flag = 1;
                p_TrackType->m_u8LRoundaboutFlag = 5; //!!!右下角已经是圆环部分(不丢线)时进入状态4 110可调
                Roundabout_To_Straight_Flag = 0;
                Roundabout_Last_Line_Out = Roundabout_Out_End;
                LastData_Y = 0;
                Angle_offset = 0;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;

            break;
        case 5:
            times = 0;
            if (Roundabout_Last_Line_Out > 0.8 * IMGH)
            {
                for (i = 0.5 * IMGH; i >= 0.4 * IMGH; i--)
                {
                    if (p_Border->m_RPnt[i].m_i16x >= p_Border->m_RPnt[i - Gap].m_i16x &&
                        p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_RPnt[i - Gap].m_i16x < IMGW - 2)
                        times++;
                    else
                        times = 0;
                }
                if (times >= 10 && Fabs(Angle_offset) > 50)
                {
                    Roundabout_To_Straight_Flag = 1;
                    x1 = 41;
                    x2 = 99;
                    y1 = p_Border->m_RPnt[41].m_i16x;
                    y2 = IMGW - 1;
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_RPnt[j].m_i16x;
                        p_Border->m_RPnt[j].m_i16x = t;
                        p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                {
                    Roundabout_To_Straight_Flag = 0;
                    j = 30;
                    times = 0;
                    for (i = IMGH - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][j] == B_WHITE)
                            times++;
                        else
                            times = 0;
                        if (times >= 10 && image[i - Gap][j] == B_BLACK && image[i - Gap - 1][j] == B_BLACK && image[i - Gap - 2][j] == B_BLACK)
                            break;
                    }
                    if (i >= 0.2 * IMGH)
                    {
                        x1 = i;
                        x2 = IMGH - 1;
                        y1 = j;
                        y2 = IMGW - 2;

                        for (j = x1; j <= x2; j++)
                        {
                            t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                            image[j][t] = 0;
                            p_Border->m_RPnt[j].m_i16x = t;
                            p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                        }
                    }
                    else
                        Error_Reserve_Flag = 1;
                }
                if (Roundabout_To_Straight_Flag)
                {
                    if (sum > 0)
                        p_TrackType->m_u8LRoundaboutFlag = 6;
                }
            }
            else
            {
                i = IMGH - 5;
                while (i-- > Roundabout_Last_Line_Out - 5)
                {
                    if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                        p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x &&
                        ((p_Border->m_RPnt[i - 2 * Gap].m_i16x != IMGW - 2 && p_Border->m_RPnt[i - 2 * Gap].m_i16x - p_Border->m_RPnt[i].m_i16x > 10) ||
                         p_Border->m_RPnt[i - 2 * Gap].m_i16x == IMGW - 2))
                    {
                        Roundabout_LBpoint[0] = i; //出环时 右下角拐点实际为圆环左下角标志点
                        for (k = i + Gap; k >= i - Gap; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                p_Border->m_RPnt[k].m_i16x <= p_Border->m_RPnt[k + 1].m_i16x)
                            {
                                Roundabout_LBpoint[0] = k;
                                break;
                            }
                        }
                        break;
                    }
                }
                if (Roundabout_LBpoint[0])
                    Roundabout_Last_Line_Out = Roundabout_LBpoint[0];
                if (Roundabout_LBpoint[0])
                {
                    x1 = 10;
                    x2 = Roundabout_LBpoint[0];
                    y1 = 1;
                    y2 = p_Border->m_RPnt[Roundabout_LBpoint[0]].m_i16x;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_RPnt[j].m_i16x;
                        p_Border->m_RPnt[j].m_i16x = t;
                        p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                {
                    x1 = 10;
                    x2 = IMGH - 1;
                    y1 = 1;
                    y2 = IMGW - 2;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_RPnt[j].m_i16x;
                        p_Border->m_RPnt[j].m_i16x = t;
                        p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
            }

            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 6:
            for (i = IMGH - 1; i >= IMGH * 0.6; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2 && p_Border->m_LPnt[i - Gap].m_i16x >= p_Border->m_LPnt[i].m_i16x && p_Border->m_RPnt[i - Gap].m_i16x <= p_Border->m_RPnt[i].m_i16x)
                    times += 1; //|| p_Border->m_PRPnt[i].m_f16x==IMGW-2
                else
                    times = 0;
            }
            if (times > 32)
            {
                p_TrackType->m_u8LRoundaboutFlag = 0;
                beep_flag = 1;
                break;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        }
    }

    if (p_TrackType->m_u8RRoundaboutFlag != 0) //环岛补线
    {
        switch (p_TrackType->m_u8RRoundaboutFlag)
        {
        case 1: //左侧直线 右边界丢线
            LeftMost = IMGW;
            RightMost = 0;
            Zebra_times = 0;
            i = IMGH;
            while (i-- > 0.3 * IMGH)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    Rflag = i;
                    for (k = i + Gap; k >= i - Gap; k--)
                    {
                        if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                            p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k + 1].m_i16x)
                        {
                            Rflag = k;
                            break;
                        }
                    }
                    break;
                }
            }
            if (Rflag)
            {
                for (i = Rflag - Gap; i > 0.3 * IMGH; i--)
                {
                    if (Zebra_start == 0)
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                            myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 &&
                            myabs(p_Border->m_RPnt[i - 3].m_i16x - p_Border->m_LPnt[i - 3].m_i16x) < 12 &&
                            p_Border->m_RPnt[i].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i].m_i16x > 1 &&
                            p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i - 1].m_i16x > 1 &&
                            p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 &&
                            p_Border->m_LPnt[i - 2].m_i16x > 1)
                        {
                            Zebra_start = i;
                            Zebra_times++;
                            Loop_Start = Zebra_start + 20 < IMGH - 1 ? Zebra_start + 20 : IMGH - 1;
                            Loop_End = Zebra_start - 40 > 5 ? Zebra_start - 40 : 5;
                            for (k = Loop_Start; k >= Loop_End; k--)
                            {
                                if (p_Border->m_RPnt[k].m_i16x > RightMost && p_Border->m_RPnt[k].m_i16x != IMGW - 2)
                                    RightMost = p_Border->m_RPnt[k].m_i16x;
                                if (p_Border->m_LPnt[k].m_i16x < LeftMost && p_Border->m_LPnt[k].m_i16x != 1)
                                    LeftMost = p_Border->m_LPnt[k].m_i16x;
                            }
                        }
                    }
                    else if (Zebra_start != 0)
                    {
                        if (p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 &&
                            myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                            Zebra_times++;
                    }
                }

                times = 0;
                Loop_Start = Zebra_start - Zebra_times / 2; // 0 <= i < IMGH
                if (Zebra_times >= 3 && Loop_Start > 0.2 * IMGH && Loop_Start < IMGH - 2 && Zebra_start != 0)
                {
                    for (k = Loop_Start - 2; k <= Loop_Start + 2; k++)
                    {
                        block_times = 0;
                        for (j = p_Border->m_RPnt[k].m_i16x; j < RightMost; j++)
                        {
                            if (image[k][j - 1] == 255 && image[k][j] == 0)
                            {
                                black_blocks = 0;
                                while (image[k][j++] == 0 && j < IMGW - 2)
                                    black_blocks++;
                                if (black_blocks > 3 && black_blocks < 8)
                                    block_times++;
                            }
                        }
                        for (j = p_Border->m_LPnt[k].m_i16x; j > LeftMost; j--)
                        {
                            if (image[k][j + 1] == 255 && image[k][j] == 0)
                            {
                                black_blocks = 0;
                                while (image[k][j--] == 0 && j > 1)
                                    black_blocks++;
                                if (black_blocks > 3 && black_blocks < 8)
                                    block_times++;
                            }
                        }
                        if (block_times >= 3)
                            times++;
                    }
                }

                if (times >= 2)
                {
                    p_TrackType->m_u8RRoundaboutFlag = 0;
                    p_TrackType->m_u8RunTurns++; //初始值存疑
                    if (p_TrackType->m_u8RunTurns != 2)
                    {
                        p_TrackType->m_u8ZebraCrossingFlag = 3;
                        Zebra_Switch_Flag = 0;
                        //                            beep_flag=1;
                        break;
                    }
                    else if (p_TrackType->m_u8RunTurns == 2)
                    {
                        /*右车库*/
                        p_TrackType->m_u8CarRunningState = 3; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                        beep_flag = 1;
                        break;
                    }
                }
            }

            if (p_TrackType->m_u8RRoundaboutFlag)
            {
                for (i = IMGH - 1; i >= IMGH * 0.85; i--)
                {
                    if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                        right_missingline_times += 1;
                    else
                        right_missingline_times = 0;
                    if (right_missingline_times > 12 && p_Border->m_RPnt[IMGH - 1].m_i16x == IMGW - 2)
                    {
                        beep_flag = 1;
                        p_TrackType->m_u8RRoundaboutFlag = 2;
                        break;
                    }
                }
                p_TrackType->m_u8RAllLostFlag = 3;
                p_TrackType->m_u8LAllLostFlag = 0;
            }
            break;
        case 2: //环岛右下标志点丢失 固定右下标志点 右下 右中补线
            LeftMost = IMGW;
            RightMost = 0;
            Zebra_times = 0;
            for (i = IMGH - 1; i > IMGH / 2; i--)
            {
                if (Zebra_start == 0)
                {
                    if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 1].m_i16x - p_Border->m_LPnt[i - 2].m_i16x) < 12 &&
                        myabs(p_Border->m_RPnt[i - 3].m_i16x - p_Border->m_LPnt[i - 3].m_i16x) < 12 &&
                        p_Border->m_RPnt[i].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 1].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 1].m_i16x > 1 &&
                        p_Border->m_RPnt[i - 2].m_i16x < IMGW - 2 &&
                        p_Border->m_LPnt[i - 2].m_i16x > 1)
                    {
                        Zebra_start = i;
                        Zebra_times++;
                        Loop_Start = Zebra_start + 20 < IMGH - 1 ? Zebra_start + 20 : IMGH - 1;
                        Loop_End = Zebra_start - 40 > 5 ? Zebra_start - 40 : 5;
                        for (k = Loop_Start; k >= Loop_End; k--)
                        {
                            if (p_Border->m_RPnt[k].m_i16x > RightMost && p_Border->m_RPnt[k].m_i16x != IMGW - 2)
                                RightMost = p_Border->m_RPnt[k].m_i16x;
                            if (p_Border->m_LPnt[k].m_i16x < LeftMost && p_Border->m_LPnt[k].m_i16x != 1)
                                LeftMost = p_Border->m_LPnt[k].m_i16x;
                        }
                    }
                }
                else if (Zebra_start != 0)
                {
                    if (p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_LPnt[i].m_i16x > 1 &&
                        myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                        Zebra_times++;
                }
            }

            times = 0;
            Loop_Start = Zebra_start - Zebra_times / 2; // 0 <= i < IMGH
            if (Zebra_times >= 3 && Loop_Start > 0.2 * IMGH && Loop_Start < IMGH - 2 && Zebra_start != 0)
            {
                for (k = Loop_Start - 2; k <= Loop_Start + 2; k++)
                {
                    block_times = 0;
                    for (j = p_Border->m_RPnt[k].m_i16x; j < RightMost; j++)
                    {
                        if (image[k][j - 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j++] == 0 && j < IMGW - 2)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    for (j = p_Border->m_LPnt[k].m_i16x; j > LeftMost; j--)
                    {
                        if (image[k][j + 1] == 255 && image[k][j] == 0)
                        {
                            black_blocks = 0;
                            while (image[k][j--] == 0 && j > 1)
                                black_blocks++;
                            if (black_blocks > 3 && black_blocks < 8)
                                block_times++;
                        }
                    }
                    if (block_times >= 3)
                        times++;
                }
            }

            if (times >= 2)
            {
                p_TrackType->m_u8RRoundaboutFlag = 0;
                p_TrackType->m_u8RunTurns++; //初始值存疑
                if (p_TrackType->m_u8RunTurns != 2)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 3;
                    Zebra_Switch_Flag = 0;
                    beep_flag = 1;
                    break;
                }
                else if (p_TrackType->m_u8RunTurns == 2)
                {
                    /*右车库*/
                    p_TrackType->m_u8CarRunningState = 3; //第二次扫到斑马线 车库标志位置1 其余情况仅当做斑马线情况
                    beep_flag = 1;
                    break;
                }
            }

            if (p_TrackType->m_u8RRoundaboutFlag)
            {
                times = 0;
                for (i = IMGH - 1; i >= IMGH - 6; i--)
                {
                    if (p_Border->m_RPnt[i].m_i16x < IMGW - 2)
                        times++;
                    else
                        times = 0;
                    if (times > 5)
                    {
                        beep_flag = 1;
                        Angle_offset = 0;
                        Roundabout_Wire_Flag = 0;
                        p_TrackType->m_u8RRoundaboutFlag = 3;
                    }
                }
                p_TrackType->m_u8RAllLostFlag = 3;
                p_TrackType->m_u8LAllLostFlag = 0;
            }
            break;
        case 3:
            times = 0;
            for (i = IMGH - Gap - 1; i >= 0.25 * IMGH; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - Gap].m_i16x > 50 && p_Border->m_RPnt[i].m_i16x - p_Border->m_RPnt[i - Gap].m_i16x < 120)
                {
                    Roundabout_RTpoint[0] = i - Gap;
                    Roundabout_RTpoint[1] = p_Border->m_RPnt[i - Gap].m_i16x;
                    if (!Roundabout_Wire_Flag)
                        Roundabout_Wire_Flag = 1;
                    if (Roundabout_RTpoint[1] > 1)
                        break;
                }
                if (p_Border->m_LPnt[i - Gap].m_i16x - p_Border->m_LPnt[i].m_i16x > 20 && p_Border->m_LPnt[i - Gap].m_i16x != 1 && p_Border->m_LPnt[IMGH - 1].m_i16x == 1 && p_Border->m_LPnt[IMGH - 2].m_i16x == 1 && p_Border->m_LPnt[IMGH - 3].m_i16x == 1 && Roundabout_Wire_Flag)
                {
                    Roundabout_RTpoint[0] = i - Gap;
                    Roundabout_RTpoint[1] = p_Border->m_LPnt[i - Gap].m_i16x;
                    break;
                }
            }
            //                if (Roundabout_RTpoint[0]>=Roundabout_Last_Line_In)  Roundabout_Last_Line_In=Roundabout_RTpoint[0];
            //                else Error_Reserve_Flag=1;

            if (!Roundabout_RTpoint[0] && !Roundabout_Wire_Flag)
            {
                i = IMGH - 5;
                times = 0;
                while (i-- > 0.2 * IMGH)
                {
                    if (p_Border->m_LPnt[i].m_i16x > 0.5 * IMGW)
                        times++;
                    if (times > 5)
                    {
                        p_TrackType->m_u8RRoundaboutFlag = 0;
                        p_TrackType->m_u8Pflag = 5;
                        break;
                    }
                }
            }
            else
            {
                if (Roundabout_RTpoint[0]) //&& Roundabout_RTpoint[0]>=Roundabout_Last_Line_In
                {
                    x1 = Roundabout_RTpoint[0];
                    x2 = Roundabout_RTpoint[0] + RRoundabout_TimeToTurn < IMGH - 1 ? Roundabout_RTpoint[0] + RRoundabout_TimeToTurn : IMGH - 1;
                    y1 = Roundabout_RTpoint[1];
                    if (x2 == IMGH - 1)
                        y2 = p_Border->m_LPnt[IMGH - 1].m_i16x;
                    else
                        y2 = p_Border->m_LPnt[Roundabout_RTpoint[0] + RRoundabout_TimeToTurn].m_i16x;
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_LPnt[j].m_i16x = t;
                        p_Border->m_PLPnt[j].m_f16x = (X_perspective[p_Border->m_LPnt[j].m_i16y][p_Border->m_LPnt[j].m_i16x]);
                    }
                    Temp_Round = p_Border->m_LPnt[Roundabout_RTpoint[0]].m_i16x;
                    for (i = x1 - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][Temp_Round] == B_BLACK)
                        {
                            for (j = Temp_Round; j <= IMGW - 3; j++)
                            {
                                if (image[i][j] == B_BLACK && image[i][j + 1] == B_WHITE)
                                {
                                    p_Border->m_LPnt[i].m_i16x = j;
                                    p_Border->m_PLPnt[i].m_f16x = (X_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
                                    Temp_Round = p_Border->m_LPnt[i].m_i16x;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            p_Border->m_LPnt[i].m_i16x = 1;
                            p_Border->m_PLPnt[i].m_f16x = (X_perspective[p_Border->m_LPnt[i].m_i16y][p_Border->m_LPnt[i].m_i16x]);
                        }
                    }
                }
                else
                    Error_Reserve_Flag = 1;
            }
            if (Fabs(Angle_offset) > 40) // || x1 > 0.8*IMGH  //右侧找线条件优化了已经
            {
                beep_flag = 1;
                p_TrackType->m_u8RRoundaboutFlag = 4;
            }
            p_TrackType->m_u8RAllLostFlag = 3; //不放外面,上拐点没找到,但没判进p //就有问题
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 4:
            for (i = IMGH - 1; i >= IMGH / 2; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i + Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                    ((p_Border->m_LPnt[i - 2 * Gap].m_i16x != 1 && p_Border->m_LPnt[i].m_i16x - p_Border->m_LPnt[i - 2 * Gap].m_i16x > 10) ||
                     p_Border->m_LPnt[i - 2 * Gap].m_i16x == 1))
                {
                    Roundabout_RBpoint[0] = i; //出环时 右下角拐点实际为圆环左下角标志点
                    Roundabout_RBpoint[1] = p_Border->m_LPnt[i].m_i16x;
                    break;
                }
            }
            //                if (Fabs(Angle_offset)>223 || (Roundabout_RBpoint[0] && Fabs(Angle_offset)>200))
            if (Fabs(Angle_offset) > 240 && Roundabout_RBpoint[0] > 0.5 * IMGH)
            {
                beep_flag = 1;
                p_TrackType->m_u8RRoundaboutFlag = 5; //!!!右下角已经是圆环部分(不丢线)时进入状态4 110可调
                Roundabout_To_Straight_Flag = 0;
                Roundabout_Last_Line_Out = Roundabout_Out_End;
                LastData_Y = 0;
                Angle_offset = 0;
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 5:
            times = 0;
            if (Roundabout_Last_Line_Out > 0.8 * IMGH)
            {
                for (i = 0.5 * IMGH; i >= 0.4 * IMGH; i--)
                {
                    if (p_Border->m_LPnt[i].m_i16x <= p_Border->m_LPnt[i - Gap].m_i16x &&
                        p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_LPnt[i - Gap].m_i16x > 1)
                        times++;
                    else
                        times = 0;
                }
                if (times >= 10 && Fabs(Angle_offset) > 50)
                {
                    Roundabout_To_Straight_Flag = 1;
                    x1 = 41;
                    x2 = 99;
                    y1 = p_Border->m_LPnt[41].m_i16x;
                    y2 = 1;
                    sum = 0;
                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_LPnt[j].m_i16x;
                        p_Border->m_LPnt[j].m_i16x = t;
                        p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                {
                    Roundabout_To_Straight_Flag = 0;
                    j = 140;
                    times = 0;
                    for (i = IMGH - 1; i >= 0.2 * IMGH; i--)
                    {
                        if (image[i][j] == B_WHITE)
                            times++;
                        else
                            times = 0;
                        if (times >= 10 && image[i - Gap][j] == B_BLACK && image[i - Gap - 1][j] == B_BLACK && image[i - Gap - 2][j] == B_BLACK)
                            break;
                    }
                    if (i >= 0.2 * IMGH)
                    {
                        x1 = i;
                        x2 = IMGH - 1;
                        y1 = j;
                        y2 = 0;

                        for (j = x1; j <= x2; j++)
                        {
                            t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                            image[j][t] = 0;
                            p_Border->m_LPnt[j].m_i16x = t;
                            p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                        }
                    }
                    else
                        Error_Reserve_Flag = 1;
                }
                if (Roundabout_To_Straight_Flag)
                {
                    if (sum < 0)
                        p_TrackType->m_u8RRoundaboutFlag = 6;
                }
            }
            else
            {
                i = IMGH - 5;
                while (i-- > Roundabout_Last_Line_Out - 5)
                {
                    if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                        p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i + Gap].m_i16x &&
                        ((p_Border->m_LPnt[i - 2 * Gap].m_i16x != 1 && p_Border->m_LPnt[i].m_i16x - p_Border->m_LPnt[i - 2 * Gap].m_i16x > 10) ||
                         p_Border->m_LPnt[i - 2 * Gap].m_i16x == 1))
                    {
                        Roundabout_RBpoint[0] = i; //出环时 右下角拐点实际为圆环左下角标志点
                        for (k = i + Gap; k >= i - Gap; k--)
                        {
                            if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                p_Border->m_LPnt[k].m_i16x >= p_Border->m_LPnt[k + 1].m_i16x)
                            {
                                Roundabout_RBpoint[0] = k;
                                break;
                            }
                        }
                        break;
                    }
                }
                if (Roundabout_RBpoint[0])
                    Roundabout_Last_Line_Out = Roundabout_RBpoint[0];
                if (Roundabout_RBpoint[0])
                {
                    x1 = 10;
                    x2 = Roundabout_RBpoint[0];
                    y1 = IMGW - 2;
                    y2 = p_Border->m_LPnt[Roundabout_RBpoint[0]].m_i16x;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_LPnt[j].m_i16x;
                        p_Border->m_LPnt[j].m_i16x = t;
                        p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                {
                    x1 = 10;
                    x2 = IMGH - 1;
                    y1 = IMGW - 2;
                    y2 = 0;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        if (j >= 0.7 * IMGH)
                            sum += t - p_Border->m_LPnt[j].m_i16x;
                        p_Border->m_LPnt[j].m_i16x = t;
                        p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 6:
            times = 0;
            for (i = IMGH - 1; i >= IMGH * 0.6; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2 && p_Border->m_LPnt[i - Gap].m_i16x >= p_Border->m_LPnt[i].m_i16x && p_Border->m_RPnt[i - Gap].m_i16x <= p_Border->m_RPnt[i].m_i16x)
                    times += 1; //|| p_Border->m_PRPnt[i].m_f16x==IMGW-2
                else
                    times = 0;
            }
            if (times > 32)
            {
                p_TrackType->m_u8RRoundaboutFlag = 0;
                beep_flag = 1;
                break;
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        }
    }

    /*斑马线*/
    if (p_TrackType->m_u8ZebraCrossingFlag != 0) //斑马线 入库 P //字必须放在环岛后 避免环岛状态清零后 没有保留误差 中线未处理的错乱情况
    {
        switch (p_TrackType->m_u8ZebraCrossingFlag)
        {
        case 1: //左侧车库的斑马线
            if (!Zebra_Switch_Flag)
            {
                RightMost = 0;
                i = IMGH;
                while (i-- > 0.2 * IMGH)
                {
                    if (Zebra_start == 0)
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                            Zebra_start = i;
                        if (p_Border->m_RPnt[i].m_i16x > RightMost && p_Border->m_RPnt[i].m_i16x < IMGW - 2)
                        {
                            Cross_RBpoint[0] = i;
                            RightMost = p_Border->m_RPnt[i].m_i16x;
                        }
                    }
                    else
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) >= 12)
                        {
                            Zebra_end = i;
                            RightMost = 0;
                            for (i = Zebra_end; i >= 0.3 * IMGH; i--)
                            {
                                if (p_Border->m_RPnt[i].m_i16x > RightMost)
                                {
                                    Cross_RTpoint[0] = i;
                                    RightMost = p_Border->m_RPnt[i].m_i16x;
                                }
                            }
                            break;
                        }
                    }
                }
                if (Zebra_start > 0.9 * IMGH)
                    Zebra_Switch_Flag = 1;
                else
                {
                    if (Cross_RTpoint[0] > 0.35 * IMGH)
                    {
                        x1 = Cross_RTpoint[0];
                        y1 = p_Border->m_RPnt[Cross_RTpoint[0]].m_i16x;
                        x2 = Cross_RBpoint[0];
                        y2 = p_Border->m_RPnt[Cross_RBpoint[0]].m_i16x;
                        for (j = x1; j <= x2; j++)
                        {
                            t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                            image[j][t] = 0;
                            p_Border->m_RPnt[j].m_i16x = t;
                            p_Border->m_PRPnt[j].m_f16x = X_perspective[j][p_Border->m_RPnt[j].m_i16x];
                        }
                    }
                    //                        else Error_Reserve_Flag=1;
                }
            }
            for (i = IMGH - 1; i >= 0.5 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) > 12)
                    times++;
                if (times > 45 && Zebra_Switch_Flag)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 0;
                    break;
                }
            }
            break;
        case 2: //搁置
            times = 0;
            for (i = IMGH - 1; i >= IMGH * 0.8; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2 && myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x > 12))
                    times += 1;
                else
                    times = 0;
                if (times > 16)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 0;
                    break;
                }
            }

            for (i = IMGH - 1; i >= 5; i--)
            {
                if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 && Zebra_start == 0)
                {
                    Zebra_start = i;
                }
                if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) > 12 && Zebra_start != 0 && Zebra_end == 0)
                {
                    Zebra_end = i;
                    break;
                }
            }
            if (Zebra_end == 0)
            {
                prospect_front = 50;
                prospect_later = 70;
            }
            else
            {
                prospect_later = Zebra_end - 1;
                if (prospect_later > 90)
                    prospect_later = 90;
                prospect_front = prospect_later - 20;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 3: //右侧车库的斑马线
            if (!Zebra_Switch_Flag)
            {
                LeftMost = IMGW;
                i = IMGH;
                while (i-- > 0.2 * IMGH)
                {
                    if (Zebra_start == 0)
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12)
                            Zebra_start = i;
                        if (p_Border->m_LPnt[i].m_i16x < LeftMost && p_Border->m_LPnt[i].m_i16x > 1)
                        {
                            Cross_LBpoint[0] = i;
                            LeftMost = p_Border->m_LPnt[i].m_i16x;
                        }
                    }
                    else
                    {
                        if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) >= 12)
                        {
                            Zebra_end = i;
                            //                                if (Zebra_end>0.7*IMGH) ZCrossing_Quit_Flag=1;
                            LeftMost = IMGW;
                            for (i = Zebra_end; i >= 0.3 * IMGH; i--)
                            {
                                if (p_Border->m_LPnt[i].m_i16x < LeftMost)
                                {
                                    Cross_LTpoint[0] = i;
                                    LeftMost = p_Border->m_LPnt[i].m_i16x;
                                }
                            }
                            break;
                        }
                    }
                }
                if (Zebra_start > 0.9 * IMGH)
                    Zebra_Switch_Flag = 1;
                else
                {
                    if (Cross_LTpoint[0] > 0.35 * IMGH)
                    {
                        x1 = Cross_LTpoint[0];
                        y1 = p_Border->m_LPnt[Cross_LTpoint[0]].m_i16x;
                        x2 = Cross_LBpoint[0];
                        y2 = p_Border->m_LPnt[Cross_LBpoint[0]].m_i16x;
                        for (j = x1; j <= x2; j++)
                        {
                            t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                            image[j][t] = 0;
                            p_Border->m_LPnt[j].m_i16x = t;
                            p_Border->m_PLPnt[j].m_f16x = X_perspective[j][p_Border->m_LPnt[j].m_i16x];
                        }
                    }
                    //                        else Error_Reserve_Flag=1;
                }
            }

            for (i = IMGH - 1; i >= 0.5 * IMGH; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x != IMGW - 2 && myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) > 12)
                    times++;
                if (times > 45 && Zebra_Switch_Flag)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 0;
                    break;
                }
            }
            p_TrackType->m_u8LAllLostFlag = 0;
            p_TrackType->m_u8RAllLostFlag = 3;

            break;
        case 4: //搁置
            times = 0;
            for (i = IMGH - 1; i >= IMGH * 0.8; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2 && myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x > 12))
                    times += 1;
                else
                    times = 0;
                if (times > 16)
                {
                    p_TrackType->m_u8ZebraCrossingFlag = 0;
                    break;
                }
            }
            for (i = IMGH - 5; i >= 5; i--)
            {
                if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) < 12 && !Zebra_start)
                    Zebra_start = i;
                if (myabs(p_Border->m_RPnt[i].m_i16x - p_Border->m_LPnt[i].m_i16x) > 12 && Zebra_start && !Zebra_end)
                {
                    Zebra_end = i;
                    break;
                }
            }
            if (Zebra_end)
            {
                x1 = Zebra_end - 20 > 5 ? Zebra_end - 20 : 5;
                x2 = IMGH - 1;
                for (i = x2; i >= 5; i--)
                    p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                y1 = p_Border->m_CPnt[(int16)x1].m_f16x;
                y2 = X_perspective[IMGH - 1][91];
                for (j = (int16)x1; j <= (int16)x2; j++)
                {
                    temp = (((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][(int16)temp] = 0;
                    p_Border->m_CPnt[j].m_f16x = temp;
                }
            }
            else
            {
                for (i = IMGH - 1; i >= 5; i--)
                {
                    p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                }
            }
            break;
        }
    }

    /*车库状态*/
    if (p_TrackType->m_u8CarRunningState != 0)
    {
        switch (p_TrackType->m_u8CarRunningState)
        {
        case 1:
            times = 0;
            for (i = IMGH * 0.9; i > 0.25 * IMGH; i--)
            {
                if (myabs(p_Border->m_LPnt[i].m_i16x - p_Border->m_RPnt[i].m_i16x) < 12 &&
                    p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_RPnt[i].m_i16x > 1)
                    break;
            }
            if (i > Garage_Left_Times * IMGH)
            {
                beep_flag = 1;
                Angle_offset = 0;
                p_TrackType->m_u8CarRunningState = 2;
            }
            break;
        case 2:
            //舵机向左打死 某特定行满足全黑 电机停转
            pid_angel.ActualAngel = Left_Max_angle;
            //pwm_duty(ATOM1_CH1_P33_9, (uint32)pid_angel.ActualAngel);
            if (Fabs(Angle_offset) > Garage_Left_Angle)
                pid_speed.SetSpeed = 0;
            break;
        case 3:
            times = 0;
            for (i = IMGH * 0.9; i >= 0.25 * IMGH; i--)
            {
                if (myabs(p_Border->m_LPnt[i].m_i16x - p_Border->m_RPnt[i].m_i16x) < 12 &&
                    p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_RPnt[i].m_i16x > 1)
                    break;
            }
            if (i > Garage_Right_Times * IMGH)
            {
                beep_flag = 1;
                Angle_offset = 0;
                p_TrackType->m_u8CarRunningState = 4;
            }
            break;
        case 4:
            //舵机向右打死 某特定行满足全黑 电机停转 状态位清零
            pid_angel.ActualAngel = Right_Max_angle;
            //pwm_duty(ATOM1_CH1_P33_9, (uint32)pid_angel.ActualAngel);
            if (Fabs(Angle_offset) > Garage_Right_Angle)
                pid_speed.SetSpeed = 0;
            break;
        }
    }

    if (p_TrackType->m_u8Pflag != 0)
    {
        switch (p_TrackType->m_u8Pflag)
        {
        case 1:
            for (i = 0.9 * IMGH; i >= 0.6 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                    times++;
                if (times > 25)
                {
                    p_TrackType->m_u8Pflag = 2;
                    break;
                }
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 2: // LPflag
            Lflag = 0;
            Rflag = 0;
            P_times = 0;
            for (i = IMGH - 1 - Gap; i >= 0.3 * IMGH; i--) //左下 右下特征点消失
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                {
                    Rflag = i;
                    for (k = Rflag + Gap; k >= Rflag - Gap; k--)
                    {
                        if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                            p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k + 1].m_i16x)
                        {
                            Rflag = k;
                            break;
                        }
                    }
                    break;
                }
            }
            for (i = Rflag; i >= Rflag - 5; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x || p_Border->m_RPnt[i - Gap].m_i16x == IMGW - 2)
                    times++;
            }
            if (Rflag > prospect_front && times > 4)
            {
                prospect_front = Rflag + 1;
                prospect_later = prospect_front + 20;
                if (prospect_later >= 99)
                    prospect_later = 99;
            }
            if (Rflag > LPflag_TimeToTurn * IMGH && times > 4)
            {
                beep_flag = 1;
                Angle_offset = 0;
                Pflag_Last_Line = Pflag_End;
                p_TrackType->m_u8Pflag = 3;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 3:
            //                if (Fabs(Angle_offset)>35)
            //                {
            //                    p_TrackType->m_u8Pflag=0;
            //                    break;
            //                }
            //                pid_angel.ActualAngel=Right_Max_angle;
            //                pwm_duty(ATOM1_CH1_P33_9,(uint32)pid_angel.ActualAngel);

            for (i = 0.5 * IMGH; i >= 0.4 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x <= p_Border->m_LPnt[i - Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_LPnt[i - Gap].m_i16x > 1)
                    times++;
                else
                    times = 0;
            }
            if (times >= 10 && Angle_offset < -15)
            {
                x1 = 41;
                x2 = 99;
                y1 = p_Border->m_LPnt[41].m_i16x;
                y2 = 1;
                sum = 0;
                for (j = x1; j <= x2; j++)
                {
                    t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][t] = 0;
                    if (j >= 0.7 * IMGH)
                        sum += t - p_Border->m_LPnt[j].m_i16x;
                    p_Border->m_LPnt[j].m_i16x = t;
                    p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                }
                if (sum < 0)
                {
                    p_TrackType->m_u8Pflag = 0;
                    break;
                }
            }
            else
            {
                if (Pflag_Last_Line < 0.8 * IMGH)
                {
                    i = IMGH;
                    Rflag = 0;
                    while (i-- > Pflag_Last_Line - 5)
                    {
                        if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x &&
                            p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i + Gap].m_i16x)
                        {
                            Rflag = i;
                            for (k = Rflag + Gap; k >= Rflag - Gap; k--)
                            {
                                if (p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k - 1].m_i16x &&
                                    p_Border->m_RPnt[k].m_i16x < p_Border->m_RPnt[k + 1].m_i16x)
                                {
                                    Rflag = k;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    Pflag_Last_Line = Rflag;
                }

                j = 140;
                times = 0;
                for (i = Pflag_Last_Line; i >= 5; i--)
                {
                    if (image[i][j] == B_WHITE)
                        times++;
                    else
                        times = 0;
                    if (times >= 10 && image[i - Gap][j] == B_BLACK && image[i - Gap - 1][j] == B_BLACK && image[i - Gap - 2][j] == B_BLACK)
                        break;
                }
                if (i >= 5)
                {
                    x1 = i;
                    x2 = IMGH - 1;
                    y1 = j;
                    y2 = 0.3 * IMGW;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_LPnt[j].m_i16x = t;
                        p_Border->m_PLPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                    Error_Reserve_Flag = 1;
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 4:
            for (i = 0.9 * IMGH; i >= 0.6 * IMGH; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
                    times++;
                if (times > 25)
                {
                    p_TrackType->m_u8Pflag = 5;
                    break;
                }
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 5: // RPflag
            Lflag = 0;
            Rflag = 0;
            for (i = IMGH - 1 - Gap; i >= 0.3 * IMGH; i--) //左下 右下特征点消失
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                    p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i + Gap].m_i16x)
                {
                    Lflag = i;
                    for (k = i + Gap; k >= i - Gap; k--)
                    {
                        if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                            p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k + 1].m_i16x)
                        {
                            Lflag = k;
                            break;
                        }
                    }
                    break;
                }
            }

            for (i = Lflag; i >= Lflag - 5; i--)
            {
                if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x || p_Border->m_LPnt[i - Gap].m_i16x == 1)
                    times++;
            }
            if (Lflag > prospect_front && times > 4)
            {
                prospect_front = Lflag;
                prospect_later = prospect_front + 20;
                if (prospect_later >= 99)
                    prospect_later = 99;
            }
            if (Lflag > RPflag_TimeToTurn * IMGH && times > 4)
            {
                Angle_offset = 0;
                Pflag_Last_Line = Pflag_End;
                p_TrackType->m_u8Pflag = 6;
            }
            p_TrackType->m_u8RAllLostFlag = 3;
            p_TrackType->m_u8LAllLostFlag = 0;
            break;
        case 6:
            //                if (Fabs(Angle_offset)>35)
            //                {
            //                    p_TrackType->m_u8Pflag=0;
            //                    break;
            //                }
            //                pid_angel.ActualAngel=Left_Max_angle;
            //                  pwm_duty(ATOM1_CH1_P33_9,(uint32)pid_angel.ActualAngel);

            for (i = 0.5 * IMGH; i >= 0.4 * IMGH; i--)
            {
                if (p_Border->m_RPnt[i].m_i16x >= p_Border->m_RPnt[i - Gap].m_i16x &&
                    p_Border->m_RPnt[i].m_i16x < IMGW - 2 && p_Border->m_RPnt[i - Gap].m_i16x < IMGW - 2)
                    times++;
                else
                    times = 0;
            }
            if (times >= 10 && Angle_offset > 15) //测试
            {
                x1 = 41;
                x2 = 99;
                y1 = p_Border->m_RPnt[41].m_i16x;
                y2 = IMGW - 2;
                sum = 0;
                for (j = x1; j <= x2; j++)
                {
                    t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                    image[j][t] = 0;
                    if (j >= 0.7 * IMGH)
                        sum += t - p_Border->m_RPnt[j].m_i16x;
                    p_Border->m_RPnt[j].m_i16x = t;
                    p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                }
                if (sum > 0)
                {
                    p_TrackType->m_u8Pflag = 0;
                    break;
                }
            }
            else
            {
                //                    x1=40;
                //                    x2=IMGH-1;
                //                    y1=0;
                //                    y2=IMGW*0.8;
                //                    for (j=x1;j<=x2;j++)
                //                    {
                //                      t=(int16)(((y1-y2)*j+x1*y2-x2*y1)/(x1-x2));
                //                      image[j][t]=0;
                //                      p_Border->m_RPnt[j].m_i16x=t;
                //                      p_Border->m_PRPnt[j].m_f16x=X_perspective[j][t];
                //                    }
                if (Pflag_Last_Line < 0.8 * IMGH)
                {
                    i = IMGH;
                    Lflag = 0;
                    while (i-- > Pflag_Last_Line - 5)
                    {
                        if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x &&
                            p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i + Gap].m_i16x)
                        {
                            Lflag = i;
                            for (k = i + Gap; k >= i - Gap; k--)
                            {
                                if (p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k - 1].m_i16x &&
                                    p_Border->m_LPnt[k].m_i16x > p_Border->m_LPnt[k + 1].m_i16x)
                                {
                                    Lflag = k;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                    Pflag_Last_Line = Lflag;
                }

                j = 40;
                times = 0;
                for (i = Pflag_Last_Line; i >= 5; i--)
                {
                    if (image[i][j] == B_WHITE)
                        times++;
                    else
                        times = 0;
                    if (times >= 10 && image[i - Gap][j] == B_BLACK && image[i - Gap - 1][j] == B_BLACK && image[i - Gap - 2][j] == B_BLACK)
                        break;
                }
                if (i >= 5)
                {
                    x1 = i;
                    x2 = IMGH - 1;
                    y1 = j;
                    y2 = IMGW * 0.7;

                    for (j = x1; j <= x2; j++)
                    {
                        t = (int16)(((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2));
                        image[j][t] = 0;
                        p_Border->m_RPnt[j].m_i16x = t;
                        p_Border->m_PRPnt[j].m_f16x = X_perspective[j][t];
                    }
                }
                else
                    Error_Reserve_Flag = 1;
            }
            p_TrackType->m_u8LAllLostFlag = 3;
            p_TrackType->m_u8RAllLostFlag = 0;
            break;
        case 7: // Pturn
            if (Fabs(Angle_offset) > 35)
            {
                p_TrackType->m_u8Pflag = 0;
                break;
            }
            if (Pturn == LEFT)
            {
                p_TrackType->m_u8RAllLostFlag = 3;
                p_TrackType->m_u8LAllLostFlag = 0;
            }
            else if (Pturn == RIGHT)
            {
                p_TrackType->m_u8LAllLostFlag = 3;
                p_TrackType->m_u8RAllLostFlag = 0;
            }
            break;
        }
    }
}

void examine(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    int16 i, j;
    int16 Lfind = 0, Rfind = 0, Lflag, Rflag;
    for (i = IMGH - 2; i >= 5; i--)
    {
        if (myabs(p_Border->m_LPnt[i].m_i16x - p_Border->m_LPnt[i + 1].m_i16x) > 20) //横向偏差过大 判断为不合理边界 根据上一行边界重新寻找
        {
            Lflag = 0;
            for (j = p_Border->m_LPnt[i + 1].m_i16x - 5; j <= p_Border->m_LPnt[i + 1].m_i16x + 5 && j >= 1 && j <= IMGW - 2; j++)
            {
                if (image[i][j - 1] == 0 && image[i][j] == 255)
                {
                    p_Border->m_LPnt[i].m_i16x = j;
                    Lflag = 1;
                    break;
                }
            }
            if (!Lflag)
                p_Border->m_LPnt[i].m_i16x = 1;
        }
        if (myabs(p_Border->m_RPnt[i + 1].m_i16x - p_Border->m_RPnt[i].m_i16x) > 20) //横向偏差过大 判断为不合理边界 根据上一行边界重新寻找
        {
            Rflag = 0;
            for (j = p_Border->m_RPnt[i + 1].m_i16x + 5; j >= p_Border->m_RPnt[i + 1].m_i16x - 5 && j >= 1 && j <= IMGW - 2; j--)
            {
                if (image[i][j] == 0 && image[i][j - 1] == 255)
                {
                    p_Border->m_RPnt[i].m_i16x = j;
                    Rflag = 1;
                    break;
                }
            }
            if (!Rflag)
                p_Border->m_RPnt[i].m_i16x = IMGW - 2;
        }
    }
}

void CenterlineGet(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    int16 i, j, x1, x2, y1, y2; //根据赛道数据拟合
    float k1, k2, n, t;
    float dx, dy, temp_sin, temp_cos, d;

    if (g_TrackType.m_u8RampFlag || g_TrackType.m_u8RampReadyFlag)
    {
        i = IMGH;
        while (i-- > 5)
        {
            p_Border->m_CPnt[i].m_f16x = (p_Border->m_LPnt[i].m_i16x + p_Border->m_RPnt[i].m_i16x) / 2;
            p_Border->m_CPnt[i].m_f16y = p_Border->m_LPnt[i].m_i16y;
        }
    }
    else if (g_TrackType.m_u8LAllLostFlag)
    {
        i = IMGH;
        switch (g_TrackType.m_u8LAllLostFlag)
        {
        case 1: //正常贴线
            while (i-- > 5)
            {
                dx = Fabs(p_Border->m_PRPnt[i - 1].m_f16x - p_Border->m_PRPnt[i + 1].m_f16x);
                dy = Fabs(p_Border->m_PRPnt[i - 1].m_f16y - p_Border->m_PRPnt[i + 1].m_f16y);
                d = FSqrt((dx * dx) + (dy * dy));
                temp_sin = dy / d;
                temp_cos = dx / d;
                p_Border->m_CPnt[i].m_f16x = (p_Border->m_PRPnt[i - 1].m_f16x + p_Border->m_PRPnt[i + 1].m_f16x) / 2 - Perspective_Normal_Width / 2 * temp_sin;
                p_Border->m_CPnt[i].m_f16y = (p_Border->m_PRPnt[i - 1].m_f16y + p_Border->m_PRPnt[i + 1].m_f16y) / 2 + Perspective_Normal_Width / 2 * temp_cos;
            }
            break;
        case 2: //贴近边线
            while (i-- > 5)
            {

                if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                {
                    if (i >= IMGH * 0.8)
                        continue;
                    else
                        break;
                }
                dx = Fabs(p_Border->m_PRPnt[i - 1].m_f16x - p_Border->m_PRPnt[i + 1].m_f16x);
                dy = Fabs(p_Border->m_PRPnt[i - 1].m_f16y - p_Border->m_PRPnt[i + 1].m_f16y);
                d = FSqrt((dx * dx) + (dy * dy));
                temp_sin = dy / d;
                temp_cos = dx / d;
                p_Border->m_CPnt[i].m_f16x = (p_Border->m_PRPnt[i - 1].m_f16x + p_Border->m_PRPnt[i + 1].m_f16x) / 2 - Perspective_Normal_Width / 2 * temp_sin;
                p_Border->m_CPnt[i].m_f16y = (p_Border->m_PRPnt[i - 1].m_f16y + p_Border->m_PRPnt[i + 1].m_f16y) / 2 - Perspective_Normal_Width / 2 * temp_cos;
            }
            x1 = (int16)p_Border->m_CPnt[i + 1].m_f16y;
            x2 = (int16)p_Border->m_CPnt[i + 2].m_f16y;
            y1 = (int16)p_Border->m_CPnt[i + 1].m_f16x;
            y2 = (int16)p_Border->m_CPnt[i + 2].m_f16x;
            j = x1;
            while (j-- > 5)
            {
                t = (float)((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2);
                if ((p_Border->m_RPnt[j].m_i16x != IMGW - 2 && j < x1 - 10) || t >= IMGW - 6)
                    break;
                else
                {
                    p_Border->m_CPnt[j].m_f16x = t;
                    p_Border->m_CPnt[j].m_f16y = p_Border->m_CPnt[j + 1].m_f16y - 1;
                }
            }
            break;
        case 3: //简单平移
            while (i-- > 5)
            {
                p_Border->m_CPnt[i].m_f16x = p_Border->m_PRPnt[i].m_f16x - Perspective_Normal_Width / 2;
                p_Border->m_CPnt[i].m_f16y = p_Border->m_PRPnt[i].m_f16y;
            }
            break;
        }
    }
    else if (g_TrackType.m_u8RAllLostFlag)
    {
        i = IMGH;
        switch (g_TrackType.m_u8RAllLostFlag)
        {
        case 1: //按斜率补线
            while (i-- > 5)
            {
                dx = Fabs(p_Border->m_PLPnt[i - 1].m_f16x - p_Border->m_PLPnt[i + 1].m_f16x);
                dy = Fabs(p_Border->m_PLPnt[i - 1].m_f16y - p_Border->m_PLPnt[i + 1].m_f16y);
                d = FSqrt((dx * dx) + (dy * dy));
                temp_sin = dy / d;
                temp_cos = dx / d;
                p_Border->m_CPnt[i].m_f16x = (p_Border->m_PLPnt[i - 1].m_f16x + p_Border->m_PLPnt[i + 1].m_f16x) / 2 + Perspective_Normal_Width / 2 * temp_sin;
                p_Border->m_CPnt[i].m_f16y = (p_Border->m_PLPnt[i - 1].m_f16y + p_Border->m_PLPnt[i + 1].m_f16y) / 2 + Perspective_Normal_Width / 2 * temp_cos;
            }
            break;
        case 2: //环岛入环单边补线
            while (i-- > 5)
            {
                if (p_Border->m_LPnt[i].m_i16x == 1)
                {
                    if (i >= 0.8 * IMGH)
                        continue;
                    else
                        break;
                    dx = Fabs(p_Border->m_PLPnt[i - 1].m_f16x - p_Border->m_PLPnt[i + 1].m_f16x);
                    dy = Fabs(p_Border->m_PLPnt[i - 1].m_f16y - p_Border->m_PLPnt[i + 1].m_f16y);
                    d = FSqrt((dx * dx) + (dy * dy));
                    temp_sin = dy / d;
                    temp_cos = dx / d;
                    p_Border->m_CPnt[i].m_f16x = (p_Border->m_PLPnt[i - 1].m_f16x + p_Border->m_PLPnt[i + 1].m_f16x) / 2 - Perspective_Normal_Width / 2 * temp_sin;
                    p_Border->m_CPnt[i].m_f16y = (p_Border->m_PLPnt[i - 1].m_f16y + p_Border->m_PLPnt[i + 1].m_f16y) / 2 - Perspective_Normal_Width / 2 * temp_cos;
                }
            }
            x1 = (int16)p_Border->m_CPnt[i + 1].m_f16y;
            x2 = (int16)p_Border->m_CPnt[i + 2].m_f16y;
            y1 = (int16)p_Border->m_CPnt[i + 1].m_f16x;
            y2 = (int16)p_Border->m_CPnt[i + 2].m_f16x;
            j = x1;
            while (j--)
            {
                t = (float)((y1 - y2) * j + x1 * y2 - x2 * y1) / (x1 - x2);
                if ((p_Border->m_LPnt[j].m_i16x != 1 && j < x1 - 10) || t <= 6)
                    break;
                else
                {
                    p_Border->m_CPnt[j].m_f16x = t;
                    p_Border->m_CPnt[j].m_f16y = p_Border->m_CPnt[j + 1].m_f16y - 1;
                }
            }
            break;
        case 3: //简单平移
            while (i-- > 5)
            {
                p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                p_Border->m_CPnt[i].m_f16y = p_Border->m_PLPnt[i].m_f16y;
            }
            break;
        }
    }
    else
    {
        i = IMGH;
        while (i-- > 5)
        {
            if (p_Border->m_LPnt[i].m_i16x != 1 && p_Border->m_RPnt[i].m_i16x != IMGW - 2)
            {
                p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                p_Border->m_CPnt[i].m_f16y = p_Border->m_PLPnt[i].m_f16y;
            }
            else if (p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2)
            {
                p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                p_Border->m_CPnt[i].m_f16y = p_Border->m_PLPnt[i].m_f16y;
            }
            else if (p_Border->m_LPnt[i].m_i16x == 1 && p_Border->m_RPnt[i].m_i16x == IMGW - 2) //双边丢线 先直接取中点
            {
                p_Border->m_CPnt[i].m_f16x = 90;
                p_Border->m_CPnt[i].m_f16y = (p_Border->m_PLPnt[i].m_f16y + p_Border->m_PRPnt[i].m_f16y) / 2;
            }
            else if (p_Border->m_LPnt[i].m_i16x == 1 || p_Border->m_RPnt[i].m_i16x == IMGW - 2)
            {
                if (p_Border->m_LPnt[i].m_i16x == 1)
                {
                    p_Border->m_CPnt[i].m_f16x = p_Border->m_PRPnt[i].m_f16x - Perspective_Normal_Width / 2;
                    p_Border->m_CPnt[i].m_f16y = p_Border->m_PRPnt[i].m_f16y;
                }
                else if (p_Border->m_RPnt[i].m_i16x == IMGW - 2)
                {
                    p_Border->m_CPnt[i].m_f16x = p_Border->m_PLPnt[i].m_f16x + Perspective_Normal_Width / 2;
                    p_Border->m_CPnt[i].m_f16y = p_Border->m_PLPnt[i].m_f16y;
                }
            }
        }
    }
}

void err_Cal(TRACK_BORDER_INFO *p_Border)
{
    int16 i, flag = 0;
    float err;
    float a, c = 2, k, t, sum = 0, y = 0, sum2 = 0, y2 = 0, sum3 = 0, y3 = 0;
    int16 missing_lines = 0;

    if (g_TrackType.m_u8RampFlag || g_TrackType.m_u8RampReadyFlag)
    {
        prospect_front = 0.85 * IMGH;
        prospect_later = 0.95 * IMGH;
    }
    else if (g_TrackType.m_u8Pflag != 0)
    {
        //图像里更改了P //字十字前瞻
    }
    else if (g_TrackType.m_u8ReadyFlag)
    {
        //图像中更改
        g_TrackType.m_u8ReadyFlag = 0;
    }
    else
    {
        if (Gear == 3 || Gear == 2)
        {
            Normal_Prospect_Front = 40;
            Normal_Prospect_Later = 60;
        }
        else
        {
            Normal_Prospect_Front = 45;
            Normal_Prospect_Later = 65;
        }
        prospect_front = Normal_Prospect_Front;
        prospect_later = Normal_Prospect_Later;
    }
    t = (prospect_later + prospect_front) / 2;
    a = (prospect_later - prospect_front) / 4;

    for (i = prospect_later; i >= prospect_front; i--) // err //均值从图像上往下取 遍历更完整 // 大错特错 上面的线是不准的会导致最下面的偏差不取 猛转
    {
        if (g_TrackType.m_u8LRoundaboutFlag >= 3)
        {
            k = 0.4 * FExp(-((i - t) * 1.0 / a) * ((i - t) * 1.0 / a) / 2.0);
            if (k < 0)
                k = 0;
            if (g_Border.m_RPnt[i].m_i16x < IMGW - 2)
            {
                sum = sum + k * p_Border->m_CPnt[i].m_f16x;
                y += k;
                flag = 1;
            }
        }
        else if (g_TrackType.m_u8RRoundaboutFlag >= 3)
        {
            k = 0.4 * FExp(-((i - t) * 1.0 / a) * ((i - t) * 1.0 / a) / 2.0);
            if (k < 0)
                k = 0;
            if (g_Border.m_LPnt[i].m_i16x > 1)
            {
                sum = sum + k * p_Border->m_CPnt[i].m_f16x;
                y += k;
                flag = 1;
            }
        }
        else if (g_TrackType.m_u8CrossFlag != 0)
        {
            k = 0.4 * FExp(-((i - t) * 1.0 / a) * ((i - t) * 1.0 / a) / 2.0);
            if (Fabs(p_Border->m_CPnt[i - Gap].m_f16x - p_Border->m_CPnt[i].m_f16x) > 10)
                break;
            if (k < 0)
                k = 0;
            sum = sum + k * p_Border->m_CPnt[i].m_f16x;
            y += k;
            flag = 1;
        }
        else if (g_TrackType.m_u8ThreeRoadsFlag != 0 || g_TrackType.m_u8ZebraCrossingFlag != 0)
        {
            k = 0.4 * FExp(-((i - t) * 1.0 / a) * ((i - t) * 1.0 / a) / 2.0);
            if (k < 0)
                k = 0;
            sum = sum + k * p_Border->m_CPnt[i].m_f16x;
            y += k;
            flag = 1;
        }
        else
        {
            if (((g_TrackType.m_u8LAllLostFlag != 0 && g_Border.m_RPnt[i].m_i16x < IMGW - 2) || (g_TrackType.m_u8RAllLostFlag != 0 && g_Border.m_LPnt[i].m_i16x > 1) || (g_TrackType.m_u8LAllLostFlag == 0 && g_TrackType.m_u8RAllLostFlag == 0 && (g_Border.m_RPnt[i].m_i16x < IMGW - 2 || g_Border.m_LPnt[i].m_i16x > 1))))
            {
                if (myabs(g_Border.m_RPnt[i].m_i16x - g_Border.m_LPnt[i].m_i16x) > 12)
                {
                    k = 0.4 * FExp(-((i - t) * 1.0 / a) * ((i - t) * 1.0 / a) / 2.0);
                    if (k < 0)
                        k = 0;
                    sum = sum + k * p_Border->m_CPnt[i].m_f16x;
                    y += k;
                    flag = 1;
                }
            }
        }
    }

    if (Error_Reserve_Flag)
    {
        flag = 0;
        Error_Reserve_Flag = 0;
    }

    if (flag)
    {
        err = sum / y;
        last_err = err;
    }
    else
        err = last_err;
    err_duty = err - Center;
}

float LeastSquareCalc_Curve(TRACK_BORDER_INFO *p_Border, int16 StartLine, int16 EndLine, int16 type)
{
    int16 i = 0;
    float Sum_X2 = 0, Sum_Y = 0, Sum_X4 = 0, Sum_YX2 = 0, Average_X2 = 0, Average_Y = 0, Average_X4 = 0, Average_YX2 = 0, Sum = 0, curve_a, curve_b;
    Sum = EndLine - StartLine;
    if (type == 0)
    { //中线
        for (i = EndLine; i > StartLine; i--)
        {
            //            if(i < 99-p_Border->m_Eq_CPnt_Length) break;
            Sum_X2 += p_Border->m_CPnt[i].m_f16y * p_Border->m_CPnt[i].m_f16y;
            Sum_Y += p_Border->m_CPnt[i].m_f16x;
            Sum_X4 += i * i * i * i;
            Sum_YX2 += p_Border->m_CPnt[i].m_f16x * p_Border->m_CPnt[i].m_f16y * p_Border->m_CPnt[i].m_f16y;
        }
    }
    else if (type == 1)
    { //右边线
        for (i = EndLine; i > StartLine; i--)
        {
            Sum_X2 += i * i;
            Sum_Y += p_Border->m_PRPnt[i].m_f16x;
            Sum_X4 += i * i * i * i;
            Sum_YX2 += p_Border->m_PRPnt[i].m_f16x * i * i;
        }
    }
    else if (type == 2)
    { //左边线
        for (i = EndLine; i > StartLine; i--)
        {
            Sum_X2 += i * i;
            Sum_Y += p_Border->m_PLPnt[i].m_f16x;
            Sum_X4 += i * i * i * i;
            Sum_YX2 += p_Border->m_PLPnt[i].m_f16x * i * i;
        }
    }
    Average_X2 = Sum_X2 / Sum;
    Average_Y = Sum_Y / Sum;
    Average_X4 = Sum_X4 / Sum;
    Average_YX2 = Sum_YX2 / Sum;
    curve_a = (Average_YX2 - Average_Y * Average_X2) / (Average_X4 - Average_X2 * Average_X2);
    curve_b = Average_Y - curve_a * Average_X2;
    return curve_a * curve_a + curve_b;
}
void Judge_S(TRACK_BORDER_INFO *p_Border)
{
    uint8 i, t, a;
    int16 j, max, min, k;
    uint8 Left_missing = 0, Right_missing = 0, times;
    float sum, y;
    uint8 Lflag = 0, Rflag = 0;
    for (i = 0.8 * IMGH; i >= 0.3 * IMGH; i--)
    {
        if (p_Border->m_LPnt[i].m_i16x == 1 &&
            p_Border->m_LPnt[i - 1].m_i16x == 1 &&
            p_Border->m_LPnt[i - 2].m_i16x == 1 &&
            p_Border->m_LPnt[i - 3].m_i16x == 1 &&
            p_Border->m_LPnt[i - 4].m_i16x == 1)
        {
            Left_missing = i;
            break;
        }
        if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x && Lflag == 0)
            Lflag = 1;
    }
    if (i == 30)
        Left_missing = 30;
    for (i = 0.8 * IMGH; i >= 0.3 * IMGH; i--)
    {
        if (p_Border->m_RPnt[i].m_i16x == IMGW - 2 &&
            p_Border->m_RPnt[i - 1].m_i16x == IMGW - 2 &&
            p_Border->m_RPnt[i - 2].m_i16x == IMGW - 2 &&
            p_Border->m_RPnt[i - 3].m_i16x == IMGW - 2 &&
            p_Border->m_RPnt[i - 4].m_i16x == IMGW - 2)
        {
            Right_missing = i;
            break;
        }
        if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x && Rflag == 0)
            Rflag = 1;
    }
    if (i == 30)
        Right_missing = 30;
    if (Left_missing == 30 && Right_missing == 30)
    {
        for (i = 80; i >= 30; i -= 2)
        {
            if (p_Border->m_LPnt[i].m_i16x > 1 && p_Border->m_RPnt[i].m_i16x < IMGW - 2)
            {
                k = 1;
                sum = sum + k * ((p_Border->m_LPnt[i].m_i16x + p_Border->m_RPnt[i].m_i16x) / 2);
                y += k;
            }
        }
        min = p_Border->m_LPnt[80].m_i16x;
        max = p_Border->m_RPnt[80].m_i16x;
        for (j = 0.8 * IMGH; j > Left_missing; j -= 2)
        {
            if (min > p_Border->m_LPnt[j].m_i16x)
                min = p_Border->m_LPnt[j].m_i16x;
            if (max < p_Border->m_RPnt[j].m_i16x)
                max = p_Border->m_RPnt[j].m_i16x;
        }
        times = 0;
        for (j = min; j <= max; j++) //左边线最大值——max //右边线最小值——min  //max<min
        {
            if (p_Border->m_u16MyLineBAr[j] > 70)
                times++; // 70的值需要调试
        }
        if (Fabs(sum / y - Center) < 10 && times > 40 && (Lflag == 1 || Rflag == 1))
        {
            g_TrackType.m_u8SmallSFlag = 1;
        }
        else
            g_TrackType.m_u8SmallSFlag = 0;
    }
    else
        g_TrackType.m_u8SmallSFlag = 0;
}

void straightaway_curve(TRACK_BORDER_INFO *p_Border, int16 type)
{
    uint8 i, t, a;
    uint8 Left_missing = 0, Right_missing = 0, times;
    int16 j, max, min;
    uint16 StraightLine, n;
    int32 Sumx = 0, Sumy = 0, Sumxx = 0, Sumxy = 0;
    uint8 Lflag = 0, Rflag = 0;
    float a0 = 0, a1 = 0, sum = 0, y = 0, k;
    if (type == 0)
    {
        for (i = 0.8 * IMGH; i >= 0.25 * IMGH; i--)
        {
            if (p_Border->m_LPnt[i].m_i16x == 1 &&
                p_Border->m_LPnt[i - 1].m_i16x == 1 &&
                p_Border->m_LPnt[i - 2].m_i16x == 1 &&
                p_Border->m_LPnt[i - 3].m_i16x == 1 &&
                p_Border->m_LPnt[i - 4].m_i16x == 1)
            {
                Left_missing = i;
                break;
            }
            if (p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i - Gap].m_i16x && Lflag == 0)
                Lflag = 1;
        }
        for (i = 0.8 * IMGH; i >= 0.25 * IMGH; i--)
        {
            if (p_Border->m_RPnt[i].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 1].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 2].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 3].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 4].m_i16x == IMGW - 2)
            {
                Right_missing = i;
                break;
            }
            if (p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i - Gap].m_i16x && Rflag == 0)
                Rflag = 1;
        }
    }
    else if (type == 1)
    { //左边线可靠
        for (i = 0.8 * IMGH; i >= 0.25 * IMGH; i--)
        {
            if (p_Border->m_LPnt[i].m_i16x == 1 &&
                p_Border->m_LPnt[i - 1].m_i16x == 1 &&
                p_Border->m_LPnt[i - 2].m_i16x == 1 &&
                p_Border->m_LPnt[i - 3].m_i16x == 1 &&
                p_Border->m_LPnt[i - 4].m_i16x == 1)
            {
                Left_missing = i;
                break;
            }
        }
        Right_missing = 99;
    }
    else if (type == 2)
    { //右边线可靠
        for (i = 0.8 * IMGH; i >= 0.25 * IMGH; i--)
        {
            if (p_Border->m_RPnt[i].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 1].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 2].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 3].m_i16x == IMGW - 2 &&
                p_Border->m_RPnt[i - 4].m_i16x == IMGW - 2)
            {
                Right_missing = i;
                break;
            }
        }
        Left_missing = 99;
    }
    if (Left_missing == 0)
        Left_missing = 0.24 * IMGH;
    if (Right_missing == 0)
        Right_missing = 0.24 * IMGH;

    if (Left_missing == Right_missing && Left_missing == 24)
        Single_missing = 1;
    else
        Single_missing = 0;

    if (Left_missing > Right_missing)
    { //右边线可靠
        for (j = 0.8 * IMGH; j > Right_missing; j--)
        {
            StraightLine = j;
            n += 1;
            Sumx += p_Border->m_RPnt[j].m_i16x;
            Sumy += j;
            Sumxx += (p_Border->m_RPnt[j].m_i16x * p_Border->m_RPnt[j].m_i16x);
            Sumxy += (p_Border->m_RPnt[j].m_i16x * j);
            if (j <= 0.8 * IMGH - 10)
            {
                a0 = (Sumxx * Sumy - Sumx * Sumxy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                a1 = (n * Sumxy - Sumx * Sumy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                if (myabs((int16)((j - 1 - a0) / a1) - p_Border->m_RPnt[j - 1].m_i16x) > 2 && myabs((int16)((j - 2 - a0) / a1) - p_Border->m_RPnt[j - 2].m_i16x) > 2 && myabs((int16)((j - 3 - a0) / a1) - p_Border->m_RPnt[j - 3].m_i16x) > 2 && myabs((int16)((j - 4 - a0) / a1) - p_Border->m_RPnt[j - 4].m_i16x) > 2)
                {
                    last_last_straight_lines = last_straight_lines;
                    last_straight_lines = straight_lines;
                    straight_lines = j;
                    break;
                }
            }
        }
        if (j == Right_missing)
            straight_lines = Right_missing;
    }
    else if (Left_missing < Right_missing)
    { //左边线可靠
        for (j = 0.8 * IMGH; j > Left_missing; j--)
        {
            StraightLine = j;
            n += 1;
            Sumx += p_Border->m_LPnt[j].m_i16x;
            Sumy += j;
            Sumxx += (p_Border->m_LPnt[j].m_i16x * p_Border->m_LPnt[j].m_i16x);
            Sumxy += (p_Border->m_LPnt[j].m_i16x * j);
            if (j <= 0.8 * IMGH - 10)
            {
                a0 = (Sumxx * Sumy - Sumx * Sumxy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                a1 = (n * Sumxy - Sumx * Sumy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                if (myabs((int16)((j - 1 - a0) / a1) - p_Border->m_LPnt[j - 1].m_i16x) > 2 && myabs((int16)((j - 2 - a0) / a1) - p_Border->m_LPnt[j - 2].m_i16x) > 2 && myabs((int16)((j - 3 - a0) / a1) - p_Border->m_LPnt[j - 3].m_i16x) > 2 && myabs((int16)((j - 4 - a0) / a1) - p_Border->m_LPnt[j - 4].m_i16x) > 2)
                {
                    last_last_straight_lines = last_straight_lines;
                    last_straight_lines = straight_lines;
                    straight_lines = j;
                    break;
                }
            }
        }
        if (j == Left_missing)
            straight_lines = Left_missing;
    }
    else if (Left_missing == Right_missing && Left_missing == 0.24 * IMGH)
    {
        for (j = 0.8 * IMGH; j > Left_missing; j--)
        {
            StraightLine = j;
            n += 1;
            Sumx += p_Border->m_LPnt[j].m_i16x;
            Sumy += j;
            Sumxx += (p_Border->m_LPnt[j].m_i16x * p_Border->m_LPnt[j].m_i16x);
            Sumxy += (p_Border->m_LPnt[j].m_i16x * j);
            if (j <= 0.8 * IMGH - 10)
            {
                a0 = (Sumxx * Sumy - Sumx * Sumxy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                a1 = (n * Sumxy - Sumx * Sumy) / (1.0 * (n * Sumxx - Sumx * Sumx));
                if (myabs((int16)((j - 1 - a0) / a1) - p_Border->m_LPnt[j - 1].m_i16x) > 2 && myabs((int16)((j - 2 - a0) / a1) - p_Border->m_LPnt[j - 2].m_i16x) > 2 && myabs((int16)((j - 3 - a0) / a1) - p_Border->m_LPnt[j - 3].m_i16x) > 2 && myabs((int16)((j - 4 - a0) / a1) - p_Border->m_LPnt[j - 4].m_i16x) > 2)
                {
                    last_last_straight_lines = last_straight_lines;
                    last_straight_lines = straight_lines;
                    straight_lines = j;
                    break;
                }
            }
        }
        if (j == Left_missing)
            straight_lines = Left_missing;
    }
    if (Left_missing < Right_missing)
        g_TrackType.m_u8RAllLostFlag = 3;
    if (Left_missing > Right_missing)
        g_TrackType.m_u8LAllLostFlag = 3;
    if (Left_missing == Right_missing && Left_missing == 0.24 * IMGH)
    {
        if (straight_lines < 35)
            g_TrackType.m_u8RAllLostFlag = 3;
        else
            g_TrackType.m_u8LAllLostFlag = 3;
    }
    //    if(straight_lines < 35 && Left_missing <= Right_missing && Lflag == 0) g_TrackType.m_u8RAllLostFlag=3;
    //    if(straight_lines < 35 && Left_missing >= Right_missing && Rflag == 0) g_TrackType.m_u8LAllLostFlag=3;
}

// void straightaway_curve(TRACK_BORDER_INFO *p_Border,int16 type){
//     uint8 i,k;
//     uint8 Left_missing=0,Right_missing=0,times;
//     int16 j,max,min;
//     uint16 StraightLine,n;
//     int32 Sumx=0,Sumy=0,Sumxx=0,Sumxy=0;
//     uint8 Lflag=0,Rflag=0;
//     float a0=0,a1=0;
//     if(type == 0)
//     {
//         for(i=0.9*IMGH;i>=0.25*IMGH;i--)
//         {
//             if(p_Border->m_LPnt[i].m_i16x == 1 &&
//                p_Border->m_LPnt[i-1].m_i16x == 1 &&
//                p_Border->m_LPnt[i-2].m_i16x == 1 &&
//                p_Border->m_LPnt[i-3].m_i16x == 1 &&
//                p_Border->m_LPnt[i-4].m_i16x == 1
//                )
//             {
//                 Left_missing = i;
//                 break;
//             }
//             if(p_Border->m_LPnt[i].m_i16x > p_Border->m_LPnt[i-Gap].m_i16x && Lflag == 0) Lflag = 1;
//         }
//         for(i=0.9*IMGH;i>=0.25*IMGH;i--)
//         {
//             if(p_Border->m_RPnt[i].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-1].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-2].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-3].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-4].m_i16x == IMGW-2
//                )
//             {
//                 Right_missing = i;
//                 break;
//             }
//             if(p_Border->m_RPnt[i].m_i16x < p_Border->m_RPnt[i-Gap].m_i16x && Rflag == 0) Rflag = 1;
//         }
//     }
//     else if(type == 1)
//     {    //左边线可靠
//         for(i=0.9*IMGH;i>=0.25*IMGH;i--)
//         {
//             if(p_Border->m_LPnt[i].m_i16x == 1 &&
//                p_Border->m_LPnt[i-1].m_i16x == 1 &&
//                p_Border->m_LPnt[i-2].m_i16x == 1 &&
//                p_Border->m_LPnt[i-3].m_i16x == 1 &&
//                p_Border->m_LPnt[i-4].m_i16x == 1)
//             {
//                 Left_missing = i;
//                 break;
//             }
//         }
//         Right_missing = 99;
//     }
//     else if(type == 2)
//     {    //右边线可靠
//         for(i=0.9*IMGH;i>=0.25*IMGH;i--)
//         {
//             if(p_Border->m_RPnt[i].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-1].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-2].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-3].m_i16x == IMGW-2 &&
//                p_Border->m_RPnt[i-4].m_i16x == IMGW-2)
//             {
//                 Right_missing = i;
//                 break;
//             }
//         }
//         Left_missing = 99;
//     }
//     if(Left_missing == 0)Left_missing = 0.24*IMGH;
//     if(Right_missing == 0)Right_missing = 0.24*IMGH;
//
//     if(Left_missing > Right_missing){   //右边线可靠
//         for(j=0.9*IMGH;j>Right_missing;j--){
//             StraightLine = j;
//             n+=1;
//             Sumx+=p_Border->m_RPnt[j].m_i16x;
//             Sumy+=j;
//             Sumxx+=(p_Border->m_RPnt[j].m_i16x*p_Border->m_RPnt[j].m_i16x);
//             Sumxy+=(p_Border->m_RPnt[j].m_i16x*j);
//             if(j<=0.9*IMGH-8){
//                 a0 = (Sumxx*Sumy - Sumx*Sumxy)/(1.0*(n*Sumxx - Sumx*Sumx));
//                 a1 = (n*Sumxy - Sumx*Sumy)/(1.0*(n*Sumxx - Sumx*Sumx));
//                 if( myabs((int16)((j-1-a0)/a1) - p_Border->m_RPnt[j-1].m_i16x) > 2
//                         && myabs((int16)((j-2-a0)/a1) - p_Border->m_RPnt[j-2].m_i16x) > 2
//                         && myabs((int16)((j-3-a0)/a1) - p_Border->m_RPnt[j-3].m_i16x) > 2
//                         && myabs((int16)((j-4-a0)/a1) - p_Border->m_RPnt[j-4].m_i16x) > 2)
//                 {
//                     last_last_straight_lines = last_straight_lines;
//                     last_straight_lines = straight_lines;
//                     straight_lines = j;
//                     break;
//                 }
//             }
//         }
//         if(j == Right_missing) straight_lines = Right_missing;
//     }else if(Left_missing < Right_missing){                              //左边线可靠
//         for(j=0.9*IMGH;j>Left_missing;j--){
//             StraightLine = j;
//             n+=1;
//             Sumx+=p_Border->m_LPnt[j].m_i16x;
//             Sumy+=j;
//             Sumxx+=(p_Border->m_LPnt[j].m_i16x*p_Border->m_LPnt[j].m_i16x);
//             Sumxy+=(p_Border->m_LPnt[j].m_i16x*j);
//             if(j<=0.9*IMGH-8){
//                 a0 = (Sumxx*Sumy - Sumx*Sumxy)/(1.0*(n*Sumxx - Sumx*Sumx));
//                 a1 = (n*Sumxy - Sumx*Sumy)/(1.0*(n*Sumxx - Sumx*Sumx));
//                 if(myabs((int16)((j-1-a0)/a1) - p_Border->m_LPnt[j-1].m_i16x) > 2
//                         && myabs((int16)((j-2-a0)/a1) - p_Border->m_LPnt[j-2].m_i16x) > 2
//                         && myabs((int16)((j-3-a0)/a1) - p_Border->m_LPnt[j-3].m_i16x) > 2
//                         && myabs((int16)((j-4-a0)/a1) - p_Border->m_LPnt[j-4].m_i16x) > 2)
//                 {
//                     last_last_straight_lines = last_straight_lines;
//                     last_straight_lines = straight_lines;
//                     straight_lines = j;
//                     break;
//                 }
//             }
//         }
//         if(j == Left_missing) straight_lines = Left_missing;
//     }else if(Left_missing == Right_missing && Left_missing == 0.24*IMGH){
//         max = p_Border->m_LPnt[90].m_i16x;
//         min = p_Border->m_RPnt[90].m_i16x;
//         for(j=0.9*IMGH-1;j>Left_missing;j-=2){
//             if(max < p_Border->m_LPnt[j].m_i16x) max=p_Border->m_LPnt[j].m_i16x;
//             if(min > p_Border->m_RPnt[j].m_i16x) min=p_Border->m_RPnt[j].m_i16x;
//         }
//         times=0;
//         for (j=max;j<=min;j++) //左边线最大值——max //右边线最小值——min  //max<min
//         {
//             if (p_Border->m_u16MyLineBAr[j]>70) times++;  //70的值需要调试
//         }
//         if((max+min)/2 <=100 && (max+min)/2 >= 80 && min-max>30 && times>0.8*(min-max)){
//             last_last_straight_lines = last_straight_lines;
//             last_straight_lines = straight_lines;
//             straight_lines = 0.24*IMGH;
//         }
//     }
//     if(straight_lines < 40 && Left_missing <= Right_missing && Lflag == 0) g_TrackType.m_u8RAllLostFlag=3;
//     if(straight_lines < 40 && Left_missing >= Right_missing && Rflag == 0) g_TrackType.m_u8LAllLostFlag=3;
// }

void draw_centerline(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    // int16 i;
    // i = IMGH - 5;
    // while (i-- > 5)
    // {
    //     ips200_drawpoint((int16)p_Border->m_PLPnt[i].m_f16x, (int16)p_Border->m_PLPnt[i].m_f16y, RGB565_RED);
    //     if (p_Border->m_CPnt[i].m_f16x >= 188)
    //         p_Border->m_CPnt[i].m_f16x = 187;
    //     if (p_Border->m_CPnt[i].m_f16y >= 100)
    //         p_Border->m_CPnt[i].m_f16y = 99;
    //     ips200_drawpoint((int16)p_Border->m_CPnt[i].m_f16x, (int16)p_Border->m_CPnt[i].m_f16y, RGB565_BLACK);
    //     ips200_drawpoint((int16)p_Border->m_PRPnt[i].m_f16x, (int16)p_Border->m_PRPnt[i].m_f16y, RGB565_BLUE);
    // }
}

//最小二乘法拟合y=kx+b
float Linear_fitting(TRACK_BORDER_INFO *p_stBorder, int16 startline, int16 endline)
{
    float height, A = 0, B = 0, C = 0, D = 0;
    int i;
    float k;
    height = endline - startline + 1;
    for (i = startline; i <= endline; i++)
    {
        A += p_stBorder->m_CPnt[i].m_f16x * p_stBorder->m_CPnt[i].m_f16x;
        B += p_stBorder->m_CPnt[i].m_f16y;
        C += p_stBorder->m_CPnt[i].m_f16x;
        D += p_stBorder->m_CPnt[i].m_f16x * p_stBorder->m_CPnt[i].m_f16y;
    }
    k = (D - B * C * 1.0 / height) / (A - C * C / height);
    return k;
}

void regression(TRACK_BORDER_INFO *p_stBorder, int16 startline, int16 endline, uint8 type)
{
    int16 i = 0;
    int16 sumlines = endline - startline;
    float sumX = 0;
    float sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    Variance = 0;
    if (type == 1)
    {
        for (i = startline; i < endline && p_stBorder->m_LPnt[i].m_i16x > 1; i++)
        {
            sumX += i;
            sumY += p_stBorder->m_LPnt[i].m_i16x;
        }
        if (sumlines != 0)
        {
            averageX = (float)(sumX / sumlines); // x的平均值
            averageY = (float)(sumY / sumlines); // y的平均值
        }
        else
        {
            averageX = 0; // x的平均值
            averageY = 0; // y的平均值
        }
        for (i = startline; i < endline && p_stBorder->m_LPnt[i].m_i16x > 1; i++)
        {
            sumUp += (p_stBorder->m_LPnt[i].m_i16x - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0)
            parameterB = 0;
        else
            parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
        for (i = startline; i < endline && p_stBorder->m_LPnt[i].m_i16x > 1; i++)
        {
            Variance += Fabs(parameterB * i + parameterA - p_stBorder->m_LPnt[i].m_i16x);
        }
    }
    else if (type == 2)
    {
        for (i = startline; i < endline && p_stBorder->m_RPnt[i].m_i16x < IMGW - 2; i++)
        {
            sumX += i;
            sumY += p_stBorder->m_RPnt[i].m_i16x;
        }
        if (sumlines != 0)
        {
            averageX = (float)(sumX / sumlines); // x的平均值
            averageY = (float)(sumY / sumlines); // y的平均值
        }
        else
        {
            averageX = 0; // x的平均值
            averageY = 0; // y的平均值
        }
        for (i = startline; i < endline && p_stBorder->m_RPnt[i].m_i16x < IMGW - 2; i++)
        {
            sumUp += (p_stBorder->m_RPnt[i].m_i16x - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0)
            parameterB = 0;
        else
            parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
        for (i = startline; i < endline && p_stBorder->m_RPnt[i].m_i16x < IMGW - 2; i++)
        {
            Variance += Fabs(parameterB * i + parameterA - p_stBorder->m_RPnt[i].m_i16x);
        }
    }
}
uint8 Angle_judge(uint8 type, int16 bottomline, int16 middleline, int16 topline)
{
    float x1, y1, x2, y2;
    if (type == 0)
    {
        if (g_Border.m_PLPnt[topline].m_f16x == 1)
            return 0;
        x1 = (float)(g_Border.m_PLPnt[bottomline].m_f16x - g_Border.m_PLPnt[middleline].m_f16x);
        x2 = (float)(g_Border.m_PLPnt[topline].m_f16x - g_Border.m_PLPnt[middleline].m_f16x);
        y1 = (float)(bottomline - middleline);
        y2 = (float)(topline - middleline);
        res = (x1 * x2 + y1 * y2) / (FSqrt(x1 * x1 + y1 * y1) * FSqrt(x2 * x2 + y2 * y2));
        if (res < 0)
            return 1; //钝角
        else
            return 0;
    }
    else if (type == 1)
    {
        if (g_Border.m_PRPnt[topline].m_f16x == IMGW - 2)
            return 0;
        x1 = (float)(g_Border.m_PRPnt[bottomline].m_f16x - g_Border.m_PRPnt[middleline].m_f16x);
        x2 = (float)(g_Border.m_PRPnt[topline].m_f16x - g_Border.m_PRPnt[middleline].m_f16x);
        y1 = (float)(bottomline - middleline);
        y2 = (float)(topline - middleline);
        res = (x1 * x2 + y1 * y2) / (FSqrt(x1 * x1 + y1 * y1) * FSqrt(x2 * x2 + y2 * y2));
        if (res < 0)
            return 1; //钝角
        else
            return 0;
    }
    return -1;
}

float Roadwidth_Cal(int16 line)
{
    return (g_Border.m_RPnt[line].m_i16x - g_Border.m_LPnt[line].m_i16x);
}
float process_curvity(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float K;
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    //面积的符号表示方向
    float q1 = (float)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    float AB = sqrt(q1);
    q1 = (float)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    float BC = sqrt(q1);
    q1 = (float)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    float AC = sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}
void Equidistant_Sample(TRACK_BORDER_INFO *p_Border, float dist)
{
    int16 i = 99;
    int16 len = 1;
    p_Border->m_Eq_CPnt[99].m_f16x = p_Border->m_CPnt[99].m_f16x;
    p_Border->m_Eq_CPnt[99].m_f16y = p_Border->m_CPnt[99].m_f16y;
    p_Border->m_Eq_CPnt_Length = 99;
    while ((((g_TrackType.m_u8LAllLostFlag != 0 && g_Border.m_RPnt[i].m_i16x < IMGW - 2) || (g_TrackType.m_u8RAllLostFlag != 0 && g_Border.m_LPnt[i].m_i16x > 1) || (g_TrackType.m_u8LAllLostFlag == 0 && g_TrackType.m_u8RAllLostFlag == 0 && (g_Border.m_RPnt[i].m_i16x < IMGW - 2 || g_Border.m_LPnt[i].m_i16x > 1))) || g_TrackType.m_u8CrossFlag != 0) && len < p_Border->m_Eq_CPnt_Length && i > 6)
    {
        float x0 = p_Border->m_CPnt[i].m_f16x;
        float y0 = p_Border->m_CPnt[i].m_f16y;
        float x1 = p_Border->m_CPnt[i - 1].m_f16x;
        float y1 = p_Border->m_CPnt[i - 1].m_f16y;
        do
        {
            float x = p_Border->m_Eq_CPnt[100 - len].m_f16x;
            float y = p_Border->m_Eq_CPnt[100 - len].m_f16y;
            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;
            float dist0 = FSqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = FSqrt(dx1 * dx1 + dy1 * dy1);
            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;
            if (r0 < 0 || r1 < 0)
                break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            p_Border->m_Eq_CPnt[99 - len].m_f16x = x0;
            p_Border->m_Eq_CPnt[99 - len].m_f16y = y0;
            len++;
        } while (len < p_Border->m_Eq_CPnt_Length);
        i--;
    }
    p_Border->m_Eq_CPnt_Length = len;
}
void Out_Protect(uint8 (*image)[IMGW])
{
    uint8 i;
    int16 Sum = 0;
    for (i = 0; i < 188; i++)
    {
        if (image[90][i] == 255)
            Sum++;
    }
    if (Sum < 50)
        g_TrackType.m_u8CarRunningState = 2;
}
