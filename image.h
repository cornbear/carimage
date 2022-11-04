#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include <string.h>

#include "type.h"
#include "hardware.h"
#include "pid.h"
#include "Mymath.h"

#define YES_2 2 //未找到，备选状态,确定可能需要二次确定或者第二确定状态
#define YES 1   //已找到，但违反单调性，需要补线
#define NO 0

/*定义二值化图像黑色值与白色值*/
#define B_BLACK 0
#define B_WHITE 255

#define LEFT 1
#define RIGHT 2
#define STRAIGHT 3

#define BEEP_PIN P15_7

typedef struct int_point_info_
{
    int16 m_i16x;
    int16 m_i16y;
} INT_POINT_INFO;

typedef struct float_point_info_
{
    float m_f16x;
    float m_f16y;
} FLOAT_POINT_INFO;

typedef struct trackborder_info_
{
    /*经典算法边界数据*/
    INT_POINT_INFO m_LPnt[IMGH];      /*左边边线数组*/
    FLOAT_POINT_INFO m_PLPnt[IMGH];   /*左边线逆透视数组*/
    INT_POINT_INFO m_RPnt[IMGH];      /*右边边线数组*/
    FLOAT_POINT_INFO m_PRPnt[IMGH];   /*右边线逆透视数组*/
    FLOAT_POINT_INFO m_CPnt[IMGH];    /*中线数组*/
    FLOAT_POINT_INFO m_Eq_CPnt[IMGH]; /*等距变换后的中线数组*/
    int16 m_Eq_CPnt_Length;           /*等距变换后的中线数组长度*/
    int16 m_i16LPointCnt;             /*左边找到点数*/
    int16 m_i16RPointCnt;             /*右边找到点数*/

    /*int16 m_i16LEndNum;                  左边找到点数
    int16 m_i16REndNum;                  右边找到点数
    */
    int16 m_i16LLostPointCnt; /*左边丢失点总数*/
    int16 m_i16RLostPointCnt; /*右边丢失点总数*/
    uint32 m_u32LAllArea;     /*左边白色点总面积*/
    uint32 m_u32RAllArea;     /*右边白色点总面积*/
    uint32 m_u32AllArea;      /*中间部分总面积（横向）*/

    int8 m_i8LMonotonicity[IMGH]; /*左边单调性 (复用:找线阶段用它来描述需要是否需要补线,或者补线的类型,左边)*/
    int8 m_i8LMonotonicityCnt;    /*违反点调性的点数*/
    int8 m_i8RMonotonicity[IMGH]; /*右边单调性 (复用:同上,描述右边)*/
    int8 m_i8RMonotonicityCnt;    /*违反点调性的点数*/
    INT_POINT_INFO Cturn;         /*中间顶点 主要用于三叉*/

    //        F32 angle_L;
    //        F32 angle_R;
    //        int16 m_i16OutWidthROWNum;  //超宽函数/
    //        int16 m_i16WOutWidthROWNum; //非常严重的超宽状态
    //        int16 m_i16OutWidthStart;   //超宽开始行
    //        int16 m_i16OutWidthEnd;     //超宽结束行

    float Threshold;
    uint8 LastThreshold;

    INT_POINT_INFO m_LMaxPoint; /*左边最大点,在正常情况下,最大点不应该出现在边线的中间,只会出现在边线结束,出现在中间,那么这个最大点应该就*/
    INT_POINT_INFO m_RMinPoint;

    /*新思路算法定义数据*/
    INT_POINT_INFO m_OptimalPoint;    /*实际最大列所在的最优点*/
    INT_POINT_INFO m_LOptimalPoint;   //实际左半边最大列所在的最优点
    INT_POINT_INFO m_ROptimalPoint;   //实际右半边最大列所在的最优点
    INT_POINT_INFO m_CenterLinePoint; //图像中间优所在的最优点

    int16 m_i16JumpPointCnt; /*跳变点,全图盲搜的黑跳白,白跳黑的跳变点*/
                             /*基于最优点的面积*/
    int32 m_i32OptimalLAreaAr[IMGH];
    int32 m_i32OptimalRAreaAr[IMGH];
    /*基于中线点的面积*/
    int32 m_i16CenterLAreaAr[IMGH];
    int32 m_i16CenterRAreaAr[IMGH];
    /*记录每列从底部向上的白点数数组*/
    uint16 m_u16LineBAr[IMGW];
    uint16 m_u16MyLineBAr[IMGW];
} TRACK_BORDER_INFO;

/*赛道类型结构体*/
typedef struct track_type_info_
{
    uint32 m_u32CrossTime;      /*十字标识别到的时间戳,用于超时解除识别*/
    uint32 m_u32RampTime;       /*坡道标识别到的时间戳,用于超时解除识别*/
    uint32 m_u32OutCarTime;     /*出库标识别到的时间戳,用于超时解除识别*/
    uint32 m_u32InCarTime;      /*入库标识别到的时间戳,用于超时解除识别*/
    uint32 m_u32RoundaboutTime; /*环岛标识别到的时间戳,用于超时解除识别*/
    uint32 m_u32SmallSTime;     /*小S 标识别到的时间戳,用于超时解除识别 */
    uint32 m_u32StraightTime;   /*直道标识别到的时间戳,用于超时解除识别*/

    uint8 m_u8StraightFlag;      /*直道标志位*/
    uint8 m_u8BendFlag;          /*弯道标志位*/
    uint8 m_u8ReadyFlag;         /*调整前瞻防抖标志位*/
    uint8 m_u8CrossingReadyFlag; /*十字防抖标志位*/
    uint8 m_u8CrossFlag;         /*十字标志位*/
    uint8 m_u8LRoundaboutFlag;   /*左环岛标志位*/
    uint8 m_u8RRoundaboutFlag;   /*右环岛标志位*/
    uint8 m_u8RampFlag;          /*坡道标志位*/
    uint8 m_u8RampReadyFlag;     /*坡道准备阶段*/
    uint8 m_u8SmallSFlag;        /*小S 标志位*/
    uint8 m_u8Pflag;             /*P字标志位*/
    uint8 m_u8LAllLostFlag;      /*左单边全丢标志位*/
    uint8 m_u8RAllLostFlag;      /*右单边全丢标志位*/

    uint8 m_u8ThreeRoadsFlag;  /*三岔口标志位 0:未检测到 1:检测到 2:左进 3:右进 */
    uint8 m_u8ThreeRoadsDir;   /*三叉路口左转右转标志位*/
    uint8 m_u8ThreeRoadsstart; /*三叉阶段标志位 0进入  1退出*/

    /*包括出库,出库完成入库*/

    uint8 m_u8CarBarnState; /*出库状态*/
    uint8 m_u8CarBarnDir;   /*出库入库方向,什么方向出库就什么方向入库(由拨码开关来决定)*/

    uint8 m_u8UltraWideFlag;     /*超宽标志位*/
    uint8 m_u8RunTurns;          /*运行圈数*/
    uint8 m_u8ZebraCrossingFlag; /*斑马线*/
    uint8 m_u8CarRunningState;   //车库状态

    uint8 m_u8StraightToBendFlag; /*直道有效行*/

    uint8 m_u8twoturn; /*当出现两个拐点时 为1 用于判断三叉和十字*/

} TRACK_TYPE_INFO;
typedef struct line_error_info_
{
    int16 m_i16StraightROW;     //直道前瞻
    int16 m_i16BendROW;         //弯道前瞻
    int16 m_i16StraightSpeed;   //直道速度
    int16 m_i16BendSpeed;       //弯道速度
    int16 m_i16RoundaboutROW_4; //入环阶段前瞻
    int16 m_i16RoundaboutROW_5; //环中阶段前瞻
    int16 m_i16RoundaboutROW_6; //出环岛阶段前瞻
    float m_f32LineAllError;
    float m_f32COLAllError;
    INT_POINT_INFO m_ErPointBuffer[20]; //选取7个点出来控制,此处引入前瞻控制
} LINE_ERROR_INFO;

void CenterlineGet(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border);
void draw_centerline(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border);
INT_POINT_INFO getOptimumColumn(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_stBorder, TRACK_TYPE_INFO *p_TrackType);
void Perspective_Change(TRACK_BORDER_INFO *p_Border);
void examine(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border);
void state_judgement(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType);
void test(uint8 (*image)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType);
uint8 Angle_judge(uint8 type, int16 bottomline, int16 middleline, int16 topline);
void err_Cal(TRACK_BORDER_INFO *p_Border);
float LeastSquareCalc_Curve(TRACK_BORDER_INFO *p_Border, int16 StartLine, int16 EndLine, int16 type);
float Linear_fitting(TRACK_BORDER_INFO *p_stBorder, int16 startline, int16 endline);
void regression(TRACK_BORDER_INFO *p_stBorder, int16 startline, int16 endline, uint8 type);
void straightaway_curve(TRACK_BORDER_INFO *p_Border, int16 type);
float process_curvity(float x1, float y1, float x2, float y2, float x3, float y3);
float Roadwidth_Cal(int16 line);
void Equidistant_Sample(TRACK_BORDER_INFO *p_Border, float dist);
void Judge_S(TRACK_BORDER_INFO *p_Border);

void getBorder(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border);
void GetCenter(TRACK_BORDER_INFO *p_Border);                                                        //计算中线
void GetTrackType(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType); //赛道类型转化
void Line_GetMonotonicity(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType);
void Line_GetThreeRoads(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_TrackType);
void GetLineError(TRACK_BORDER_INFO *p_Border, LINE_ERROR_INFO *p_LineError, TRACK_TYPE_INFO *p_TrackType);
void Out_Protect(uint8 (*image)[IMGW]);
#endif /* CODE_IMAGE_H_ */
