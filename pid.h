/*
 * struct.h
 *
 *  Created on: 2022年1月5日
 *      Author: qww
 */
#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "type.h"

typedef struct
{
    float Current_Error_left;   //当前误差
    float Last_Error_left;      //上一次误差
    float Previous_Error_left;  //上上次误差
    float Current_Error_right;  //当前误差
    float Last_Error_right;     //上一次误差
    float Previous_Error_right; //上上次误差
} Err_S;

typedef struct
{
    float Kp_left, Ki_left, Kd_left; //比例系数、积分、微分系数
    float Kp_right, Ki_right, Kd_right;
    float SetSpeed; //定义设定值
    float ActualSpeed_left, ActualSpeed_right, Last_ActualSpeed_left, Last_ActualSpeed_right;
    float voltage_left, voltage_right, last_voltage_left, last_voltage_right;
} PID_S;
typedef struct
{
    float Current_Error; //当前误差
    float Last_Error;    //上次误差
} Err_D;

typedef struct
{
    float Kp, Ki, Kd;  //比例系数、积分、微分系数
    float ActualAngel; //定义设定值
    float SetAngel;
} PID_D;

#endif
