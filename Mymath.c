/*
 * math.c
 *
 *  Created on: 2022年4月22日
 *      Author: qww
 */

/*开平方函数*/
float FSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x; // evil floating point bit level hacking
    // i = 0x5f3759df - (i >> 1);  // what the fuck?
    i = 0X5F3504F3 - (i >> 1); //精度更高
    x = *(float *)&i;
    x = x * (1.5f - (xhalf * x * x));
    return 1 / x;
}

/*指数函数*/
float FExp(float x)
{
    x = 1.0 + x / 4096;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    return x;
}
float Fabs(float x)
{
    if (x < 0)
        return -x;
    else
        return x;
}

int  myabs(int dat)
{
    if(dat>=0)  return dat;
    else        return -dat;
}