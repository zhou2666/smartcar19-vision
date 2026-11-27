#ifndef _BSP_H_
#define _BSP_H_
/*******************************************************************************************************************/
/***Includes***/
//{
#include "init.h"       //初始化
#include "pid.h"        //PID参数计算
#include "gyroscope.h"  //陀螺仪相关
#include "moter.h"      //电机驱动
#include "carry.h"      //搬运相关
#include "massage.h"    //无线串口相关(调试模式使用)
#include "fmath.h"      //数学
#include "filter.h"     //滤波
#include "openart_uart.h"//摄像头
#include "display.h"     //OLED显示
#include "flash_param.h" //Flash模块
#include "route_planning.h"//路径规划
#include "ant.h"       //蚁群算法
#include "tofa.h"       //蚁群算法
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "bluetooth.h"
#include "MatKey.h"
//}
/*******************************************************************************************************************/
/***Basic Function Defines***/
//{
#define LIMIT(x, min, max)  (((x) < (min)) ? (min) : ( ((x) > (max)) ? (max) : (x)))        //限幅
#define RANGE(x, max, min)  (((x) <= (min)) ? (min) : ( ((x) >= (max)) ? (max) : (x)))      //确保x的范围为min~max
#define minf(x, y)   (((x) < (y)) ? (x) : (y))                                               //取小值
#define maxf(x, y)   (((x) > (y)) ? (x) : (y))                                               //取大值
#define INT(x)  ((int)(x))   //((int32)(((x)>0?1:-1)*ceil(ABS((x)))))                       //强制整型
#define SWAP(x, y)   do{(x) ^= (y); (y) ^= (x); (x) ^= (y);} while(0)                       //交换 x, y 的值
#define ARR_SIZE(a) ( sizeof( (a) ) / sizeof( ((a)[0]) ) )                                  //返回数组元素的个数
#define OFFSET(type, member)    ((uint32)(&(((type *)0)->member)))                          //获取结构体某成员偏移
//}
/*******************************************************************************************************************/
//void Game_Begin(void);
/*******************************************************************************************************************/
/***Constant Define***/
//{
#ifndef PI
#define PI  (3.14159265f)
#endif
#ifndef CP
#define CP  (0.01745329f)
#endif
#ifndef ONE_PI
#define ONE_PI  (3.14159265f)
#endif 
#ifndef TWO_PI
#define TWO_PI  (2.000f * 3.14159265f)
#endif
#ifndef Exp
#define Exp (2.71828183f)
#endif
#define dT 0.005f
#define ms 0
#define us 1
//}
/*******************************************************************************************************************/
/***Eggs***/
//{
#define mian main   // You know what I mean :D
//}
/*******************************************************************************************************************/
#endif