/*
 * circle_left.h
 *
 *  Created on: 2023年12月6日
 *      
 */

#ifndef CODE_CIRCLE_LEFT_H_
#define CODE_CIRCLE_LEFT_H_
#include "zf_common_headfile.h"


//*************************自定义变量********************************
extern struct CIRCLE circle_left;
//左圆环协方差
extern float cov_Leftcircle_L;//拐点上方的协方差
extern float cov_Leftcircle_R;//右线的协方差
extern float cov_Leftcircle_L_down;//拐点下方的协方差
extern float Circle_left_yaw;   

//extern uint8 start_circle;
extern uint8 yh_run;

//*************************自定义函数*******************************
extern void circle_left_find(void);//识别左圆环
extern void find_whitecnt_Left(void);//左拐点上方白色区域个数
extern void cov_Leftcircle(void);//左右协方差计算
extern void find_outcorner_Left(void);//寻找出环下拐点
extern void find_upcorner_Left(void);//寻找入环上拐点与出环上拐点
extern void find_downcorner_Left(void);//寻找左边下拐点
extern void circle_left_pro(void); //左圆环处理函数
extern void circle_left_in(void);//左圆环识别入环时刻
extern void find_middlecorner_Left(void);
#endif /* CODE_CIRCLE_LEFT_H_ */
