/*
 * circle_right.h
 *
 *  Created on: 2023年12月6日
 *
 */

#ifndef CODE_CIRCLE_RIGHT_H_
#define CODE_CIRCLE_RIGHT_H_
//**********************头文件引用*************************************//
#include "zf_common_headfile.h"

#define LENGTH_WHITE_THRESHOLD 5

extern struct CIRCLE circle_right;

//*********************自定义变量*************************************//
extern float cov_Rightcircle_L;     //左线协方差
extern float cov_Rightcircle_R;     //拐点上方的协方差
extern float cov_Rightcircle_R_down;//拐点下方的协方差
extern float Circle_right_yaw;//进环时的陀螺仪值
struct CIRCLE
{
    uint8_t circle_findflag;//圆环模式标志位

    uint8_t circle_left_true;//圆环识别左边标志位
    uint8_t circle_middle_true;//圆环识别中间标志位
    uint8_t circle_right_true;//圆环识别右边标志位

    uint8_t circle_downcorner_row;//圆环下拐点行
    uint8_t circle_downcorner_col;//圆环下拐点列
    uint8_t circle_downcorner_ordinal;//下拐点在整条边线的列数
    uint8_t circle_downcorner_flag;//下拐点标志位

    uint8_t circle_middlecorner_row;//中拐点
    uint8_t circle_middlecorner_col;
    uint8_t circle_middlecorner_ordinal;
    uint8_t circle_middlecorner_flag;


    uint8_t circle_upcorner_row;//上拐点
    uint8_t circle_upcorner_col;
    uint8_t circle_upcorner_ordinal;
    uint8_t circle_upcorner_flag;

    uint8_t circle_outcorner_row;//出环拐点
    uint8_t circle_outcorner_col;
    uint8_t circle_outcorner_ordinal;
    uint8_t circle_out_flag;

    uint8_t bigcircle_flag;//大圆环标志位
    uint8_t smallcircle_flag;//小圆环标志位

    uint8_t Length_white;//拐点上方空白的长度
};
//*********************自定义函数*************************************//
extern void circle_right_find(void);
extern void circle_right_in(void);
extern void circle_right_pro(void);
extern void find_downcorner(void);
extern void find_midcorner(void);
extern void find_upcorner(void);
extern void find_outcorner(void);
extern void cov_Rightcircle(void);
extern void find_whitecnt(void);
extern void find_middlecorner(void);
#endif /* CODE_CIRCLE_RIGHT_H_ */
