/*
 * cross.h
 *
 *  Created on: 2023年12月6日
 *  
 */

#ifndef CODE_CROSS_H_
#define CODE_CROSS_H_
#include "zf_common_headfile.h"

struct CROSS
{
    uint8 cross_findflag;//十字模式标志位
    uint8 cross_mode;//十字模式中的状态

    uint8 cross_L_downfindflag;                  //左下拐角标志位
    uint8 cross_R_downfindflag;                  //右下拐角标志位

    uint8 Left_downcorner_row;                   //左下拐角坐标
    uint8 Left_downcorner_col;
    uint8 Left_downcorner_ordinal;

    uint8 Right_downcorner_row;                  //右下拐角坐标
    uint8 Right_downcorner_col;
    uint8 Right_downcorner_ordinal;

    uint8 cross_L_upcorner_flag;                 //左上拐角坐标
    uint8 cross_L_upcorner_row;
    uint8 cross_L_upcorner_col;

    uint8 cross_R_upcorner_flag;                //右上拐角坐标
    uint8 cross_R_upcorner_row;
    uint8 cross_R_upcorner_col;

    uint8 cross_lost;//十字拐点状态，  1--->均存在；2----->右拐点不存在；3----->左拐点不存在
};

extern struct LEFT L_Up_line[100];//十字上方边界线
extern struct RIGHT R_Up_line[100];
extern uint8 L_Up_cnt;//十字上方边线的数量
extern uint8 R_Up_cnt;

extern int left_downchange_col;
extern int right_downchange_col;
extern int Road_width_7;
extern int Road_width_70_flag;
extern uint8 cross_mode_flag;
extern int cross_preview_flag;
extern int cross_preview_count;

extern struct CROSS cross;
extern void cross_find(void);
extern void cross_pro(void);

extern void least_sqaure_leftline(double *ka,double *kb);
extern void least_sqaure_rightline(double *ka,double *kb);
extern void least_sqaure_method(const double *x, const double *y, int n, double *k, double *b);
extern void least_sqaure_left(double *ka,double *kb);
extern void least_sqaure_right(double *ka,double *kb);

extern void leftdown_change(int *col,int row);
extern void rightdown_change(int *col,int row);

extern void find_L_up_point(uint8 k1,uint8 k2,uint8 j1);
extern void find_R_up_point(uint8 k1,uint8 k2,uint8 j1);

extern void circle_R_fitting(uint8_t start_row, uint8_t end_row, float ka, float kb);
extern void circle_L_fitting(uint8_t start_row, uint8_t end_row, float ka, float kb);

extern void findline_lefthand_adaptive_up(uint8_t* image, int block_size, int clip_value, int x, int y, uint8_t* num);//左手迷宫巡线(十字左上拐点)
extern void findline_righthand_adaptive_up(uint8_t* image, int block_size, int clip_value, int x, int y, uint8_t* num);//右手迷宫巡线(十字右上拐点)

extern float ka_cross;//十字左边补线斜率
extern float kb_cross;//十字右边补线斜率
extern float ka2_cross;//十字左边补线斜率
extern float kb2_cross;//十字右边补线斜率
//最小二乘法
extern double ka3_cross;//十字右边补线斜率
extern double kb3_cross;
extern double ka4_cross;//十字左边补线斜率
extern double kb4_cross;
extern double ka5_cross;//
extern double kb5_cross;
extern double ka6_cross;//
extern double kb6_cross;
#endif /* CODE_CROSS_H_ */
