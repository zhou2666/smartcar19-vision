/*
 * My_math.h
 *
 *  Created on: 2023年12月6日
 *   
 */

#ifndef CODE_MY_MATH_H_
#define CODE_MY_MATH_H_
#include "zf_common_headfile.h"

//#define BUZZER_ON()   gpio_init(P33_10, GPO, 1, GPO_PUSH_PULL)
//#define BUZZER_OFF()  gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL)

extern void find_xielv(int start_row, int start_col, int end_row, int end_col, float* ka, float* kb);
extern void find_area(int x1, int y1, int x2, int y2, int x3, int y3, float* S);
extern float abs_float(float num);
extern void straightlong_find(void);
extern uint8 judge_white(uint8 x);
extern uint8 judge_black(uint8 x);
//长直线判断
extern float S_L;//左边线斜率
extern float S_R;//右边线斜率
extern float S_M;//中线斜率
extern uint8 straightlong_L_findflag;//左长直道标志位
extern uint8 straightlong_R_findflag;//右长直道标志位
extern uint8 white_judge(int start_ROW, int end_ROW, int start_COL, int end_COL,int tmp);
extern uint8 block_judge(int start_ROW, int end_ROW, int start_COL, int end_COL,int tmp);


#endif /* CODE_MY_MATH_H_ */
