/*
 * Barrier_right.h
 *
 *  Created on: 2024Äê11ÔÂ11ÈÕ
 *  
 */

#ifndef CODE_BARRIER_RIGHT_H_
#define CODE_BARRIER_RIGHT_H_

#include "zf_common_headfile.h"

extern struct BARRIER barrier_right;
extern void barrier_right_find(void);
extern void vector_barrier_corner_right(void);

extern void monotone_decrease(uint8 start_col,uint8 end_col);
extern void monotone_increase(uint8 start_col,uint8 end_col) ;

extern float ka_barrier_right;
extern float kb_barrier_right;
extern double barrier_angle_right;
extern uint8 barrier_right_single;
extern uint8 barrier_right_count;
extern uint8 monotone_increase_flag;
extern uint8 monotone_decrease_flag;

#endif /* CODE_BARRIER_LEFT_H_ */
