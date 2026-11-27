/*
 * Barrier_left.h
 *
 *  Created on: 2024Äê6ÔÂ7ÈÕ
 *  
 */

#ifndef CODE_BARRIER_LEFT_H_
#define CODE_BARRIER_LEFT_H_

#include "zf_common_headfile.h"
struct BARRIER
{
   uint8 find_flag;

   uint8 corner_row;
   uint8 corner_col;
   uint8 corner_flag;

};

extern struct BARRIER barrier_left;
extern void barrier_left_find(void);
extern void vector_barrier_corner_left(void);
extern float ka_barrier_left;
extern float kb_barrier_left;

extern double barrier_angle;
extern uint8 barrier_left_single;
extern uint8 barrier_left_count;

#endif /* CODE_BARRIER_LEFT_H_ */
