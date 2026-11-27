#ifndef _ZEBRA_H_
#define _ZEBRA_H_

#include "zf_common_headfile.h"

extern uint8_t point_change ;//斑马线符合标志位
extern uint8 point_change_white_L ;//白色斑马线个数
extern uint8 point_change_black_L;//黑色斑马线个数
extern uint8 change_count;

void zebra_crossing();
#endif
