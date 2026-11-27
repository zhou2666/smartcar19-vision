/*
 * situation.h
 *
 *  Created on: 2023年12月6日

 *  
 */

#ifndef CODE_SITUATION_H_
#define CODE_SITUATION_H_

#include "zf_common_headfile.h"

//中线获取与元素处理
extern float Mid_add;            //中线误差
extern float Mid_add_last; 
extern float Mid_add_sum; 

extern void get_mid_bias(void);  //获取中线偏差

#endif /* CODE_SITUATION_H_ */
