
#ifndef __ANT_H
#define __ANT_H
#include "zf_common_headfile.h"
#include "define.h"
#include "bsp.h"
int count(int *f,int a,int lenth);
double sum(double *f,int lenth);
int con_bet(double *p,int lenth);
void sort(double *p,int lenth);
int get_index(double *f,int lenth,double v);
double ABS_ant(double n);
void ant_start( uint8  *point_data,int len_point_data);
#endif 
 