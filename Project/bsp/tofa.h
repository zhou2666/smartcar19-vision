#ifndef __TOFA_H
#define __TOFA_H
#include "zf_common_headfile.h"

extern uint8  pit_ms_flag;
extern uint16 pit_ms_count;
extern uint16 dl1b_refresh_freq ;

void tofa_process(void);

#endif 
 