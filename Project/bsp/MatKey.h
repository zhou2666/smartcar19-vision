#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"


extern uint8 Key_Value;
extern uint8 flag_KeyDel;

extern void KEY_Init(void);
extern void Key_ColOut(uint8_t x);
extern uint8 ScanKey(void);
extern void Get_Key(void);
#endif
