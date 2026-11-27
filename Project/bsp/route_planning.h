/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.

 ********************************************************************************************************************/

#ifndef _route_planning
#define _route_planning
   
   
uint8 Manhattan_distance(uint8 x, uint8 y);                     //曼哈顿测距  |x-x1|+|y-y1|
void distmin_Apcompare(void);                                   //找到距小车当前位置最近的AprilTag码  (a,b)为现在点坐标,N为AprilTag码的数量 
void Posture_correction(void);                                  //姿态矫正
void Send_corrcet_message(void);
void Wait_smart_stop(void);
void correct_display(void);
void Wait_smart_correct(void);
void correct_postion(void);
void Posture_back(void);
void back_postion(void);

void Send_Stop_corrcet_message(void);
void Send_clear_message(void);
void Send_clear_2_message(void);
void Send_rc_picture_message(void);
void Send_Start_message(void);
void yellow_check(uint8 x);
void Send_goon_message(void);
void Send_Bianli_message(void);
void Find_frameless_images(void);
void Send_GoBack_message(void);
void bianli (float x ,float y);
void huiku(void);
void game_begin(void);
void game_start(void);
void Line_following();
void Send_goon_Find_frameless_message(void);
void Send_find_yellow_message(void);
#endif