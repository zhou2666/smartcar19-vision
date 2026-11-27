/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 ********************************************************************************************************************/
#ifndef display_h
#define display_h


void key_gpio_init(void);       //按键初始化
uint8 Key_Scan(uint8 model);  	//按键扫描函数	
void five_direction_key();
void ips114_control(void);      //屏幕控制
void interface_select(uint8 color, uint8 left_x, uint8 right_x, uint8 high_y, uint8 low_y, uint8 jiemian, uint8 select); //界面选择（用于屏幕控制）
void display_map_data(uint8 flag);    //地图数据显示（用于屏幕控制）



#endif