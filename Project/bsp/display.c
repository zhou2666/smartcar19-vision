/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"
/*********************************************************************************************************
  变量定义
*********************************************************************************************************/
uint8 mode = 0;         //按键按下的变量
uint8 last[2] = {0};
uint8 save_frame[4] = {0};
uint8 picture_num_old = 0;
extern float **picture_coordinate;       // 排序后坐标
#define Key_up     gpio_get_level(B14)
#define Key_down   gpio_get_level(B17)
#define Key_left   gpio_get_level(B23)
#define Key_right  gpio_get_level(B16)
#define Key_mid    gpio_get_level(B21)

#define Key_up_PRES     1
#define Key_down_PRES   2
#define Key_left_PRES   3
#define Key_right_PRES  4
#define Key_mid_PRES    5

#define True    1
#define False   0

/*********************************************************************************************************
  枚举类型
*********************************************************************************************************/
enum {
    White_1, White_2, White_3, Yellow, Yellow_1, Yellow_2, Yellow_3, Yellow_4, Yellow_5, Yellow_6, Yellow_7, Yellow_8, Yellow_9, Yellow_10, Green, Black,Gray
} backgroud; 

enum {
    NONE_OPT, PID_OPT, Map_Data_OPT, Speed_OPT, END_OPT
} interface_option; 

enum {
    NONE_PID_X, PID_1, PID_2, PID_3, END_PID_X
} PID_option_X;

enum {
    NONE_PID_Y, Kp_Speed_OPT, Ki_Speed_OPT, Kd_Speed_OPT, Kp_Location_OPT, Ki_Location_OPT, Kd_Location_OPT, END_PID_Y
} PID_option_Y; 

enum {
    NONE_SET, Main_SET, PID_SET, Map_Data_SET, Speed_SET, END_SET
} setting; 


float PID[6][3] = {{2000,2000,6000},{3.72,4.5,250},{2000,2000,0},
                   {1.7,2,0.015},{0,0,0},{20,20,0.3}};

/*********************************************************************************************************
** Function name:           key_gpio_init
** Descriptions:            按键初始化
** input parameters:        uint8 x,uint8 y(需要测距的坐标)
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void key_gpio_init(void)
{
    gpio_init(B14, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(B17, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(B23, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(B16, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(B21, GPI, 1, GPIO_PIN_CONFIG);
    
}

/*********************************************************************************************************
** Function name:           Key_Scan
** Descriptions:            按键扫描函数
** input parameters:        model=1支持连按，model=0不支持连按
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
uint8 Key_Scan(uint8 model)
{	 
	static uint8 key_state=1;//按键按松开标志
	if(model)key_state=1;  //支持连按		  
	if(key_state&&(Key_up==0||Key_down==0||Key_left==0||Key_right==0||Key_mid==0))
	{
		system_delay_ms(5);//去抖动 
		key_state=0;
		if(Key_up==0)return Key_up_PRES;
		else if(Key_down==0)return Key_down_PRES;
		else if(Key_left==0)return Key_left_PRES;
		else if(Key_right==0)return Key_right_PRES;
                else if(Key_mid==0)return Key_mid_PRES;
	}else if(Key_up==1&&Key_down==1&&Key_left==1&&Key_right==1&&Key_mid==1)key_state=1; 	    
 	return 0;// 无按键按下
}

void five_direction_key()
{
  key_num=Key_Scan(1);
  if(key_num==Key_up_PRES)//左避坡道
  {
    ramp_switch=-0.8;
  }
  else if(key_num==Key_down_PRES)
  {
    
  }
  else if(key_num==Key_left_PRES)//左斑马线
  {
    zebra_switch=1;
    zebra_speed=-0.66;
  }
  else if(key_num==Key_right_PRES)
  {
    
  }
  else if(key_num==Key_mid_PRES)
  {
    prep=1;
  }
}

void five_direction_key_test()
{
  key_num=Key_Scan(1);
  if(key_num==Key_up_PRES)//左环2号位
  {
    yc_angle_test=1;
  }
  else if(key_num==Key_down_PRES)//左环3号位
  {
    yc_angle_test=2;
  }
  else if(key_num==Key_left_PRES)//右环2号位
  {
    yc_angle_test=3;
  }
  else if(key_num==Key_right_PRES)//右环3号位
  {
    yc_angle_test=4;
  }
  else if(key_num==Key_mid_PRES)
  {
    prep=1;
  }
}

//
///*********************************************************************************************************
//** Function name:           ips114_control
//** Descriptions:            屏幕控制
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void ips114_control(void)
//{
//    static uint8 mode_interface = 0;
//    static uint8 PID_interface_X = 0;
//    static uint8 PID_interface_Y = 0;
//    static uint8 PID_interface_Plus_Minus = 0; //加减
//    static uint8 set_point = 0; //中键按下（确定）
//    static uint8 a = 0;//判断中键是否二次按下（确定）
//    //static uint8 PID_interface_Minus = 0;
//    static uint8 key = 0;
//    key = Key_Scan(0);
//    if(key)
//    {
//      switch(key)
//      {
//          case Key_up_PRES:
//              if(setting == NONE_SET){mode_interface = True; if(interface_option > NONE_OPT){interface_option--; if(interface_option==NONE_OPT)interface_option=Speed_OPT;}}
//              else if(setting==PID_SET){if(PID_interface_Plus_Minus == False){set_point = False; PID_interface_Y = True; PID_interface_X = False;if(PID_option_Y>NONE_PID_Y){PID_option_Y--;if(PID_option_Y==NONE_PID_Y)PID_option_Y=Kd_Location_OPT;}}
//                                        else {a = 0; PID[PID_option_Y-1][PID_option_X-1] += 1; set_point = True;}}
//              else {mode = 1;mode_interface = False;PID_interface_Y = False;}            
//              break;
//          case Key_down_PRES:
//              if(setting == NONE_SET){mode_interface = True;if(interface_option<END_OPT){interface_option++;if(interface_option==END_OPT)interface_option=PID_OPT;}}
//              else if(setting==PID_SET){if(PID_interface_Plus_Minus == False){set_point = False; PID_interface_Y = True; PID_interface_X = False;if(PID_option_Y<END_PID_Y){PID_option_Y++;if(PID_option_Y==END_PID_Y)PID_option_Y=Kp_Speed_OPT;}}
//                                        else {a = 0; PID[PID_option_Y-1][PID_option_X-1] -= 1; set_point = True;}}
//              else {mode = 2;mode_interface = False;PID_interface_Y = False;}  
//              break;
//          case Key_left_PRES:
//              if(setting == PID_SET){set_point = False;PID_interface_X = True;PID_interface_Y = False;if(PID_option_X>NONE_PID_X){PID_option_X--;if(PID_option_X==NONE_PID_X)PID_option_X=PID_3;}}
//              else {mode = 3;mode_interface = False;PID_interface_X = False;}
//              break;
//          case Key_right_PRES:
//              if(setting == PID_SET){set_point = False;PID_interface_X = True;PID_interface_Y = False;if(PID_option_X<END_PID_X){PID_option_X++;if(PID_option_X==END_PID_X)PID_option_X=PID_1;}}
//              else {mode = 4;mode_interface = False;PID_interface_X = False;}
//              break;
//          case Key_mid_PRES:
//              if(setting == PID_SET)
//                {
//                  a++;
//                  if(set_point == False)
//                  {
//                    if(a==1)
//                    {
//                      PID_interface_Plus_Minus = True; 
//                      mode_interface = False; PID_interface_X = False; 
//                      PID_interface_Y = False;
//                    }
//                                                                 
//                    else if(a==2)
//                    {
//                      setting=NONE_SET;
//                      mode_interface = True;
//                      a=0;
//                      PID_interface_Y = False;
//                      PID_interface_X = False,PID_interface_Plus_Minus = False;
//                    }
//                  }
//                                          
//                  else if (set_point == True) 
//                  {
//                    a=0;
//                    PID_interface_Plus_Minus = False;
//                    PID_interface_X  = True;
//                  }
//                }
//              else if(setting == Map_Data_SET){a++;if(a==2){a=0;setting=NONE_SET;mode_interface = True;PID_interface_Y = False;PID_interface_X = False,PID_interface_Plus_Minus = False;}}
//              else if(setting == Speed_SET){a++;if(a==2){a=0;setting=NONE_SET;mode_interface = True;PID_interface_Y = False;PID_interface_X = False,PID_interface_Plus_Minus = False;}}
//              else setting = Main_SET; 
//              break;
//      }
//    }
//    if(setting==NONE_SET)
//    {
//        if(interface_option==NONE_OPT)
//        {
//            ips114_showstr(44,1,"Interface to Select");
//            ips114_showstr(108,3,"PID");
//            ips114_showstr(88,4,"Map_Data");
//            ips114_showstr(100,5,"Speed");
//        }
//        if(mode_interface == True)
//        {
//            if(interface_option == PID_OPT)             interface_select(White_1, 107, 133, 48, 63, 0, 0);
//            else if(interface_option == Map_Data_OPT)  interface_select(White_2, 87, 153, 64, 79, 0, 0);
//            else if(interface_option == Speed_OPT)     interface_select(White_3, 99, 141, 80, 95, 0, 0);
//        }
//    }
//    else if(setting==PID_SET)
//    {
//        if(PID_interface_Y == True)
//        {
//            if(PID_option_Y==Kp_Speed_OPT){interface_select(Yellow_1, 136, 183, 32, 47, 1, 0); last[0] = 32; last[1] = 47;}
//            else if(PID_option_Y == Ki_Speed_OPT)    {interface_select(Yellow_2, 136, 183, 48, 63, 1, 0);  last[0] = 48; last[1] = 63;}
//            else if(PID_option_Y == Kd_Speed_OPT)    {interface_select(Yellow_3, 136, 183, 64, 79, 1, 0);  last[0] = 64; last[1] = 79;}
//            else if(PID_option_Y == Kp_Location_OPT) {interface_select(Yellow_4, 136, 183, 80, 95, 1, 0);  last[0] = 80; last[1] = 95;}
//            else if(PID_option_Y == Ki_Location_OPT) {interface_select(Yellow_5, 136, 183, 96, 111, 1, 0); last[0] = 96; last[1] = 111;}
//            else if(PID_option_Y == Kd_Location_OPT) {interface_select(Yellow_6, 136, 183, 112, 127, 1, 0);last[0] = 112;last[1] = 127;}
//            PID_option_X = PID_2;
//        }  
//        else if(PID_interface_X == True)
//        {
//            if(PID_option_X == PID_1)interface_select(Yellow_7, 80, 128, last[0], last[1], 1, 0);
//            else if(PID_option_X == PID_2)interface_select(Yellow_8, 136, 183, last[0], last[1], 1, 0);
//            else if(PID_option_X == PID_3)interface_select(Yellow_9, 191, 239, last[0], last[1], 1, 0);
//        }
//        else if((PID_interface_Plus_Minus == True)&&(PID_interface_Y == False)&&(PID_interface_X == False))
//        {
//            interface_select(Yellow_10, 0, 0, 0, 0, 1, 1);
//        }
//    }
//    else if(setting == Main_SET)
//    {
//        if((interface_option == PID_OPT)&&(PID_interface_Y == False)&&(PID_interface_X == False)&&(PID_interface_Plus_Minus == False))
//        {
//            interface_select(Yellow, 0, 0, 0, 0, 1, 0);
//            setting = PID_SET;
//        }
//        else if(interface_option == Map_Data_OPT)
//        {
//            if(backgroud != Green){ips114_clear(GREEN);backgroud = Green;}
//            display_map_data(1);
//            setting = Map_Data_SET;
//        }
//        else if(interface_option == Speed_OPT)
//        {
//            if(backgroud != Gray){ips114_clear(GRAY);backgroud = Gray;}
//            ips114_showstr(60,0,"Speed Interface");
//            ips114_showstr(20,4,"Speed:");
//            setting = Speed_SET;
//        }
//    }
////    if(Key_dial==0){static int flash_cnt = 0;
////            if (++flash_cnt % 100 == 0) {
////                flash_param_write();
////                flash_param_load();
////            }}
//}

/*********************************************************************************************************
** Function name:           interface_select
** Descriptions:            界面选择方框显示
** input parameters:        color(方框颜色),left_x(方框左值),right_x(方框右值),high_y(方框上值),low_y(方框下值),
                            jiemian---jiemian=0 主界面,jiemian=1 PID界面,
                            select---select=0 选择状态,select=1 选中状态(PID界面更改数值使用到)
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
//void interface_select(uint8 color, uint8 left_x, uint8 right_x, uint8 low_y, uint8 high_y, uint8 jiemian, uint8 select)
//{
//    if(jiemian == 0){
//        if(backgroud != color){ips114_clear(WHITE); backgroud = color;}
//        ips114_showstr(44,  1, "Interface to Select");
//        ips114_showstr(108, 3, "PID");
//        ips114_showstr(88,  4, "Map_Data");
//        ips114_showstr(100, 5, "Speed");
//    }
//    else if(jiemian == 1){
//        if(backgroud != color){ips114_clear(YELLOW); backgroud = color;}
//        ips114_showstr(68,0,"PID Interface");
//        ips114_showstr(0, 2,"Kp_Speed:");
//        ips114_showstr(0, 3,"Ki_Speed:");
//        ips114_showstr(0, 4,"Kd_Speed:");
//        ips114_showstr(0, 5,"Kp_Place:");
//        ips114_showstr(0, 6,"Ki_Place:");
//        ips114_showstr(0, 7,"Kd_Place:");
//        ips114_showfloat(80,  2, PID[0][0], 4, 0);
//        ips114_showfloat(136, 2, PID[0][1], 4, 0);
//        ips114_showfloat(192, 2, PID[0][2], 4, 0);
//        ips114_showfloat(80,  3, PID[1][0], 3, 2);
//        ips114_showfloat(136, 3, PID[1][1], 3, 2);
//        ips114_showfloat(192, 3, PID[1][2], 3, 2);
//        ips114_showfloat(80,  4, PID[2][0], 4, 0);
//        ips114_showfloat(136, 4, PID[2][1], 4, 0);
//        ips114_showfloat(192, 4, PID[2][2], 4, 0);
//        ips114_showfloat(80,  5, PID[3][0], 1, 3);
//        ips114_showfloat(136, 5, PID[3][1], 1, 3);
//        ips114_showfloat(192, 5, PID[3][2], 1, 3);
//        ips114_showfloat(80,  6, PID[4][0], 2, 2);
//        ips114_showfloat(136, 6, PID[4][1], 2, 2);
//        ips114_showfloat(192, 6, PID[4][2], 2, 2);
//        ips114_showfloat(80,  7, PID[5][0], 2, 2);
//        ips114_showfloat(136, 7, PID[5][1], 2, 2);
//        ips114_showfloat(192, 7, PID[5][2], 2, 2);
//    }   
//    if(select == 0){
//        for(int m=low_y;  m<=high_y;  m++) ips114_drawpoint(left_x,m,  PURPLE);
//        for(int m=low_y;  m<=high_y;  m++) ips114_drawpoint(right_x,m, PURPLE);
//        for(int m=left_x; m<=right_x; m++) ips114_drawpoint(m,high_y,  PURPLE);
//        for(int m=left_x; m<=right_x; m++) ips114_drawpoint(m,low_y,   PURPLE);
//        save_frame[0]=left_x; save_frame[1]=right_x; save_frame[2]=low_y; save_frame[3]=high_y;
//    }
//    else if(select == 1){
//        for(int m=save_frame[2]; m<=save_frame[3]; m++)ips114_drawpoint(save_frame[0],m,BLACK);
//        for(int m=save_frame[2]; m<=save_frame[3]; m++)ips114_drawpoint(save_frame[1],m,BLACK);
//        for(int m=save_frame[0]; m<=save_frame[1]; m++)ips114_drawpoint(m,save_frame[3],BLACK);
//        for(int m=save_frame[0]; m<=save_frame[1]; m++)ips114_drawpoint(m,save_frame[2],BLACK);
//    }
//}

/*********************************************************************************************************
** Function name:           display_map_data
** Descriptions:            地图数据显示
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
//void display_map_data(uint8 flag)
//{
//    if(flag == 1){
//      openart1_send();
//     systick_delay_ms(100); 
//      if(picture_num_old != (openart1.picture_num/2)) ips114_clear(WHITE);
//      picture_num_old = openart1.picture_num/2;  
//      ips114_showuint8(0, 0, openart1.picture_num/2);
//    }
//    else if(flag == 2){
//      ips114_clear(WHITE);
//    }
//    for(int i=0; i<(openart1.picture_num/2); i++){
//          if(i<7){
//            ips114_showuint8(0, i+1, i+1);
//            ips114_showuint8(30, i+1, openart1.picture_array[2*i]);
//            ips114_showuint8(60, i+1, openart1.picture_array[2*i+1]);
//          }
//          else{
//            ips114_showuint8(120, i-6, i+1);
//            ips114_showuint8(150, i-6, openart1.picture_array[2*i]);
//            ips114_showuint8(180, i-6, openart1.picture_array[2*i+1]);
//          }
//    }
//} 
//   