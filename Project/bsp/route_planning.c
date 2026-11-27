/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"

/*********************************************************************************************************
小车寻找的图片的数组
*********************************************************************************************************/

//uint8 locate_now[2]={0, 0};             //小车当前位置
float locate_now[2]={0.0, 0.0};
extern uint8 num;
int time_a=0;
extern uint8 abv;
/*********************************************************************************************************
** Function name:           Manhattan_distance
** Descriptions:            曼哈顿测距   |x-x1|+|y-y1|
** input parameters:        uint8 x,uint8 y(需要测距的坐标)
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
uint8 Manhattan_distance(uint8 x, uint8 y)
{
  uint8 distance = 0;
  distance = abs(locate_now[0] - x);     //CarState.Block_X_Now,CarState.Block_Y_Now分别为小车目前的坐标
  return distance;
}



/*********************************************************************************************************
** Function name:           distmin_Apcompare
** Descriptions:            比较现在点与所有图片的距离，找到最小距离，这个用于识别地图的时候可以用
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
float x_speed=0;
float y_speed=0;
void distmin_Apcompare(void)
{
  static uint8 j = 1;
  if(j<=openart1.picture_num/2 )
  {
    need_grid[0] = (float)(temp_openart1[2*j]  - locate_now[0])-1;                //存储距小车最近的图片x轴
    need_grid[1] = (float)(temp_openart1[2*j+1]  - locate_now[1])-0.5;            //存储距小车最近的图片y轴      
//    printf("%f,%f\n",need_grid[0],need_grid[1]);
    locate_now[0] = temp_openart1[2*j] ;
    locate_now[1] = temp_openart1[2*j+1] ;
    j++;
  }
}


/*********************************************************************************************************
** Function name:           yellow_check
** Descriptions:            摄像头检测到黄线开始扔图片
** input parameters:        x  哪一边
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void yellow_check(uint8 x)  
{
  encoder_ctrl(1);
  switch(x)
  {
  case 1:
    {
      angle_PID_function(0);   
      gory_PID_function(angle_pid.output);  
      if(yellow_flag!=1)
      {
        outputL[0] = 0;
        outputL[1] = 0;
      }
      if(direction==0)  //小车可能停在长方形的左右方向，所以需要判断方向
        speed_PID_function(0.6,0.3); 
      else if(direction==1)
        speed_PID_function(-0.6,0.3); 
      break;
    }
  case 2:
    {
      angle_PID_function(0);
      gory_PID_function(angle_pid.output);  
      if(ABS(outputL[0])<2)
        speed_PID_function(0.975,-1);
      else
        speed_PID_function(1.7,-0.1);
      break;
    }
  case 3:
    {
      angle_PID_function(0);
      gory_PID_function(angle_pid.output);    
      if(ABS(outputL[1])<3)
        speed_PID_function(-2,-0.725);
      else
        speed_PID_function(-0.175,-0.825);
      break;
    }
  case 4:
    { 
      
      angle_PID_function(0);
      gory_PID_function(angle_pid.output);  
      if(ABS(outputL[0])<1)
        speed_PID_function(-0.675,1);
      else
        speed_PID_function(-1.3,0);
      break;
    }
  }
  
  speed_output[0] = speed_pid[0].output ;
  speed_output[1] = speed_pid[1].output  ;
  speed_output[2] = gory_pid.output;
  dispense_speed();
  moter_control();
  encoder_ctrl(2);
}
/*********************************************************************************************************
** Function name:           Posture_correction
** Descriptions:            姿态矫正
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Posture_correction(void)
{
  // distmin_Apcompare();            //曼哈顿测距  //小车获取坐标
  // Wait_smart_stop();     //等待小车停止
  Send_corrcet_message(); //给摄像头发送矫正标志位
  Wait_smart_correct();
}


/*********************************************************************************************************
** Function name:           Send_corrcet_message
** Descriptions:            发送矫正姿态信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_corrcet_message(void)
{
  openart_mode = Correct;//小车处于矫正姿态模式
  over(10);
  openart1_send();                //将小车模式发送给openart1
  //  openart2_send();
}

void Send_rcsl_picture_message(void)
{
  openart_mode = Correct;//小车处于矫正姿态模式
  over(10);
  openart2_send();                //将小车模式发送给openart1
  //  openart2_send();
}

///*********************************************************************************************************
//** Function name:          Send_correct_wd_message
//** Descriptions:            发送  识别 弯道上照片信息
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void Send_rcwd_message(void)
//{
//  openart_mode = correct_wd;//小车处于矫正姿态模式
//  over(200);
//  openart2_send();                //将小车模式发送给openart2
//}




/*********************************************************************************************************
** Function name:           Send_my_element_picture_message
** Descriptions:            发送 进入元素 矫正 照片信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_my_element_picture_message(void)
{
  openart_mode = element_picture;
  over(10);
  openart1_send();                //将小车模式发送给 openart1 ,开始 矫正 元素内部照片
}
/*********************************************************************************************************
** Function name:           Send_rc_picture_message
** Descriptions:            发送 识别 照片信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_rc_picture_message(void)
{
  openart_mode = element_picture;
  over(10);
  openart2_send();                //将小车模式发送给 openart2 ,开始 识别 照片
}




/*********************************************************************************************************
** Function name:           Send_my_element_letter_message
** Descriptions:            发送 元素外围 矫正 字母信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_my_element_letter_message(void)
{
  openart_mode = element_letter;
  over(10);
  openart1_send();                //将小车模式发送给 openart1 ,开始 矫正 元素外围字母
}
/*********************************************************************************************************
** Function name:           Send_rc_letter_message
** Descriptions:            发送 识别 字母信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_rc_letter_message(void)
{
  openart_mode = element_letter;
  over(10);
  openart2_send();                //将小车模式发送给 openart2 ,开始 识别 元素外围字母
}




/*********************************************************************************************************
** Function name:           Send_my_zebra_figure_message
** Descriptions:            发送 斑马线后 矫正 数字信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_my_zebra_figure_message(void)
{
  openart_mode = zebra_figure;
  over(10);
  openart1_send();                //将小车模式发送给 openart1 ,开始 矫正 斑马线后数字
}
/*********************************************************************************************************
** Function name:           Send_rc_figure_message
** Descriptions:            发送 识别 数字信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_rc_figure_message(void)
{
  openart_mode = zebra_figure;
  over(10);
  openart2_send();                //将小车模式发送给 openart2 ,开始 识别 斑马线后数字
}




/*********************************************************************************************************
** Function name:           Send_clear_message
** Descriptions:            发送清除openart所有状态信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_clear_message(void)
{
  openart_mode = clear;//小车处于矫正姿态模式
  over(200);
  openart1_send();                //将小车模式发送给openart1

}

void Send_clear_2_message(void)
{
  openart_mode = clear;//小车处于矫正姿态模式
  over(200);
  openart2_send();                //将小车模式发送给openart2
}

/*********************************************************************************************************
** Function name:          Send_first_search_message
** Descriptions:            发送清除openart所有状态信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_first_search_message(void)
{
  openart_mode = first_search;//小车处于矫正姿态模式
  over(200);
  openart1_send();                //将小车模式发送给openart1

}






/*********************************************************************************************************
** Function name:           Send_Start_message
** Descriptions:            发送开始信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Send_Start_message(void)
{
  openart_mode = Start;//小车处于矫正姿态模式
  over(10);
  openart1_send();                //将小车模式发送给openart1
  //  openart2_send();                //将小车模式发送给openart2
}
///*********************************************************************************************************
//** Function name:           Send_pickfinish_message
//** Descriptions:            发送元素内照片捡完信息
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void Send_pickfinish_message(void)
//{
//  openart_mode = pick_finish;//小车处于矫正姿态模式
//  over(10);
//  openart1_send();                //将小车模式发送给openart1
//  //  openart2_send();                //将小车模式发送给openart2
//}
///*********************************************************************************************************
//** Function name:           Send_GoBack_message
//** Descriptions:            发送回库信息
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void Send_GoBack_message(void)
//{
//  openart_mode = GoBack;//小车处于矫正姿态模式
//  Delay(10,ms);
//  openart1_send();     //将小车模式发送给openart2
//}

/*********************************************************************************************************
** Function name:           Send_Stop_corrcet_message
** Descriptions:            发送停止矫正信息
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
//void Send_Stop_corrcet_message(void)
//{
//  openart_mode = Stop_corrcet;//小车处于矫正姿态模式
//  over(10);
//  openart1_send();                //将小车模式发送给openart1
//  //  openart2_send();                //将小车模式发送给openart2
//  
//}


/*********************************************************************************************************
** Function name:           Wait_smart_stop
** Descriptions:            等待小车停止
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Wait_smart_stop(void)
{
  smartcar_state = 1;  
  
  while(smartcar_state == 1);          //等待小车停止
}




/*********************************************************************************************************
** Function name:           Wait_smart_correct
** Descriptions:            等待小车姿态矫正
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void Wait_smart_correct(void)
{ 
  smartcar_state = 2;  
  while(smartcar_state == 2);          //等待小车矫正 
}

/*********************************************************************************************************
** Function name:           Find_frameless_images
** Descriptions:            遍历场地，直到看到场地黄色边框
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
//void Find_frameless_images()
//{
//  encoder_ctrl(1);
//  angle_PID_function(0);
//  gory_PID_function(angle_pid.output);
//  if(direction==0)    //车向右走
//  {
//    if(ABS(outputL[1])<run_distance)
//    {
//      check_yellow_commencement=0;
//      if( outputL[0]>0.06)//防止车上下飘，给反方向速度
//        y_speed1=-0.4;
//      else if( outputL[0]<-0.04)
//        y_speed1=0.6;
//      else 
//        y_speed1=0;
//      speed_PID_function(0.5,y_speed1);
//    }
//    else
//    {
//      no_picture = 1;
//      smartcar_state=0;
//      direction++;
//      bianli_cs++;
//      direction%=2;
//      outputL[0] = 0;
//      outputL[1] = 0;
//      check_yellow_commencement=1;
//    }
//  }
//  else if(direction==1)//车向左走
//  {
//    if( ABS(outputL[1])<run_distance)
//    {
//      check_yellow_commencement=0;
//      if( outputL[0]>0.06)
//        y_speed1=-0.4;
//      else if( outputL[0]<-0.04)
//        y_speed1=0.6;
//      else 
//        y_speed1=0;
//      speed_PID_function(-0.5,y_speed1);
//    }
//    else
//    {
//      // speed_PID_function(0,y_speed1);
//      no_picture =1;
//      check_yellow_commencement=1;
//      smartcar_state=0;
//      direction++;
//      bianli_cs++;
//      direction%=2;
//      outputL[0] = 0;
//      outputL[1] = 0;
//    }
//  }
//  
//  speed_output[0] = speed_pid[0].output ;
//  speed_output[1] = speed_pid[1].output  ;
//  speed_output[2] = gory_pid.output;
//  dispense_speed();
//  moter_control();
//  encoder_ctrl(2);
//  check=1;
//}
//
//
///*********************************************************************************************************
//** Function name:           Find_digit_images
//** Descriptions:            遍历场地，直到看到场地黄色边框
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void Find_digit_images()
//{
//  encoder_ctrl(1);
//  angle_PID_function(Initial_yaw);
//  gory_PID_function(angle_pid.output);
//  if(direction==0)    //车向右走
//  {
//    if(ABS(outputL[1])<run_distance)
//    {
//      if( outputL[0]>0.06)//防止车上下飘，给反方向速度
//        y_speed1=-0.4;
//      else if( outputL[0]<-0.04)
//        y_speed1=0.6;
//      else 
//        y_speed1=0;
//      speed_PID_function(1.25,y_speed1);
//    }
//  }
//  speed_output[0] = speed_pid[0].output;
//  speed_output[1] = speed_pid[1].output;
//  speed_output[2] = gory_pid.output;
//  dispense_speed();
//  moter_control();
//  encoder_ctrl(2);
//  
//}

/*********************************************************************************************************
** Function name:           Line_following
** Descriptions:            循迹全场
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 

void Line_following()
{
  encoder_ctrl(1);
  turn_pid_function();
  gory_PID_function(turn_pid.output);
  
  
  
  if(element_flag==1||sprint==1||sprint==2||(dl1b_distance_mm<800&&dl1b_refresh_freq>=10))
  {
    if(element_flag==1)
    {
      kk=11;
      speed_PID_function(0,0.85);
      
      pick_complete=0;
      yc1=0;
      yc2=3;
      yc3=0;
      yb_flag=0;
      yc_forward_y=0;
      output_ycforward[0]=0;
    }
    if(sprint==1)
    {
      kk=12;
      speed_PID_function(0,0);
    }
    if(sprint>=2)
    {
      kk=13;
      speed_PID_function(0,2);
    }
    //  speed_PID_function(0,1);
    
    if(dl1b_distance_mm<800&&dl1b_refresh_freq>=10)
    {
      speed_PID_function(0,0.2);
      kk=14;
    }
  }
  
//  if(ramp_lowspeed==1)
//  {
//    speed_PID_function(0,0);
//  }
//  if(tofa_flag==1)
//  {
//    speed_PID_function(0,0);
//  }
  else if(barrier_left_single==1||barrier_right_single==1)
  {
    speed_PID_function(0,0.9);
    
  }
  else if(speed_falg1==1)
  {
    speed_PID_function(0,1.7);
  }
  else
  {
    
    if(ABS(Mid_add)<=5)
    {
      kk=1;
      speed_PID_function(0,1.2);
    }
  
    else if(ABS(Mid_add)>5&&ABS(Mid_add)<=10)
    {
      kk=2;
      speed_PID_function(0,1.1);
    }

    else if(ABS(Mid_add)>10)
    {
      kk=3;
      speed_PID_function(0,0.9);
    }
      
  }
    
  speed_output[0] = speed_pid[0].output;
  speed_output[1] = speed_pid[1].output;
  speed_output[2] = gory_pid.output;
  dispense_speed();
  moter_control();
  encoder_ctrl(2);
}



///*********************************************************************************************************
//** Function name:           huiku
//** Descriptions:            回库
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void huiku(void)
//{
//  encoder_ctrl(1);
//  angle_PID_function(0);   
//  gory_PID_function(angle_pid.output);   
//  speed_PID_function(0,-0.8);
//  speed_output[0] = 1*speed_pid[0].output;
//  speed_output[1] = 1*speed_pid[1].output;
//  speed_output[2] = 1*gory_pid.output;  
//  dispense_speed();
//  moter_control();
//  encoder_ctrl(2);
//}

/*********************************************************************************************************
** Function name:           game_begin
** Descriptions:            小车的整个流程
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
extern int abd;
int fzflag=0;
int qjflag=0;
void game_begin(void)
{
  openart_mode = 0;
  need_grid[0] = 0;//X
  need_grid[1] = 0;//Y
  need_angle = 0;
  
  
  
  //  while(start_flag!=1); //等待摄像头初始化完成
  //    while(gpio_get(B21)!=0);//等待按键按下
  //  need_grid[0] = 0;//X
  //  need_grid[1] = 2.5;//Y
  //  smartcar_state = 1;
  //  while(smartcar_state == 1);//走出库
  need_grid[0] = 0;//X
  need_grid[1] = 0;//Y
  need_angle = 0;
  over(240);
  
  
  //  PWM3_ctrl(147,1);
  
  
  //  test_ts();
  
  //  //////*************************遍历*******************************************
  //  bcc=0;
  //  direction_compare=1;
  //  outputL[0] = 0;
  //  outputL[1] = 0;
  
//  while(prep!=1)
//  {
//    five_direction_key();
//    tft180_show_uint(0, 0, prep, 3);
//    tft180_show_uint(0, 96, key_num, 3);
//    
//  }
//  while(prep!=1)
//  {
//    five_direction_key_test();
//    tft180_show_uint(0, 0, prep, 3);
//    if(key_num==1)
//      tft180_show_string(0, 24, "L2");
//    else if(key_num==2)
//      tft180_show_string(0, 24, "L3");
//    else if(key_num==3)
//      tft180_show_string(0, 24, "R2");
//    else if(key_num==4)
//      tft180_show_string(0, 24, "R3");
//    
//    
//    tft180_show_string(0, 36, "R_yaw");
////    tft180_show_uint(60, 36, Reduction_yaw, 3);
//    tft180_show_float(60, 36, Reduction_yaw, 3, 3);
////    deal_image();     
//  }
//  while(prep==1)            
//  {
    smartcar_state=0;
    while(smartcar_state==0);
//  }
  while(1)
  {
//    sq();
//  smartcar_state=0;
//  //          over(240);
//  ts();
//  //        over(240);
//  
//  //          smartcar_state=2;
//  yb();
  }



//  while(bianli_over!=1)            //看到拐点结束遍历
//  {
//    for(int m = 0;m <7;m++)
//    {
//      if(m!=6)
//      {
//        r_list_B[m] =carmer_yaw;
//      }
//      else if (m==6)
//      {
//        r_list_B[m] =-carmer_yaw;
//      }
//      
//      
//      if(m == 0)
//      {
//        need_grid[0] = x_list_B[m];
//        need_grid[1] = y_list_B[m];
//        need_angle = r_list_B[m];
//      }
//      else
//      {
//        need_grid[0] = x_list_B[m] - x_list_B[m - 1]; 
//        need_grid[1] = y_list_B[m] - y_list_B[m - 1]; 
//        need_angle = r_list_B[m];     
//      }
//      smartcar_state = 5;
//      while(smartcar_state == 5);
//    }
//    
//    
//    
//    //    over(240);
//    //    sq();
//    //    over(300);
//    //    ts();
//    //    over(240);
//    //    servo_back();
//    //    over(10);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    //    over(300);
//    
//    
//    
//    //    if(no_picture == 0)//&&fzflag!=1&&qjflag!=1
//    //    {
//    //      Send_corrcet_message();
//    //      if(smartcar_state!=2)
//    //      {
//    //        smartcar_state=4;
//    //        while(smartcar_state==4);//等待看到黄线或者看到图片
//    //      }
//    //      outputL[0] = 0;
//    //    }
//    //    
//    //    if(smartcar_state==0&&no_picture==1)//没有看到图片
//    //    {  
//    //      Send_find_yellow_message();
//    //      outputL[1] = 0;
//    //      check_yellow_commencement=0;
//    //      smartcar_state=5;
//    //      while(smartcar_state==5);
//    //      if(bianli_over!=1)
//    //        while(ABS(Reduction_yaw)>1.5);
//    //      run_distance=2.5;
//    //      no_picture = 0;
//    //    }
//    
//    
//    //    if(smartcar_state==2)          //看到图片
//    //    {
//    //      while((smartcar_state==2)&&(bianli_over!=1));       //等待矫正完成
//    //      if(bianli_over!=1)
//    //      {
//    //        while(wait_identify_flag!=1);
//    //        wait_identify_flag=0;
//    //        over(10);
//    //        //        sq2();
//    //        //        if(wk_y>=0.08)
//    //        //        {
//    //        //          smartcar_state=8; 
//    //        //        }
//    //        //        while(smartcar_state==8);
//    //        over(300);    
//    //        //        servo_back2();
//    //        over(10);
//    //        
//    //      }
//    //      else
//    //        smartcar_state=0;
//    //    }
//    
//    //    
//    ////    else if(fzflag==1)
//    ////    {
//    ////      over(300);
//    ////      over(300);
//    ////      smartcar_state=12;
//    ////      while(smartcar_state==12);
//    ////      over(200);
//    ////      gpio_set(C12,0);
//    ////      fzflag=0;
//    ////      qjflag=1;
//    ////    }
//    ////    else if(qjflag==1)
//    ////    {
//    ////      over(300);
//    ////      over(300);
//    ////      over(300);
//    ////      smartcar_state=13;
//    ////      while(smartcar_state==13);
//    ////      qjflag=0;
//    ////    }
//    //    abv=0;
//  }
  //  direction_compare=0;
  //  
  //  
  //  //////////*************************扔图片***********************************************
  //  //  Send_goon_message();
  //  //  smartcar_state=0;
  //  //  PWM2_ctrl(80,1);
  //  //  PWM1_ctrl(80,1);
  //  //  smartcar_state=0;
  //  //  smartcar_state=0;
  //  //  over(90);
  //  //  steering_degree(5);
  //  //  outputL[0] = 0;
  //  //  outputL[1] = 0;
  //  //  for(int i=0;i<4;i++)
  //  //  {
  //  //    over(300);
  //  //    outputL[0] = 0;
  //  //    outputL[1] = 0;
  //  //    smartcar_state=3;
  //  //    if(i==0)
  //  //    {
  //  //      while((ABS(outputL[0])<0.6)&&(ABS(outputL[1])<1));//第一次扔图片走固定距离
  //  //      smartcar_state=0;
  //  //      Throw_area=2;
  //  //    }
  //  //    while(smartcar_state == 3);
  //  //    if(i!=3)
  //  //      dcs_work(4-i);
  //  //    else
  //  //    {
  //  //      gpio_set(C26,1);//最后一次扔图片
  //  //      over(390);
  //  //      gpio_set(C26,0);
  //  //    }
  //  //    abd=0;
  //  //    yellow_flag=0;
  //  //  }
  //  //////////////*************************回库***********************************************
  //  smartcar_state=0;
  //  Send_GoBack_message();
  //  outputL[0] = 0;
  //  outputL[1] = 0;
  //  smartcar_state=0;
  //  while(huiku_flag!=1);
  //  if(hk_distance[0]==2)
  //    need_grid[0]=(hk_distance[1]*12)/(20.0*proportion);//摄像头传过来离边界的距离。由于传过来的是像素点，公式变换一下
  //  else if(hk_distance[0]==0)
  //    need_grid[0]=-(hk_distance[1]*12)/(20.0*proportion);
  //  need_grid[0]=need_grid[0];
  //  need_grid[1]=0;
  //  smartcar_state=7;
  //  while(smartcar_state==7);//等待小车距离边线一定距离
  //  smartcar_state=9;
  //  while(smartcar_state==9);//等待小车倒车回库
}



void element_one(void)
{
  for(mm = 0;mm <2;mm++)  //右转到正对着圆环
  {
    Send_clear_message();//清除art1所有状态
    if(mm==0)//元素第一步：转身至正对元素
    {
      //              r_list_B[mm] =carmer_yaw;
      if(element_state==1)//左环
      {
        need_grid[0] = x_list_L[mm];
        need_grid[1] = y_list_L[mm];
        need_angle = r_list_L[mm];
        Initial_yaw=r_list_L[mm];
      }
      else if(element_state==2)//右环
      {
        need_grid[0] = x_list_R[mm];
        need_grid[1] = y_list_R[mm];
        need_angle = r_list_R[mm];
        Initial_yaw=r_list_R[mm];
      }
      else if(element_state==3)//十字（p环）
      {  //判断p环趋势
        if(k_left>-0.2&&k_right>0.7)         //为左p环  向左转
        {
          cross_switch=1;
          
        }
        else if(k_left<-0.7&&k_right<0.2)   //为右p环  向右转
        {
          cross_switch=2;
          
        }
        else                              
        {
          if(ka5_cross>0&&ka6_cross>0)
          {
            cross_switch=1;              //为左p环  向左转
          }
          else if(ka5_cross<0&&ka6_cross<0)
          {
            cross_switch=2;              //为右p环  向右转
          }
          else 
          {
           element_state=0;
           element_flag=0;
           smartcar_state=4;
           
          }
        }
        
        if(cross_switch==1)//为左p环  向左转
        {
          need_grid[0] = x_list_LC[mm];
          need_grid[1] = y_list_LC[mm];
          need_angle = r_list_LC[mm];
          Initial_yaw=r_list_LC[mm];
        }
        else if(cross_switch==2)//为右p环  向右转
        {
          need_grid[0] = x_list_RC[mm];
          need_grid[1] = y_list_RC[mm];
          need_angle = r_list_RC[mm];
          Initial_yaw=r_list_RC[mm];
        }
      }
      
      Send_clear_message();
      smartcar_state = 5;
      while(ABS(Reduction_yaw-need_angle)>1);//等待旋转至车身角度与实际角度小于1度
      while(smartcar_state == 5);//等待完成5状态
      //
      //              if(ABS(Reduction_yaw - need_angle)<5)
      //              {
      //                if(wk_ys>=10)
      //                {
      //                  over(200);
      //                  over(200);
      //                  Send_my_element_picture_message(); //第二次向openart发送看到元素信息，进入03状态
      //                  over(200);
      //                  over(200);
      //                  Send_my_element_picture_message(); //第二次向openart发送看到元素信息，进入03状态
      //                }
      //                
      //              }
    }
    else//元素第二步：向前走一步
    {
      //              r_list_B[mm] =carmer_yaw;
      if(element_state==1)
      {
        need_grid[0] = x_list_L[mm] - x_list_L[mm - 1]; 
        need_grid[1] = y_list_L[mm] - y_list_L[mm - 1];
        need_angle = r_list_L[mm];
        
        yc1++;
        Initial_yaw=r_list_L[mm];
      }
      else if(element_state==2)
      {
        need_grid[0] = x_list_R[mm] - x_list_R[mm - 1]; 
        need_grid[1] = y_list_R[mm] - y_list_R[mm - 1];
        need_angle = r_list_R[mm];
        
        yc1++;
        Initial_yaw=r_list_R[mm];
      }
      else if(element_state==3)
      {
        if(cross_switch==1)
        {
          need_grid[0] = x_list_LC[mm] - x_list_LC[mm-1];
          need_grid[1] = y_list_LC[mm] - y_list_LC[mm-1];
          need_angle = r_list_LC[mm];
          
          yc1++;
          Initial_yaw=r_list_LC[mm];
        }
        else if(cross_switch==2)
        {
          need_grid[0] = x_list_RC[mm] - x_list_RC[mm-1];
          need_grid[1] = y_list_RC[mm] - y_list_RC[mm-1];
          need_angle = r_list_RC[mm];
          
          yc1++;
          Initial_yaw=r_list_RC[mm];
        }
      }
      Send_clear_message();
      
      while(yc_forward_y<foward_distanse)
      {
        smartcar_state = 1;
      }
//      yc_forward_y=0;
//      output_ycforward[0];
//      smartcar_state = 5;
//      while(smartcar_state == 5);
      over(200);
      if(ABS(Reduction_yaw - need_angle)<5)
      {
        Send_my_element_picture_message(); //第二次向openart发送看到元素信息，进入03状态
        
        over(200);
        Send_my_element_picture_message(); //第二次向openart发送看到元素信息，进入03状态
      }
      
    }
    //            pick_complete=1;
//    sc_monitor1++;
//    if(sc_monitor1==10000)
//    {
//      smartcar_state==10;
//    }

  }
}


void element_place(void)
{
  wk_pick_y=0;
  output_pick[0]=0;
  spin_finish=0;
  lz_flag=0;
  element_flag=0;
  Send_clear_message();
  yb_flag=0;
  if(yc2>=3&&yc2<=8)//跑5个放置点
  {
    if(no_picture_flag!=1)//不是空环（元素内有照片）
    {
      if(element_state==1)
      {
        need_grid[0] = x_list_L[yc2];
        need_grid[1] = y_list_L[yc2];
        //              need_grid[0] = 0;
        //              need_grid[1] = 0;
        need_angle = r_list_L[yc2];
        
        Initial_yaw=r_list_L[yc2];
      }
      else if(element_state==2)
      {
        need_grid[0] = x_list_R[yc2];
        need_grid[1] = y_list_R[yc2];
        //              need_grid[0] = 0;
        //              need_grid[1] = 0;
        need_angle = r_list_R[yc2];
        
        Initial_yaw=r_list_R[yc2];
      }
      else if(element_state==3)
      {
        if(cross_switch==1)
        {
          need_grid[0] = x_list_LC[yc2];
          need_grid[1] = y_list_LC[yc2];
          //                need_grid[0] = 0;
          //                need_grid[1] = 0;
          need_angle = r_list_LC[yc2];
          Initial_yaw=r_list_LC[yc2];
        }
        else if(cross_switch==2)
        {
          need_grid[0] = x_list_RC[yc2];
          need_grid[1] = y_list_RC[yc2];
          //                need_grid[0] = 0;
          //                need_grid[1] = 0;
          need_angle = r_list_RC[yc2];
          Initial_yaw=r_list_RC[yc2];
        }
      }
      //            PWM5_ctrl(136,1);//关门
      ys_angle=0;
      smartcar_state = 5;
      while(smartcar_state == 5)
      {
        Send_clear_message();
      }
      while(ABS(Reduction_yaw-need_angle)>1);
      if(yc2==8)
      {
        while(ABS(Reduction_yaw-need_angle)>5);
      }
      
      //            over(200);
      //            over(200);
      //            if(yc2<8&&ABS(Reduction_yaw-need_angle)<3)
      //              Send_go_position_message();
      //            over(200);
      //            over(200);
      //            if(yc2<8&&ABS(Reduction_yaw-need_angle)<3)
      //              Send_go_position_message();
      //            over(200);
      //            over(200);
      //            over(200);
      //            over(200);
      e_p_flag=1;//进入遍历放置区
      smartcar_state = 7;
      while(smartcar_state == 7)
      {
        if(yc2<8&&ABS(Reduction_yaw-need_angle)<3)
        {
          if(wk_ys>=0.45)//遍历过0.45米
            Send_my_element_letter_message();//开art1找数字
        }
        if(yc2==3)
          Send_my_element_letter_message();
      }
      
      yc2++;//放置区计数器++
      if(yc2==8&&(element_state==1||element_state==2))
      { //元素结束开启单边巡线出环
        Single_line=1;
        single_continue=1;           ///////////////////////////////
        if(element_state==1)
           single_flag=1;
        else if(element_state==2)
           single_flag=2;
      }
      if(yc2>=9)//跑完元素周围放置照片区域，停车，所有标志位置0
      {
        smartcar_state=4;
        element_state=0;
      }
    }
    else if(no_picture_flag==1)//空环（元素内没照片）
    {
//      kkkk=10086;
      if(element_state==1)
      {
        need_grid[0] = 1.2;
//        need_grid[1] = 3.5;
        need_grid[1] = 2;

        //              need_grid[0] = 0;
        //              need_grid[1] = 0;
        need_angle = 0;
        Initial_yaw=0;
//        kkkk=1;
      }
      else if(element_state==2)
      {
        need_grid[0] = -1.2;
//        need_grid[1] = 3.5;
        need_grid[1] = 2;

        //              need_grid[0] = 0;
        //              need_grid[1] = 0;
        need_angle = 0;
        
        Initial_yaw=0;
//        kkkk=2;
      }
      else if(element_state==3)
      {
        if(cross_switch==1)
        {
          need_grid[0] = 2.2;
          need_grid[1] = 2.5;
          //                need_grid[0] = 0;
          //                need_grid[1] = 0;
          need_angle = -90;
          Initial_yaw=-90;
//          kkkk=3;
        }
        else if(cross_switch==2)
        {
          need_grid[0] = -2;
          need_grid[1] = 2.5;
          //                need_grid[0] = 0;
          //                need_grid[1] = 0;
          need_angle = 90;
          Initial_yaw= 90;
//          kkkk=4;
        }
      }
      //            PWM5_ctrl(136,1);//关门
      ys_angle=0;
      smartcar_state = 5;
//      kkkk=5;
      while(smartcar_state == 5);
      yc2=9;

      if(yc2>=9)//跑完元素周围放置照片区域，停车，所有标志位置0
      {
        if(element_state==1||element_state==2)
        {  //元素结束开启单边巡线出环
          Single_line=1;             
          single_continue=1;
          if(element_state==1) 
            single_flag=1;  
          else if(element_state==2)
            single_flag=2;
        }
        smartcar_state=4;
           //置零标志位
        element_state=0;
        no_picture_flag=0;
        pick_complete=0;
      }
    }
  } 
}

void element_zebra(void)//斑马线操作
{
//  sprint=1;
  Send_clear_message();
  if(yc3>=0&&yc3<=3)
  {
    if(yc3>=0&&yc3<3)
    {
      if(zebra_commutation_flag!=1)
      {
        need_grid[0] = 0;
        need_grid[1] = 0;
        if(zebra_switch==0)
          need_angle = r_list_B[yc3];
        else if(zebra_switch==1)
          need_angle = r_list_B_L[yc3];
      }
      else if(zebra_commutation_flag==1)
      {
        need_grid[0] = 0;
        need_grid[1] = 0;//1.0
        if(zebra_switch==0)
          need_angle = -r_list_B[yc3];
        else if(zebra_switch==1)
          need_angle = -r_list_B_L[yc3];
      }
    }
    else if(yc3==3)
    {
      need_grid[0] = x_list_B[yc3];
      need_grid[1] = y_list_B[yc3];
      need_angle = 0;
    }
    
    
    ys_angle=0;
    smartcar_state = 5;
    //          while(ABS(Reduction_yaw-need_angle)>1)
    //          {
    //            smartcar_state = 5;
    //          }
    while(smartcar_state == 5)
    {
      Send_clear_message();
    }
    if(yc3==3)
    {
      while(ABS(Reduction_yaw-need_angle)>5);
    }
    over(600);
    over(200);
    sprint=2;
    element_flag=0;
    //          over(200);
    //          over(200);
    //          if(yc3<4&&ABS(Reduction_yaw-need_angle)<3)
    //            Send_go_position_message();
    //          over(200);
    //          over(200);
    //          if(yc3<4&&ABS(Reduction_yaw-need_angle)<3)
    //            Send_go_position_message();
    //          over(200);
    //          over(200);
    //          over(200);
    //          over(200);
    //          Send_clear_message();
    smartcar_state = 7;
    while(smartcar_state == 7)
    {
      if(yc3>0&&yc3<3&&ABS(Reduction_yaw-need_angle)<2)
      {
        if(ABS(wk_ys)>=0.35)
          Send_my_zebra_figure_message();
      }
      if(yc3==0)
      {
        if(ABS(wk_ys)>=0.2)
          Send_my_zebra_figure_message();
      }
    }
    yc3++;
  }
  else if(yc3>=4)
  {
    
    smartcar_state=4;
    element_state=0;
    zebra_findflag=0; //最后一次识别斑马线标志位置零
    zebra_commutation_flag=0;
  }
}

void correct_recognize_common()
{
  //  kkk=1111;
  while((smartcar_state==2))       //等待矫正完成
  {
    //    kk=11;    
  }
  gpio_high(B15);//开补光灯
  wait_identify_picture_flag=0;//清空识别完成图片标志位
  wait_identify_letter_flag=0;//清空识别完成数字标志位
  //矫正完art关机
  Send_clear_message();//清除art1所有状态
//  gpio_high(B15);

  //  Send_clear_2_message();
  if(fake_picture==0)//真的有照片，不是误判
  {
    while(wait_identify_picture_flag==0&&wait_identify_letter_flag==0)//等待识别完成
    {
      if(art2_begin_count==0)
      {
        if(element_state==0)//识别散落图片情况下
        {
          //      if(wd_flag==0)
          Send_rcsl_picture_message();//向art2发送识别散落照片信息
          //      else if(wd_flag==1)
          //        Send_rcwd_message();
        }
        if(pick_complete==0&&element_state!=0&&element_state!=4)//在元素识别照片情况下
        {
          Send_rc_picture_message();//向art2发送识别识别元素内照片信息
          //      kk=1;
        }
        
        if(pick_complete==1)//识别元素外围字母
        {
          Send_rc_letter_message();//向art2发送识别识别元素外围字母信息
          //      kk=2;
          back_find=0;
        }
        
        if(element_state==4)//斑马线后识别数字
        {
          Send_rc_figure_message();//向art2发送识别识别斑马线后数字信息
        }
        art2_begin_count++;//开机计数器++避免多次开机识别
      }
      //    kk=10;

    }
    //  sc_monitor1=0;
    //  sc_monitor2=0;
    //  kk=888;
    
    //识别完成双art关机
    Send_clear_2_message();
    Send_clear_message();
    art2_begin_count=0;//art2开机计数器清0
    //      kk=0;
    gpio_low(B15);//关补光灯

    wait_identify_picture_flag=0;//照片识别完成标志位清零
    over(10);
    over(300);
    goon_flag=0;//状态跳转至0，给goon_flag==0避免车跑了
    if((element_state==0||pick_complete!=1)&&element_state!=4)//散落照片或元素内（除斑马线）机械臂操作
    {
      if(element_state==0)//散落照片
      {
        //            Send_clear_2_message();
        species_scattered(species_picture);//分类照片
        steering_degree_in(0,big_species);//转收纳盒
        over(240);
        sq();//下爪
        goon_flag=0;
        if(/*wk_y>=0.08&&*/element_state==0)
        {
          smartcar_state=8;//抓完照片回原位
        }
        
        ts();//抬机械臂
        over(240);
        sc=0;
        servo_back();//机械臂回待机位置
        over(600);
        smartcar_state=12;//状态12保持车不动
        while(smartcar_state==12);
        goon_flag=1;//状态跳转至0，给goon_flag==1让车继续跑循迹
        over(120);
        
      }
      else if(element_state!=0)//元素内集中照片
      {
        //      Send_clear_2_message();
        species_concentrated(species_picture);
        steering_degree_in(small_count,0);
        //            over(240);
        sq();
        smartcar_state=0;
        //          over(240);
        ts();
        //            over(240);
        //            Send_rc_picture_message();
        //            over(120);
        //            Send_rc_picture_message();
        if(carry_count!=0)//不是3的奇数张照片时
          lz_flag=1;//连抓标志位至1，主函数跳转到连抓状态
        //          smartcar_state=2;
        yb();//机械臂预备抓取位置
        over(390);
        sc=0;
        //          over(240);
        //          over(240);
        //          Send_go_position_message();
      }
      
    }
    if((pick_complete==1||element_state==4)&&wait_identify_letter_flag==1)//元素外围放置区，斑马线放置区
    {
      Send_clear_2_message();
      skip_pick=0;
      place_picture(species_alphabet);
      if(classify_error==0)
      {
        smartcar_state = 6;
        while(smartcar_state == 6); 
        over(300);
        over(300);
        over(300);
        over(300);
        over(300);
        
        fz();
        wk_ys=0;
        over(240);
      }
      servo_back();
      over(390);
      
      sc=0;
      wait_identify_letter_flag=0;
    }
  }
  else if(fake_picture==1)//没照片，误判
  {
//    kkk=1000;
    
    smartcar_state=8;
    servo_back();
    gpio_low(B15);
    servo_back();
    over(600);
    smartcar_state=12;
    while(smartcar_state==12);
    goon_flag=1;
//    while(smartcar_state==8);
  }

}

void correct_recognize_lz(void)
{
  if(carry_count!=3)//不是3的整数倍张照片
  {
    //  kkk=2222;
    //          Send_clear_message();
    //        fifo_openart1_rx_buffer=0;
    while(wait_identify_picture_flag!=1&&pick_complete!=1);//等待识别完成  
    if(pick_complete!=1)
    {
      wait_identify_picture_flag=0;
      species_concentrated(species_picture);
      steering_degree_in(small_count,0);
      sq();
      smartcar_state=0;
      //          over(240);
      ts();
      //        over(240);
      
      //          smartcar_state=2;
      yb();//机械臂抓完一张直接回预备抓取位置，马上准备下爪抓第二张照片
      over(390);
      sc=0;
    }
    else//如果art2判断元素内照片捡完了
    {
      lz_flag=0;
      carry_count=0;
      Send_my_element_picture_message();//开art1确认是否真的没照片了
    }
  }
  else //照片计到3的整数倍张
  {
    lz_flag=0;//连抓标志位清0
    carry_count=0;//捡照片计数器清零
    Send_my_element_picture_message();;//向art1发送矫正元素照片信息，进行矫正
    //（进行这一次矫正是为了避免由于机械臂连抓产生的惯性导致车发生位移而抓不到照片的情况）

  }
  
}


/*********************************************************************************************************
** Function name:           game_start
** Descriptions:            小车的整个流程
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 

int n=0;
int x_flag=0;
void game_start(void)
{
  openart_mode = 0;
  need_grid[0] = 0;//X
  need_grid[1] = 0;//Y
  need_angle = 0;
  smartcar_state = 0;
  
  
//  while(start_flag!=1); //等待openart1摄像头初始化完成
  //   while(start_flag_2!=1); //等待openart2摄像头初始化完成
  while(prep!=1)//发车按键没被按下
  {
    five_direction_key();
    tft180_show_uint(0, 0, prep, 3);

    tft180_show_uint(0, 24, key_num, 3);
    
    tft180_show_string(0, 36, "R_yaw");
//    tft180_show_uint(60, 36, Reduction_yaw, 3);
    tft180_show_float(60, 36, Reduction_yaw, 3, 3);
    
    tft180_show_float(0, 50, dl1b_distance_mm, 4, 3);

    
    pre_image();//显示总钻风图像 
  }
  while(prep==1)//发车按键被按下        
  {
    if(clear_tft==1) //清一次屏
    {
     tft180_clear();
     clear_tft=0;
    }
    
//      tft180_show_float(0, 36, (float)(openart1.error_array[1]), 2, 1);
//      tft180_show_float(40, 36, encoder[0], 2, 1);
//      tft180_show_uint(80, 36, compulsion_monitor, 2);        
//
//
//      tft180_show_float(0, 56, (float)(openart1.error_array[3]), 2, 1);
//      tft180_show_float(40, 56, encoder[1], 2, 1);
//      tft180_show_uint(80, 56, dd, 2);        
//      
//      tft180_show_float(0, 70, jz_inc_err[1], 2, 1);
//      tft180_show_float(40, 70, encoder[2], 2, 1);
//
//      
//      tft180_show_float(0, 90, jz_inc_err[0], 2, 1);
//      tft180_show_float(40, 90, encoder[3], 2, 1);

//    if(element_state==0&&Single_line==0&&sprint!=2&&art_begin==1/*&&nono_pic_falg!=1*/)
    if(element_state==0&&Single_line==0&&sprint!=2&&art_begin==1)//不在元素内、不是总钻风单边寻线状态、不是冲刺状态、斑马线发车后记步达到一定距离
    {
      Send_corrcet_message();//打开openart1
//      Send_first_search_message();
    }
    
    if(smartcar_state!=2)//不是矫正状态（art没有看到照片）
    {
//      element_state=3;
//      carmer_yaw=0;
//      pick_complete=1;
//      yc1=1;
//      cross_switch=2;
      
      if(tofa_flag!=1)//tof没有判断到坡道
      {
        if(element_state==1||element_state==2||element_state==3) //当总钻风看到元素（左环、右环、十字）
        {
          //        pick_complete=0;
          if(pick_complete!=1&&spin_finish!=1)//未收到捡完元素照片标志位&&完成旋转至正对着元素
          {
            Send_my_element_picture_message(); //第一次向openart发送03标志位，openart所有状态清0
          }//同时经常发生openart向rt1064发送0XF6，rt1064接收不到的情况，导致pick_complete一直为0，无法进行下一步操作，
          //因此当pick_complete!=1时,下位机向openart发送Send_my_element_letter_message(),openart便向下位机发送0XF6，
          //确保能令pick_complete=1;
          
          //        element_flag=1;
          //        over(120);
          if(yc1==0&&pick_complete!=1)//元素内照片没捡完
          {
            element_one();
          }
          
          if(pick_complete==1/*&&jz_success!=1*/)//元素内所有照片捡完以后
          {
            element_place();
          }
        }
        else if(element_state==4)//总钻风看到斑马线
        {
          element_zebra();
        }
        else if(element_state==0)//总钻风没有看到元素
        {
          if(sprint==2)
            Send_clear_message();
          need_angle=0;
//          fake_picture=0;
          smartcar_state=4;//循迹
          while(smartcar_state==4&&element_state==0)//一直向前等待看到元素或者散落的图片
          {
            if(art_begin_count==0&&art_begin==1)
            {
              Send_corrcet_message();
              art_begin_count=1;
            }
            if(element_state==0&&Single_line==0&&art_02_flag==1)//单边寻线出环后开启art1
            {
              Send_corrcet_message();
              art_02_flag=0;
              //      Send_first_search_message();
            }
            if(tofa_flag==1)
              smartcar_state=0;
          }
      
        }
      }
      else if(tofa_flag==1)
      {
          Send_clear_message();
          smartcar_state = 3;
          while(smartcar_state == 3);
      }
    }
    
    if(smartcar_state==2||lz_flag==1)          //看到图片 进入矫正环
    {
//      kk=111;
//      if(element_state!=0)
//        while(ABS(Reduction_yaw-need_angle)>1);
      if(lz_flag==0)//单抓状态（每张照片都要矫正）
      {
        if(element_state==0)
          yb();
        correct_recognize_common();//单抓
        kk=15151;
      }
      else if(lz_flag==1)//连抓状态（三张照片矫正一次）
      {
        correct_recognize_lz();//连抓
        kk=15152;
      }
      //      Initial_yaw=Reduction_yaw;

    }
//    if(smartcar_state==13)
//    {
//      while(smartcar_state==13);
//      while(ABS(Reduction_yaw-need_angle)>1);
//      over(300);
//      Send_corrcet_message();
//    }
  }
  //===============================================  
  
}



