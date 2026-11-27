/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "define.h"  
#include "bsp.h"
#include "stdio.h"

volatile int stering_counter = 0;
extern volatile bool stering_start;
extern int time_a;
volatile bool steering_flag = 0;
int carry_time=0;
//extern uint8 openart1_rx_buffer; 
//extern uint8 openart2_rx_buffer;
extern uint8 no_picture;
extern float pwm4;

void CSI_IRQHandler(void)
{
  CSI_DriverIRQHandler();     //调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
  __DSB();                    //数据同步隔离
}
int iii=5;
void PIT_IRQHandler(void)
{
  if(pit_flag_get(PIT_CH0))
  {
    pit_flag_clear(PIT_CH0);
    //陀螺仪读取
    gyroscope_read();
    //获取参考陀螺仪数值
    icmGetValues_static();
    // printf("%f,%f,%f,%f,%f,%f\r\n",icm_data_static.acc_x,icm_data_static.acc_y,icm_data_static.acc_z,icm_data_static.gyro_x,icm_data_static.gyro_y,icm_data_static.gyro_z);
    //从陀螺仪得到欧拉角
    icmAHRSupdate(&icm_data,&Q_info,&gyrocope_err);
    Euler(&eulerAngle,&Q_info);   
    //从参考（假）陀螺仪得到欧拉角
    icmAHRSupdate(&icm_data_static,&Q_info_static,&gyrocope_err_static);
    Euler(&eulerAngle_static,&Q_info_static);  
    //数据处理
    if(ABS(eulerAngle.final_yaw - eulerAngle.final_yaw_last) >= 2.0 * ABS(eulerAngle_static.final_yaw - eulerAngle_static.final_yaw_last))
    {
      Reduction_yaw += (eulerAngle.final_yaw - eulerAngle.final_yaw_last)- (eulerAngle_static.final_yaw - eulerAngle_static.final_yaw_last);
    }
    //printf("%f\n ",Reduction_yaw);
    //gpio_set(B9,0);
  }
  
  if(pit_flag_get(PIT_CH1))
  {
    pit_flag_clear(PIT_CH1);
    angle_function++;
    flag_pid++;          //PID计数标志位
    static int32 tim_ms;
    tim_ms++; 
    
    if(flag_pid >= flag_pid_n)              
    {                       // flag_pid_n PID矫正调节时间
      flag_pid = 0;
      pid_fucntion_flag = 1; //PID动作标志位
    }
    if(angle_function >= 1)
    {
      angle_function = 0;  //角度PID动作标志位
      angle_function_act = 1;
    }
    
    if(smartcar_state == 1)                        
    {
      yc_forward_y=output_ycforward[0];
      encoder_ctrl(1);
      angle_PID_function(need_angle);
      gory_PID_function(angle_pid.output);
      
      if(+ yc_forward_y<foward_distanse)
        speed_PID_function(0,0.70);       //(0.35,0.3)
      else
        speed_PID_function(0,0);       //(0.35,0.3)
      speed_output[0] = speed_pid[0].output;
      speed_output[1] = speed_pid[1].output;
      speed_output[2] = gory_pid.output;
      dispense_speed();
      moter_control();
      encoder_ctrl(2);
    }
    //***********************************************************************
    else if(smartcar_state == 2 )//矫正
    {
//      tft180_show_float(0, 36, (float)(openart1.error_array[1]), 2, 1);
////      tft180_show_float(40, 36, encoder[0], 2, 1);
//      tft180_show_uint(80, 36, compulsion_monitor, 2);        
////
////
//      tft180_show_float(0, 56, (float)(openart1.error_array[3]), 2, 1);
////      tft180_show_float(40, 56, encoder[1], 2, 1);
//      tft180_show_uint(80, 56, dd, 2);        
//      
//      tft180_show_float(0, 70, jz_inc_err[1], 2, 1);
//      tft180_show_float(40, 70, encoder[2], 2, 1);
//
//      
//      tft180_show_float(0, 90, jz_inc_err[0], 2, 1);
//      tft180_show_float(40, 90, encoder[3], 2, 1);
      
//      if(lz_flag==1)
//      {
//        sc_monitor1++;
//        if(sc_monitor1==800)
//        {
////          Send_clear_message();
////          Send_clear_2_message();
//          smartcar_state=0;
//          pick_complete=1;
//        }
//      }
      if(flag_pid_error!=ture_)      //误差接收完毕标志位
      {
//        if(pick_complete==1)
//        {
//          k24=0.28;//（y轴矫正到位，x未到位）系数
//          k25=0.19;
//          k26=0.18;
//          
//          k34=0.140;//（x轴矫正到位，y未到位）系数
//          k35=0.090;
//          k36=0.165;
//        }
        
        angle_PID_function(Initial_yaw);
        gory_PID_function(angle_pid.output);
        speed_output[0] = 0;
        speed_output[1] = 0;
        speed_output[2] = gory_pid.output;
        
        dispense_speed();  //PWM参数传递  
        moter_control();
      }
      if(flag_pid_error==ture_)
      {
        if(1)
        {
          jz_y=outputL[0];
          jz_x=outputL[1];
          sc_y=output_sc[0];
          sc_x=output_sc[1];

//          sc++;
          if(ABS(Mid_add)<10)//直道看到照片刹车
          {
            sc_flag=1;
            if(sc_y<=zd_sc)
            {
              sc_y=zd_sc;
              sc_finish=1;
            }
            if(sc_y>zd_sc&&element_state==0&&sc_finish==0)
            {
              encoder_ctrl(1);
              angle_PID_function(Initial_yaw);
              gory_PID_function(angle_pid.output);
              if(sc_y>=0.15)
                speed_PID_function(0,-0.2);
              else if(sc_y<0.1&&sc_y>zd_sc)
                speed_PID_function(0,-0.1);
              for(int n=0;n<2;n++)
              {
                if(ABS(speed_pid[n].output)>=10000)
                {
                  speed_pid[n].output=SIGN(speed_pid[n].output)*10000;
                }
              }
              
              speed_output[0] = speed_pid[0].output;
              speed_output[1] = speed_pid[1].output;
              speed_output[2] = gory_pid.output;
              dispense_speed();
              moter_control();
              encoder_ctrl(2);
//              move(0,-0.7,Initial_yaw);
            }
            else
            {
              my_move(Initial_yaw);
              // flag_pid_error=0;
              picture_flag1=0;
            }
          }
          else if(ABS(Mid_add)>=10)
          {
            sc_flag=2;
            if(sc_y<=wd_sc)
            {
              sc_y=wd_sc;
              sc_finish=1;

            }
            if(sc_y>wd_sc&&element_state==0&&sc_finish==0)
            {
              encoder_ctrl(1);
              angle_PID_function(Initial_yaw);
              gory_PID_function(angle_pid.output);
              if(sc_y>=-0.02)
                speed_PID_function(SIGN(Mid_add)*0.35,-0.25);
              else if(sc_y<-0.02&&sc_y>wd_sc)
                speed_PID_function(SIGN(Mid_add)*0.15,-0.2);
              
              for(int n=0;n<2;n++)
              {
                if(ABS(speed_pid[n].output)>=10000)
                {
                  speed_pid[n].output=SIGN(speed_pid[n].output)*10000;
                }
              }
              if(ABS(gory_pid.output)>=12000)
              {
                gory_pid.output=SIGN(gory_pid.output)*12000;
              }
              speed_output[0] = speed_pid[0].output;
              speed_output[1] = speed_pid[1].output;
              speed_output[2] = gory_pid.output;
              dispense_speed();
              moter_control();
              encoder_ctrl(2);
//              move(0,-0.7,Initial_yaw);
            }
            else
            {
              my_move(Initial_yaw);
              // flag_pid_error=0;
              picture_flag1=0;
            }
          }
          
        }
      }
    }
    //*********************************************************************************       
    else if (smartcar_state==3)//坡道避障
    {
      encoder_ctrl(1);
      angle_PID_function(0);
      gory_PID_function(angle_pid.output);
      ramp_x=outputL[1];
      ramp_y=outputL[0];
      
      if(nnnn==0)//如果卡坡道往后退
      {
        if(ramp_y>-0.01&&dl1b_distance_mm<200)
        {
          speed_PID_function(0,-3);
          ramp_kz=1;
        }
        else
        {
          outputL[0]=0;
          outputL[1]=0;
          ramp_y=0;
          ramp_x=0;
          speed_PID_function(0,0);
          nnnn++;
        }
      }
      else if(nnnn==1)//横移出坡道
      {
        if(ABS(ramp_x)<=0.35)
          speed_PID_function(ramp_switch,0);
        else
        {
          outputL[0]=0;
          outputL[1]=0;
          ramp_y=0;
          ramp_x=0;
          speed_PID_function(0,0);
          nnnn++;
        }
      }
      else if(nnnn==2)//前进
      {
        if(ramp_kz!=1)//没卡坡道
        {
          if(ABS(ramp_y)<=1.65)
            speed_PID_function(0,0.8);
          else
          {
            outputL[0]=0;
            outputL[1]=0;
            ramp_y=0;
            ramp_x=0;
            speed_PID_function(0,0);
            nnnn++; 
          }
        }
        else if(ramp_kz==1)//卡坡道
        {
          if(ABS(ramp_y)<=2.30)
            speed_PID_function(0,0.8);
          else
          {
            outputL[0]=0;
            outputL[1]=0;
            ramp_y=0;
            ramp_x=0;
            speed_PID_function(0,0);
            nnnn++; 
          }
        }
      }
      else if(nnnn==3)//斜移入坡道
      {

        if(ABS(ramp_x)<=0.45)
          speed_PID_function(-ramp_switch,0.35);
        else
        {
          outputL[0]=0;
          outputL[1]=0;
          ramp_y=0;
          ramp_x=0;
          speed_PID_function(0,0);
          nnnn++;
        }
      }
      else if(nnnn==4)
      {
        outputL[0]=0;
        outputL[1]=0;
        ramp_y=0;
        ramp_x=0;
        tofa_flag=0;
        nnnn=0;
        smartcar_state=0;
      }
        
      speed_output[0] = speed_pid[0].output;
      speed_output[1] = speed_pid[1].output;
      speed_output[2] = gory_pid.output;
      dispense_speed();
      moter_control();
      encoder_ctrl(2);
    }
    //******************若坡道后没元素*******************************//
//    else if (smartcar_state==3)//坡道避障
//    {
//      encoder_ctrl(1);
//      angle_PID_function(0);
//      gory_PID_function(angle_pid.output);
//      ramp_x=outputL[1];
//      ramp_y=outputL[0];
//      
//      if(nnnn==0)//如果卡坡道往后退
//      {
//        if(ramp_y>-0.01&&dl1b_distance_mm<200)
//        {
//          speed_PID_function(0,-3);
//          ramp_kz=1;
//        }
//        else
//        {
//          outputL[0]=0;
//          outputL[1]=0;
//          ramp_y=0;
//          ramp_x=0;
//          speed_PID_function(0,0);
//          nnnn++;
//        }
//      }
//      else if(nnnn==1)//横移出坡道
//      {
//        if(ABS(ramp_x)<=0.35)
//          speed_PID_function(ramp_switch,0);
//        else
//        {
//          outputL[0]=0;
//          outputL[1]=0;
//          ramp_y=0;
//          ramp_x=0;
//          speed_PID_function(0,0);
//          nnnn++;
//        }
//      }
//      else if(nnnn==2)//前进
//      {
//        if(ramp_kz!=1)//没卡坡道
//        {
//          if(ABS(ramp_y)<=1.9)
//            speed_PID_function(0,0.8);
//          else
//          {
//            outputL[0]=0;
//            outputL[1]=0;
//            ramp_y=0;
//            ramp_x=0;
//            speed_PID_function(0,0);
//            nnnn++; 
//          }
//        }
//        else if(ramp_kz==1)//卡坡道
//        {
//          if(ABS(ramp_y)<=2.2)
//            speed_PID_function(0,0.8);
//          else
//          {
//            outputL[0]=0;
//            outputL[1]=0;
//            ramp_y=0;
//            ramp_x=0;
//            speed_PID_function(0,0);
//            nnnn++; 
//          }
//        }
//      }
//      else if(nnnn==3)//斜移入坡道
//      {
//
//        if(ABS(ramp_x)<=0.40)
//          speed_PID_function(-ramp_switch,0.5);
//        else
//        {
//          outputL[0]=0;
//          outputL[1]=0;
//          ramp_y=0;
//          ramp_x=0;
//          speed_PID_function(0,0);
//          nnnn++;
//        }
//      }
//      else if(nnnn==4)
//      {
//        outputL[0]=0;
//        outputL[1]=0;
//        ramp_y=0;
//        ramp_x=0;
//        tofa_flag=0;
//        nnnn=0;
//        smartcar_state=0;
//      }
//        
//      speed_output[0] = speed_pid[0].output;
//      speed_output[1] = speed_pid[1].output;
//      speed_output[2] = gory_pid.output;
//      dispense_speed();
//      moter_control();
//      encoder_ctrl(2);
//    }
    //**************************************************************//
    else if (smartcar_state==4)//循迹
    {
       if(Single_line!=0&&single_flag!=0)
      {
        if(barrier_left_single==0&&barrier_right_single==0)
        {
           if(single_continue==0)  //只跑元素
          {
            wk_single=output_single[0];
            if(wk_single>=1.4)    //1.4
            {
              Single_line=0;     //关闭单边寻线
              single_flag=0;
              speed_falg1=0;
              wk_single=0;        //记步清零
              output_single[0]=0;
              output_single[1]=0;
              art_02_flag=1;//art1发送0x02标志位
              //      Send_first_search_message();
            }
          }
          else if(single_continue==1)
          {
            wk_single=output_single[0];
            if(wk_single>=1.5)  //1.4
            {
              Single_line=0;//关闭单边寻线
              single_flag=0;
              wk_single=0;//记步清零
              output_single[0]=0;
              output_single[1]=0;
              art_02_flag=1;//art1发送0x02标志位
              //      Send_first_search_message();
            }
          }
        }
        if(barrier_left_single==1)
        {
           wk_single=output_single[0];
          if(wk_single>=0.7)
          {
            Single_line=0;   //关闭单边寻线
            single_flag=0;
            barrier_left_single=0;
            wk_single=0;    //记步清零
            output_single[0]=0;
            output_single[1]=0;
            art_02_flag=1;//art1发送0x02标志位
            //      Send_first_search_message();
          }
        }
        if(barrier_right_single==1)
        {
            wk_single=output_single[0];
          if(wk_single>=0.55)
          {
            Single_line=0;//关闭单边寻线
            single_flag=0;
            barrier_right_single=0;
            wk_single=0; //记步清零
            output_single[0]=0;
            output_single[1]=0;
            art_02_flag=1;//art1发送0x02标志位
            //      Send_first_search_message();
          }
        }
      }
      if(zebra_jibu==1) //用于斑马线开始 识别到斑马线后开始计部
      {
         wk_single=output_start[0];
          if(wk_single>=0.75)
          {
            zebra_start=1;
            art_begin=1;
            wk_single=0;
            output_start[0]=0;
            output_start[1]=0;
            zebra_jibu=0;
            
          }
          
      }
      if(stop_flag==1)  //识别到斑马线后 停车计步
      {   
         wk_single=output_stop[0];
        if(wk_single>=0.85)
        {
          
          wk_single=0;
          output_stop[0]=0;
          output_stop[1]=0;
          stop_flag=0;
          prep=0;
          smartcar_state=0;
        }
      
      }
      goon_flag=0;
      if(mt9v03x_finish_flag)   
      {      
        deal_image();        //总转风图像采集处理
        mt9v03x_finish_flag = 0;
        Reduction_yaw=0;
        
      }
//      if(tofa_start==1)
//      {
//        tofa_process();
//      }
      Line_following();  //循迹函数
        
//      kk=1;
      
//      else if(no_picture==1)
//      {
//        smartcar_state=0;
//      }
//      smartcar_state=0;

    }
    else if (smartcar_state==5)//看到元素/放置区旋转至正确姿态
    {
      Final_yaw=Reduction_yaw+carmer_yaw;
      move(need_grid[0],need_grid[1],need_angle);
      
//      move(0,3,0);
      
    }
    else if (smartcar_state==6)//转身扔照片
    {
//      if((element_state==1||element_state==2)&&yc2==8)
//      {
//        
//        move(0,0,-180);
//      }
//      else
//        move(2.5,1.8,need_angle);
        ys_angle=1;
        if(element_state==4)
        {
          move(0,0,need_angle-90);
        }
        else
        {
          move(0,0,need_angle-90);
        }
        output_zebra[0]=0;

    }  
    else if (smartcar_state==7)//遍历元素外围放置区
    {
//      tft180_show_float(0, 50, output_zebra[0], 1, 3);      
//      tft180_show_float(0, 70, zebra_forward, 1, 3);

      encoder_ctrl(1);
      angle_PID_function(need_angle);
      gory_PID_function(angle_pid.output);
      wk_ys=outputL[0]+outputL[1];
      if(wk_ys>1.8)
        back_find=1;
      if(yc2<8)
      {
        if(element_state==1||element_state==2)//圆环
        {
          if(back_find!=1)
          {
            if(yc2==3)//1号位
            {
              if(hk_nwj==1)             //小框
                speed_PID_function(0.525,0.45);       //(0.35,0.3)
              else if(hk_nwj==2)        //大框
                speed_PID_function(0.525,0.50);       //(0.35,0.3)
            }
            else if(yc2==7)//5号位
            {
//              speed_PID_function(0.60,0.390);      //(0.55,0.12)
              if(hk_nwj==1)             //小框
                speed_PID_function(0.60,0.350);      //(0.55,0.12)
              else if(hk_nwj==2)        //大框
                speed_PID_function(0.60,0.380); 
            }
            else//234号位
            {
              if(hk_nwj==1)
                speed_PID_function(0.9,0.320);      //(0.5,-0.05)
              else if(hk_nwj==2)
                speed_PID_function(0.9,0.330);      //(0.5,-0.05)
            }
          }
          else//因为卡轮子导致y轴少跑而找不到放置区补救措施
            speed_PID_function(-0.8,0);
        }
        else if(element_state==3)//十字
        {
          if(cross_switch!=0)
          {
            if(back_find!=1)
            {
              if(yc2==3&&cross_switch==2)//右p第一步
              {
                if(hk_nwj==1)//小框
                {
                  speed_PID_function(-0.2,0.2);
                }
                else if(hk_nwj==2)//大框
                {
                  speed_PID_function(-0.2,0.2);
                }
              }
              else if(yc2==3&&cross_switch==1)//左p第一步
              {
                if(hk_nwj==1)//小框
                {
                  speed_PID_function(0.5,0.5);
                }
                else if(hk_nwj==2)//大框
                {
                  speed_PID_function(0.3,0.5);
                }
              }
              else//2345号位
              {
                if(hk_nwj==1)           //小框
                {
                  speed_PID_function(0.70,0.25); 
                }
                else if(hk_nwj==2)      //大框
                {
                  speed_PID_function(0.70,0.30); 
                }
              } 
            }
            else
              speed_PID_function(-0.8,0);
          }
        }
        
      }
      else if(yc2>=8&&(element_state==1||element_state==2||element_state==3))
      {
        element_state=0;
        pick_complete=0;
        smartcar_state=4;
        remember_i=0;
        e_p_flag=0;

      }
    
      if(yc3<3)
      {
        if(element_state==4)
        {
          wk_zebra=output_zebra[1];
//          wk_zebra_y=output_zebra[0];
//          wk_zebraF_y=output_zebra[2];
//          wk_zebraS_y=output_zebra[3];

          if( output_zebra[0]>(0.02+zebra_forward))//防止车上下飘，给反方向速度
            y_speed1=-0.2;
          else if(output_zebra[0]<(-0.02+zebra_forward))
            y_speed1=0.6;
          else
            y_speed1=0;

//          if(zebra_commutation_flag==0&&wk_zebraF_y<zebra_forward)
//            speed_PID_function(zebra_speed,y_speed1+0.3);
//          else if(zebra_commutation_flag==1&&wk_zebraS_y<zebra_forward)
//            speed_PID_function(zebra_speed,y_speed1+0.3);
//          else 
            speed_PID_function(zebra_speed,y_speed1);

          if(ABS(wk_zebra)>=2.0&&zebra_commutation_flag!=1)
          {
            zebra_commutation_flag=1;
            break_out_flag=1;
            output_zebra[0]=0;
            zebra_forward=0.12;
            yc3--;
          }
          if(break_out_flag==1)
          {
            smartcar_state=0;
            break_out_flag=0;
          }
        }
      }
      else if(yc3>=3&&element_state==4)
      {
        element_state=0;
        smartcar_state=4;
        zebra_commutation_flag=0;
        e_p_flag=0;

      }
             
      speed_output[0] = speed_pid[0].output;
      speed_output[1] = speed_pid[1].output;
      speed_output[2] = gory_pid.output;
      dispense_speed();
      moter_control();
      encoder_ctrl(2);
    }
    else if (smartcar_state==8)//捡完照片回原位
    {
      if(element_state==0)
      {
//        move(-3*bl_x,-1.5*wk_y,0);
        //        move(-1.3*SIGN(jz_x),0,0);
        if(fake_picture==0)
        {
          if(dispersive_position==2)
            move(1.3,0.5,0);
          else if(dispersive_position==0)
            move(-1.3,0.5,0);
          else
            move(0,0,0);
        }
        else if(fake_picture==1)
        {
          if(dispersive_position==2)
            move(2,0.5,0);
          else if(dispersive_position==0)
            move(-2,0.5,0);
          else
            move(0,0,0);
        }

      }

    } 
    else if (smartcar_state==9)//测试
    {
      encoder_ctrl(1);
//      angle_PID_function(117);
//      angle_PID_function(98);
//      angle_PID_function(42);
//      angle_PID_function(-15);
//      angle_PID_function(-77);
      wk_ys=outputL[0]+outputL[1];
      if(wk_ys>1.8)
        back_find=1;
      angle_PID_function(0);
      gory_PID_function(angle_pid.output);
      if(back_find!=1)
      {
        speed_PID_function(0.60,0.388); 
      }
      else
        speed_PID_function(-0.6,0);

      speed_output[0] = speed_pid[0].output;
      speed_output[1] = speed_pid[1].output;
      speed_output[2] = gory_pid.output;
      dispense_speed();
      moter_control();
      encoder_ctrl(2);
    }
    else if (smartcar_state==10)
    {
//      if(element_state==2||element_state==1)
//      {
//        move(0,3.5,0);
//        kkkk=1;
//      }  
//      else if(element_state==3&&cross_switch==1)//左p
//      {
//        move(3,0,-90);
//        kkkk=2;
//      }
//      else if(element_state==3&&cross_switch==2)//右p
//      {
//        move(-3,0,90);
//        kkkk=3;
//      }
//        
//      element_state=0;
//      pick_complete=0;
////      smartcar_state=4;
//      remember_i=0;
    }
    else if (smartcar_state==12)//保持不动
    {
//      if(yc_angle_test==1)
//        move(0,0,117);//左环
//      else if(yc_angle_test==2)
//        move(0,0,58);
//      else if(yc_angle_test==3)
//        move(0,0,-53);//右环
//      else if(yc_angle_test==4)
//        move(0,0,-124);
//      else
        move(0,0,0);
    }
    else if (smartcar_state==13)//未使用
    {
      if(dispersive_picture==1)
        move(0,0,dispersive_angle);
      if(dispersive_picture==2)
        move(0,0,dispersive_angle);
    }  
    else //这里是小车停止的状态smartca_state0
    {
      flag_pid = 0;
      pid_fucntion_flag = 1;
      if (ys_angle==1/*||dispersive_picture!=0*/)
      {
        if(ys_angle==1)
        {
          if(element_state==4)
          {
            move(0,0,need_angle-90);
          }
          else
          {
            move(0,0,need_angle-90);
          }
        }
//          move(0,0,need_angle-90);
//        else if(dispersive_picture!=0)//未使用
//          move(0,0,dispersive_angle);  
      }
      else
      {
        if(element_state!=0)
        {
          if(element_state!=4&&pick_complete!=1&&spin_finish==1)//元素转完
          {
            wk_pick_y=output_pick[0];
            encoder_ctrl(1);
            if(angle_function_act == 1)
            {
              angle_function_act = 0;
              angle_PID_function(need_angle);
              gory_PID_function(angle_pid.output);
            }
            location_PID_function_hard(0);
            speed_PID_function(0,location_pid_hard[0].output);
//            if(wk_pick_y>0.01)
//              speed_PID_function(0,-0.3);
//            else if(wk_pick_y<-0.01)
//              speed_PID_function(0,0.3);
            speed_output[0] = speed_pid[0].output;
            speed_output[1] = speed_pid[1].output;
            speed_output[2] = gory_pid.output;
            dispense_speed();
            moter_control();
            encoder_ctrl(2);
          }
          else
            move(0,0,need_angle);
        }
        else if(element_state==0)
        {
          move(0,0,0);
        }
        if(goon_flag==1)
        {
          smartcar_state=4;
        }
        
      }
      
//      if(lz_flag==1)
//      {
//        sc_monitor1++;
//        if(sc_monitor1==800)
//        {
////          Send_clear_message();
////          Send_clear_2_message();
//          smartcar_state=0;
//          pick_complete=1;
//        }
//      }
          
      
    }
    
//    if((goon_flag==1)&&(bcc==11)&&(smartcar_state==4))//当机械臂抬起的过程中，摄像头继续开始工作
//    {
//      Send_corrcet_message();        //  发送矫正姿态信息
//      goon_flag=0;
//      bcc=0;
//    }
//    
    
    
    
    switch(delay_temp)
    {
      case 10: delay_complete_flag= time_delay(2,delay_complete_flag);break;
      case 90: delay_complete_flag= time_delay(15,delay_complete_flag);break;
      case 96: delay_complete_flag= time_delay(16,delay_complete_flag);break; 
      case 120: delay_complete_flag= time_delay(20,delay_complete_flag);break;
      case 150: delay_complete_flag= time_delay(25,delay_complete_flag);break;
      case 200: delay_complete_flag= time_delay(33,delay_complete_flag);break;
      case 240: delay_complete_flag= time_delay(40,delay_complete_flag);break;
      case 300: delay_complete_flag= time_delay(50,delay_complete_flag);break;
      case 390: delay_complete_flag= time_delay(65,delay_complete_flag);break;
      case 600: delay_complete_flag= time_delay(100,delay_complete_flag);break;  
      case 1800: delay_complete_flag= time_delay(300,delay_complete_flag);break;
      case 2000: delay_complete_flag= time_delay(366,delay_complete_flag);break;  
      case 3600: delay_complete_flag= time_delay(600,delay_complete_flag);break;
    }
  }
  
  
  if(pit_flag_get(PIT_CH2))
  {
    pit_flag_clear(PIT_CH2);
   
//    send_massage(107); 
//    tft180_show_uint(0,112,pick_complete,1);
    
  } 
  
  if(pit_flag_get(PIT_CH3))
  {
    pit_flag_clear(PIT_CH3);
  }
  __DSB();
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
    #if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                      // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
    #endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去      
        extern void uart_rx_interrupt_handler1();
        uart_rx_interrupt_handler1();
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断 
//        flexio_camera_uart_handler();
//        
//        gnss_uart_callback();
       extern void uart_rx_interrupt_handler4();
       uart_rx_interrupt_handler4();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        wireless_module_uart_handler();
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}









/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/


