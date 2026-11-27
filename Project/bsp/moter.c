/*******************************************************************************************************************/
/***Include***/
//{
#include "zf_common_headfile.h"
#include "define.h"
#include "bsp.h"
//}
/*******************************************************************************************************************/
 
/***Functions***/
//{
//------------------------------------------------------------------------------------------------------------------
// @brief       麦克纳姆轮解算
// @
// @return      void
//------------------------------------------------------------------------------------------------------------------
void Kinematics_Forward()
{
//从四个编码器获取上个时间段T所积累的脉冲数
//并根据小车电机的朝向做相应的变化
    	input[0] = -encoder[0];
	input[1] = encoder[1];
	input[2] = -encoder[2];
	input[3] = encoder[3];
//计算小车在上一个时间段T所走过的距离//单位：米
        for(int n=0;n<4;n++)
	{
          dv_w_times_dt[n] = 1.0*input[n];
          encoder_sum_current[n] = input[n];
        }
//将数据变换到最初的状态方便后续计算        
        dv_w_times_dt[0] = -dv_w_times_dt[0];
	dv_w_times_dt[1] =  dv_w_times_dt[1];
	dv_w_times_dt[2] = -dv_w_times_dt[2];
	dv_w_times_dt[3] =  dv_w_times_dt[3];
//计算在小车坐标系下小车的纵向和横向移动距离
        dv_t_times_dt[0] = ( dv_w_times_dt[0] + dv_w_times_dt[1] + dv_w_times_dt[2] + dv_w_times_dt[3])/(4.0*a[0]);
        dv_t_times_dt[1] = ( dv_w_times_dt[0] - dv_w_times_dt[1] - dv_w_times_dt[2] + dv_w_times_dt[3])/(4.0*a[1]);
//        if(need_grid[1]<=1)
//        {
//        dv_t_times_dt[0]=dv_t_times_dt[0] /ybm[0]
//        }
//根据陀螺仪的返回角度，计算在地面坐标系下小车的纵向和横向移动距离
        
#if(0)
使用二阶旋转矩阵
      [cosθ -sinθ] [x]                          [cosθ  sinθ] [x]
      [sinθ  cosθ] [y]     逆时针               [-sinθ cosθ] [y]     顺时针
#endif
        speed_filter[0].input = dv_t_times_dt[0]* PID_RATE;
        speed_filter[1].input = dv_t_times_dt[1] * PID_RATE; 
//速度滤波        
        filter_function(F_speed);
//数值装载
        encoder_data[0].encoder_speed[0] = speed_filter[0].output;
        encoder_data[1].encoder_speed[0] = speed_filter[1].output;
        outputV[0] = (encoder_data[0].encoder_speed[0]);
        outputV[1] = (encoder_data[1].encoder_speed[0]);
//计算上一时间段T的对地微小路程
         
        aL[0] = dv_t_times_dt[0] ;
        aL[1] = dv_t_times_dt[1];///1.08

//计算平行线、对角线上轮子转速差
        ec12=ABS(ABS(encoder[0])-ABS(encoder[1]));
        ec13=ABS(ABS(encoder[0])-ABS(encoder[2]));
        ec14=ABS(ABS(encoder[0])-ABS(encoder[3]));
        ec23=ABS(ABS(encoder[1])-ABS(encoder[2]));
        
//        ecj12=encoder[0]*encoder[1];
//        ecj34=encoder[2]*encoder[3];
//微小路程加和求总路程
        if(element_state!=0&&element_state!=4&&smartcar_state==0&&pick_complete!=1&&spin_finish==1)
        {
          output_pick[0]+= aL[0];
        }
        if((ec12<50&&ec13<50&&ec14<50)||(ec14<50&&ec23<50))//平行移动或斜向移动并且轮子没有空转
        {
          outputL[0] += aL[0];
          outputL[1] += aL[1];
          if(element_state==4&&smartcar_state==7)
          {
//            if(zebra_commutation_flag==0)
//              output_zebra[2] += aL[0];
//            else if(zebra_commutation_flag==1)
//              output_zebra[3] += aL[0];
            output_zebra[0] += aL[0];
            output_zebra[1] += aL[1];
          }
          if(Single_line!=0&&single_flag!=0)//单边 
          {
            output_single[0]+= aL[0];
            output_single[1]+= aL[1];
          }
          if(zebra_jibu==1)                //发车
          {
            output_start[0]+= aL[0];
            output_start[1]+= aL[1];
          }
          if(stop_flag==1)                 //停车
          {
            output_stop[0]+= aL[0];
            output_stop[1]+= aL[1];
          }
          
          if(flag_pid_error==ture_)
          {
            output_sc[0]+= aL[0];
            output_sc[1]+= aL[1];
          }
          if(smartcar_state==1)
          {
            output_ycforward[0]+= aL[0];
            output_ycforward[1]+= aL[1];
          }
          
//          outputjz[0] += aL[0];
//          outputjz[1] += aL[1];
//          output_ramp[0] += aL[0];
//          output_ramp[1] += aL[1];
        }
        
//        if(ec12<40&&ec13<40&&ecj12>0&&ecj34>0)
//        {
//          if(flag_pid_error==ture_)         //刹车记步
//          {
//            output_sc[0]+= aL[0];
//            output_sc[1]+= aL[1];
//          }
//        }
 //数值重装载       
        for(int n = 0;n < 2;n++)
        {
          encoder_data[n].encoder_speed[2] = encoder_data[n].encoder_speed[1];
          encoder_data[n].encoder_speed[1] = encoder_data[n].encoder_speed[0];
          encoder_data[n].encoder_acc[1] = encoder_data[n].encoder_acc[0];
        }
}

//------------------------------------------------------------------------------------------------------------------
// @brief       编码器的统合函数
// @param       mode 模式选择
// @mode_0      初始化
// @mode_1      获取编码器的数据
// @mode_2      清空编码器的值    
// @return      void
//------------------------------------------------------------------------------------------------------------------
void encoder_ctrl(int mode)
{
  switch(mode)
  {
    case 0:
      {
        encoder_quad_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);
        encoder_quad_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);
        encoder_quad_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C25);
        encoder_quad_init(QTIMER3_ENCODER2,QTIMER3_ENCODER2_CH1_B18,QTIMER3_ENCODER2_CH2_B19);
        break;
      }
     
    case 1:{
      for(int n=0;n<4;n++)
      {
        encoder_filter[n].input = encoder_get_count(moter[n].moter_timerch);
      }
      filter_function(F_encoder);
      for(int n=0;n<4;n++)
      {
        encoder[n] = encoder_filter[n].output;
      }
      Kinematics_Forward();
        break;
      }
        
    case 2:{
        for(int n=0;n<4;n++)
      {
        encoder_clear_count(moter[n].moter_timerch);
      }
        break;
      }
      default:break;
  
  }
    
}
//------------------------------------------------------------------------------------------------------------------
// @brief       电机驱动函数
// @return      void
//------------------------------------------------------------------------------------------------------------------
void moter_control()
{
    for(int n=0;n<4;n++)
    {
      if(moter[n].moter_pwm>=0)
      {
        pwm_set_duty(moter[n].moter_pwmch_A,0);
        pwm_set_duty(moter[n].moter_pwmch_B,moter[n].moter_pwm );
      }
      if(moter[n].moter_pwm<0)
      {
        pwm_set_duty(moter[n].moter_pwmch_B,0);
        pwm_set_duty(moter[n].moter_pwmch_A,-moter[n].moter_pwm );
      }
    }
  
    
//使用逐飞的驱动板
//  if(moter[1].moter_pwm>=0)
//    gpio_set(D12, 1);
//  else
//    gpio_set(D12,0); 
//  if(moter[0].moter_pwm>=0)
//    gpio_set(D13,1);
//  else
//    gpio_set(D13,0);
//  if(moter[2].moter_pwm>=0)
//    gpio_set(D1, 1);
//  else
//    gpio_set(D1,0);
//  if(moter[3].moter_pwm>=0)
//    gpio_set(D0,  1);
//  else
//    gpio_set(D0, 0);
//  pwm_set_duty(PWM1_MODULE1_CHA_D14 ,abs(moter[1].moter_pwm));//电机   右前 
//  pwm_set_duty(PWM1_MODULE1_CHB_D15 ,abs(moter[0].moter_pwm) );//电机  左前    
//  pwm_set_duty(PWM2_MODULE3_CHA_D2 ,abs(moter[3].moter_pwm) );//电机  右后
//  pwm_set_duty(PWM2_MODULE3_CHB_D3 ,abs(moter[2].moter_pwm) );//电机  左后
   
  
  
 
}
//------------------------------------------------------------------------------------------------------------------
// @brief       速度分配函数
// @return      void
//------------------------------------------------------------------------------------------------------------------
void dispense_speed()
{
//定义k：小车速度分配系数
  float k;
//计算总速度模量
  move_sum = ABS(speed_output[0]) + ABS(speed_output[1]) + ABS(speed_output[2] * 1);
//超限判断
  if(move_sum>(PWM_MAX))
  {
//等比例变换
//保证纵向横向和旋转分量的比例关系不变
    k = (PWM_MAX)/move_sum;
    speed_output[0] = k * speed_output[0];
    speed_output[1] = k * speed_output[1];
    speed_output[2] = k * speed_output[2];
  }
//计算四个轮子应有的PWM值  
    moter[0].moter_pwm = (int32)(speed_output[0] + speed_output[1] - speed_output[2] * 1);
    moter[1].moter_pwm = (int32)(speed_output[0] - speed_output[1] + speed_output[2] * 1);
    moter[2].moter_pwm = (int32)(speed_output[0] - speed_output[1] - speed_output[2] * 1);
    moter[3].moter_pwm = (int32)(speed_output[0] + speed_output[1] + speed_output[2] * 1);
//数值传递
    for(int n = 0;n < 4;n++)
    {
      moter[n].moter_pwm = (int32)(1.0 * moter[n].moter_pwm);
    }
}
//}
/*******************************************************************************************************************/
#if(0)
Reference Resources:
https://www.bilibili.com/video/BV1sY411H74R/?spm_id_from=333.337.search-card.all.click  
https://www.bilibili.com/read/cv11415123
#endif
/*******************************************************************************************************************/