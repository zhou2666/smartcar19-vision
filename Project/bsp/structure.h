/*******************************************************************************************************************/
#ifndef __STRUCTURE_H
#define __STRUCTURE_H
/*******************************************************************************************************************/
/***Includes***/
//{
#include "zf_common_headfile.h"
//}
/*******************************************************************************************************************/
/***Statements***/
//{
/**Moters**/
//{
typedef struct
{
  encoder_index_enum moter_timerch;//编码器定时器通道
  encoder_channel1_enum moter_timer_pin;//编码器定时器引脚
  pwm_channel_enum moter_pwmch_B;//副pwm引脚
  pwm_channel_enum moter_pwmch_A;//主pwm引脚
  int32 moter_pwm;//电机上的pwm占空比
  int16 moter_encoder;//下一次编码器的读输出值
  int16 moter_encoder_last;//下一次编码器的读输出值
  float moter_now;
  float moter_last;
}moter_init;
//}
/**PID**/
//{
typedef struct
{
  volatile float Kp;                    //比例系数
  volatile float Ki;                    //积分系数
  volatile float Kd;                    //微分系数
  volatile float ExpectedValue;         //系统期望值
  volatile float ActualValue;           //系统实际值
  volatile float err[3];                //系统近三次误差[本次,上次,前次]
  volatile float err_o[11];             
  volatile float err_add;               //系统误差累计(最终)
  volatile float Ud[2];                    //微分输出
  volatile float output_dead_bar;        //输出死区
  volatile float deter_output;           //相对增量
  volatile float err_m;                 //系统误差最近一次极值
  volatile float inc_err[2];            //系统误差变化量[本次,上次]   
  volatile float abs_err[3];            //系统允许误差绝对值[最大值,中值,最小值]
  volatile float dead_bar;               //系统控制死区
  volatile float output_last;            //系统PID输出传递值
  volatile float output;                //系统PID输出
  volatile float err_sum;
  volatile float err_temp;
}pid_init;
//}
/**Filters**/
//{
typedef struct
{
  float input;
  float baroAlt;
  float filter_hander;
  float output;
}fliter_init;
//}
/***PID_imports***/
typedef struct
{
  volatile float acc;
  volatile float speed;
  volatile float location;
}data_init;
/**Gyroscopes**/
//{
typedef struct 
{
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
}icm_param_t;

typedef struct 
{
  float q0;
  float q1;
  float q2;
  float q3;
}quater_param_t;

typedef struct 
{
  float pitch;
  float roll;
  float yaw;
  float yaw_last;
  float cad[2];
  float final_yaw;
  volatile int k;
  float ceil;            //修正系数
  float final_yaw_last;
}euler_param_t;

typedef struct 
{
  int16 Xdata;
  int16 Ydata;
  int16 Zdata;
  float Xacc_data;
  float Yacc_data;
}GyroOffset_init;

typedef struct 
{
  float acc_x;
  float acc_y;
  float acc_z;
}ACC_init;

typedef struct
{
  float encoder_speed[3];             //本次，上次，前次
  float encoder_acc[2];               //本次，上次
  float encoder_jerk;
}encoder_reader_init;

typedef struct
{
float vx;
float vy;
float vz;
float ex; 
float ey; 
float ez; 
float I_ex;  
float I_ey;  
float I_ez;  
}gyrocope_err_init;




//}

//}
/*******************************************************************************************************************/
#endif
