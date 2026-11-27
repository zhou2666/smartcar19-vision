/*******************************************************************************************************************/
/***Include***/
//{
#include "zf_common_headfile.h"
#include "define.h"
#include "bsp.h"

//}
/*******************************************************************************************************************/
/***Functions***/
////{
////------------------------------------------------------------------------------------------------------------------
//// @brief       延时函数
//// @param       wanna_time      延迟时间
//// @param       mode            模式选择
//// @mode ms                     毫秒模式
//// @mode us                     微秒模式
//// @return      void
////--------------------------------------------------------------------------------------------------------------
//void Delay(uint64 wanna_time,int mode)
//{
//  switch(mode)
//  {
//    case(ms) :{pit_delay_ms(PIT_CH3,wanna_time);break;}
//    case(us) :{pit_delay_us(PIT_CH3,wanna_time);break;}
//    default  :break;
//  }
//}
//------------------------------------------------------------------------------------------------------------------
// @brief       定时器初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void tim_setup()
{ 
//陀螺仪中断
  pit_ms_init(PIT_CH0,6);  
  interrupt_set_priority(PIT_IRQn,5);
//定时器中断用于舵机  
  pit_ms_init(PIT_CH2,6);  
  interrupt_set_priority(PIT_IRQn,5); 
//定时器中断用于PID  
  pit_ms_init(PIT_CH1,6); 
  interrupt_set_priority(PIT_IRQn,5);
  interrupt_global_enable(0);
}
////------------------------------------------------------------------------------------------------------------------
//// @brief       条件中断初始化
//// @return      void
////------------------------------------------------------------------------------------------------------------------
//void Requirement_init()
//{
//  gpio_interrupt_init(C4,RISING,GPIO_INT_CONFIG);
//  interrupt_set_priority(GPIO2_Combined_0_15_IRQn,4);
//}

//------------------------------------------------------------------------------------------------------------------
// @brief       麦克纳姆轮初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void Kinematics_Init()
{
//计算脉冲距离转换比
 pulse_per_meter = (float)(ENCODER_RESOLUTION/(WHEEL_DIAMETER*PI))/linear_correction_factor;
//数值装载
  r_x = D_X/2;
  r_y = D_Y/2;
  rx_plus_ry_cali = (r_x + r_y)/angular_correction_factor;
}
//------------------------------------------------------------------------------------------------------------------
// @brief       数据值初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void dataBase_init()
{
  eulerAngle.pitch = 0;              // 欧拉角
  eulerAngle.roll = 0;
  eulerAngle.yaw = 0;
  eulerAngle.yaw_last = 0;
  eulerAngle.final_yaw = 0;
  eulerAngle.cad[0] = 0;
  eulerAngle.cad[1] = 0;
  eulerAngle.k = 0;
  eulerAngle.ceil = 1.00;
  eulerAngle.final_yaw_last = 0;
  
  GyroOffset.Xdata = 0;                     //零漂误差
  GyroOffset.Ydata = 0;               
  GyroOffset.Zdata = 0;
  GyroOffset.Xacc_data = 0;
  GyroOffset.Yacc_data = 0;
  
  icm_data_static.acc_x = 0;
  icm_data_static.acc_y = 0;
  icm_data_static.acc_z = 0;
  icm_data_static.gyro_x = 0;
  icm_data_static.gyro_y = 0;
  icm_data_static.gyro_z = 0;
  
  icm_data_random.acc_x = STATIC_ACC_X;
  icm_data_random.acc_y = STATIC_ACC_Y;
  icm_data_random.acc_z = STATIC_ACC_Z;
  icm_data_random.gyro_x = STATIC_GYRO_X;
  icm_data_random.gyro_y = STATIC_GYRO_Y;
  icm_data_random.gyro_z = STATIC_GYRO_Z;
  
  eulerAngle_static.pitch = 0;
  eulerAngle_static.roll = 0;
  eulerAngle_static.yaw = 0;
  eulerAngle_static.yaw_last = 0;
  eulerAngle_static.final_yaw = 0;
  eulerAngle_static.cad[0] = 0;
  eulerAngle_static.cad[1] = 0;
  eulerAngle_static.k = 0;
  eulerAngle_static.ceil = 0.95832;
  eulerAngle_static.final_yaw_last = 0;
  
  gyrocope_err.I_ex = 0;
  gyrocope_err.I_ey = 0;
  gyrocope_err.I_ez = 0;
  
  gyrocope_err_static.I_ex = 0;
  gyrocope_err_static.I_ey = 0;
  gyrocope_err_static.I_ez = 0;
}
//------------------------------------------------------------------------------------------------------------------
// @brief       电机pwm使能
// @return      void
//--------------------------------------------------------------------------------------------------------------
 
int time_delay(int time,uint8 b)
{
    if(b==0)
    {
        init_flag++;
        if(init_flag>=time)
        {
        init_flag=0;
        return 1;
        }
        else 
        {
        return 0;
        }
    }
    else if(b==1)
    {
       return 1;
    }
    return 0;
    
}

void over(int  a) //延时函数，需要在中断编写，这样可以在主函数的while加延时，也不影响中断
{
    delay_temp=a;
    while(init_flag!=0);
    delay_complete_flag=0;
    delay_temp=0;
    init_flag=1;
}


void moter_pwm_init()
{
  pwm_init(PWM1_MODULE1_CHB_D15,17000,0);//电机  L F
  pwm_init(PWM1_MODULE0_CHB_D13,17000,0);//电机 R F
  pwm_init(PWM1_MODULE3_CHA_D0,17000,0);//电机  L B
  pwm_init(PWM2_MODULE3_CHA_D2,17000, 0);//电机 R B 
  
  
  pwm_init(PWM1_MODULE1_CHA_D14,17000,0);//电机  L F 
  pwm_init(PWM1_MODULE0_CHA_D12,17000, 0);//电机 R F 
  pwm_init(PWM1_MODULE3_CHB_D1,17000,0);//电机  L B 
  pwm_init(PWM2_MODULE3_CHB_D3,17000,0 );//电机 R B 
//   gpio_init(D12, GPO, 0, GPIO_PIN_CONFIG);
//
// pwm_init(PWM1_MODULE1_CHA_D14,17000,0);//电机   右前 
//     gpio_init(C21, GPO, 0, GPIO_PIN_CONFIG);
//    
//  gpio_init(D13, GPO, 0, GPIO_PIN_CONFIG);
//  pwm_init(PWM1_MODULE1_CHB_D15,17000,0);//电机  左前 
//  
//    gpio_init(D0, GPO, 0, GPIO_PIN_CONFIG);
//  pwm_init(PWM2_MODULE3_CHA_D2,17000,0);//电机  左前 
//  
//    gpio_init(D1, GPO, 0, GPIO_PIN_CONFIG);
//  pwm_init(PWM2_MODULE3_CHB_D3,17000,0);//电机  左前 
}
 
//------------------------------------------------------------------------------------------------------------------
// @brief       舵机控制初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void carry_init()
{
  pwm_init(PWM4_MODULE2_CHA_C30,50,1250); //舵机 1
  pwm_init(PWM4_MODULE3_CHA_C31,50,1250); //舵机 2
  pwm_init(PWM2_MODULE2_CHB_C11,50,1250); //舵机 3
  gpio_init(C12,GPO,1,GPIO_PIN_CONFIG);//电磁铁
//  steering_bullt_A(degreeA_Init);
//  steering_bullt_B(1,degreeB_Init);
//  steering_bullt_B(2,degreeC_Init);;
//  Electromagnet(0);
}
//------------------------------------------------------------------------------------------------------------------
// @brief       PID初始化（距离）
// @return      void
//------------------------------------------------------------------------------------------------------------------
void pid_Init_A()
{
   for(int n=0;n<2;n++)
  {
    speed_pid[n].Kp = kp_speed[n];//
    speed_pid[n].Ki = ki_speed[n];
    speed_pid[n].Kd = kd_speed[n];
    speed_pid[n].err[0] = 0;
    speed_pid[n].err[1] = 0;
    speed_pid[n].err[2] = 0;
    speed_pid[n].err_add = 0;
    speed_pid[n].err_m = 0;
    speed_pid[n].dead_bar= 0.01;
    speed_pid[n].output = 0;
    speed_pid[n].Ud[0] = 0;speed_pid[n].Ud[1] = 0;
    speed_pid[n].output_last = 0;
    speed_pid[n].deter_output = 0;
    speed_pid[n].err_sum = 0;
    speed_pid[n].output_dead_bar = 20;
    speed_pid[n].inc_err[0] = 0;
    speed_pid[n].inc_err[1] = 0;
    speed_pid[n].abs_err[0] = 2.0;
    speed_pid[n].abs_err[1] = 1.0;
    speed_pid[n].abs_err[2] = 0.3;
  }
  
  for(int n=0;n<4;n++)//
  {
    encoder_pid[n].Kp = kp_encoder[n];
    encoder_pid[n].Ki = ki_encoder[n];
    encoder_pid[n].Kd = kd_encoder[n];
    encoder_pid[n].err[0] = 0;
    encoder_pid[n].err[1] = 0;
    encoder_pid[n].err[2] = 0;
    encoder_pid[n].err_add = 0;
    encoder_pid[n].err_m = 0;
    encoder_pid[n].dead_bar= 0.1;
    encoder_pid[n].deter_output = 0;
    encoder_pid[n].output = 0;
    encoder_pid[n].Ud[0] = 0;encoder_pid[n].Ud[1] = 0;
    encoder_pid[n].output_last = 0;
    encoder_pid[n].err_sum = 0;
    encoder_pid[n].output_dead_bar = 20;
    encoder_pid[n].inc_err[0] = 0;
    encoder_pid[n].inc_err[1] = 0;
    encoder_pid[n].abs_err[0] = 100;
    encoder_pid[n].abs_err[1] = 50;
    encoder_pid[n].abs_err[2] = 20;
  }
  
   for(int n=0;n<2;n++)
  {
    location_pid[n].err[0] = 0;
    location_pid[n].err[1] = 0;
    location_pid[n].err[2] = 0;
    location_pid[n].err_add = 0;
    location_pid[n].err_m = 0;
    location_pid[n].dead_bar= 0.01;
    location_pid[n].output = 0;
    location_pid[n].deter_output = 0;
    location_pid[n].output_last = 0;
    location_pid[n].err_sum = 0;
    location_pid[n].output_dead_bar = 0.005;
    location_pid[n].Ud[0] = 0;location_pid[n].Ud[1] = 0;
    location_pid[n].inc_err[0] = 0;
    location_pid[n].inc_err[1] = 0;
    location_pid[n].abs_err[0] = 2.0;
    location_pid[n].abs_err[1] = 1.0;
    location_pid[n].abs_err[2] = 0.2;
  }
}
//------------------------------------------------------------------------------------------------------------------
// @brief       PID初始化（角度）
// @return      void
//------------------------------------------------------------------------------------------------------------------
void pid_Init_B()
{
    angle_pid.Kp = kpidl_angle[0];
    angle_pid.Ki = kpidl_angle[1];
    angle_pid.Kd = kpidl_angle[2];
    angle_pid.err[0] = 0;
    angle_pid.err[1] = 0;
    angle_pid.err[2] = 0;
    angle_pid.err_add = 0;
    angle_pid.err_m = 0;
    angle_pid.dead_bar = 1;
    angle_pid.output = 0;
    angle_pid.deter_output = 0;
    angle_pid.output_dead_bar = 0.005 * PI;;
    angle_pid.Ud[0] = 0;angle_pid.Ud[1] = 0;
    angle_pid.output_last = 0;
    angle_pid.err_sum = 0;
    angle_pid.inc_err[0] = 0;
    angle_pid.inc_err[1] = 0;
    angle_pid.abs_err[0] = 60.0;
    angle_pid.abs_err[1] = 30.0;
    angle_pid.abs_err[2] = 10.0;
    
    gory_pid.Kp = kpidl_gory[0];//
    gory_pid.Ki = kpidl_gory[1];
    gory_pid.Kd = kpidl_gory[2];
    gory_pid.err[0] = 0;
    gory_pid.err[1] = 0;
    gory_pid.err[2] = 0;
    gory_pid.err_add = 0;
    gory_pid.err_m = 0;
    gory_pid.deter_output = 0;
    gory_pid.dead_bar = 0.06 * PI;
    gory_pid.output = 0;
    gory_pid.output_dead_bar = 20;
    gory_pid.Ud[0] = 0;gory_pid.Ud[1] = 0;
    gory_pid.output_last = 0;
    gory_pid.err_sum = 0;
    gory_pid.inc_err[0] = 0;
    gory_pid.inc_err[1] = 0;
    gory_pid.abs_err[0] = 0.6 * PI;
    gory_pid.abs_err[1] = 0.3 * PI;
    gory_pid.abs_err[2] = 0.05 * PI;
    
    turn_pid.Kp = kpidl_turn1[0];
    turn_pid.Ki = kpidl_turn1[1];
    turn_pid.Kd = kpidl_turn1[2];
    turn_pid.err[0] = 0;
    turn_pid.err[1] = 0;
    turn_pid.err[2] = 0;
    turn_pid.err_add = 0;
    turn_pid.err_m = 0;
    turn_pid.deter_output = 0;
    turn_pid.dead_bar = 0.06 * PI;
    turn_pid.output = 0;
    turn_pid.output_dead_bar = 20;
    turn_pid.Ud[0] = 0;
    turn_pid.Ud[1] = 0;
    turn_pid.output_last = 0;
    turn_pid.err_sum = 0;
    turn_pid.inc_err[0] = 0;
    turn_pid.inc_err[1] = 0;
    turn_pid.abs_err[0] = 0.6 * PI;
    turn_pid.abs_err[1] = 0.3 * PI;
    turn_pid.abs_err[2] = 0.05 * PI;
}
//------------------------------------------------------------------------------------------------------------------
// @brief      电机控制初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void Moter_init()
{
  for(int n=0;n<4;n++)
  {
//转载电机控制引脚数据
    moter[n].moter_timerch = moter_timerch[n];
    moter[n].moter_timer_pin = moter_timer_pin[n];
    moter[n].moter_pwmch_B = moter_pwmch_B[n];
    moter[n].moter_pwmch_A = moter_pwmch_A[n];
//初始化控制动态数据
    moter[n].moter_pwm = 0;
    moter[n].moter_encoder = 0;
    moter[n].moter_encoder_last = 0;
    moter[n].moter_now = 0;
    moter[n].moter_last = 0;
  }  
}
//------------------------------------------------------------------------------------------------------------------
// @brief      电机数据集初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void encoder_data_INIT()
{
  for(int n  = 0;n < 2;n++)
  {
    encoder_data[n].encoder_speed[0] = 0;
    encoder_data[n].encoder_speed[1] = 0;
    encoder_data[n].encoder_speed[2] = 0;
/*Abolished     No use*/  
    encoder_data[n].encoder_acc[0] = 0;
    encoder_data[n].encoder_acc[1] = 0;
/*Abolished     No use*/   
    encoder_data[n].encoder_jerk = 0;
  }
}
//void key_init()
//{
//gpio_init(B17, GPI, 1, GPIO_PIN_CONFIG);
//gpio_init(B16, GPI, 1, GPIO_PIN_CONFIG);
//gpio_init(B10, GPI, 1, GPIO_PIN_CONFIG);
//gpio_init(B21, GPI, 1, GPIO_PIN_CONFIG);
//gpio_init(B20, GPI, 1, GPIO_PIN_CONFIG);
//} 
extern int mmm[2];
extern int mmm1[2];
extern int mmm3[2];
extern int xwc;
int nb[2];
//extern uint16 pwm4;
//------------------------------------------------------------------------------------------------------------------
// @brief      全部初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void  All_Init()
{
  clock_init(SYSTEM_CLOCK_600M);  // 不可删除 
  debug_init();                   // 调试端口初始化 	        
  system_delay_ms(300);           //等待主板其他外设上电完成
//    scanf("%d\n",&pwm_a);
  //  scanf("%d\n",&mmm[0]);
  //  scanf("%d\n",&mmm1[0]);
  //  scanf("%d\n",&mmm3[0]);

  //  scanf("%d\n",&mmm[1]); 
  //  scanf("%d\n",&mmm1[1]);
  //  scanf("%d\n",&mmm3[1]);

  //  scanf("%f\n",&k24);
  //  scanf("%f\n",&k25);
  //  scanf("%f\n",&k26);

  //  scanf("%f\n",&k34);
  //  scanf("%f\n",&k35);
  //  scanf("%f\n",&k36);

 
  //  scanf("%d\n",&mmm[0]);
  //  scanf("%d\n",&mmm[1]); 
  //
  //  scanf("%d\n",&mmm1[0]);
  //  scanf("%d\n",&mmm1[1]);

  //  scanf("%d\n",&mmm3[0]);
  //  scanf("%d\n",&mmm3[1]);
  //  
  //  scanf("%f\n",&kpidl_gory[0]);
  //  scanf("%f\n",&kpidl_gory[1]);
  //  scanf
  //  scanf("%f\n",&kpidl_turn1[0]);
  //  scanf("%f\n",&kpidl_turn1[2]);
      
  //  scanf("%f\n",&kp_location_02[0]);
  //  scanf("%f\n",&ki_location_02[0]);
  //  scanf("%f\n",&kd_location_02[0]);

  //  scanf("%f\n",&kp_location_02[1]);
  //  scanf("%f\n",&ki_location_02[1]);
  //  scanf("%f\n",&kd_location_02[1]);
  
  //  scanf("%f\n",&kp_location_1[0]);
  //  scanf("%f\n",&ki_location_1[0]);
  //  scanf("%f\n",&kd_location_1[0]);
 
  //  scanf("%f\n",&kp_location_1[1]);
  //  scanf("%f\n",&ki_location_1[1]);
  //  scanf("%f\n",&kd_location_1[1]);

  key_gpio_init();//五向按键
  servo_init();     //舵机初始化
//  gpio_init(B15,GPO,0,GPIO_PIN_CONFIG);//补光灯
  system_delay_ms(100);
//moter_pwm_init();                                //电机PWM初始化  
//system_delay_ms(100);                            //等待上电完成
//dcs_init();
//key_init();
//ips114_init();
  openart_init ();                              //openart串口初始化 
  image_init();                                 //总钻风+TFT屏幕初始化
//SCANF("%d\n",&species);
//SCANF("%f\n",&x_list_B[0]);
//SCANF("%f\n",&y_list_B[0]);
  
  Queue_Init();                                 //加速度滤波初始化 
  dataBase_init();                              //数值初始化 
  Kinematics_Init();                            //麦克纳姆轮计算初始化
////gpio_init(B9,GPO,0,GPIO_PIN_CONFIG);            //B9指示灯初始化   可能会导致陀螺仪不正常
//gpio_init(B11,GPO,1,GPIO_PIN_CONFIG);
//adc_init(ADC_1,ADC1_CH3_B14,ADC_8BIT);           //ADC初始化
  encoder_data_INIT();                         //电机数据集初始化  
  moter_pwm_init();                             //电机PWM初始化
  system_delay_ms(200);  
  ///***Missing Debug Part***/                  
  pid_Init_A();                                 //PID初始化
  pid_Init_B();                                 //PID初始化
  wireless_uart_init();                         //无线串口初始化
  Moter_init();                                //电机动作初始化 
  encoder_ctrl(0);                              //启动编码器
  encoder_ctrl(2);                              //编码器置零
  //  bluetooth_send_init();                          				
  icm20602_init();                             //陀螺仪初始化
  gyroOffsetInit();
  dl1b_init();                                //tof模块初始化
  tim_setup();                                  //定时器初始化 
}
//}
/*******************************************************************************************************************/