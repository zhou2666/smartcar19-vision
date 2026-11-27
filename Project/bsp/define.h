#ifndef _DEFINE_H
#define _DEFINE_H
/*******************************************************************************************************************/
/***Includes***/
//{
#include "zf_common_headfile.h"
#include "structure.h"
//}
/*******************************************************************************************************************/
/***Defines***/
//{
#define BLUETOOTH_CH9141_INDEX                  (UART_8)                        // 蓝牙模块对应使用的串口号
#define BLUETOOTH_CH9141_BUAD_RATE              (115200)                        // 蓝牙模块对应使用的串口波特率
#define BLUETOOTH_CH9141_TX_PIN                 (UART8_RX_D17)                  // 蓝牙模块对应模块的 TX 要接到单片机的 RX
#define BLUETOOTH_CH9141_RX_PIN                 (UART8_TX_D16)                  // 蓝牙模块对应模块的 RX 要接到单片机的 TX
#define BLUETOOTH_CH9141_RTS_PIN                (D26)                           // 蓝牙模块对应模块的 RTS 引脚

#define BLUETOOTH_CH9141_BUFFER_SIZE            (64)                            // 默认的蓝牙模块使用的接收缓冲区大小
#define BLUETOOTH_CH9141_TIMEOUT_COUNT          (500) 


#ifndef TARGET_tYPE
#define TARGET_tYPE float
#endif
#ifndef N_1
#define N_1 6
#endif
#ifndef N_2
#define N_2 6
#endif
#ifndef N_3
#define N_3 3
#endif
#define PWM_S_MIN 1250     
#define PWM_S_MAX 6250  
#define PWM_MAX 34000                   //最大PWM限幅
#define PID_RATE 166.666f                    //PID调节PWM值的频率
#define T_PID   6
#define ENCODER_RESOLUTION 2297        //编码器分辨率, 轮子转一圈，编码器产生的脉冲数//2555
#define WHEEL_DIAMETER 0.063            //轮子直径,单位：米
#define D_X  0.201                      //底盘Y轴上两轮中心的间距
#define D_Y 0.178                       //底盘X轴上两轮中心的间距
#define Times_Max_A 2                    //最大预定刹车制动次数
#define Times_Max_B 3      
#define Times_Max_D 300                    //最大预定刹车制动次数//在这改吧 400
 
#define delta_T     0.006f               // 陀螺仪采样周期1ms 即频率1KHZ
#define G (9.80665f)                    //重力加速度
#define Fmax 3.0                       //进给速度   2
#define Amax 6.0                       //最大加速度
#define Sref 1.5                       //位置标段 //Notice :Sref = Fmax*Fmax/Amax
#define F_acc 1
#define F_speed 2
#define F_encoder 3
#define F_openart 4
#define d_speed 3.0f


#define RL_fix     1.00f
#define FB_fix     0.8f
#define C_ALL     openart1.correct_posture[1]
#define C_NULL    openart1.correct_posture[0]
#define degreeA_Init 141
#define degreeB_Init 133
#define degreeB_Catch 10
#define degreeB_Catch_back 107
#define degreeC_Init 175
#define degreeC_Catch 258//263
#define degreeC_Back_pull 58
#define STATIC_ACC_X    -0.112979612f
#define STATIC_ACC_Y    -0.009641198f
#define STATIC_ACC_Z    0.991866235f
#define STATIC_GYRO_X   0.000657963f
#define STATIC_GYRO_Y   0.002055521f
#define STATIC_GYRO_Z   -0.002366952f
/********************openart_mode********************/
#define Discern 0
#define Start 0x01
#define Correct 0x02

#define element_picture 0x03//矫正元素内照片
#define element_letter 0x04//矫正元素周围字母
#define zebra_figure 0x05//矫正斑马线后数字
#define clear 0x06

#define first_search 0x07 //第一周帧看到图片
//#define correct_wd 0x08
#define goon_Find_frameless 0x09

 

//}
/*******************************************************************************************************************/
/***Externs***/
/********************Variate********************/
/***Constants***/
//{ 
extern float x_list_A[11];
extern float y_list_A[11];
extern float r_list_A[11];

extern float x_list_B[16];
extern float y_list_B[16];
extern float r_list_B[16];

extern float x_list_B_L[16];
extern float y_list_B_L[16];
extern float r_list_B_L[16];

extern float x_list_R[16];
extern float y_list_R[16];
extern float r_list_R[16];

extern float x_list_L[16];
extern float y_list_L[16];
extern float r_list_L[16];

extern float x_list_LC[16];
extern float y_list_LC[16];
extern float r_list_LC[16];

extern float x_list_RC[16];
extern float y_list_RC[16];
extern float r_list_RC[16];

extern float x_list_ramp_R[16];
extern float y_list_ramp_R[16] ;
extern float r_list_ramp_R[16];


extern int32 dead_bar_positive_correct[4];
extern int32 dead_bar_negative_correct[4];
extern int32 dead_bar_positive[4];
extern int32 dead_bar_negative[4];
extern float coordinate[2];  

extern encoder_index_enum moter_timerch[4];////编码器定时器通道
extern encoder_channel1_enum moter_timer_pin[4];//编码器定时器引脚  

extern pwm_channel_enum moter_pwmch_B[4];//副pwm引脚
extern pwm_channel_enum moter_pwmch_A[4];//主pwm引脚引脚
extern float x_grid[10];
extern float y_grid[10];
extern float liner_fixed_line[3];
extern float liner_fixed_clone[3];
//}
/***Structures***/
//{ 
extern fliter_init acc_filter[2];
extern fliter_init speed_filter[2];
extern fliter_init encoder_filter[4];
extern fliter_init openart_filter[2];
extern encoder_reader_init encoder_data[2];
extern moter_init moter[4];
extern pid_init location_pid[2];
extern pid_init location_pid_hard[2];

extern pid_init speed_pid[2];
extern pid_init encoder_pid[4];
extern pid_init angle_pid;
extern pid_init gory_pid;
extern pid_init turn_pid;
extern ACC_init acc;
extern quater_param_t Q_info;
extern quater_param_t Q_info_static;
extern euler_param_t eulerAngle;              // 欧拉角
extern icm_param_t icm_data;                  // ICM20602采集的六轴数值
extern GyroOffset_init GyroOffset;            //零漂误差
extern icm_param_t icm_data_static,icm_data_random;
extern euler_param_t eulerAngle_static; 
// extern Ant ant[30];
//}
/***Basic Var***/  
//{
extern int location_state;
extern float r_x,r_y;
extern float x,y;
extern float pulse_per_meter;
extern float rx_plus_ry_cali;
extern float angular_correction_factor;
extern float linear_correction_factor;
extern float dv_t_times_dt[2];
extern float dv_w_times_dt[4];
extern float outputJ[2];
extern float outputA[2];
extern float outputV[2];
extern float outputL[2];
extern float outputjz[2];
extern float output_ramp[2];
extern float output_zebra[4];
extern float output_single[2];
extern float output_pick[2];
extern float output_start[2];
extern float output_stop[2];
extern float output_sc[2];
extern float output_ycforward[2];

extern float encoder[4];
extern float encoder_target[4];
extern TARGET_tYPE value_buf_acc[2][N_1];
extern TARGET_tYPE value_buf_speed[2][N_2];
extern TARGET_tYPE value_buf_encoder[4][N_3];
extern TARGET_tYPE value_buf_openart[4][N_3];
extern float input[4];
extern float speed_output[3];
extern float P_output_V_target[2];
extern float encoder_sum_current[4];
extern float move_sum;
extern float need_grid[2];
extern float need_correct[2];
extern float aL[2];
extern bool go_ahead;
extern volatile int flag_pid_n;
extern volatile int openart_mode;
extern volatile int openart_mode_1;
extern volatile int correct_state;
extern volatile int remove_state;
extern volatile int back_state;
extern volatile float close_pid_var[2];
extern volatile float Reduction_yaw;

extern float need_angle;  
//}
/********************PIDs********************/
//{
extern float kpidl_angle[4];
extern float kpidl_gory[4];

extern float kpidl_turn1[3];
extern float kpidl_turn1[3];
extern float kpidl_turn2[3];

extern float kp_speed[2];
extern float ki_speed[2];
extern float kd_speed[2];
extern float limit_V[2];

extern float kp_location_02[2];
extern float ki_location_02[2];
extern float kd_location_02[2];

extern float kp_location_1[2];
extern float ki_location_1[2];
extern float kd_location_1[2];

extern float kp_location_3[2];
extern float ki_location_3[2];
extern float kd_location_3[2];

extern float kp_location_5[2];
extern float ki_location_5[2];
extern float kd_location_5[2];

extern float kp_location_p5[2];
extern float ki_location_p5[2];
extern float kd_location_p5[2];

extern float kp_location_06[2];
extern float ki_location_06[2];
extern float kd_location_06[2];

extern float kp_location_p6[2];
extern float ki_location_p6[2];
extern float kd_location_p6[2];
extern float limit_S[2];

extern float kp_encoder[4];
extern float ki_encoder[4];
extern float kd_encoder[4];
extern float limit_E[4];
//}
/**********Gyroscopes***************/
//{
extern gyrocope_err_init gyrocope_err,gyrocope_err_static;
extern float icm_kp;                                    // 加速度计的收敛速率比例增益 0.17,0.004
extern float icm_ki;                                    // 陀螺仪收敛速率的积分增益
extern float vx, vy, vz;                               // 当前姿态计算得来的重力在三轴上的分量
extern float ex, ey, ez;                               // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
extern float yaw;
extern float deter;
//}
/********************Flags********************/
//{
extern int i;                                              //for循环计数
extern volatile int  smartcar_state;                             //弱小车状态定义(防止合代码重定义)
extern volatile bool bar_flag[3];                                //PID限幅标志
extern int32 flag;
extern volatile int flag_pid;
extern volatile int flag_L_pid[2];
extern volatile int pid_fucntion_flag;
//extern float pid_counter[2][4];
extern volatile int moniter[2];
extern volatile bool roll_next;
extern int cp[2];
extern volatile int self_correct;
extern volatile int self_correct_last;
extern int abc;
//extern volatile int xx;
extern volatile int compulsion_monitor;
extern volatile int location_monitor;

extern volatile int angle_function;
extern bool angle_function_act;
extern volatile bool forced_stop;
extern volatile bool stering_start;
extern float mut[2];
extern float a[2];
extern int kkkk;
extern int kkk;
extern int kk;


extern uint8 species_picture;
extern uint8 species_picture_last;
extern uint8 species_alphabet;
extern uint8 small_count;
extern uint8 species_cs;


extern int bcc;
extern float pwm4;

extern uint8 start_circle;
extern uint8 zebra_circle;
extern uint8 zebra_findflag_start;
extern uint8 zebra_jibu;
extern uint8 INCREASE;

//}
/**********蚁群算法***************/
//{
extern int start_flag;
extern int start_flag_2;
extern int flag_pid_error;
extern float temp_openart1[70];
//}
extern uint8 picture_flag1;
extern int time_flag1;

extern uint8 delay_complete_flag;   
extern uint8 wait_identify_picture_flag; 
extern uint8 wait_identify_letter_flag;
extern  int  delay_temp; 
extern int init_flag;

      
extern int abb;
extern int bbc;

extern uint8  Throw_area ;
extern uint8  yellow_flag ;
/**********************************************************************
bluetooth的变量 
************************************************************************/
extern uint8 location_x[32];
extern uint8 location_y[32];
extern uint8 type[32];
extern uint8 one_bag_receive_flag ;
/*******************************************************************************************************************/
 /********************************************************************************
遍历变量
********************************************************************************/
extern uint8_t complete_flag;
extern uint8_t direction;
extern uint8_t discover_flag;
extern uint8_t direction_compare; 
 
extern uint8_t  monitor_uint8;
extern uint8 wk_flag;
extern uint8 openart1_jz_flag;
extern uint8 openart2_jz_flag;
extern uint8 bianli_cs;
extern float wk_y;
extern float wk_x;
extern float bl_x;
extern float yc_forward_y;

extern uint8 goon_flag;

extern uint8 goon_flag2;

extern uint8 goon_flag3;

extern uint8 no_picture;

extern float remember_p[16];
extern float a_o[5];
extern float angle_order;
extern uint8 big_species;
extern float angle_wing;
/********************************************************************************
串口数据及缓存区
********************************************************************************/
extern uint8 uart_get_data1[64];          // 串口接收数据缓冲区
extern uint8 fifo_get_data1[64];           // fifo 输出读出缓冲区
extern uint8 fifo_openart2_rx_buffer; // fifo 输出读出缓冲区
extern uint8 get_data1;                  // 接收数据变量
extern uint32 fifo_data_count1;          // fifo 数据个数
extern fifo_struct uart_data_fifo1;
extern uint8 gpio_status1;

extern uint8 uart_get_data4[64];          // 串口接收数据缓冲区
extern uint8 fifo_get_data4[64];          // fifo 输出读出缓冲区
extern uint8 fifo_openart1_rx_buffer;   // fifo 输出读出缓冲区
extern uint8 get_data4;                  // 接收数据变量
extern uint32 fifo_data_count4;          // fifo 数据个数
extern fifo_struct uart_data_fifo4;
extern uint8 gpio_status4;

//extern uint8 uart_get_data8[64];          // 串口接收数据缓冲区
//extern uint8 fifo_get_data8[64];          // fifo 输出读出缓冲区
//extern uint8 fifo_openart1_rx_buffer;   // fifo 输出读出缓冲区
//extern uint8 get_data8;                  // 接收数据变量
//extern uint32 fifo_data_count8;          // fifo 数据个数
//extern fifo_struct uart_data_fifo8;
//extern uint8 gpio_status8;




/*******************************************************************************************************************/
 /********************************************************************************
回库变量
********************************************************************************/
 extern uint8 huiku_flag;
extern uint8 hk_distance[2];
 extern uint8 Return_brake_flag;
 extern uint8 bianli_over;
 extern  float run_distance;
 extern uint8 check_yellow_commencement;
extern float each_speed[4];
extern uint8 check;
extern float proportion;
//============================================
extern float Initial_yaw;
extern float carmer_yaw;
extern float Final_yaw;
extern float bug;
extern int ture_;
extern int false_;
extern uint8 my_state;
extern uint8 my_state_y;


extern float k34;
extern float k35;
extern float k36;

extern float k24;
extern float k25;
extern float k26;

extern uint16 sc;
extern uint8 element_state;  //元素状态  //左环1，右环2，十字3，斑马线4
extern uint8 element_flag;
extern uint8 pick_complete;
extern uint8 locat_state;
extern uint8 left_action;
extern uint8 right_action;
extern uint8 diuxian_left;
extern uint8 diuxian_right;

extern uint8 yc1;
extern uint8 yc2;
extern uint8 yc3;
extern int mm;
extern float y_speed1;
extern int remember_i;
extern float jz_inc_err[2];
extern int jz_success;
extern uint8 cross_switch;
extern int angle_monitor;
extern uint8 prep;
extern uint8 key_num;
extern uint16 pwm_a;
extern uint8 sprint;
extern uint8 speed_flag;

extern uint8 mid_flag1;
extern uint8 mid_flag2;
extern double ka5_cross;
extern double kb5_cross;
extern double ka6_cross;
extern double kb6_cross;
extern float k_left;
extern float k_right;
extern uint8 zebra_straight;

extern uint8 ys_angle;
extern float wk_ys;
extern float wk_zd;
extern float wk_zebra;
extern float wk_zebra_y;
extern float wk_zebraF_y;
extern float wk_zebraS_y;

extern float wk_single;
extern float wk_pick_y;

extern float ramp_x;
extern float ramp_y;
extern uint8 ramp_kz;
extern uint8 ramp_lowspeed;
extern float ramp_switch;
extern float jz_x;
extern float jz_y;
extern uint8 yb_flag;
extern uint8 finsh_rc_letter_flag;
extern uint8 lz_flag;
//extern uint8 zd_sc;
extern uint8 wd_sc_time;
extern float zd_sc;
extern float wd_sc;
extern float sc_x;
extern float sc_y;
extern float ec12;
extern float ec13;
extern float ec14;
extern float ec23;
extern float ecj12;
extern float ecj34;
extern uint8 back_find;
extern uint16 my_monitor1;
extern uint16 sc_monitor2;
extern uint8 skip_pick;
extern uint8 no_picture_flag;
extern uint8 tofa_flag;
extern uint8 nnnn;
extern uint8 nosc_flag;
extern uint8 zebra_switch;
extern float zebra_speed;
extern uint8 zebra_commutation_flag;
extern float zebra_forward;
extern uint8 break_out_flag;
extern uint8 dispersive_picture;
extern uint8 dispersive_angle;
extern uint8 dispersive_position;
extern uint8 classify_error;
extern uint8 tofa_per_flag;
extern uint16 kkkkkkk;
extern uint8 classify_error;
extern uint8 wd_flag;
extern uint8 zebra_start;
extern uint8 Single_line;
extern uint8 single_flag;
extern uint8 art_02_flag;
extern uint8 yc_angle_test;
extern uint8 spin_finish;
extern uint8 carry_count;
extern uint8 fake_picture;
extern uint8 stop_flag;
extern uint8 dd;
extern uint8 yuzhi;
extern float foward_distanse;
extern uint8 dd_fre;
extern uint8 sc_finish;
extern uint8 sc_flag;
extern uint8 hk_nwj;
extern uint8 e_p_flag;
extern uint8 tofa_start;
extern uint8 clear_tft;
extern uint8 barrier_start;
extern uint8 single_continue;
extern uint8 ycsf_flag;
extern uint8 kl_distanse;
extern uint8 art_begin;
extern uint8 art_begin_count;
extern uint8 art2_begin_count;
extern uint8 speed_falg1;
extern uint8 nono_pic_falg;
#endif
