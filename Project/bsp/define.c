/*******************************************************************************************************************/
/***Include***/
//{
#include "structure.h"
//}
/*******************************************************************************************************************/
/***Defines***/
//{
#define TARGET_tYPE float
#define N_1 20
#define N_2 6
#define N_3 3
//}
/*******************************************************************************************************************/
/********************Variate********************/
/***Constants***/
//{
//float x_list_A[12] = {0,-2,-2,2,2,2,-2,-2,2,2,-0.5,-0.5};
//float y_list_A[12] = {3,3,8,8,12,12,12,9,9,3,3,-0.7};
//float r_list_A[12] = {0,0,0,0,0,180,180,180,180,180,180,180};
//
//float x_list_B[16] = {3,11,19,23,15,20,11,20,0,2.1,-2.1,-2.1,2.1,2.1,-0.2,-0.2};
//float y_list_B[16] = {5,3,4,6,5,9,3,18,0,12.5,12.5,8,8,3,3,-0.3};
//float r_list_B[16] = {0,0,0,0,0,0,0,180,180,180,180,180,180,180,180,180};

/********************蚁群算法********************/
/**********************************************************************
bluetooth的变量 
************************************************************************/
uint8 location_x[32]={0};
uint8 location_y[32]={0};
uint8 type[32]={0};
uint8 one_bag_receive_flag = 0;// 解析标志位   1：正在解析数据   0：空闲



int location_state = 0;
/********************************************************************************
遍历变量
********************************************************************************/
uint8_t  complete_flag =0;//0为有框图片未找完，1为有框图片找完
uint8_t  direction=0;//0为向右，1为向左
uint8_t  discover_flag=0;//0未找到 1找到
uint8_t  direction_compare = 0;

uint8_t  monitor_uint8=0;

/********************************************************************************
串口数据及缓存区
********************************************************************************/
uint8 uart_get_data1[64];          // 串口接收数据缓冲区
uint8 fifo_get_data1[64];
uint8 fifo_openart2_rx_buffer;          // fifo 输出读出缓冲区
uint8 get_data1 = 0;               // 接收数据变量
uint32 fifo_data_count1 = 0;       // fifo 数据个数
uint8 gpio_status1;
fifo_struct uart_data_fifo1;

uint8 uart_get_data4[64];          // 串口接收数据缓冲区
uint8 fifo_get_data4[64] ;
uint8 fifo_openart1_rx_buffer;          // fifo 输出读出缓冲区
uint8 get_data4 = 0;               // 接收数据变量
uint32 fifo_data_count4 = 0;       // fifo 数据个数
uint8 gpio_status4;
fifo_struct uart_data_fifo4;

//uint8 uart_get_data8[64];          // 串口接收数据缓冲区
//uint8 fifo_get_data8[64] ;
//uint8 fifo_openart3_rx_buffer;          // fifo 输出读出缓冲区
//uint8 get_data8 = 0;               // 接收数据变量
//uint32 fifo_data_count8 = 0;       // fifo 数据个数
//uint8 gpio_status8;
//fifo_struct uart_data_fifo8;


//float x_list_B[14] = {0,-2,-2,2,2,-2,-2,0,0,2.5,-2.5,2.5,-0.4,-0.4};
//float y_list_B[14] = {3,3,8,8,12,12,17,17,17,14,10,7,3,-0.1};
//float r_list_B[14] = {0,0,0,0,0,0,0,0,180,180,180,180,180,180};
//电机死区电压转PWM
int32 dead_bar_positive[4] = {7096,6613,8913,8395};//STOP:3800 2657 3285 2988 ACTION:5706 3700 3713 4634
int32 dead_bar_negative[4] = {8541,8203,8725,8853};//STOP:4128 2561 3020 2497 ACTION:4939 4930 3797 3573
int32 dead_bar_positive_correct[4] = {3000,3000,3000,3000};//STOP:3800 2657 3285 2988 ACTION:5706 3700 3713 4634
int32 dead_bar_negative_correct[4] = {3000,3000,3000,3000};//STOP:4128 2561 3020 2497 ACTION:4939 4930 3797 35
//坐标寄存处
/*abolished！！！*/
float coordinate[2] = {0,0};  
//编码器定时器通道
pwm_channel_enum moter_pwmch_B[4] = {PWM1_MODULE1_CHA_D14,PWM1_MODULE0_CHA_D12,PWM1_MODULE3_CHB_D1,PWM2_MODULE3_CHB_D3};//副pwm引脚
pwm_channel_enum moter_pwmch_A[4] = {PWM1_MODULE1_CHB_D15,PWM1_MODULE0_CHB_D13,PWM1_MODULE3_CHA_D0,PWM2_MODULE3_CHA_D2};//主pwm引脚引脚

encoder_index_enum moter_timerch[4] = {QTIMER1_ENCODER1,QTIMER1_ENCODER2,QTIMER3_ENCODER2,QTIMER2_ENCODER1};////编码器定时器通道
encoder_channel1_enum moter_timer_pin[4] = {QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER2_CH1_C2,QTIMER3_ENCODER2_CH1_B18,QTIMER2_ENCODER1_CH1_C3};//编码器定时器引脚 
//坐标对列（测试用）
__attribute__((weak))float x_grid[10] = {0,0,0,0,0,0,0,0,0,0};
__attribute__((weak))float y_grid[10] = {0,0,0,0,0,0,0,0,0,0};
//横向纵向修正系数（测试用）
float liner_fixed_clone[3] = {0.45,0.45,0.45};
float liner_fixed_line[3] = {0.45,0.45,0.24};
//}
/***Structures***/
//{
//加速度滤波单元（测试用）
fliter_init acc_filter[2];
//速度滤波单元
fliter_init speed_filter[2];
//编码器滤波单元
fliter_init encoder_filter[4];
//摄像头误差滤波
fliter_init openart_filter[2];
//编码器数值存储单元
encoder_reader_init encoder_data[2];
//电机单元
moter_init moter[4];
//位置PID单元
pid_init location_pid[2];
pid_init location_pid_hard[2];

//速度PID单元
pid_init speed_pid[2];
//编码器PID单元（测试用）
pid_init encoder_pid[4];
//角度PID单元
pid_init angle_pid;
//角速度PID单元
pid_init gory_pid;
//转向环PID单元
pid_init turn_pid;
//加速度存储单元
ACC_init acc;
// 欧拉角
euler_param_t eulerAngle; 
// ICM20602采集的六轴数值
icm_param_t icm_data; 
//零漂误差
GyroOffset_init GyroOffset;       
//（假）陀螺仪数据存储单元
icm_param_t icm_data_static,icm_data_random;
//（假）欧拉角数据存储单元
euler_param_t eulerAngle_static;  
//}
/***Basic Var***/  
//{
float r_x,r_y;
float x,y;
//脉冲/米转换比
float pulse_per_meter = 0;
//麦克纳姆轮修正系数
float rx_plus_ry_cali = 0.3;
float angular_correction_factor = 1.0;
float linear_correction_factor = 1.0;
//两项速度
float dv_t_times_dt[2] = {0,0};
//四轮角速度
float dv_w_times_dt[4] = {0,0,0,0};
//加加速度输出（测试用）
float outputJ[2] = {0,0};
//加速度输出（测试用）
float outputA[2] = {0,0};
//速度输出
float outputV[2] = {0,0};
//*******距离输出(记步器)*************//
float outputL[2] = {0,0};
float outputjz[2] = {0,0};
float output_ramp[2] = {0,0};
float output_zebra[4] = {0,0,0,0};
float output_single[2] = {0,0};
float output_pick[2] = {0,0};
float output_start[2]={0,0};
float output_stop[2]={0,0};
float output_sc[2]={0,0};
float output_ycforward[2]={0,0};
//************************************//
//***************二级记步器**************************//
float wk_y=0;//矫正时，纵向移动的距离
float wk_x=0;//矫正时，横向移动的距离
float bl_x=0;//这一行遍历的距离
float zd_sc=-0.001;
float wd_sc=-0.01;
float sc_x=0;
float sc_y=0;
float jz_x=0;
float jz_y=0;
float wk_ys=0;//从一个元素放置区遍历到另一个放置区的记步器
float wk_zd=0;
float wk_zebra=0;
float wk_zebra_y=0;
float wk_zebraS_y=0;
float wk_zebraF_y=0;
float wk_single=0;
float wk_pick_y=0;
float ramp_x=0;
float ramp_y=0;
float yc_forward_y=0;
//***********************************************//
//编码器输出
float encoder[4] = {0,0,0,0};
//编码器脉冲目标值（测试用）
float encoder_target[4] = {0,0,0,0};
//麦克纳姆轮计算输入
float input[4] = {0,0,0,0};
//三项速度输出
float speed_output[3] = {0,0,0};
//滤波器数组
TARGET_tYPE value_buf_acc[2][N_1];
TARGET_tYPE value_buf_speed[2][N_2];
TARGET_tYPE value_buf_encoder[4][N_3];
TARGET_tYPE value_buf_openart[2][N_3];
//？？？
float encoder_sum_current[4] = {0,0,0,0};
//输出PWM总和
float move_sum;
// 四元数初始化（真）
quater_param_t Q_info = {1, 0, 0, 0};   
// 四元数初始化（假）
quater_param_t Q_info_static = {1,0,0,0};
//陀螺仪误差初始化
gyrocope_err_init gyrocope_err,gyrocope_err_static;
// 加速度计的收敛速率比例增益 0.17,0.004
float icm_kp= 0.17;         
// 陀螺仪收敛速率的积分增益
float icm_ki= 0.002;       
// 当前姿态计算得来的重力在三轴上的分量
float vx, vy, vz; 
// 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
float ex, ey, ez;       
//偏航角（测试用）
float yaw;
//（测试用）
float deter;
//矫正次数
int correct_times;
//dispaly
int m;
//坐标传递数组
float need_grid[2] = {0,0};
__attribute__((weak))float need_correct[2] = {0,0};
float need_angle = 0; 
//微路程
float aL[2] ={0,0};
//}

int init_flag=1;

float remember_p[16]={0};
float a_o[5]={1.1, 2.5, 3.6, 4.8, 5.6};
float angle_order=0;

uint8 big_species=0;
float angle_wing=0;

/********************PID数据集********************/
//{
//float kpidl_angle[4] = {0.04995,0.00027,0,400}; //0.0231   0.0517
float kpidl_angle[4] = {0.046*0.78,0.0005,0,400}; //0.0231   0.0517
//float kpidl_gory[4] = {2590.0,108.0,0,10000};
//float kpidl_gory[4] = {2500,72,0,10000};
float kpidl_gory[4] = {7000,400,0,10000};

//float kpidl_turn1[3] = {0.038,0,0.00};//0.018,0.0018    0.024. 0.003
//float kpidl_turn2[3] = {0.0470,0,0.0036};   
//float kpidl_turn3[3] = {0.0110,0,0.0010};

float kpidl_turn1[3] = {0.052,0,0.00};//0.038//0.043
float kpidl_turn2[3] = {0.042,0,0.0036};   
float kpidl_turn3[3] = {0.0110,0,0.0010};


float kp_speed[2] = {8000,7000};//600 6800 //前y后x
float ki_speed[2] = {160,150};  //25 245
float kd_speed[2] = {0,0};      //5000
float limit_V[2] = {100000,100000};//40

float kp_location_02[2] = {2,3};//3.2//前y后x
float ki_location_02[2] = {0.1528,0.003};//0.028,0.0014
float kd_location_02[2] = {0,0};//0.0033,0.0020

float kp_location_1[2] = {4.8 ,3};//3.2
float ki_location_1[2] = {0.4  ,0.003};//0.028,0.0014
float kd_location_1[2] = {60 ,0};//0.0033,0.0020

float kp_location_3[2] = {1.3 ,3.5};//3.2
float ki_location_3[2] = {0.01528,0.003};//0.028,0.0014
float kd_location_3[2] = {0,0};//0.0033,0.0020

float kp_location_5[2] = {1.23 ,3};//3.2{0.02558,0.02626};//0.028,0.0014
float ki_location_5[2] = {0.01528,0.003};//3.2{0.02558,0.02626};//0.028,0.0014{0.831494,0.847290};//3.2{0.02558,0.02626};//0.028,0.0014
float kd_location_5[2] = {0,0};//0.0033,0.0020

float kp_location_p5[2] = {1.23,3};//3.2{0.02558,0.02626};//0.028,0.0014
float ki_location_p5[2] = {0.01528,0.003};//3.2{0.02558,0.02626};//0.028,0.0014
float kd_location_p5[2] = {0,0};//0.0033,0.0020

float kp_location_p6[2] = {20};
float ki_location_p6[2] = {0};
float kd_location_p6[2] = {0};

float limit_S[2] = {200,200};

float kp_encoder[4] = {300,308,321,333};//没啥用
float ki_encoder[4] = {14,13,11,12};
float kd_encoder[4] = {235,252,180,200};
float limit_E[4] = {5000,5000,5000,5000};

float k24=0.32;
float k25=0.34;
float k26=0.30;

float k34=0.155;
float k35=0.160;
float k36=0.180;

//}
/********************Flags********************/
//{
//for循环计数
int i = 0;    
//小车状态定义
__attribute__((weak))volatile int smartcar_state = 0;     
//PID限幅标志（测试用）
volatile bool bar_flag[3] = {0,0,0};   
//dispaly.c中的标志位
int32 flag = 0;
//PID计数标志位
volatile int flag_pid = 0;
//位置环PID计数标志位
volatile int flag_L_pid[2] = {0,0};        
//PID动作标志位
volatile int pid_fucntion_flag = 1;                   
//volatile float pid_counter[2][4] = {{0,0,0,0},{0,0,0,0}};
//PID监视单元
volatile int moniter[2] = {0,0};
//PID是否继续标志位
volatile bool roll_next = 1;
int cp[2] = {0,0};
//自矫正标志位
volatile int self_correct = 0;
volatile int self_correct_last = 0;
int abc = 0;
//位置环强制停止百分比取值
volatile float close_pid_var[2] = {1.00,1.00};

//监视
volatile int compulsion_monitor = 0;
volatile int location_monitor = 0;

//volatile int xx = 0;
//角度PID动作标志位
volatile int angle_function = 0;
volatile bool angle_function_act = 0;
//强制停止标志位
volatile bool forced_stop = 0;
//舵机动作标志位
volatile bool stering_start = 0;
//前进标志位
bool go_ahead;
//PID矫正调节时间
volatile int flag_pid_n = 1;
//openart工作模式
volatile int openart_mode = 0;
volatile int openart_mode_1 = 0;
//小车矫正状态判断
volatile int correct_state = 0;
volatile int remove_state = 0;
volatile int back_state = 0;

volatile float Reduction_yaw = 0;
float mut[2] = {1,1};
//float a[2]={13040,14295};//10647.5    1m
//float a[2]={11736,12866};             2m

float a[2]={12100,12526.47}; 

////  左环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6        
//float x_list_L[16] ={  0,      0,      0,       4,      5,      5,       5,      3.5,     0,     0,};
//float y_list_L[16] ={  0,      1,      0,       6,     -2,     -2,     -1.5,    -8,       0,     2,};
//float r_list_L[16] ={  90,     90,    90,      180,    120,     55,      0,      0,       0,    90,    0,180,180,180};

////  右环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
//float x_list_R[16] ={  0,      0,       0,      3,        3,       3,      3,     -2,      1,       0,};
//float y_list_R[16] ={  0,      0,       0,      3,        0,       0,      0,      2,      0,       2,};
//float r_list_R[16] ={ -90,    -90,     -90,     90,       38,     -38,    -90,     90,     90,      90,    0,180,180,180};

////  斑马线              0        1         2          
//float x_list_B[16] ={  5,       5,        5     };
//float y_list_B[16] ={  0,      -1.5,     -1.5   };
//float r_list_B[16] ={ -90,     -90,      -90    };



////  左环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6        
//float x_list_L[16] ={  0,      0,       0,      0,      0,      0,       0,      0,       0,        0,};
//float y_list_L[16] ={  0,      0.3,     0,      0,      0,      0,       0,      0,       3.5,      0,};
//float r_list_L[16] ={  90,     90,      90,     180,    117,    55,      0,      -90,     -360,     0,};
//
////  右环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
//float x_list_R[16] ={  0,      0,        0,     0,      0,      0,       0,      0,       0,      0,};
//float y_list_R[16] ={  0,      0.3,      0,     0,      0,      0,       0,      0,       3.5,    0,};
//float r_list_R[16] ={ -90,    -90,     -90,     0,    -52,     -122,    -180,    -270,   -360,    0,};
//
////  左十字              0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
//float x_list_LC[16] ={  0,      0,      0,       0,      0,      0,       0,      0,       3.5,     0,};
//float y_list_LC[16] ={  0,      0.3,      0,       0,      0,      0,       0,      0,      0,     0,};
//float r_list_LC[16] ={  90,     90,     90,      150,    108,     42,      -15,    -77,    -90,    0,};
//
////  右十字              0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
//float x_list_RC[16] ={  0,       0,       0,      0,      0,      0,       0,      0,       0,     0,};
//float y_list_RC[16] ={  0,       0.3,       0,      0,      0,      0,       0,      0,      4,     0,};
//float r_list_RC[16] ={  -90,     -90,     -90,      65,     23,     -43,     -103,    -163,   -270,    0,};
//
////  斑马线              0        1         2        3
//float x_list_B[16] ={  0,       0,      0 ,      0 };
//float y_list_B[16] ={  0,       0,      0 ,      0 };
//float r_list_B[16] ={ -90,     -90,     -90,     -360 };
//
//
////  斑马线              0        1         2        3
//float x_list_B_L[16] ={  0,       0,      0 ,      0 };
//float y_list_B_L[16] ={  0,       0,      0 ,      0 };
//float r_list_B_L[16] ={ 90,       90,     90,      0 };


//  左环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6        
float x_list_L[16] ={  0,      0,       0,      0,      0,      0,       0,      0,       0,        0,};
float y_list_L[16] ={  0,      0.3,     0,      0,      0,      0,       0,      0,       0,      0,};
float r_list_L[16] ={  90,     90,      90,     178,    119,    53,      -4,      -94,     0,     0,};//注意5号位只要4号位-90就行了

//  右环               0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
float x_list_R[16] ={  0,      0,        0,     0,      0,      0,       0,      0,       0,      0,};
float y_list_R[16] ={  0,      0.3,      0,     0,      0,      0,       0,      0,       0,    0,};
float r_list_R[16] ={ -90,    -90,     -90,     -1,    -47,     -124,    -179,   -269,   -359,    0,};//注意5、6号位只要4号位-90就行了（4号位与5号位垂直，5号位与6号位垂直），不要量！

//  左十字              0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
float x_list_LC[16] ={  0,      0,      0,       0,      0,      0,       0,      0,       2.7,     0,};
float y_list_LC[16] ={  0,      0.3,      0,       0,      0,      0,       0,      0,      0,     0,};
float r_list_LC[16] ={  90,     90,     90,      160,    104,     44,      -17,    -77,    -90,    0,};

//  右十字              0        1       2       3/1     4/2     5/3      6/4     7/5      8/6       
float x_list_RC[16] ={  0,       0,       0,      0,      0,      0,       0,      0,       0,     0,};
float y_list_RC[16] ={  0,       0.3,       0,      0,      0,      0,       0,      0,      3.5,     0,};
float r_list_RC[16] ={  -90,     -90,     -90,      73,     14,     -53,     -111,    -164,   -270,    0,};//注意6号位，5号位-90即可

//  斑马线              0        1         2        3
float x_list_B[16] ={  0,       0,      0 ,      0 };
float y_list_B[16] ={  0,       0,      0 ,      0 };
float r_list_B[16] ={ -90,     -90,     -90,     -360 };


//  斑马线              0        1         2        3
float x_list_B_L[16] ={  0,       0,      0 ,      0 };
float y_list_B_L[16] ={  0,       0,      0 ,      0 };
float r_list_B_L[16] ={ 90,       90,     90,      0 };



//float x_list_B[16] ={ 20,    20,     1,     1,      20,      16,   3,0,    32,   32,  22,  16,   10,  9,  5, 2};
//float y_list_B[16] = { 0,     3,    3,      6,      6,       19 ,  3,0,     3,   13,  12,  12,   10, 14, 12, 7};
//float r_list_B[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,180,180,180};

int kk=0;
int kkk = 0;//这个一般是小车遇到问题把这个作为标志量打印出来查找问题出处的
int kkkk = 0;//这个一般是小车遇到问题把这个作为标志量打印出来查找问题出处的
int start_flag=0;//进入矫正环标志位
int start_flag_2 = 0;
int flag_pid_error=0;//误差接收完毕标志位
float temp_openart1[70];//储存蚁群算法输出坐标
uint8 picture_flag1 = 0;//到达位置后未识别到图片标志位
int time_flag1=0;//矫正超时标志位


uint8 species_picture=0;
uint8 species_picture_last=0;
uint8 species_alphabet=0;
uint8 small_count=0;
uint8 species_cs=0;

uint8 wait_identify_picture_flag=0;//等待识别完毕标志位
uint8 wait_identify_letter_flag=0;//等待识别完毕标志位
uint8 delay_complete_flag=0;//定时循环标志位
int  delay_temp=0;//延时参数run_distance暂时储存
uint8 no_picture = 0;//遍历未检测出图片标志位
int abb=0;
int bbc=0;
float run_distance=2.5;//遍历距离 
uint8  yellow_flag=0; //识别出黄线标志位
uint8  Throw_area=1;//扔图片位置标志位//分别对应前，右，后，左
uint8 hk_distance[2]={1,0};//接收回库横向偏移量 
uint8 huiku_flag=0;////回库完成标志位
uint8 Return_brake_flag=0;
uint8 bianli_over=0;//遍历结束标志位
uint8 check_yellow_commencement=0;
uint8 bianli_cs=0;
uint8 wk_flag=0;
uint8 openart1_jz_flag=0;
uint8 openart2_jz_flag=0;


float each_speed[4]={0};
uint8 check = 0;
volatile int bcc=0;
uint8 goon_flag=0;
uint8 goon_flag2=0;
uint8 goon_flag3=0;
float proportion=40.0;
float pwm4=180;
uint8 start_circle=1;
uint8 zebra_circle=0;
uint8 zebra_findflag_start=1;

uint8 INCREASE=0;
float Initial_yaw=0;
float carmer_yaw=-90;
float Final_yaw=0;
float bug=0;

volatile int ture_=1;
volatile int false_=1;
uint8 my_state=0;
uint8 my_state_y=0;



uint16 sc=0;
uint8 element_state=0;//元素状态  //左环1，右环2，十字3，斑马线4
uint8 element_flag=0;//元素识别标志位
uint8 pick_complete=0;//捡完元素图片标志位
uint8 locat_state=0;
uint8 left_action=1;
uint8 right_action=1;
uint8 diuxian_left=0;
uint8 diuxian_right=0;
uint8 yc1=0;
uint8 yc2=3;//放置区计数器
uint8 yc3=0;
int mm=0;
uint8 nnnn=0;
float y_speed1=0;
int remember_i=0;//记忆集中照片拾取顺序
float jz_inc_err[2]={0};
int jz_success=0;
uint8 cross_switch=0;//1为左十字2为右十字
int angle_monitor=0;
uint8 prep=0;
uint8 key_num=0;
uint16 pwm_a=0;
uint8 sprint=0;
uint8 speed_flag=0;


uint8 zebra_straight=0;  //之后需分配一个按键给此

uint8 ys_angle=0;

uint8 ramp_kz=0;
uint8 ramp_lowspeed=0;
float ramp_switch=0.8;

uint8 yb_flag=0;
uint8 finsh_rc_letter_flag=0;
uint8 lz_flag=0;//连抓标志位
//uint8 zd_sc=65;
uint8 wd_sc_time=75;

float ec12=0;
float ec13=0;
float ec14=0;
float ec23=0;
float ecj12=0;
float ecj34=0;
uint8 back_find=0;
uint16 my_monitor1=0;
uint16 sc_monitor2=0;
uint8 skip_pick=0;
uint8 no_picture_flag=0;
uint8 tofa_flag=0;//识别坡道标志位
uint8 nosc_flag=0;
uint8 zebra_switch=0;
float zebra_speed=0.66;//遍历斑马线放置区速度
uint8 zebra_commutation_flag=0;//转身遍历另一边斑马线标志位
float zebra_forward=0.03;
uint8 break_out_flag=0;
uint8 dispersive_picture=0;
uint8 dispersive_angle=0;
uint8 dispersive_position=0;
uint8 classify_error=0;
uint8 tofa_per_flag=0;
uint16 kkkkkkk=0;
uint8 wd_flag=0;
uint8 art_02_flag=0;
uint8 yc_angle_test=0;
uint8 spin_finish=0;//完成旋转至正对元素标志位
uint8 carry_count=0;//元素内捡照片计数器
uint8 fake_picture=0;
uint8 dd=0;
uint8 yuzhi=0;
float foward_distanse=0.04;
uint8 dd_fre=75;
uint8 sc_finish=0;
uint8 sc_flag=0;
uint8 hk_nwj=2;//1为外径30（小框）2为内径30（大框）
uint8 e_p_flag=0;//进入遍历放置区状态标志位
uint8 clear_tft=1;
uint8 ycsf_flag=0;
uint8 kl_distanse=19;//(小框)//小框23大框20
uint8 art_begin=0;//opemart1开启标志位
uint8 art_begin_count=0;//art1开机计数器
uint8 art2_begin_count=0;//art2开机计数器

uint8 mid_flag1=0;
uint8 mid_flag2=0;
double ka5_cross;
double kb5_cross;
double ka6_cross;
double kb6_cross;
float k_left;
float k_right;
uint8 zebra_start=0;
uint8 zebra_jibu=0;
uint8 Single_line=0;
uint8 single_flag=0;
uint8 stop_flag=0;
uint8 speed_falg1=0;
uint8 nono_pic_falg;
uint8 single_continue=0;
uint8 tofa_start=1;
uint8 barrier_start=1;


//}
/*******************************************************************************************************************/