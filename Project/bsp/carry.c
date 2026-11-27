/*******************************************************************************************************************/
/***Include***/
//{
/*******************************************************************************************************************/
/***Include***/
//{
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"
#include "route_planning.h"
//}
/*******************************************************************************************************************/
/***Functions***/
//{

// **************************** 代码区域 ****************************
#define SERVO_MOTOR_PWM1                (PWM2_MODULE2_CHA_C10)                      // 定义主板上舵机对应引脚  机械臂下舵机（双轴270）
#define SERVO_MOTOR_PWM2                (PWM2_MODULE2_CHB_C11)                      // 定义主板上舵机对应引脚  机械臂上舵机（单轴270）
#define SERVO_MOTOR_PWM4                (PWM2_MODULE1_CHB_C9)                       // 定义主板上舵机对应引脚  机械臂中舵机（双轴270）

#define SERVO_MOTOR_PWM3                (PWM2_MODULE0_CHB_C7)                       // 定义主板上舵机对应引脚  收纳盒舵机（单轴360）
#define SERVO_MOTOR_PWM5                (PWM2_MODULE1_CHA_C8)                       // 定义主板上舵机对应引脚 
//////#define SERVO_MOTOR_PWM6                (PWM2_MODULE0_CHA_C6)                       // 定义主板上舵机对应引脚 

#define SERVO_MOTOR_FREQ                (300)                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define pwm1  154 //100
#define pwm4  53 //100
#define pwm2  135//170

#define pwm3   3

#define pwm5   60


// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))    //舵机角度设置为0 - 180，90度为中值
#define SERVO_MOTOR_DUTY1(x)        ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/180.0))   //舵机角度设置为0 - 360，180度为中值
#define SERVO_MOTOR_DUTY270(x)        ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/135.0))   //舵机角度设置为0 -270，135度为中值
#define SERVO_MOTOR_DUTY_mn(x)      ((float)PWM_DUTY_MAX/(1000.0/(float)50)*(0.5+(float)(x)/90.0))                 //舵机角度设置为0 - 180，90度为中值
static uint16 servo1_duty = 0, servo2_duty = 0,servo3_duty = 3,servo4_duty = 0,servo5_duty = 0;

int xwc=0;
extern int mmm1[2];
void servo_init() //初始角度
{
  pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(52));//下
  pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY270(210));//中
  pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY270(145));//上
  
  
  pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY1(85));//盒   //5  45 88 140 190 230 270  318
                                                                          
  
  pwm_init(SERVO_MOTOR_PWM5, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(146));//门

  gpio_init(C12,GPO,0,GPIO_PIN_CONFIG);
  servo1_duty = pwm1;servo2_duty = pwm2; servo3_duty = pwm3;servo4_duty = 0;servo5_duty = 0;
}
void PWM1_ctrl(uint16 _servo1_angle  ,float _step_count) //舵机控制代码 前一个参数为角度 后一个参数用来控制舵机速度 值越大 速度越慢
{
  float servo1_start = (float)servo1_duty;
  float servo1_step = (float)(_servo1_angle - servo1_duty)/_step_count;
  while(1)
  {
    if(fabsf(servo1_start - (float)_servo1_angle) >= servo1_step)servo1_start += servo1_step;
    else servo1_start = _servo1_angle;
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)servo1_start));
    if(fabsf(servo1_start - (float)_servo1_angle) < 1 )
    {
      servo1_duty = (uint16)_servo1_angle;
      break;
    }
  }
}

void PWM2_ctrl(uint16 _servo2_angle , float _step_count)//同上
{
  float  servo2_start = (float)servo2_duty;
  float  servo2_step = (float)(_servo2_angle - servo2_duty)/_step_count;
  
  while(1)
  {
    if(fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
    else servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY270((uint16)servo2_start));
    
    if(fabsf(servo2_start - (float)_servo2_angle) < 1)
    {
      servo2_duty = (uint16)_servo2_angle;
      break;
    }
  }
}


void PWM3_ctrl(uint16 _servo2_angle , float _step_count) //同上
{
  float  servo2_start = (float)servo3_duty;
  float  servo2_step = (float)(_servo2_angle - servo3_duty)/_step_count;
  while(1)
  {
    if(fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
    else servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY1((uint16)servo2_start));
    
    if(fabsf(servo2_start - (float)_servo2_angle) < 1)
    {
      servo3_duty = (uint16)_servo2_angle;
      break;
    }
  }
}

//void PWM4_ctrl(uint16 _servo4_angle , float _step_count)//同上
//{
//  float  servo4_start = (float)servo4_duty;
//  float  servo4_step = (float)(_servo4_angle - servo4_duty)/_step_count;
//  
//  while(1)
//  {
//    if(fabsf(servo4_start - (float)_servo4_angle) >= servo4_step)servo4_start += servo4_step;
//    else servo4_start = _servo4_angle;
//    pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY_mn((uint16)servo4_start));
//    
//    if(fabsf(servo4_start - (float)_servo4_angle) < 1)
//    {
//      servo4_duty = (uint16)_servo4_angle;
//      break;
//    }
//  }
//}

//
void PWM4_ctrl(uint16 _servo2_angle , float _step_count) //同上
{
  float  servo2_start = (float)servo4_duty;
  float  servo2_step = (float)(_servo2_angle - servo4_duty)/_step_count;
  while(1)
  {
    if(fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
    else servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY270((uint16)servo2_start));
    
    if(fabsf(servo2_start - (float)_servo2_angle) < 1)
    {
      servo4_duty = (uint16)_servo2_angle;
      break;
    }
  }
}

void PWM5_ctrl(uint16 _servo2_angle , float _step_count)//同上
{
  float  servo2_start = (float)servo5_duty;
  float  servo2_step = (float)(_servo2_angle - servo5_duty)/_step_count;
  
  while(1)
  {
    if(fabsf(servo2_start - (float)_servo2_angle) >= servo2_step)servo2_start += servo2_step;
    else servo2_start = _servo2_angle;
    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY((uint16)servo2_start));
    
    if(fabsf(servo2_start - (float)_servo2_angle) < 1)
    {
      servo5_duty = (uint16)_servo2_angle;
      break;
    }
  }
}





void sq()//控制舵机拾图片
{
  gpio_low(B15);
  wait_identify_picture_flag=0;
  wait_identify_letter_flag=0;
  Send_clear_message();
  Send_clear_2_message();
  if(element_state!=0&&element_state!=4&&smartcar_state==0&&pick_complete!=1&&spin_finish==1)
  {
    carry_count++;
  }
  if(yb_flag!=1)
  {
    PWM1_ctrl(65,1);
    PWM4_ctrl(110,1);
    PWM2_ctrl(145,1);
    over(300);
    over(240);
  }
  if(yb_flag==1)
  {
     PWM1_ctrl(110,1);
     PWM4_ctrl(pwm4,1);
     over(120);
     PWM1_ctrl(130,1);
     PWM4_ctrl(pwm4,1);
  }
//  if(element_state==0)
//  {
//    species_scattered(species_picture);
//    steering_degree_in(0,big_species);
//  }
//  if(element_state!=0)
//  {
//    wait_identify_picture_flag=0;
//    species_concentrated(species_picture);
//    steering_degree_in(small_count,0);
//  }
  
  move(0,0,0);
//  over(10);
  gpio_high(C12);
  over(120);
  PWM1_ctrl(pwm1,1);
  PWM4_ctrl(pwm4,1);
  
  if(servo1_duty != pwm1 ||servo4_duty!=pwm4)
  {
    PWM1_ctrl(pwm1,1);
    PWM4_ctrl(pwm4,1);
  }
//  PWM5_ctrl(136,1);
  
  over(390);
  over(120);
}




void ts()
{
  over(90);
  PWM1_ctrl(125,1);  //大臂抬升45度
  over(90);
  
//  if(ABS(species_picture-species_picture_last)>=3)
//  {
//    over(96);
//  }
//  species_last=species;
  PWM2_ctrl(250,1);  //转手腕
  over(200);
  move(0,0,0);
  PWM1_ctrl(80,1);  //大臂抬升60度
  PWM4_ctrl(55,1);  //小臂旋转5度 
  over(90);
  PWM1_ctrl(65,1);  
  PWM4_ctrl(50,1);  
  over(200);
  move(0,0,0);
//  if(lz_flag==1||(element_state!=0&&element_state!=4))
//  {
//    Send_rc_picture_message();
//  }

  PWM4_ctrl(15,1);  //小臂旋转到位
  over(300);
  if(lz_flag==1||(element_state!=0&&element_state!=4))
  {

//    if(carry_count==0)
//      Send_my_element_picture_message();
    if(carry_count!=0)
    {
      Send_rc_picture_message();
      gpio_high(B15);
    }
  }
//  if(wk_y<0.08&&element_state==0)
//    smartcar_state=8;
  over(390);
  gpio_low(C12);
  over(200);
}
  
void yb()
{
//  if(lz_flag==1||(element_state!=0&&element_state!=4))
//  {
//    Send_rc_picture_message();
//  }
  PWM1_ctrl(65,1);  //大臂抬升到位
  PWM4_ctrl(80,1);  //小臂到位
  over(120);
  PWM2_ctrl(145,1);  //转手腕
  over(300);
  yb_flag=1;
}
  
void fz()
{
  //  PWM5_ctrl(81,1);
  over(300);
  over(90);
  over(10);
  over(300);
  PWM5_ctrl(pwm5,1);
  if(servo5_duty != pwm5)
  {
    PWM5_ctrl(pwm5,1);
  }
  
  over(300);
  over(300);
}
  

void servo_back()//舵机返回原位
{
  pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY270((uint16)210));
  pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY((uint16)146));
  over(300);
//  pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY1((uint16)180));
  pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY270((uint16)145));
  pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)52));  

  
  servo1_duty = pwm1;servo2_duty = pwm2;servo3_duty = pwm3;servo4_duty = pwm4;servo5_duty = pwm5;
}




void test_sq()
{
  PWM4_ctrl(pwm4,1);
}

void test_ts()
{
  PWM3_ctrl(147,1);
}


//////////////////////////////////////////////******************************************/////////////////////////////////////////////////////////////////////////////
//void sq2() //拾取照片放入收纳盒
//{
//  gpio_set(C12, 1);
//  PWM1_ctrl(pwm1,0.8);
//  if(servo1_duty != pwm1)
//  {
//    PWM1_ctrl(pwm1,0.8);
//  }
//  over(300);
//  over(300);
//  
//  pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//  servo1_duty = pwm1;
//  over(300);
//  over(300);
//  
//  PWM2_ctrl(pwm2,0.8);
//  if(servo2_duty != pwm2)
//  {
//    PWM2_ctrl(pwm2,0.8);
//  }
//  over(300);
//  over(300);
//  over(300);
//  over(300);
//  
//  gpio_set(C12, 0);
//  over(300);
//  over(300);
//  over(300);
//  pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//  servo2_duty = pwm2;
//}


//    int ct=1;
//void sq_wing() //翅膀拾取照片
//{
//  if(ct=1)
//  {
//    gpio_set(1号, 1);
//    PWM5_ctrl(pwm5,0.8); 
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo5_duty = pwm5;
//    over(300);
//    over(300);
//  }
//  else if(ct=2)
//  {
//    gpio_set(2号, 1);
//    PWM5_ctrl(pwm5,0.8);  
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo5_duty = pwm5;
//    over(300);
//    over(300);
//  }
//  else if(ct=3)
//  {
//    gpio_set(3号, 1);
//    PWM5_ctrl(pwm5,0.8);  
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM5, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo5_duty = pwm5;
//    over(300);
//    over(300);
//  }
//  else if(ct=4)
//  {
//    gpio_set(4号, 1);
//    PWM6_ctrl(pwm5,0.8); 
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM6, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo6_duty = pwm6;
//    over(300);
//    over(300);
//  }
//  else if(ct=5)
//  {
//    gpio_set(5号, 1);
//    PWM6_ctrl(pwm5,0.8);  
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM6, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo6_duty = pwm6;
//    over(300);
//    over(300);
//  }
//  else if(ct=6)
//  {
//    gpio_set(6号, 1);
//    PWM6_ctrl(pwm5,0.8);  
//    over(300);
//    over(300);
//  
//    pwm_set_duty(SERVO_MOTOR_PWM6, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//    servo6_duty = pwm6;
//    over(300);
//    over(300);
//  }
//    carry_wing[species][1]=ct;
//  ct++;
//  if(ct==7)
//  {
//    ct=1;
//  }

//}


//////////////////////////////////////////////*******************************************/////////////////////////////////////////////////////////////////////
//void servo_work() //控制舵机拾取后把图片放在盒子里面         //暂时用不上
//{
//  over(90);
//  PWM1_ctrl(50,1);//100   30
//  species_shibie(species_picture,0);
//  steering_degree(species_picture);
//  over(300);
//  if(ABS(species_picture-species_picture_last)>=3)
//  {
//    over(96);
//  }
//  species_picture_last=species_picture;
//  PWM1_ctrl(43,50);//100
//  PWM2_ctrl(0,5);//100
//  over(600);
//  PWM1_ctrl(95,1);
//  if(wk_y<0.08)
//    smartcar_state=8;
//  over(300);
//  PWM2_ctrl(0,1);//170  80
//  PWM1_ctrl(20,10);//20
//  over(300);
//  bcc=11;
//  over(200);
//  gpio_set_level(C12, 0);
//  PWM1_ctrl(25,100);
//  over(200);
//  over(200);
//  PWM1_ctrl(15,100);
//}

//void servo_back()//舵机返回原位
//{
//  pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY((uint16)178));
//  over(390);
//  pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY((uint16)0));
//  servo1_duty = pwm1;servo2_duty = pwm2;
//}

//void dcs_init(void)//电磁铁初始化                   // 暂时用不了
//{
//  gpio_init(B28,GPO,0,GPIO_PIN_CONFIG);//1
//  gpio_init(B29,GPO,0,GPIO_PIN_CONFIG);//2
//  gpio_init(B30,GPO,0,GPIO_PIN_CONFIG);//3
//  gpio_init(B31,GPO,0,GPIO_PIN_CONFIG);//4
//  gpio_init(C26,GPO,0,GPIO_PIN_CONFIG);//5
//}
//
//void dcs_work(uint8 n)//电磁铁按顺序上电打开活板门                //暂时用不上
//{ 
//  steering_degree(n);
//  over(240);
//  over(240);
//  if(n<=5)
//  {
//    switch(n+1)
//    {
//    case 4:gpio_set_level(B28,1);break;//4
//    case 5:gpio_set_level(B29,1);break;//5
//    case 3:gpio_set_level(B30,1);break;//3
//    case 2:gpio_set_level(B31,1);break;//2
//    case 6:gpio_set_level(C26,1);break;//1号
//    }
//    over(300);
//    switch(n+1)
//    {
//    case 4:gpio_set_level(B28,0);break;//4
//    case 5:gpio_set_level(B29,0);break;//5
//    case 3:gpio_set_level(B30,0);break;//3
//    case 2:gpio_set_level(B31,0);break;//2
//    case 6:gpio_set_level(C26,0);break;//1号
//    }
//  }
//}

//void steering_degree_in(int degree)//控制收纳装置转动                 
//{
//  switch(degree)
//  {
//  case 1:PWM3_ctrl(0,1)  ; break;
//  case 2:PWM3_ctrl(66,1) ; break;
//  case 3:PWM3_ctrl(151-14,1); break;
//  case 4:PWM3_ctrl(223-13,1); break;
//  case 5:PWM3_ctrl(297-14,1); break;
//  case 6:PWM3_ctrl(223-13,1); break;
//  case 7:PWM3_ctrl(297-14,1); break;
//  case 8:PWM3_ctrl(297-14,1); break;
//  }
//}
//  
////------------------------------------------------------------------------------------------------------------------
//// @brief       species_scattered    散落照片
//// @param       small_species        照片小类（A--1，B--2，     ，O--15）
//// @return      void
////------------------------------------------------------------------------------------------------------------------
//
////A//1枪支，2爆炸物，3匕首，4警棍， 5消防斧
////B//6急救包， 7手电，8对讲机, 9防弹背心,10望远镜,11头盔
////C//12消防车，13救护车，14装甲车，15摩托车
//void species_scattered(int small_species)    //识别种类 对species进行赋值
//{
//  if(small_species==1||small_species==2||small_species==3||small_species==4||small_species==5)
//    big_species=1;
//  else if(small_species==6||small_species==7||small_species==8||small_species==9||small_species==10||small_species==11)
//    big_species=2;
//  else if(small_species==12||small_species==13||small_species==14||small_species==15)
//    big_species=3;
//}
//
////------------------------------------------------------------------------------------------------------------------
//// @brief       species_concentrated    集中照片记忆
//// @param       type_species            照片小类（A--1，B--2，     ，O--15）
//// @return      void
////------------------------------------------------------------------------------------------------------------------
//void species_concentrated(int species_p)    //识别种类后将图片种类依次存到数组
//{ 
//  remember_p[species_p]=remember_i;
//  remember_i++;
//}
//
//
//void species_shibie(int small_species,int big_species)    //识别种类 对species进行赋值
//{
//  
//  if(big_species==1)//武器放置区
//  {
//    if(small_species==0||small_species==9||small_species==10||small_species==1)
//      species_picture=5;
//    else if(small_species==2||small_species==3)
//      species_picture=4;
//    else if(small_species==11||small_species==4)
//      species_picture=3;
//    else if(small_species==14||small_species==13)
//      species_picture=2;
//    else if(small_species==12||small_species==7||small_species==6)
//      species_picture=1;
//    else if(small_species==5||small_species==8)
//      species_picture=6;
//  }
//  //0苹果,1香蕉， 2豆，3白菜，4辣椒， 5玉米，6黄瓜， 7榴莲
//  //8茄子, 9葡萄,10橘子,11花生，12红薯，13萝卜，14水稻 
//  else if(big_species==2)//蔬菜
//  {
//    if(small_species==3||small_species==4||small_species==8||small_species==6||small_species==13)
//      species_picture=5;
//    else if(small_species==2||small_species==10)
//      species_picture=4;
//    else if(small_species==11||small_species==0)
//      species_picture=3;
//    else if(small_species==14||small_species==1)
//      species_picture=2;
//    else if(small_species==12||small_species==7)
//      species_picture=1;
//    else if(small_species==5||small_species==9)
//      species_picture=6;
//  }
//  //0苹果,1香蕉， 2豆，3白菜，4辣椒， 5玉米，6黄瓜， 7榴莲
//  //8茄子, 9葡萄,10橘子,11花生，12红薯，13萝卜，14水稻 
//  else if(big_species==3)//粮食
//  {
//    if(small_species==2||small_species==5||small_species==12||small_species==14||small_species==11)
//      species_picture=5;
//    else if(small_species==10||small_species==3)
//      species_picture=4;
//    else if(small_species==0||small_species==4)
//      species_picture=3;
//    else if(small_species==1||small_species==13)
//      species_picture=2;
//    else if(small_species==7||small_species==6)
//      species_picture=1;
//    else if(small_species==9||small_species==8)
//      species_picture=6;
//  }
//  else if(big_species==0)//粮食
//  {
//    if(small_species==2||small_species==10||small_species==3)
//      species_picture=5;
//    
//    else if(small_species==11||small_species==0||small_species==4)
//      species_picture=4;
//    
//    else if(small_species==14||small_species==1||small_species==13)
//      species_picture=3;
//    
//    else if(small_species==12||small_species==7||small_species==6)
//      species_picture=1;
//    
//    else if(small_species==5||small_species==9||small_species==8)
//      species_picture=2;
//  }
//}

//void servo_work()   
//{
//  over(90);
//  species_scattered(species_picture);        //得到big_species
//  species_concentrated(species_picture);     //得到small_count
//  steering_degree_in(small_count,big_species); 
//  place_picture_huan(species_alphabet);       //通过art所传字母对应数字
//  place_picture_straight(species_alphabet);   //数字字母1-16,2-17,3-15
//  over(300);
//  
//}
void steering_degree_in(int small_degree,int big_degree)//控制收纳装置转动                 
{
  
  if(element_state==0)
  {
    switch(big_degree)
    {
    case 1:PWM3_ctrl(32,1);break;
    case 2:PWM3_ctrl(85,1);break;
    case 3:PWM3_ctrl(127,1);break;
    default:break;
    }
  }
  if((element_state==1||element_state==2||element_state==3)&&pick_complete==0)
  {
    switch(small_degree)
    {
    case 1:PWM3_ctrl(173,1); break;
    case 2:PWM3_ctrl(213,1); break;
    case 3:PWM3_ctrl(263,1); break;
    case 4:PWM3_ctrl(307,1); break;
    case 5:PWM3_ctrl(350,1); break;
    default:break;
    }
  }
}  

//------------------------------------------------------------------------------------------------------------------
// @brief       species_scattered    散落照片
// @param       small_species        照片小类（一，二，三，O--15）
// @return      void
//------------------------------------------------------------------------------------------------------------------

//一武器    //1枪支，2爆炸物，3匕首，4警棍， 5消防斧
//二物资    //6急救包， 7手电，8对讲机, 9防弹背心,10望远镜,11头盔
//三交通工具//12消防车，13救护车，14装甲车，15摩托车
void species_scattered(int small_species)    //识别种类 对species进行赋值
{
  if(small_species==1||small_species==2||small_species==3||small_species==4||small_species==5)
    big_species=1;//武器
  else if(small_species==6||small_species==7||small_species==8||small_species==9||small_species==10||small_species==11)
    big_species=2;//物资
  else if(small_species==12||small_species==13||small_species==14||small_species==15)
    big_species=3;//交通工具
}

//------------------------------------------------------------------------------------------------------------------
// @brief       species_concentrated    集中照片记忆
// @param       type_species            照片小类（A--1，B--2，F-15）
// @return      void
//------------------------------------------------------------------------------------------------------------------
void species_concentrated(int species_p)    //识别种类后将图片种类依次存到数组
{
  remember_p[remember_i]=species_p;    //1  3 3 5 8   9
  remember_i++; 
  small_count=remember_i;
  for(int i=0;i<remember_i-1;i++)
  {   
    if(remember_p[i]==species_p)
    {
      small_count=i+1; 
      remember_i=remember_i-1;
      break;
    } 
  }
  
}
//------------------------------------------------------------------------------------------------------------------
// @brief       place_picture         放置图片
// @param       alphabet_count        字母种类对应数字1-A
// @return      void
// @备注        圆环，十字状态下执行
//------------------------------------------------------------------------------------------------------------------       
void place_picture(int alphabet_count)
{
  if(element_state==1||element_state==2||element_state==3)
  {
    if(remember_p[0]==alphabet_count)
    {
      PWM3_ctrl(241,1);
      classify_error=0;
    }
    else if(remember_p[1]==alphabet_count)
    {
      PWM3_ctrl(286,1);
      classify_error=0;
    }
    else if(remember_p[2]==alphabet_count)
    {
      PWM3_ctrl(330,1); 
      classify_error=0;
    }
    else if(remember_p[3]==alphabet_count)
    {
      PWM3_ctrl(12,1);
      classify_error=0;
    }
    else if(remember_p[4]==alphabet_count)
    {
      PWM3_ctrl(59,1);
      classify_error=0;
    }
    else
      classify_error=1;
  }
  else if(element_state==4)
  {
    if(16==alphabet_count)   //字母1
    {
      PWM3_ctrl(105,1);
      classify_error=0;
    }
    if(17==alphabet_count)   //字母2
    {
      PWM3_ctrl(147,1);
      classify_error=0;
    }
    if(18==alphabet_count)  //字母3
    {
      PWM3_ctrl(197,1);
      classify_error=0;
    }
  }
}
///*******************************************************************************************************************/