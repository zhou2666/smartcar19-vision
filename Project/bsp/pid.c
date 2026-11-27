/*******************************************************************************************************************/
/***Include***/
//{
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"
uint8 decelerate_flag[2]={0,0};
uint8 temp=0;
//}
/*******************************************************************************************************************/
/***Functions***/
//{

//------------------------------------------------------------------------------------------------------------------
// @brief       角度环PID
// @param       degree  目标角度            (角度制)
// @return      void
//------------------------------------------------------------------------------------------------------------------
void angle_PID_function(float degree)
{
    if((yc2==8&&element_state==3&&smartcar_state==5)||zebra_commutation_flag==1)//move先转再跑
    {
      angle_pid.Kp  = 0.046*0.65; //0.0231   0.0517
    }
    else
      angle_pid.Kp  = 0.046*0.78; //0.0231   0.0517
  //数据装载
  angle_pid.ExpectedValue = degree;
  angle_pid.ActualValue = Reduction_yaw;
  //误差计算
  angle_pid.err[0] = angle_pid.ExpectedValue - angle_pid.ActualValue;
  angle_pid.inc_err[0] = angle_pid.err[0] - angle_pid.err[1];
  //积分限幅
  if(angle_pid.err[0] != 0 && angle_pid.inc_err[0] != 0)
  {
    angle_pid.err_add += angle_pid.err[0];
    angle_pid.err_add = (ABS(angle_pid.err_add)>kpidl_angle[3])?(SIGN(angle_pid.err_add) * kpidl_angle[3]):angle_pid.err_add;
  }
  //取出极值点
  if(angle_pid.inc_err[0] * angle_pid.inc_err[0] < 0)
  {
    angle_pid.err_m = angle_pid.inc_err[1];
  }
  //不完全微分
  /*Abolished*/
  //  angle_pid.Ud[0] = angle_pid.Kd * 0.7 * angle_pid.inc_err[0] + 0.3 * angle_pid.Ud[1];
  //微分计算（懒）
  angle_pid.Ud[0] = angle_pid.Kd  * angle_pid.inc_err[0];
  //角度PID输出计算  
  angle_pid.output = angle_pid.Kp * angle_pid.err[0] + angle_pid.Ki * angle_pid.err_add + angle_pid.Ud[0];
  
  if(/*element_state!=0&&*/(smartcar_state==0||smartcar_state==1||smartcar_state==2||smartcar_state==6))//比较硬的角度环，会导致横移抖动，仅在角度要求高的情况下被使用
  {
    if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>=0.5&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=1)
    {

        angle_pid.output*=8;   
      //      kk=11;
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>=1&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=2)
    {
        angle_pid.output*=10;   
      //      kk=12;
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>=2&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=10)
    {
      if(element_state!=0)
        angle_pid.output*=4.4; 
      else
        angle_pid.output*=4;  

      //      kk=13;
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>10&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=30)
    {
      if(element_state!=0)
        angle_pid.output*=2.2; 
      else
        angle_pid.output*=1.3; 

    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>30&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=60)
    {
      if(element_state!=0)
        angle_pid.output*=1.35;   
      else
        angle_pid.output*=1.1;
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>60&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=80)
    {
      if(element_state!=0)
        angle_pid.output*=1.3;   
      else
        angle_pid.output*=1.1; 
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>80&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=100)
    {
      angle_pid.output*=1.2;   
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>100&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=180)
    {
      angle_pid.output*=0.9;   
    }
    else if(ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)>180&&ABS(angle_pid.ExpectedValue-angle_pid.ActualValue)<=380)
    {
      angle_pid.output*=0.45;   
    }
  }
  
  
  //数据迭代  
  angle_pid.err[2] = angle_pid.err[1];
  angle_pid.err[1] = angle_pid.err[0];
  angle_pid.inc_err[1] = angle_pid.inc_err[0];
  angle_pid.Ud[1] = angle_pid.Ud[0];
  angle_pid.output_last = angle_pid.output;
}
//------------------------------------------------------------------------------------------------------------------
// @brief       距离比较补偿                  (位置环)
//此函数用作定点跑时补偿车辆打滑和编码器误差
// @param       x       横向距离
// @param       y       纵向距离
// @return      void
//------------------------------------------------------------------------------------------------------------------
int b=0;
void compare(float x,float y)
{
  location_pid[0].ExpectedValue = y * 0.2;
  location_pid[1].ExpectedValue = x * 0.2 * RL_fix;
  
  if(location_pid[0].ExpectedValue<=1)
    close_pid_var[0] = 1-0.15;
  else if(location_pid[0].ExpectedValue<=2)
    close_pid_var[0] = 1.01-0.15;
  else if(location_pid[0].ExpectedValue<=3)
    close_pid_var[0] = 1.02-0.15;
  else
    close_pid_var[0] = 1.03-0.15;
  
  if(location_pid[1].ExpectedValue<=1)
    close_pid_var[1] = 1+0.1;
  else if(location_pid[1].ExpectedValue<=2)
    close_pid_var[1] = 1.01+0.1;
  else if(location_pid[1].ExpectedValue<=3)
    close_pid_var[1] = 1.05+0.1;
  else
    close_pid_var[1] = 1.07+0.1;
  
}
//------------------------------------------------------------------------------------------------------------------
// @brief       位置环PID                  (位置环)
// @param       x       横向距离
// @param       y       纵向距离
// @return      void
//------------------------------------------------------------------------------------------------------------------ 
void location_PID_function_hard(float y)
{
    location_pid_hard[0].Kp = kp_location_p6[0];
    location_pid_hard[0].Ki = ki_location_p6[0];
    location_pid_hard[0].Kd = kd_location_p6[0];
    
    //数据装载
    location_pid_hard[0].ActualValue = wk_pick_y;
    location_pid_hard[0].err[0] = location_pid_hard[0].ExpectedValue - location_pid_hard[0].ActualValue;
    //计算误差数据
    location_pid_hard[0].inc_err[0] = location_pid_hard[0].err[0] - location_pid_hard[0].err[1];
    location_pid_hard[0].err_add += location_pid_hard[0].err[0];
    //积分限幅    
    // location_pid[n].err_add = (ABS(location_pid[n].err_add)>limit_S[n])?(limit_S[n]):(location_pid[n].err_add);
    //取出误差极值
    if(location_pid_hard[0].inc_err[0] * location_pid_hard[0].inc_err[0] < 0)
    {
      location_pid_hard[0].err_m = location_pid_hard[0].inc_err[1];
    }
    //不完全微分
    /*Abolished*/
    //    location_pid[n].Ud[0] = location_pid[n].Kd * 0.7 * location_pid[n].inc_err[0] + 0.3 * location_pid[n].Ud[1];
    //微分计算（懒）
    location_pid_hard[0].Ud[0] = location_pid_hard[0].Kd * 1.0 * location_pid_hard[0].inc_err[0];
  
    location_pid_hard[0].output = location_pid_hard[0].Kp * location_pid_hard[0].err[0] + location_pid_hard[0].Ki * location_pid_hard[0].err_add + location_pid_hard[0].Ud[0];


    if(ABS(location_pid_hard[0].output)>1.5)
    {
      location_pid_hard[0].output = 1.5 * SIGN(location_pid_hard[0].output);
    }
    location_pid_hard[0].output = location_pid_hard[0].output;


  //到达停判
  if(ABS(location_pid_hard[0].ExpectedValue - location_pid_hard[0].ActualValue)<0.01)
  {
    location_pid_hard[0].output=0;
  }
  //数据迭代
  location_pid_hard[0].err[2] = location_pid_hard[0].err[1];
  location_pid_hard[0].err[1] = location_pid_hard[0].err[0];
  location_pid_hard[0].inc_err[1] = location_pid_hard[0].inc_err[0];
  location_pid_hard[0].Ud[1] = location_pid_hard[0].Ud[0];
  location_pid_hard[0].output_last = location_pid_hard[0].output;
}

void location_PID_function(float x,float y)
{
  //误差定义
  //  float   err[2] = {0,0};
  //自矫正结束
  //根据情况选取策略
  
  compare(x,y);//对期望值进行误差补偿
  //根据情况选取Kp Ki Kd 的值 和单位调节时间
  if((location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)<0.08)//横向距离和纵向距离的平方和小于0.08m   max=28cm
  {
    for(int n = 0;n < 2;n++)
    {
      location_pid[n].Kp = kp_location_02[n];
      location_pid[n].Ki = ki_location_02[n];
      location_pid[n].Kd = 0;  
    }
    flag_pid_n = 1;
    locat_state=1;
  }
  else if((location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)>0.08 && (location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)<=2)//横向距离和纵向距离的平方和大于0.08 小于2     28-141  1-7格
  {
    for(int n = 0;n < 2;n++)
    {
      location_pid[n].Kp = kp_location_1[n];
      location_pid[n].Ki = ki_location_1[n];
      location_pid[n].Kd = kd_location_1[n];  
    } 
    
    flag_pid_n = 10;
    locat_state=2;
  }
  else if((location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)>2 && (location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)<=18)//横向距离和纵向距离的平方和大于2 小于18       141-424   7-21格
  {
    for(int n = 0;n < 2;n++)
    {
      location_pid[n].Kp = kp_location_3[n];
      location_pid[n].Ki = ki_location_3[n];
      location_pid[n].Kd = kd_location_3[n];  
    } 
    
    flag_pid_n = 20;
    locat_state=3;
  }
  else if((location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)>18 && (location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)<=50)//横向距离和纵向距离的平方和大于18 小于50
  {
    for(int n = 0;n < 2;n++)
    {
      location_pid[n].Kp = kp_location_5[n];
      location_pid[n].Ki = ki_location_5[n];
      location_pid[n].Kd = kd_location_5[n];  
    }  
    
    flag_pid_n = 20;
  }
  else if((location_pid[0].ExpectedValue * location_pid[0].ExpectedValue + location_pid[1].ExpectedValue * location_pid[1].ExpectedValue)>50)//横向距离和纵向距离的平方和大于50
  {
    for(int n = 0;n < 2;n++)
    {
      location_pid[n].Kp = kp_location_p5[n];
      location_pid[n].Ki = ki_location_p5[n];
      location_pid[n].Kd = kd_location_p5[n];  
    }  
    flag_pid_n = 20;
  }
  
  
  
  for(int n = 0;n < 2;n++)
  {
    //数据装载
    location_pid[n].ActualValue = outputL[n];
    
    location_pid[n].err[0] = location_pid[n].ExpectedValue - location_pid[n].ActualValue;
    
    //计算误差数据
    location_pid[n].inc_err[0] = location_pid[n].err[0] - location_pid[n].err[1];
    location_pid[n].err_add += location_pid[n].err[0];
    //积分限幅    
    // location_pid[n].err_add = (ABS(location_pid[n].err_add)>limit_S[n])?(limit_S[n]):(location_pid[n].err_add);
    //取出误差极值
    if(location_pid[n].inc_err[0] * location_pid[n].inc_err[0] < 0)
    {
      location_pid[n].err_m = location_pid[n].inc_err[1];
    }
    //不完全微分
    /*Abolished*/
    //    location_pid[n].Ud[0] = location_pid[n].Kd * 0.7 * location_pid[n].inc_err[0] + 0.3 * location_pid[n].Ud[1];
    //微分计算（懒）
    location_pid[n].Ud[0] = location_pid[n].Kd * 1.0 * location_pid[n].inc_err[0];
  }
  for(int n = 0;n < 2;n++)
  {
    //位置PID输出值计算
    
    location_pid[n].output = location_pid[n].Kp * location_pid[n].err[0] + location_pid[n].Ki * location_pid[n].err_add + location_pid[n].Ud[0];
  }
  //输出限幅
  for(int n =  0;n < 2 ;n++)
  {
    if(ABS(location_pid[n].output)>1.5)
    {
      location_pid[n].output = 1.5 * SIGN(location_pid[n].output);
    }
    location_pid[n].output = 0.85 * location_pid[n].output + 0.15 * location_pid[n].output_last;
    //    if(ABS(location_pid[n].output)<=0.5)
    //      location_pid[n].output=SIGN(location_pid[n].output)*0.5; 
  }
  
  //  for(int n =  0;n < 2 ;n++)
  //  {
  ////  if(location_pid[n].ExpectedValue<0.5&&location_pid[n].ExpectedValue>0)
  ////    location_pid[n].output=0.5;
  ////    else  
  //    location_pid[n].output=location_pid[n].ExpectedValue;
  //  }
  //   for(int n =  0;n < 2 ;n++)
  //  {
  //    if(ABS(location_pid[n].output)>1.5)
  //    {
  //      location_pid[n].output = 1.5 * SIGN(location_pid[n].output);   
  //    }
  //    location_pid[n].output = location_pid[n].output  ;
  //  }
  
  ////超过调节时间 强行停止 防止卡死  停止判据1   超时判断    
  //  if(compulsion_monitor >= Times_Max_D)
  //  {
  //    compulsion_monitor = 0;
  //    outputL[0] = 0;
  //    outputL[1] = 0;
  //    pid_Init_A();
  //    smartcar_state = 0;
  //    check =5;
  //    self_correct = 0; 
  //    kkk=1;
  //  }
  
  if(ABS((location_pid[0].ActualValue)>=close_pid_var[0] * ABS(location_pid[0].ExpectedValue))&&(ABS(Reduction_yaw - need_angle)<5)&&(ABS(location_pid[0].ExpectedValue)!=0))//只要纵向达到目标位置纵向位置环输出零
  {
    
    if(smartcar_state != 0)
    {
//      location_monitor = 0;
      location_pid[0].output=0;
    }
    
//    kkk=2;
  }
  
  if((ABS(location_pid[1].ActualValue)>=close_pid_var[1] * ABS(location_pid[1].ExpectedValue))&&(ABS(Reduction_yaw - need_angle)<5)&&(ABS(location_pid[1].ExpectedValue)!=0))//只要横向达到目标位置横向位置环输出零
  {
    if(smartcar_state != 0)
    {
//      location_monitor = 0;
      location_pid[1].output=0;
      self_correct = 0; 
//      kkk=3;
    }
  }
  //到达停判
  if((ABS(location_pid[0].ActualValue)>=close_pid_var[0] * ABS(location_pid[0].ExpectedValue))&&(ABS(location_pid[1].ActualValue)>=close_pid_var[1] * ABS(location_pid[1].ExpectedValue))&&((location_pid[0].ExpectedValue!=0)||(location_pid[1].ExpectedValue!=0))&&(smartcar_state != 0)&&(ABS(Reduction_yaw - need_angle)<5))//达到目标位置刹车
  {
    location_monitor = 0;
    outputL[0] = 0;
    outputL[1] = 0;
    pid_Init_A();
    smartcar_state = 0;
    self_correct = 0;  
    check = 4;
//    kkk=4;
    
  }
  else if((location_pid[0].ExpectedValue==0)&&(location_pid[1].ExpectedValue==0))
  {
    location_monitor = 0;
    outputL[0] = 0;
    outputL[1] = 0;
    pid_Init_A();
    smartcar_state = 0;
    self_correct = 0; 
    check = 3;
  }
  //停止判据3  位置点附近判断
  else if((ABS(location_pid[0].inc_err[0])==0)&&(ABS(location_pid[1].inc_err[0])==0)&&((smartcar_state != 0))/*&&(ABS(Reduction_yaw - need_angle)<5)*/)//卡住
  {
    location_monitor++;
    if(((location_pid[0].err[0] * location_pid[0].err[0] + location_pid[1].err[0] * location_pid[1].err[0])<0.02)&&((smartcar_state == 5)||(smartcar_state == 8)))//最小距停判
    {
      location_monitor = 0;
      outputL[0] = 0;
      outputL[1] = 0;
      pid_Init_A();
      smartcar_state = 0;
      self_correct = 0; 
      check = 3;
//      kkk=5;
    }
    if(location_monitor==80)
    {
      location_monitor = 0;
      outputL[0] = 0;
      outputL[1] = 0;
      pid_Init_A();
      smartcar_state = 0;
      self_correct = 0; 
      check = 3;
//      kkk=6;
    }
  }
  //数据迭代
  for(int n = 0;n < 2;n++)
  {
    location_pid[n].err[2] = location_pid[n].err[1];
    location_pid[n].err[1] = location_pid[n].err[0];
    location_pid[n].inc_err[1] = location_pid[n].inc_err[0];
    location_pid[n].Ud[1] = location_pid[n].Ud[0];
    location_pid[n].output_last = location_pid[n].output;
  }
}

//****************************************************************************
int time_flag=0;
int www[2]={1000,1500};
int mmm[2]={3050,3100};  //kp 前y后x    //23000 2500    //400 450    //23000 35000// 27500,35000   //1*430,1*650     //310  200
float mmm1[2]={0,0};   //ki                                     //10,  1.85//4.00,1.85
int mmm2[2]={100,500}; //err_add限幅

int mmm3[2]={20000,20000};  //kd
int mmm4[2]={0,0};
int mmm5[2]={0,0};

float nnn[2]={4,7};      //用于判断误差小于多少时给积分i

float nnn1[2]={10,20};    //误差限幅  前y 后x


float nnn2[2]={16,20};
float nnn3[2]={26,30};
float nnn4[2]={36,40};


float xs[6]={0,0,0,0,0,0};  



int vvv=0;
int ccc=0; 
double func(double x)
{
  return 1.25 * pow(0.91, fabs(x));
}
//------------------------------------------------------------------------------------------------------------------
// @brief       矫正PID                   (矫正环)
// @param       x       横向距离
// @param       y       纵向距离
// @return      void
//------------------------------------------------------------------------------------------------------------------

void my_location_PID_function(void)
{
  
  //自矫正结束
  //根据情况选取策略
  
  
  location_pid[1].err[0]=( (float)(openart1.error_array[1]))*12/(proportion*1.0);
  location_pid[0].err[0]=( (float)(openart1.error_array[3]))*12/(proportion*1.0);
  
  location_pid[1].err_o[0]=(float)(openart1.error_array[1]);
  location_pid[0].err_o[0]=(float)(openart1.error_array[3]);
  
  //******************************************************************************************
  for(int n=0;n<2;n++)                                      //误差限幅
  {
    if(location_pid[n].err[0] >= nnn1[n])
    {
      location_pid[n].err[0] = nnn1[n];
    }
    
  }
  //*****************************************************************************************
  if(openart1.error_array[0]==2||openart1.error_array[0]==3)//对误差正负进行赋值
  {
    location_pid[1].err[0]= -location_pid[1].err[0];
  }
  if(openart1.error_array[2]==0||openart1.error_array[2]==1)
  {
    location_pid[0].err[0]= -location_pid[0].err[0];
  }
  
  
  
  if(ABS(location_pid[0].err[0])<=1.4&&ABS(location_pid[1].err[0])>=1&&ABS(location_pid[1].err[0])<=9)//y轴矫正到位，x在1-9之间
  {
    my_state=2;
    if(ABS(location_pid[1].err[0])<=4)
    {
      location_pid[1].Kp = 1.1*k24*mmm[1];
      my_state=24;
    }
    else if(ABS(location_pid[1].err[0])>4&&ABS(location_pid[1].err[0])<=9)
    {
      location_pid[1].Kp = 1.1*k25*mmm[1];
      my_state=25;
    }
    else
    {
      location_pid[1].Kp = 1.1*k26*mmm[1];
      my_state=26;
    }
    location_pid[0].Kp = 0;
  }
  
  else if(ABS(location_pid[1].err[0])<=1.4&&ABS(location_pid[0].err[0])>=1&&ABS(location_pid[0].err[0])<=5)//x轴矫正到位，y在1-5之间
  {
    my_state_y=3;
    if(ABS(location_pid[0].err[0])<=4)//微小距离易卡住给大p
    {
      location_pid[0].Kp = k34*func(location_pid[0].err[0])*mmm[0];
      my_state_y=34;
    }
    if(ABS(location_pid[0].err[0])>4&&ABS(location_pid[0].err[0])<=5)//微小距离易卡住给大p
    {
      location_pid[0].Kp = k35*func(location_pid[0].err[0])*mmm[0];
      my_state_y=35;
    }
    else if(ABS(location_pid[0].err[0])>5)
    {
      location_pid[0].Kp = k36*func(location_pid[0].err[0])*mmm[0];
      my_state_y=36;
    }
    location_pid[1].Kp = 0;
  }
  
  else//xy均未到位||y在6，x到位||x在10，y到位
  {
    
    //    my_state=1;
    for(int n = 0;n < 2;n++)
    {
      if(n==0)
      {
        if(ABS(location_pid[0].err[0])>=10)////y在10
        {
          my_state_y=110;
          location_pid[n].Kp = 1.0*0.1350*func(location_pid[0].err[0])*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[0].err[0])>=8&&ABS(location_pid[0].err[0])<10)////y在6
        {
          my_state_y=18;
          location_pid[n].Kp = 1.0*0.085*func(location_pid[0].err[0])*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[0].err[0])>=6&&ABS(location_pid[0].err[0])<8)////y在6
        {
          my_state_y=16;
          location_pid[n].Kp = 1.0*0.0750*func(location_pid[0].err[0])*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[0].err[0])<=5&&ABS(location_pid[0].err[0])>=1)//y在1-5
        {
          my_state_y=15;
          location_pid[n].Kp = 1.0*0.200*func(location_pid[0].err[0])*mmm[n];          //1708, 2581.9
        }
        //        if(ABS(location_pid[0].err[0])>=2)
        //        {
        //          my_state=12;
        ////          if(compulsion_monitor>50)//超时卡住就给一个大pid帮助小车动起来         
        ////            location_pid[n].Kp = 1.2*func(location_pid[0].err[0])*mmm[n];        
        ////          else
        ////            location_pid[n].Kp = 0.8*func(location_pid[0].err[0])*mmm[n];
        //        }
        //        else if(ABS(location_pid[0].err[0])>=2)
        //        {
        //          if(compulsion_monitor>10)
        //            location_pid[n].Kp = 1.5*func(location_pid[0].err[0])*mmm[n];       //2967, 4303
        //          else
        //            location_pid[n].Kp = 1.2*func(location_pid[0].err[0])*mmm[n];
        //        }
        else if(ABS(location_pid[0].err[0])<1)
        {
          location_pid[n].Kp = 0;
        }
      }
      else if(n==1)//x轴
      {
        if(ABS(location_pid[1].err[0])>=10)//x在10
        {
//          my_state=110;
          location_pid[n].Kp = 1.1*1.686*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[1].err[0])<10&&ABS(location_pid[1].err[0])>=6)//1<x<10
        {
          my_state=19;
          location_pid[n].Kp = 1.1*1.568*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[1].err[0])<6&&ABS(location_pid[1].err[0])>=1)//1<x<10
        {
          my_state=16;
          location_pid[n].Kp = 1.1*1.286*mmm[n];          //1708, 2581.9
        }
        else if(ABS(location_pid[1].err[0])<1)
        {
          my_state=1;
          location_pid[n].Kp = 0;
        }
      }
      
      //      if((ABS(location_pid[n].inc_err[0])<=0.1)&&(time_flag1==1)&&(ABS(location_pid[n].err[0])>2)&&(ABS(location_pid[n].err[0])<nnn[n]))
      //      {
      //        vvv=1;
      //        location_pid[n].Ki=mmm1[n];
      //      }
      //      else if(ABS(location_pid[n].inc_err[0])>0.5||ABS(location_pid[n].err[0])<1.7)
      //      {
      //        location_pid[n].Ki=0;
      //        vvv=2;
      //      }
      //      else
      //      {
      //        location_pid[n].Ki=0;
      //        vvv=3;
      //      }
      //      if(ABS(location_pid[0].err[0])>2&&ABS(location_pid[0].err[0])<=6)
      //      {
      //        location_pid[n].Kd=mmm3[n];
      //      }
    }
    
  }
  
  for(int n = 0;n < 2;n++)
  {
    //数据装载
    
    //计算误差数据
    
    jz_inc_err[n]=location_pid[n].err_o[1] - location_pid[n].err_o[0];
    
    if(location_pid[n].err[1]!=0)
    {
      location_pid[n].inc_err[0] = location_pid[n].err[0] - location_pid[n].err[1];
    }
    location_pid[n].err_add += location_pid[n].err[0];
    //积分限幅
    location_pid[n].err_add = (ABS(location_pid[n].err_add)>mmm2[n])?(SIGN(location_pid[n].err_add)*mmm2[n]):(location_pid[n].err_add);
    //不完全微分
    /*Abolished*/
    //    location_pid[n].Ud[0] = location_pid[n].Kd * 0.7 * location_pid[n].inc_err[0] + 0.3 * location_pid[n].Ud[1];
    //微分计算（懒）
    location_pid[n].Ud[0] = location_pid[n].Kd * 1.0 * location_pid[n].inc_err[0];
  }
  for(int n = 0;n < 2;n++)
  {
    //位置PID输出值计算
    location_pid[n].output = location_pid[n].Kp * location_pid[n].err[0] + location_pid[n].Ki * location_pid[n].err_add + location_pid[n].Ud[0];
  }
  for(int n = 0;n < 2 ;n++)
  {
    if(ABS(location_pid[n].output)> 1.0 * PWM_MAX)
    {
      location_pid[n].output = 1.0 * SIGN(location_pid[n].output) * PWM_MAX;
    }
    location_pid[n].output=location_pid[n].output + SIGN(location_pid[n].output)*www[n];
    
    if(ABS( location_pid[n].err[0])<= 1)
    {
      if(smartcar_state != 0)
      {
        location_pid[n].output=0;
      }
    }
  }
  float k;
  float sum;
  //计算总速度模量
  sum = ABS(location_pid[1].output) + ABS(location_pid[0].output);
  //超限判断
  if(sum>(10000))
  {
    //等比例变换
    //保证纵向横向和旋转分量的比例关系不变
    k = (10000)/sum;
    location_pid[0].output = k * location_pid[0].output;
    location_pid[1].output = k * location_pid[1].output;
  }
  
  
  
  ////  if((compulsion_monitor>80)&&(ABS(location_pid[1].err[0])<6)&&ABS(location_pid[1].err[0]) > 2)
  ////  {
  ////    compulsion_monitor=50;
  ////    location_pid[1].output =SIGN(location_pid[1].err[0])* 10000;//横向卡住超时就给10000大输出让车动起来
  ////  }
  ////  
  ////  if((compulsion_monitor>80)&&(ABS(location_pid[0].err[0])<5.5)&&ABS(location_pid[0].err[0]) > 2)
  ////  {
  ////    compulsion_monitor=50;
  ////    location_pid[0].output =SIGN(location_pid[0].err[0])* 7000;//纵向卡住超时就给7000大输出让车动起来
  ////  }
  ////  //数据迭代
  ////  
  ////  if(ABS(location_pid[0].err[0])<=1.5)//由于存在横向达到目标位置而另一方向未到在矫正过程中因为累积误差导致本达到位置移动故在误差小于1.5时i项不再累积
  ////    location_pid[0].err_add=0;
  ////  if(ABS(location_pid[1].err[0])<=1.5)//同上
  ////    location_pid[1].err_add=0;
  ////  
  ////  
  ////  
  ////  if((ABS(location_pid[0].inc_err[0])<=0.002)&&(ABS(location_pid[1].inc_err[0])<=0.002)&&(ABS(Reduction_yaw - need_angle)<8)&&(ABS(location_pid[1].err[0])<7)&&(ABS(location_pid[0].err[0])<4))//误差变化率很小且误差小于一定范围
  ////  {
  ////    if((compulsion_monitor>40)&&(time_flag1==0))//若超时计数累积到40且未进入过超时判断则进入超时判断
  ////    {
  ////      if(ABS(location_pid[1].err[0])>1.9)
  ////        location_pid[1].err_add=100;//给一个较大i帮助小车快速积分
  ////      
  ////      if (ABS(location_pid[0].err[0])>1.9)//同上
  ////        location_pid[0].err_add=20;
  ////      time_flag1=1;
  ////    }
  ////    
  ////    //      if((compulsion_monitor>50))        //强制停止监视标志位
  ////    //      {
  ////    //        if((ABS(location_pid[0].err[0])<=2.1)&&(ABS(location_pid[1].err[0])<=2.1))//超过一定时间且横向与纵向误差小于2.1则停止矫正
  ////    //        {
  ////    //          Send_Stop_corrcet_message();
  ////    //          compulsion_monitor = 0;
  ////    //          smartcar_state = 0;
  ////    //        }
  ////    //      }
  ////    
  ////    //      if((compulsion_monitor>25))//同上
  ////    //      {
  ////    //        if((ABS(location_pid[0].err[0])<=2)&&(ABS(location_pid[1].err[0])<=2))
  ////    //        {
  ////    //          Send_Stop_corrcet_message();
  ////    //          compulsion_monitor = 0;
  ////    //          smartcar_state = 0;
  ////    //        }
  ////    //      }
  ////    compulsion_monitor++;
  ////  }
  ////  else 
  ////  {
  ////    compulsion_monitor=0;
  ////  }
  ////  
  ////  //  if((ABS(location_pid[0].err[0])<=6)&&(ABS(location_pid[1].err[0])<=5))//大距离超时判断
  ////  //  {
  ////  //    ccc++;
  ////  //    if(ccc >=1800)
  ////  //    {
  ////  //      compulsion_monitor = 0;
  ////  //      Send_Stop_corrcet_message();
  ////  //      ccc=0;
  ////  //      smartcar_state = 0;
  ////  //    }
  ////  //  } 
  
  
  if(((ABS(jz_inc_err[0])<2)&&(ABS(jz_inc_err[1])<2))||((ABS(encoder[0])==0&&ABS(encoder[1])==0&&ABS(encoder[2])==0&&ABS(encoder[3])==0))/*&&(ABS(Reduction_yaw - need_angle))<8*/)//卡住计数
  {
    compulsion_monitor++;
  }
  else
  {
    compulsion_monitor=0;
  }
  
  
  //********************元素内卡住*****************************//
  if(element_state!=0)//元素内卡住
  {
//    if(ycsf_flag==1)
//    {
//      kl_distanse=40;
//    }
//    else
//    {
//      kl_distanse=20;
//    }
    
    if(pick_complete==1/*||element_state==4*/)//放置区
    {
      if((compulsion_monitor>dd_fre-2-5)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0)//放置区y普通卡住
      {
        if(compulsion_monitor>dd_fre-5)
        {
          compulsion_monitor=50-10;
          dd=0;
        }
        else
          dd=1;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 18000;//放置区y卡住
      }
      if((compulsion_monitor>dd_fre-3-5)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0 &&openart1.error_array[3]>=kl_distanse&&openart1.error_array[2]==0)//放置区y卡轮胎
      {
        if(compulsion_monitor>dd_fre-5)
        {
          compulsion_monitor=50-10;
          dd=0;
        }
        else
          dd=2;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 25000;//放置区y卡住
      }
    }
    else//拾取区
    {
      if((compulsion_monitor>dd_fre-2+5)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0 )//元素内捡图片y卡住
      {
        if(compulsion_monitor>dd_fre+5)
        {
          compulsion_monitor=50;
          dd=0;
        }
        else
          dd=3;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 18000;//元素内捡图片y卡住
      }
      if((compulsion_monitor>dd_fre-3+5)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0 &&openart1.error_array[3]>=kl_distanse&&openart1.error_array[2]==0)//小圆环拾取区y卡轮胎
      {
        if(compulsion_monitor>dd_fre+5)
        {
          compulsion_monitor=50;
          dd=0;
        }
        else
          dd=3;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 30000;//放置区y卡住
      }
    }
    
    
    if((compulsion_monitor>dd_fre-4-15)/*&&(ABS(location_pid[1].err[0])>10)*/&&ABS(location_pid[1].err[0])!=0)//元素内x卡住
    {
      if(compulsion_monitor>dd_fre-15)
      {
        compulsion_monitor=50-20;
        dd=0;
      }
      else
        dd=4;
      location_pid[1].output =SIGN(location_pid[1].err[0])* 36888;//元素内x卡住
    }
    
    
    if(element_state==4)//斑马线卡住
    {
      if((compulsion_monitor>dd_fre-5-15)/*&&(ABS(location_pid[1].err[0])>10)*/&&ABS(location_pid[1].err[0])!=0)//x
      {
        if(compulsion_monitor>dd_fre-15)
        {
          compulsion_monitor=50-15;
          dd=0;
        }
        else
          dd=5;
        location_pid[1].output =SIGN(location_pid[1].err[0])* 45000;
      }
      
      
     if((compulsion_monitor>dd_fre-2)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0)//y普通卡住
      {
        if(compulsion_monitor>dd_fre)
        {
          compulsion_monitor=50;
          dd=0;
        }
        else
          dd=6;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 20000;
      }
      if((compulsion_monitor>dd_fre-3)/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0 &&openart1.error_array[3]>=kl_distanse/*&&openart1.error_array[2]==0*/)//y卡轮胎
      {
        if(compulsion_monitor>dd_fre)
        {
          compulsion_monitor=50;
          dd=0;
        }
        else
          dd=7;
        location_pid[0].output =SIGN(location_pid[0].err[0])* 35000;
      }
      
    }
  }
  //********************元素外卡住*****************************//
  else if(element_state==0)//元素外卡住
  {
    if((compulsion_monitor>(dd_fre-3+8))/*&&(ABS(location_pid[0].err[0])<6)*/&&ABS(location_pid[0].err[0])!=0)//元素外散落图片y卡住
    {
      if(compulsion_monitor>dd_fre+8)
      {
        compulsion_monitor=50;
        dd=0;
      }
      else
        dd=10;
      location_pid[0].output =SIGN(location_pid[0].err[0])* 13888;//元素外散落图片y卡住
      
    }
    
    if((compulsion_monitor>(dd_fre-3))/*&&(ABS(location_pid[1].err[0])>10)*/&&ABS(location_pid[1].err[0])!=0&&ABS(openart1.error_array[1])>=6)//元素外散落图片x卡住(大距离)
    {
      if(compulsion_monitor>dd_fre)
      {
        compulsion_monitor=50;
        dd=0;
      }
      else 
        dd=11;
      location_pid[1].output =SIGN(location_pid[1].err[0])* 50000;//元素外散落图片x卡住(大距离)
    }
    
    if((compulsion_monitor>(dd_fre-3))/*&&(ABS(location_pid[1].err[0])>10)*/&&ABS(location_pid[1].err[0])!=0&&ABS(openart1.error_array[1])<6)//元素外散落图片x卡住(小距离)
    {
      if(compulsion_monitor>dd_fre)
      {
        compulsion_monitor=50;
        dd=0;
      }
      else 
        dd=12;
      location_pid[1].output =SIGN(location_pid[1].err[0])* 30000;//元素外散落图片x卡住(小距离)
    }
  }
  
//  
//  if(location_pid[1].err[0]==0&&location_pid[0].err[0]&&jz_success!=1)
//    my_monitor1++;
//  else
//    my_monitor1=0;
//  if(my_monitor1==500)
//  {
//    if(my_monitor1>500)
//    {
//      my_monitor1=0;
//    }
//    location_pid[0].output =-30000;
//    location_pid[1].output =30000;
//  }
  
  
  
  //   for(int n=0;n<2;n++)
  //   {
  if(ABS(location_pid[0].err[0])<=2)//反转刹车
  {
    if(outputV[0]>0)
    {
      location_pid[0].output=-1000; 
    }
    else if(outputV[0]<0)
    {
      location_pid[0].output=1000; 
    } 
  }
  // }
  if(ABS(location_pid[1].err[0])>=20)
  {
    if(ABS(Mid_add)<10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 11888;
    if(ABS(Mid_add)>=10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 11888;
    my_state=120;
  }
  else if(ABS(location_pid[1].err[0])>=15&&ABS(location_pid[1].err[0])<20)
  {
    if(ABS(Mid_add)<10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 11088;
    if(ABS(Mid_add)>=10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 11288;
    my_state=115;
  }
  else if(ABS(location_pid[1].err[0])>=14&&ABS(location_pid[1].err[0])<15)
  {
    if(ABS(Mid_add)<10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 10988;
    if(ABS(Mid_add)>=10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 10988;
    my_state=114;
  }
  else if(ABS(location_pid[1].err[0])>=10&&ABS(location_pid[1].err[0])<13)
  {
    if(ABS(Mid_add)<10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 10988;
    if(ABS(Mid_add)>=10)
      location_pid[1].output =SIGN(location_pid[1].err[0])* 10988;
    my_state=110;
  }
  if(ABS(location_pid[1].err[0])>=8&&smartcar_state==4)//斑马线
  {
    location_pid[1].output =SIGN(location_pid[1].err[0])* 14000;

  }
  
  
  
  for(int n = 0;n < 2;n++)
  {
    for(int i=9;i>=0;i--)
    {
      location_pid[n].err_o[i+1] = location_pid[n].err_o[i];
    }
    location_pid[n].err[2] = location_pid[n].err[1];
    location_pid[n].err[1] = location_pid[n].err[0];
    location_pid[n].inc_err[1] = location_pid[n].inc_err[0];
    location_pid[n].Ud[1] = location_pid[n].Ud[0];
    location_pid[n].output_last = location_pid[n].output;
  }
  
  // if((ABS(location_pid[0].err[0])<=0.025)&&(ABS(location_pid[1].err[0])<=0.025))
  //    if(compulsion_monitor >= 60)
  //    {
  //      compulsion_monitor = 0;
  //      Send_Stop_corrcet_message();
  //    }
  
  
}
//------------------------------------------------------------------------------------------------------------------
// @brief       速度环PID
// @param       sx       横向目标速度
// @param       sy       纵向目标速度
// @return      void
//------------------------------------------------------------------------------------------------------------------
void speed_PID_function(float sx,float sy)
{
  float   err[2] = {0,0};
  //  float   Online_K = 1.075f;
  //  float   Offine_K = 0.65f;
  speed_pid[0].ExpectedValue = sy;
  speed_pid[1].ExpectedValue = sx;
  for(int n = 0;n < 2;n++)
  {
    //数据装载
    speed_pid[n].ActualValue = outputV[n];
    //误差计算
    err[n] = (ABS(speed_pid[n].ExpectedValue - speed_pid[n].ActualValue)>speed_pid[n].dead_bar)?(speed_pid[n].ExpectedValue - speed_pid[n].ActualValue):0;
  }
  //车地坐标变换
  
  speed_pid[0].err[0] = err[0];
  speed_pid[1].err[0] = err[1]; 
  
  
  //误差相关值计算
  for(int n = 0;n < 2;n++)
  {
    speed_pid[n].inc_err[0] = speed_pid[n].err[0] - speed_pid[n].err[1];
    if(speed_pid[n].err[0] != 0 && speed_pid[n].inc_err[0] != 0)
    {
      speed_pid[n].err_add += speed_pid[n].err[0];
    }
    //积分限幅
    if(ABS(speed_pid[n].err_add)>limit_V[n])
    {
      speed_pid[n].err_sum = 0;
    }
    else
    {
      speed_pid[n].err_sum = speed_pid[n].err[0];
    }
    //取出误差峰值
    if(speed_pid[n].inc_err[0] * speed_pid[n].inc_err[0] < 0)
    {
      speed_pid[n].err_m = speed_pid[n].err[1];
    }
    //不完全微分
    /*Abolished*/
    //    speed_pid[n].Ud[0] = speed_pid[n].Kd * 0.7 * speed_pid[n].inc_err[0] + 0.3 * speed_pid[n].Ud[1];
    //微分计算（懒）      
    speed_pid[n].Ud[0] = speed_pid[n].Kd * 1.0 * (speed_pid[n].inc_err[0] - speed_pid[n].inc_err[1]);
  }
  for(int n = 0;n < 2;n++)
  {
    //速度PID输出计算
    speed_pid[n].output = speed_pid[n].output_last + speed_pid[n].Kp * speed_pid[n].inc_err[0] + speed_pid[n].Ki * speed_pid[n].err_sum + speed_pid[n].Ud[0]; 
    //输出处理
    if(ABS(speed_pid[n].output)> 1.0 * PWM_MAX)
    {
      speed_pid[n].output = 1.0 * SIGN(speed_pid[n].output) * PWM_MAX;
    }
    speed_pid[n].output = 0.85 * speed_pid[n].output + 0.15 * speed_pid[n].output_last;
    if(ABS(speed_pid[n].output)<speed_pid[n].output_dead_bar)
    {
      speed_pid[n].output = speed_pid[n].output_last;
    }
  }
  if(speed_pid[0].output>7500)
  {
    speed_flag=7;
  }
  else if(speed_pid[0].output<=6500&&speed_pid[0].output<=7500)
  {
    speed_flag=6;
  }
  else if(speed_pid[0].output<=6500)
  {
    speed_flag=5;
  }
  //数据迭代
  for(int n = 0;n < 2;n++)
  {
    speed_pid[n].err[2] = speed_pid[n].err[1];
    speed_pid[n].err[1] = speed_pid[n].err[0];
    speed_pid[n].inc_err[1] = speed_pid[n].inc_err[0];
    speed_pid[n].Ud[1] = speed_pid[n].Ud[0];
    speed_pid[n].output_last = speed_pid[n].output;
  }
}
//------------------------------------------------------------------------------------------------------------------
// @brief       角速度环PID
// @param       gory       目标角速度
// @return     void
//------------------------------------------------------------------------------------------------------------------
void gory_PID_function(float gory)
{
  //数据装载
  gory_pid.ExpectedValue = gory;
  gory_pid.ActualValue = (ABS(eulerAngle.cad[0])>0.1)?(eulerAngle.cad[0]):0;
  //误差计算i
  gory_pid.err[0] = (ABS(gory_pid.ExpectedValue - gory_pid.ActualValue)>gory_pid.dead_bar)?(gory_pid.ExpectedValue - gory_pid.ActualValue):0;
  gory_pid.inc_err[0] = gory_pid.err[0] - gory_pid.err[1];
  //计算积分以及积分限幅
  if(gory_pid.err[0] != 0 && gory_pid.inc_err[0] != 0)
  {
    gory_pid.err_add += gory_pid.err[0];
  }     
  if(ABS(gory_pid.err_add)>kpidl_gory[3])
  {
    gory_pid.err_sum = 0;//
  }
  else
  {
    gory_pid.err_sum = gory_pid.err[0];
  }
  //取出误差极值
  if(gory_pid.inc_err[0] * gory_pid.inc_err[0] < 0)
  {
    gory_pid.err_m = gory_pid.inc_err[1];
  }
  //不完全微分
  /*Abolished*/
  //gory_pid.Ud[0] = gory_pid.Kd * 0.7 * (gory_pid.inc_err[0] - gory_pid.inc_err[1]) + 0.3 * gory_pid.Ud[1];
  //微分计算（懒）  
  gory_pid.Ud[0] = gory_pid.Kd * 1.0* (gory_pid.inc_err[0] - gory_pid.inc_err[1]);
  //角速度PID输出计算
  gory_pid.output = gory_pid.output_last + gory_pid.Kp * gory_pid.inc_err[0] + gory_pid.Ki * gory_pid.err_sum + gory_pid.Ud[0];
  //输出处理
  gory_pid.output = 1.0 * gory_pid.output;
  if(ABS(gory_pid.output)> 1.0 * PWM_MAX)
  {
    gory_pid.output = 1.0 * SIGN(gory_pid.output) * PWM_MAX;
  }
  gory_pid.output = 0.85 * gory_pid.output + 0.15 * gory_pid.output_last;
  if(ABS(gory_pid.output)<gory_pid.output_dead_bar)
  {
    gory_pid.output = gory_pid.output_last;
  }
  //数据迭代   
  gory_pid.err[2] = gory_pid.err[1];
  gory_pid.err[1] = gory_pid.err[0];
  gory_pid.inc_err[1] = gory_pid.inc_err[0];
  gory_pid.Ud[1] = gory_pid.Ud[0];
  gory_pid.output_last = gory_pid.output;
}

/*------------------------------------------------------------------------------------------------------------------
// @brief       转向环PID
// @param       Mid_add       中线误差
// @return      void
------------------------------------------------------------------------------------------------------------------*/

void turn_pid_function()
{
  
  //数据装载
  turn_pid.ExpectedValue = 0;
  turn_pid.ActualValue =atan2(Mid_add,26)*180/PI;//中线误差通过反正切求出夹角，再将夹角转为角度制
  //误差计算
  turn_pid.err[0] = turn_pid.ExpectedValue - turn_pid.ActualValue;
  turn_pid.inc_err[0] =turn_pid.err[0] - turn_pid.err[1];   
  //取出极值点
  if(turn_pid.inc_err[0] * turn_pid.inc_err[0] < 0)
  {
    turn_pid.err_m = turn_pid.inc_err[1];
  }
  //微分计算（懒）
  turn_pid.Ud[0] = turn_pid.Kd  * turn_pid.inc_err[0];
  //转向环PID输出计算
  turn_pid.output = turn_pid.Kp * turn_pid.err[0] + turn_pid.Ud[0];
  
  if(ABS(turn_pid.output)>=2.5)
    turn_pid.output=SIGN(turn_pid.output)*2.5;
  //数据迭代 
  turn_pid.err[2] = turn_pid.err[1];
  turn_pid.err[1] = turn_pid.err[0];
  turn_pid.inc_err[1] = turn_pid.inc_err[0];
  turn_pid.Ud[1] = turn_pid.Ud[0];
  turn_pid.output_last = turn_pid.output;
}



int abd=0;
int abv=0;
//------------------------------------------------------------------------------------------------------------------
// @brief       移动函数
// @param       x       横向距离
// @param       y       纵向距离
// @param      degree   角度 
// @return      void
//------------------------------------------------------------------------------------------------------------------
void move(float x,float y,float degree)
{
  encoder_ctrl(1);
  //编码器数据读取
  if(angle_function_act == 1)
  {
    angle_function_act = 0;
    angle_PID_function(degree);
    gory_PID_function(angle_pid.output);
    
    if((yc2==8&&element_state==3&&smartcar_state==5)||(no_picture_flag==1&&element_state!=0)||(element_state==4&&zebra_commutation_flag==1))//元素情况
    {
      speed_output[2] = gory_pid.output;
      dispense_speed();
    }
  }
  //角度环动作计算
  
  if(forced_stop == 1)         //强制停止标志位
    //是否强制停止
  {
    if(ABS(aL[0] * PID_RATE)<0.1 && ABS(aL[1] * PID_RATE)<0.1)          //PID_RATE  PID调节PWM值的频率
    {
      forced_stop = 0;
      compulsion_monitor = 0;
      outputL[0] = 0;
      outputL[1] = 0;
      pid_Init_A();
      smartcar_state = 0;
      self_correct = 0;             //自矫正标志位
      check = 2;
    }
    speed_PID_function(0,0);
    speed_output[0] = speed_pid[0].output;
    speed_output[1] = speed_pid[1].output;
    speed_output[2] = gory_pid.output;
    dispense_speed();
  }
  
  if((yc2==8&&element_state==3&&smartcar_state==5)||(no_picture_flag==1&&element_state!=0)||(element_state==4&&zebra_commutation_flag==1))//旋转完再跑xy
  {
    
    if(ABS(Reduction_yaw-degree)<5)
    {
      if(smartcar_state!=0)
        //正常计算
      {
        if(pid_fucntion_flag == 1)
        {
          pid_fucntion_flag = 0;
          //      if(y==0&&x!=0)
          //      {
          //        if( outputL[0]>0.06)//防止车上下飘，给反方向速度
          //          y_speed1=-0.6;
          //        else if( outputL[0]<-0.04)
          //          y_speed1=0.6;
          //        else 
          //          y_speed1=0;
          //        
          //       location_PID_function(x,y_speed1);  
          //      }
          //      else
          location_PID_function(x,y);
        }  
        
        if(smartcar_state==5&&no_picture_flag==1&&(element_state==1||element_state==2))
          speed_PID_function(1.3*location_pid[1].output,1.3*location_pid[0].output);
        else
          speed_PID_function(location_pid[1].output,location_pid[0].output);
        //速度环动作计算
        
        speed_output[0] = 1*speed_pid[0].output;
        speed_output[1] = 1*speed_pid[1].output;
        speed_output[2] = 1*gory_pid.output;  
        dispense_speed();
        //PWM参数传递   
      }
      
      else//非动作情况
      {
        speed_PID_function(0,0);
        speed_output[0] = speed_pid[0].output;
        speed_output[1] = speed_pid[1].output;
        speed_output[2] = gory_pid.output; 
        
        dispense_speed();
        //        if((abd<8)&&( direction_compare==1))
        //    if((abd<12)&&(Return_brake_flag==1))
        //    { 
        //      abd++;                 //每运行一次加一
        //      float speed[4]={0,0,0,0};
        //      for(int n=0;n<4;n++)
        //      {
        //        if(ABS(encoder[n])>3.0)
        //          speed[n]=-SIGN(encoder[n])*30000;
        //      }
        //      moter[0].moter_pwm = (int32)(speed[0]  - speed_output[2] * 1);
        //      moter[1].moter_pwm = (int32)(speed[1]  + speed_output[2] * 1);
        //      moter[2].moter_pwm = (int32)(speed[2]  - speed_output[2] * 1);
        //      moter[3].moter_pwm = (int32)(speed[3]  + speed_output[2] * 1);
        //    }
      }
    }
  }
  else//边旋转边跑xy
  {
    if(smartcar_state!=0)                              
      //正常计算
    {
      if( pid_fucntion_flag == 1)
      {
        pid_fucntion_flag = 0;
        //      if(y==0&&x!=0)
        //      {
        //        if( outputL[0]>0.06)//防止车上下飘，给反方向速度
        //          y_speed1=-0.6;
        //        else if( outputL[0]<-0.04)
        //          y_speed1=0.6;
        //        else 
        //          y_speed1=0;
        //        
        //       location_PID_function(x,y_speed1);  
        //      }
        //      else
        location_PID_function(x,y);       
      }
      //位置环动作计算
      //    if(smartcar_state==5)
      //      speed_PID_function(0,0.5);
      //    
      //    else if(smartcar_state==7)
      //      speed_PID_function( 0.7, 0);
      
      //    else
      
      if(smartcar_state==8)
        speed_PID_function(1.5*location_pid[1].output,1.5*location_pid[0].output);
      else
        speed_PID_function(location_pid[1].output,location_pid[0].output);

      
      //速度环动作计算
      
      speed_output[0] = 1*speed_pid[0].output;
      speed_output[1] = 1*speed_pid[1].output;
      speed_output[2] = 1*gory_pid.output;  
      dispense_speed();
      //PWM参数传递   
    }
    
    else//非动作情况
    {
      speed_PID_function(0,0);
      speed_output[0] = speed_pid[0].output;
      speed_output[1] = speed_pid[1].output;
      speed_output[2] = gory_pid.output; 
      
      dispense_speed();
      //        if((abd<8)&&( direction_compare==1))
      //    if((abd<12)&&(Return_brake_flag==1))
      //    { 
      //      abd++;                 //每运行一次加一
      //      float speed[4]={0,0,0,0};
      //      for(int n=0;n<4;n++)
      //      {
      //        if(ABS(encoder[n])>3.0)
      //          speed[n]=-SIGN(encoder[n])*30000;
      //      }
      //      moter[0].moter_pwm = (int32)(speed[0]  - speed_output[2] * 1);
      //      moter[1].moter_pwm = (int32)(speed[1]  + speed_output[2] * 1);
      //      moter[2].moter_pwm = (int32)(speed[2]  - speed_output[2] * 1);
      //      moter[3].moter_pwm = (int32)(speed[3]  + speed_output[2] * 1);
      //    }
    }
  }
  
  
  if(e_p_flag==1&&smartcar_state==0&&jz_success==1&&wait_identify_letter_flag==0)
  {
//    kkk=12512;
    speed_output[2] =0; 
    dispense_speed();
  }
  
  for(int n=0;n<2;n++)
  {
    if(ABS(speed_output[n])>=10000)
    {
      speed_output[n]=SIGN(speed_output[n])*10000;
    }
  }
  
//      kkk=0;

  
  //  location_PID_function(x, y);
  //  angle_PID_function(degree);
  //  gory_PID_function(angle_pid.output);
  //  
  //  speed_output[0] = speed_pid[0].output;
  //  speed_output[1] = speed_pid[1].output;
  //  speed_output[2] = gory_pid.output;
  
  //  dispense_speed();
  moter_control();
  encoder_ctrl(2);
  //速度分配以及电机动作
  
}


int abw=0;
void my_move (float degree)
{
  encoder_ctrl(1);  //编码器数据读取 
  abd=0;
//  kkk=0;            //到达目标位置方式
  angle_PID_function(degree);   
  gory_PID_function(angle_pid.output);        
  my_location_PID_function();       //矫正环动作计算
  //速度环动作计算
  speed_output[0] = 1*location_pid[0].output;
  //   speed_output[0]=0;
  speed_output[1] = 1*location_pid[1].output;
  speed_output[2] = 1*gory_pid.output;  
  dispense_speed();  //PWM参数传递 
  moter_control();
  encoder_ctrl(2);
  //速度分配以及电机动作   
}

void no_move(float y,float degree)
{
  encoder_ctrl(1);
  //编码器数据读取
  if(angle_function_act == 1)
  {
    angle_function_act = 0;
    angle_PID_function(degree);
    gory_PID_function(angle_pid.output);
  }
  location_PID_function_hard(y);
  speed_PID_function(0,location_pid[0].output);
  //速度环动作计算
  
  speed_output[0] = 1*speed_pid[0].output;
  speed_output[1] = 1*speed_pid[1].output;
  speed_output[2] = 1*gory_pid.output;  
  dispense_speed();
  //PWM参数传递   
}



void test_move(float x,float y,float degree)
{
  encoder_ctrl(1);
  angle_function_act = 0;
  angle_PID_function(0);   
  gory_PID_function(angle_pid.output);   
  abd++;
  if(abd>605)
    speed_PID_function(0,0);
  else
    speed_PID_function(0,2);
  
  speed_output[0] = speed_pid[0].output;
  speed_output[1] =0;
  speed_output[2] = gory_pid.output;
  dispense_speed();
  moter_control();
  encoder_ctrl(2);
  //速度分配以及电机动作
}
/*******************************************************************************************************************/
#if(0)
Reference Resources:
https://zhuanlan.zhihu.com/p/72764897
https://blog.csdn.net/qq_34213260/article/details/106227213
https://blog.csdn.net/best_xiaolong/article/details/109634365
https://www.bilibili.com/video/BV1U341167wo/?spm_id_from=333.999.0.0
https://www.bilibili.com/video/BV1A3411x7tN/?spm_id_from=333.999.0.0
https://www.bilibili.com/video/BV1oU4y1K7T8/?spm_id_from=333.999.0.0
https://www.bilibili.com/video/BV1oF411z7Rr/?spm_id_from=333.999.0.0
https://www.bilibili.com/video/BV1LA411V7w5/?spm_id_from=333.999.0.0  
#endif
  /*******************************************************************************************************************/