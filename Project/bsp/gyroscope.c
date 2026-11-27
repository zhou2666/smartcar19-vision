//*******************************************************************************************************************/
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
// @brief       （真）陀螺获取值
// @return      void
//------------------------------------------------------------------------------------------------------------------
void icmGetValues(void) 
{
   float alpha = 0.1586;//0.1586        0.6

//一阶低通滤波，单位g
    icm_data.acc_x = (((float) (icm20602_acc_x)) * alpha) / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float) (icm20602_acc_y)) * alpha) / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float) (icm20602_acc_z)) * alpha) / 4096 + icm_data.acc_z * (1 - alpha);
//角度迭代  计算角速度时使用  
    eulerAngle.cad[1] = eulerAngle.cad[0];
    
//! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    icm_data.gyro_x = ((float) icm20602_gyro_x - GyroOffset.Xdata) * PI / 180 / 16.4f; 
    icm_data.gyro_y = ((float) icm20602_gyro_y - GyroOffset.Ydata) * PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) icm20602_gyro_z - GyroOffset.Zdata) * PI / 180 / 16.4f;
//传递角速度     用于角速度PID
    eulerAngle.cad[0] = icm_data.gyro_z;
//角速度低通滤波    
    eulerAngle.cad[0] = 0.7 * eulerAngle.cad[0] + 0.3 * eulerAngle.cad[1];
 
    
    
//传递加速度    
    acc.acc_x = icm_data.acc_x;
    acc.acc_y = icm_data.acc_y;
    acc.acc_z = icm_data.acc_z;
}
//------------------------------------------------------------------------------------------------------------------
// @brief       （假）陀螺获取值
// @return      void
//------------------------------------------------------------------------------------------------------------------
void icmGetValues_static(void)
{
   float alpha = 0.1586;//0.1586        0.6

    //一阶低通滤波，单位g
    icm_data_static.acc_x = (icm_data_random.acc_x * alpha) + icm_data_static.acc_x * (1 - alpha);
    icm_data_static.acc_y = (icm_data_random.acc_y * alpha) + icm_data_static.acc_y * (1 - alpha);
    icm_data_static.acc_z = (icm_data_random.acc_z * alpha) + icm_data_static.acc_z * (1 - alpha);
    
    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    icm_data_static.gyro_x = (icm_data_random.gyro_x);
    icm_data_static.gyro_y = (icm_data_random.gyro_y);
    icm_data_static.gyro_z = (icm_data_random.gyro_z);
     
}
//{
//------------------------------------------------------------------------------------------------------------------
// @brief       陀螺仪读取
// @return      void
//------------------------------------------------------------------------------------------------------------------
void gyroscope_read()
{
//获取加速度数值
  icm20602_get_acc();
//获取角速度数值  
  icm20602_get_gyro();
//初步数据处理
  icmGetValues();
}
//------------------------------------------------------------------------------------------------------------------
// @brief       四元数计算
// @return      void
//------------------------------------------------------------------------------------------------------------------
void icmAHRSupdate(icm_param_t* icm,quater_param_t* QQ,gyrocope_err_init* ERR)            
{
// 采样周期一半
    float halfT = 0.5 * delta_T; 
// 当前姿态计算得来的重力在三轴上的分量    
//    float vx, vy, vz;     
// 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差    
//    float ex, ey, ez;        
    
//四元数    
    float q0 = QQ->q0;  
    float q1 = QQ->q1;
    float q2 = QQ->q2;
    float q3 = QQ->q3;   
//先相乘，方便后续计算    
    float q0q0 = q0 * q0;  
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
// 正常静止状态为-g 反作用力。
// 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况    
    if(icm->acc_x * icm->acc_y * icm->acc_z == 0) 
        return;
// 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
    float norm = InvSqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z); 
    icm->acc_x = icm->acc_x * norm;
    icm->acc_y = icm->acc_y * norm;
    icm->acc_z = icm->acc_z * norm;
// 载体坐标系下重力在三个轴上的分量
    ERR->vx = 2 * (q1q3 - q0q2);
    ERR->vy = 2 * (q0q1 + q2q3);
    ERR->vz = q0q0 - q1q1 - q2q2 + q3q3;
// g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ERR->ex = icm->acc_y * ERR->vz - icm->acc_z * ERR->vy;
    ERR->ey = icm->acc_z * ERR->vx - icm->acc_x * ERR->vz;
    ERR->ez = icm->acc_x * ERR->vy - icm->acc_y * ERR->vx;
// 误差累加
    ERR->I_ex += halfT * ERR->ex;  
    ERR->I_ey += halfT * ERR->ey;
    ERR->I_ez += halfT * ERR->ez;
// 使用PI控制器消除向量积误差(陀螺仪漂移误差)
    icm->gyro_x = icm->gyro_x + icm_kp * ERR->ex + icm_ki * ERR->I_ex;
    icm->gyro_y = icm->gyro_y + icm_kp * ERR->ey + icm_ki * ERR->I_ey;
    icm->gyro_z = icm->gyro_z + icm_kp * ERR->ez + icm_ki * ERR->I_ez;
// 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;
// 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
// 用全局变量记录上一次计算的四元数值    
    QQ->q0 = q0 * norm;
    QQ->q1 = q1 * norm;
    QQ->q2 = q2 * norm;
    QQ->q3 = q3 * norm;  
}
//------------------------------------------------------------------------------------------------------------------
// @brief       欧拉角计算
// @return      void
//------------------------------------------------------------------------------------------------------------------
void Euler(euler_param_t* ANGLE,quater_param_t* QQ)
{
//载入四元数  
    float q0 = QQ->q0;
    float q1 = QQ->q1;
    float q2 = QQ->q2;
    float q3 = QQ->q3;
//计算欧拉角    
    ANGLE->pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/PI; // pitch
    ANGLE->roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/PI; // roll
//角度迭代
//这里只用了一个角    
//计算欧拉角  
    ANGLE->yaw_last = ANGLE->yaw;
    ANGLE->yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/PI; // yaw
    bug=ANGLE->yaw;
//角度连续化处理
    if(ABS(ANGLE->yaw - ANGLE->yaw_last)>300)
    {
      if(ANGLE->yaw>0)
      {
        ANGLE->k--;
      }
      else if(ANGLE->yaw<0)
      {
        ANGLE->k++;
      }
    }
    ANGLE->final_yaw_last = ANGLE->final_yaw;
    ANGLE->final_yaw = 360 *  ANGLE->k + ANGLE->yaw;
}
//------------------------------------------------------------------------------------------------------------------
// @brief       陀螺仪初始化
// @return      void
//------------------------------------------------------------------------------------------------------------------
void gyroOffsetInit()     
{
  for (uint16_t i = 0; i < 50; ++i) 
  {
// 获取陀螺仪角速度
    icm20602_get_gyro();    
    icm20602_get_acc();
    GyroOffset.Xdata += icm20602_gyro_x;
    GyroOffset.Ydata += icm20602_gyro_y;
    GyroOffset.Zdata += icm20602_gyro_z;
    system_delay_ms(1);    // 最大 1Khz--->      systick_delay_ms(1);
   }
//角度初态计算
   GyroOffset.Xdata /= 50;
   GyroOffset.Ydata /= 50;
   GyroOffset.Zdata /= 50;
// 获取陀螺仪角速度
//用于建立静态参考陀螺仪
//如果旋转一下陀螺仪，不用偏航角的话就不用那么麻烦了
  for (uint16_t i = 0; i < 500; ++i) 
  {
    gyroscope_read();
    icm_data_random.acc_x += icm_data.acc_x;
    icm_data_random.acc_y += icm_data.acc_y;
    icm_data_random.acc_z += icm_data.acc_z;
      
    icm_data_random.gyro_x += icm_data.gyro_x; 
    icm_data_random.gyro_y += icm_data.gyro_y; 
    icm_data_random.gyro_z += icm_data.gyro_z;
//    over(10);
  }
//数值回复
  icm_data.acc_x = 0;
  icm_data.acc_y = 0;
  icm_data.acc_z = 0;
  icm_data.gyro_x = 0;
  icm_data.gyro_y = 0;
  icm_data.gyro_z = 0;
//建立静态参考陀螺仪   
  icm_data_random.acc_x /= 500;
  icm_data_random.acc_y /= 500;
  icm_data_random.acc_z /= 500;
  icm_data_random.gyro_x /= 500;
  icm_data_random.gyro_y /= 500;
  icm_data_random.gyro_z /= 500;
}
//}
/*******************************************************************************************************************/
#if(0)
Reference Resources:
https://blog.csdn.net/kouxi1/article/details/120877672
https://blog.csdn.net/weixin_44410512/article/details/121950016
https://www.zhihu.com/question/401011285  
#endif
/*******************************************************************************************************************/