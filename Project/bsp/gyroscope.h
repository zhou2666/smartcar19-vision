/*******************************************************************************************************************/
#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H
/*******************************************************************************************************************/
/***Includes***/
//{
//#include None
 #include "define.h"  
//}
/*******************************************************************************************************************/
/***Export Functions***/
//{
void gyroOffsetInit(void);                                      //陀螺仪零漂初始化 
void icmGetValues(void);                                        //低通滤波并进行单位转换 deg/s -> rad/s
void gyroscope_read();                                          //读取陀螺仪数据<------|(嵌合)
void icmAHRSupdate(icm_param_t* icm,quater_param_t* QQ,gyrocope_err_init* ERR);                         //获取四元数
void Euler(euler_param_t* ANGLE,quater_param_t* QQ);                                                  //转换欧拉角
void icmGetValues_static(void);                                                 //转换欧拉角
//}
/*******************************************************************************************************************/
#endif
