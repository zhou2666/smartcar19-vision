/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.

 ********************************************************************************************************************/

#ifndef openart_uart_h
#define openart_uart_h
/*********************************************************************************************************
  结构体定义--openart1(接收地图坐标,接收车姿偏差量)
*********************************************************************************************************/
typedef struct openart1_param_t 
{
    uint8 picture_array[70];     //存储串口接受数据
    uint8 *error_array;   
    uint8 picture_num;
    uint8 number;
    enum 
    {
        FH1,count1_one,count1_two,count1_three,count1_four,count1_five
    } pack_order1;         // 打包顺序：帧头、坐标数量、坐标位置
    uint8 remove_blob;         //识别到黄框=1，没识别到=0
    uint8 remove_distance;      //需要后退的距离(cm)
} openart1_param_t;
extern openart1_param_t openart1;//后摄（目标检测，矫正姿态）

/*********************************************************************************************************
  结构体定义--openart2(接收图片识别后的信息量)
*********************************************************************************************************/
typedef struct openart2_param_t 
{
    uint8 broad_heading;        //大类
    uint8 subclass;             //小类 
    uint8 *error_array; 
    bool classify_state;        //识别状态(0：识别完成；1：正在识别)
    uint8 correct_posture[6];//矫正姿态需要的格数
    uint8 back_posture[5];//矫正终点需要的格数
    enum
    {
        FH2,count2_one,count2_two,count2_three,count2_four,count2_five
    } pack_order2;         // 打包顺序：帧头、坐标数量、
} openart2_param_t;
extern openart2_param_t openart2;//前摄（识别图片）

///*********************************************************************************************************
//  结构体定义--openart3(接收图片识别后的信息量)
//*********************************************************************************************************/
//typedef struct openart3_param_t 
//{
//    uint8 *error_array; 
//    enum
//    {
//        FH3,count3_one,count3_two,count3_three,count3_four,count3_five
//    } pack_order3;         // 打包顺序：帧头、坐标数量、
//} openart3_param_t;
//extern openart3_param_t openart3;//前摄（识别图片、矫正姿态）
/*********************************************************************************************************
  枚举定义
*********************************************************************************************************/
typedef enum
{
    CATEGORY_NULL   = 0,        //无类别
    DOG             = 7,        //狗
    HORSE           = 8,        //马
    CAT             = 9,        //猫
    CATTLE          = 10,       //牛
    PIG             = 11,       //猪
    ORANGE          = 13,       //橙子
    APPLE           = 14,       //苹果
    DURIAN          = 15,       //榴莲
    GRAPE           = 16,       //葡萄
    BANANA          = 17,       //香蕉
    TRAINS          = 19,       //火车
    STEAMSHIP       = 20,       //轮船
    PLANE           = 21,       //飞机
    CAR             = 22,       //小轿车
    COACH           = 23,       //大客车
    CARRY           = 54,       //搬运
}category_enum;

/*********************************************************************************************************
  枚举定义--小车每个时刻的状态
*********************************************************************************************************/

void openart_init(void);        //openart初始化
void openart1_send(void);       //openart1数据发送
void openart2_send(void);       //openart2数据发送
void recognition_picture(void);
void send_msg(uart_index_enum uartn, uint16 second, uint16 msecond, uint8 img_pos_x, uint8 img_pos_y, uint8 cg);


#endif