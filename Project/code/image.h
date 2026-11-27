/**********************************
 * image.h
 *
 *  Created on: 2023年12月6日
 *  
 **********************************/


#ifndef CODE_image_H_
#define CODE_image_H_  

#define IMAGEW 188
#define IMAGEH 70

#include "zf_common_headfile.h"


//***************************自定义变量********************************//
extern uint8 user_image[IMAGEH][IMAGEW];//裁剪后的灰度图像
extern uint8 Binary_image[IMAGEH][IMAGEW];//裁剪后的二值化图像
extern uint8 Threshold;//动态阈值

extern uint8 speed_mode;

extern uint8 Mid_line_count2;
//左 右 中 线特征结构体 -->>坐标 标志位 有效点个数
 struct LEFT
{
    uint8 row;      //左边线行坐标
    uint8 col;      //左边线列坐标
};
struct RIGHT
{
    uint8 row;      //右边线行坐标
    uint8 col;      //右边线列坐标
};
struct MID
{
    uint8 row;      //中线行坐标
    uint8 col;      //中线列坐标
};

struct MID_LAST
{
    uint8 row;      //上一次中线行坐标
    uint8 col;      //上一次中线列坐标
};
extern uint8 Left_line_count;
extern uint8 Left_findflag;

extern uint8 Right_line_count;
extern uint8 Right_findflag;

extern uint8 Mid_line_count;
extern uint8 Mid_findflag;

//extern uint8 start_circle;

extern uint8_t count_white;

extern struct LEFT  Left_line[100];                    //左边界结构体 图像60行，则最多寻找100个点
extern struct RIGHT Right_line[100];                   //右边界结构体
extern struct MID  Mid_line[100];                       //中线结构体
extern struct MID_LAST Mid_Last_line[100];                  //上一次中线结构体
extern const uint8 Standard_Road_Wide[IMAGEH];
extern int8_t reference;//大津法阈值补充（仅用于求最大列使用），根据现场情况，灯光越暗，此值越小

extern uint8 Left_line_start;//记录种子个数，将边线种子部分显示为蓝色，爬线部分为红色
extern uint8 Right_line_start;////记录种子个数，将边线种子部分显示为蓝色，爬线部分为红色
extern uint8 Left_last_count;//记录左线上一次点数（用于单边丢线，限制点数，防止突变）
extern uint8 Right_last_count;//记录右线上一次点数（用于单边丢线，限制点数，防止突变）
extern uint8 Mid_last_count;//记录中线上一次点数（用于单边丢线，限制点数，防止突变）
extern uint8 Road_width;//记录起始点宽度
extern uint8_t Max_col;//图像白色区域最长列所在的列数

extern uint8 Left_row_start;       //左右边线搜线起点的行坐标
extern uint8 Right_row_start;
extern uint8 search_start;
extern uint8 L_row_first, L_col_first, R_row_first, R_col_first;  //记录搜索到的基础边界点行列值
extern uint8 zebra_findflag;
extern float kRa,kRb,kLa,kLb;
//******************左右手迷宫巡线算法（参考上交大）**************************//
extern const int dir_front[4][2];
extern const int dir_frontleft[4][2];
extern const int dir_frontright[4][2];


//***************************自定义函数********************************//
extern void Get_userimage(void);//对总钻风得到的图像进行裁剪，保持宽度不变，图像上方裁剪10行，下方裁剪50行 188 X 120--->>188 X 60
extern uint8_t GetOSTUthreshold(void);//对最终使用的图像user_image,使用大津法返回动态阈值
extern void BinaryImage(uint8_t *image,uint8_t *tmImage, int ThresholdV,uint8_t row,uint8_t col);//灰度图像与二值化图像分辨率必须相同！！
extern void deal_image(void);//图像处理全流程
extern void pre_image(void);//图像处理全流程

extern void findline_righthand_adaptive(uint8_t *image, int block_size, int clip_value, uint8 x, uint8 y,uint8_t* num);//右手迷宫巡线
extern void findline_lefthand_adaptive(uint8_t *image, int block_size, int clip_value, uint8 x, uint8 y,uint8_t* num);//左手迷宫巡线
extern void draw_line(void);//在tft屏幕上画出边线
extern void clear_line(void);//清除左 右 中边线
extern void sideline_process(void);//左 右 边线处理
extern void sideline_gather(void);//边线采集，对爬取的边线每行只采集一个点

extern void Find_MaxLine(void);
extern void find_start(void);
extern void get_mid(void);
extern void least_sqaure(float *kL,float *kR);
extern void image_init(void);
#endif /* CODE_IMAGE_H_ */
