/**********************************
 * image.h
 *
 *  Created on: 2023年12月6日
 *  
 **********************************/


#include "zf_common_headfile.h"
#include "define.h"


uint8 user_image[IMAGEH][IMAGEW];  //裁剪后的灰度图像
uint8 Binary_image[IMAGEH][IMAGEW];//裁剪后的二值化图像
uint8 Threshold=0;                 //动态阈值
uint8 speed_mode=0;
uint8 Left_line_count=0;
uint8 Left_findflag=0;

uint8 Right_line_count=0;
uint8 Right_findflag=0;

uint8 Mid_line_count=0;
uint8 Mid_findflag=0;

uint8 Left_findflag_point=0;
uint8 Right_findflag_point=0;
uint8 Left_row_start = IMAGEH-2,Right_row_start = IMAGEH-2; //左右边线搜线起点的行坐标
uint8 L_row_first, L_col_first, R_row_first, R_col_first;  //记录搜索到的基础边界点行列值
uint8 zebra_findflag;
float kRa,kRb,kLa,kLb;
struct LEFT  Left_line[100];                             //左边界结构体 图像60行，则最多寻找75个点
struct RIGHT Right_line[100];                            //右边界结构体
struct MID Mid_line[100];                                //中线结构体
struct MID_LAST Mid_Last_line[100];                      //上一次中线结构体
                                                          
struct LEFT  Left_line2[100];                            //左边界结构体2 图像60行，则最多寻找75个点
struct RIGHT Right_line2[100];                           //右边界结构体2
                                                          
const uint8 Standard_Road_Wide[IMAGEH]=                   
{ 
  82, 82, 80 ,79, 78, 77 ,76, 75 ,74 ,73 , //1
  72, 70, 70 ,69, 69, 69 ,67, 67 ,65 ,65 , //2
  63, 62, 62 ,60, 59, 58 ,57, 55 ,55 ,53 , //3
  53, 51, 50 ,50, 48, 47 ,45, 45 ,43 ,43 , //4
  41, 40, 39 ,38, 37, 36 ,35, 33 ,33 ,32 , //5
  31, 30, 28 ,28, 26, 26 ,25, 23 ,23 ,21 , //6
};


const uint8 Half_width[IMAGEH]=
{
 40, 40, 40, 39, 38, 38, 37, 37, 36, 35 ,
 35, 34, 34, 34, 34, 34, 33, 33, 32, 31 ,
 31, 30, 30, 29, 29, 28, 28, 27, 27, 26 ,
 25, 25, 24, 24, 23, 23, 22, 22, 21, 21 ,
 20, 19, 19, 18, 18, 17, 17, 16, 16, 15 ,
 15, 14, 14, 13, 13, 12, 12, 12, 11, 11 ,
};

/***********************************************************
 * 函数功能：对总钻风得到的图像进行裁剪
 * 返回值：无
 * *********************************************************/
void Get_userimage(void)
{
    for(int row=10;row<80;row++)
    {
        for(int col=4;col<184;col++)   //180
        {
            user_image[row-10][col] = mt9v03x_image[row][col];
        }
    }
}
/***********************************************************
 * 函数功能：**大津法求全局阈值
 * 返回值：threshold
 * *********************************************************/
uint8_t GetOSTUthreshold(void)
{
#define GrayScale 256
    int pixel[256] = { 0 };
    int threshold = 0;
    int32_t sum_gray = 0;   
    int32_t sum_pix_num = 0;
    int32_t pl_pix_num = 0;
    int32_t p2_pix_mum = 0;
    int32_t p1_sum_gray = 0;
    float m1 = 0;
    float m2 = 0;
    float V = 0;
    float variance = 0;
    int i, j, k = 0;

    for (i = 0; i < 256; i++)
        pixel[i] = 0;


    for (i = 0; i < IMAGEH; i++)
    {
        for (j = 0; j < IMAGEW; j++)
        {
            pixel[(int)user_image[i][j]]++;
        }
    }

    for (k = 0; k < GrayScale; k++)
    {
        sum_gray += k * pixel[k];
        sum_pix_num += pixel[k];
    }

    for (k = 0; k < GrayScale - 1; k++)
    {
        pl_pix_num += pixel[k];
        p2_pix_mum = sum_pix_num - pl_pix_num;
        p1_sum_gray += k * pixel[k];
        m1 = (float)p1_sum_gray / pl_pix_num;
        m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_mum;

        V = pl_pix_num * p2_pix_mum * (m1 - m2) * (m1 - m2);

        if (V > variance)
        {
            variance = V;
            threshold = k;
        }
    }
    return threshold;
}
/***********************************************************
 * 函数功能：灰度图像二值化（传入参数--二值化图像首地址--灰度图像首地址--阈值--图像行数--图像列数）
 * 返回值：无
 * *********************************************************/
void BinaryImage(uint8_t *image,uint8_t *tmImage, int ThresholdV,uint8_t row,uint8_t col)//灰度图像与二值化图像分辨率必须相同！！
{
    int i = 0, j = 0;
    for (i = 0; i < row; i++)
    {
        for (j = 0; j < col; j++)
        {
            if (*(tmImage+i* col +j) >= ThresholdV)
                *(image + i * col + j) = 0xff;
            else
                *(image + i * col + j) = 0x00;
        }
    }
}
/***********************************************************
 * 函数功能：清空边线
 * 返回值：无
 * *********************************************************/
void clear_line(void)
{
    for (int i = 0; i < Left_line_count; i++)
    {
        Left_line[i].row = 0;
        Left_line[i].col = 0;
    }
    for (int i = 0; i < Right_line_count; i++)
    {
        Right_line[i].row = 0;
        Right_line[i].col = 0;
    }
    for (int i = 0; i < Mid_line_count; i++)
    {
        Mid_line[i].row = 0;
        Mid_line[i].col = 0;
    }
    Left_line_count=0;
    Left_findflag=0;

    Right_line_count=0;
    Right_findflag=0;

    Mid_line_count=0;
    Mid_findflag=0;
    
}

/***********************************************************
 * 函数：Find_MaxLine()
 * 返回值：无
 * *********************************************************/

uint8_t count_white = 0;//最长列的白点个数
uint8_t Max_col = 0;   //图像白色区域最长列所在的列数
int8_t reference = 40;//大津法阈值补充（仅用于求最大列使用）//40  //60
void Find_MaxLine(void)
{
    uint8_t count = 0;//表示该列的白色点数
    uint8 j = 5;      //初始化从第七列开始计算
    uint8_t Basis=Threshold+reference;//基础白点的像素值，此值为图像的白色参考点
    count_white=0;
    Max_col=0;
    for (j = 5; j < 188; j += 5)
    {
        for (int i = IMAGEH - 1; i > 1; i--)
        {
            if ((uint8_t)((abs(user_image[i][j] - Basis) * 1.0 / (user_image[i][j] + Basis) * 1.0 * 100)) <= 10)//差比和求最大列
            {
                count++;
            }
            else
                break;
        }
        if (count >= count_white)
        {
            count_white = count;
            Max_col = j;
        }
//        tft180_show_uint(120,48,Max_col,3); ;
        count = 0;
    }
}

/***********************************************************
 * 函数功能：寻找扫线起点
 * 返回值 ：无
 * *********************************************************/
uint8 Left_line_start=0;//记录种子个数，将边线种子部分显示为绿色，爬线部分为红色
uint8 Right_line_start=0;//记录种子个数，将边线种子部分显示为绿色，爬线部分为红色
void find_start(void)
{
    Left_line_start=0;
    Right_line_start=0;
    //寻找左线起点
    Left_line_count = 0;
    for (uint8 i = IMAGEH - 1; i >= IMAGEH-50; i--)//种子最多寻找50行
    {
        for (uint8 j = Max_col; j > 1; j -= 1)
        {
            if ((uint8_t)((abs(user_image[i][j] - user_image[i][j - 2]) * 1.0 / (user_image[i][j] + user_image[i][j - 2]) * 1.0 * 100)) >= 18)
            {
                Left_line[Left_line_count].row = i;
                Left_line[Left_line_count].col = j;
                Left_line_count++;
                break;
            }
        }
        //        //防止强制找到种子，滤除错误边线(连续五行未找到种子 || 两个种子之间差五列以上)
        //        if(Left_line_count>1&&(abs(Left_line[Left_line_count-1].col-Left_line[Left_line_count-2].col)>=5||abs(Left_line[Left_line_count-1].row-Left_line[Left_line_count-2].row)>=5))
        //        {
        //            Left_line_count=0;
        //            break;
        //        }
        //限制种子不能在图像边缘，否则会导致爬线越界,当满足条件时跳出循环，得到种子
        if (Left_line_count>=2 && Left_line[Left_line_count-1].col >= 4&& Left_line[Left_line_count - 1].row <= 55)
        {
            Left_line_start=Left_line_count;//记录下左线种子个数
            break;
        }
        if(i == IMAGEH-50)//找到最后一行时仍没有跳出循环（即仍不符合条件）
        {
            Left_line_count = 0;//清除已找到的点数，跳出循环
            break;
        }
    }
    //寻找右线起点
    Right_line_count=0;
    for (uint8 i = IMAGEH - 1; i >= IMAGEH - 50; i--)
    {
        for (uint8 j = Max_col; j <184; j += 1)
        {

            if ((uint8_t)((abs(user_image[i][j] - user_image[i][j + 2]) * 1.0 / (user_image[i][j] + user_image[i][j + 2]) * 1.0 * 100)) >= 18)
            {
                Right_line[Right_line_count].row = i;
                Right_line[Right_line_count].col = j;
                Right_line_count++;
                break;
            }
        }
        //        //防止强制找到边线，滤除错误边线(连续五行未找到种子 || 两个种子之间差五列以上)
        //        if(Right_line_count>1&&(abs(Right_line[Right_line_count-1].col-Right_line[Right_line_count-2].col)>=5||abs(Right_line[Right_line_count-1].row-Right_line[Right_line_count-2].row)>=5))
        //        {
        //            Right_line_count=0;
        //            break;
        //        }
        //限制种子不能在图像边缘，否则会导致爬线越界
        if (Right_line_count >=2 && Right_line[Right_line_count-1].col <= 183&& Right_line[Right_line_count - 1].row <= 55)
        {
            Right_line_start=Right_line_count;//记录右线种子数
            break;
        }
        if(i == IMAGEH-50)//找到最后一行时仍没有跳出循环（即仍不符合条件）
        {
            Right_line_count = 0;//清除已找到的点数，跳出循环
            break;
        }
    }
//    tft180_show_uint(0,48,Right_line_start,3);
//    tft180_show_uint(30,48,Left_line_start,3);
 
}
/***********************************************************
 * 函数功能：左右手迷宫巡线算法（类似四邻域 参考上交大开源）
 * 返回值：无
 * *********************************************************/
const int dir_front[4][2] = { {0,  -1},{1,  0},{0,  1},{-1, 0} };
const int dir_frontleft[4][2] = { {-1, -1},{1,  -1},{1,  1},{-1, 1} };
const int dir_frontright[4][2] = { {1,  -1},{1,  1},{-1, 1},{-1, -1} };
       
void findline_lefthand_adaptive(uint8_t *image, int block_size, int clip_value, uint8 x, uint8 y,uint8_t* num)//左手迷宫巡线
{   //findline_lefthand_adaptive(user_image[0] ,7,4, Left_line[Left_line_count-1].col, Left_line[Left_line_count-1].row, &Left_line_count);
    int half = block_size / 2;
    int dir = 0, turn = 0;           //  3<x<184                          8<y<76 
    while (Left_line_count < 90&& half < x && x < IMAGEW - half - 1 && half+5< y && y < IMAGEH - half - 1 && turn < 4)
    {
        int local_thres = 0;
//        for (int dy = -half; dy <= half; dy++)         // for循环用来计算block区域的像素值之和
//        {
//            for (int dx = -half; dx <= half; dx++) 
//            {
//                local_thres += user_image[y + dy][x + dx];
//            }
//        }
//        local_thres /= block_size * block_size;
//        local_thres -= clip_value;                        // (x,y)点block区域内的阈值
       local_thres  = Threshold;
       
        int front_value = user_image[y + dir_front[dir][1]][x + dir_front[dir][0]];
        
        int frontleft_value = user_image[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];
        if (front_value < local_thres)                 // 前进方向像素为黑色
        {
            dir = (dir + 1) % 4;                      // 遇到前方为黑色需要右转一次
            turn++;
        }
        else if (frontleft_value < local_thres)       //前方像素为白色，且左前方像素为黑色
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Left_line[Left_line_count].row = y;        // 用来存放边线坐标信息
            Left_line[Left_line_count].col = x;
            Left_line_count++;
            turn = 0;
        }
        else                                         // 前方为白色，左前方为白色（墙角）
        {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;                   // 遇到墙角要斜着走
            Left_line[Left_line_count].row = y;
            Left_line[Left_line_count].col = x;
            Left_line_count++;
            turn = 0;
        }
    }
    *num = Left_line_count;
    if (Left_line_count > 5)
        Left_findflag = 1;
}
void findline_righthand_adaptive(uint8_t *image, int block_size, int clip_value, uint8 x, uint8 y,uint8_t* num)//右手迷宫巡线
{
    int half = block_size / 2;
    int dir = 0, turn = 0;

    while (Right_line_count < 90&& half < x && x < IMAGEW - 1- half && half+5< y && y < IMAGEH - 1-half && turn < 4)
    {
        int local_thres = 0;
//        for (int dy = -half; dy <= half; dy++)
//        {
//            for (int dx = -half; dx <= half; dx++) 
//            {
//                local_thres += user_image[y + dy][x + dx];
//            }
//        }
//        local_thres /= block_size * block_size;
//        local_thres -= clip_value;
        local_thres  = Threshold;

        int front_value = user_image[y + dir_front[dir][1]][x + dir_front[dir][0]];
        int frontright_value = user_image[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
        if (front_value < local_thres) 
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_value < local_thres) 
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            Right_line[Right_line_count].col= x;
            Right_line[Right_line_count].row = y;
            Right_line_count++;
            turn = 0;
        }
        else 
        {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            Right_line[Right_line_count].col= x;
            Right_line[Right_line_count].row = y;
            Right_line_count++;
            turn = 0;
        }
    }
    *num = Right_line_count;

    if (Right_line_count > 5)
        Right_findflag = 1;
}

/***********************************************************
 * 函数功能：制左右边线以及中线
 * 返回值：无
 * *********************************************************/
void draw_line(void)
{
    if (Left_findflag)
    {
        for (int i = 0; i < Left_line_count; i++)
        {
//            if(i<Left_line_start)
//                tft180_draw_point((uint8)(Left_line[i].col/2), (uint8)(Left_line[i].row/2), RGB565_GREEN);
//            else
                tft180_draw_point((uint8)(Left_line[i].col/2), (uint8)(Left_line[i].row/2), RGB565_RED);
        }
    }
    
    if (Right_findflag)
    {
        for (int i = 0; i < Right_line_count; i ++)
        {
//            if(i<Right_line_start)
                tft180_draw_point((uint8)(Right_line[i].col/2), (uint8)(Right_line[i].row/2), RGB565_GREEN);
//            else
//                tft180_draw_point((uint8)(Right_line[i].col/2), (uint8)(Right_line[i].row/2), RGB565_RED);
        }
    }
    if (Mid_line_count)
    {
        for (int i = 0; i < Mid_line_count; i ++)
        {
            tft180_draw_point((uint8)(Mid_line[i].col/2), (uint8)(Mid_line[i].row/2), RGB565_BLUE);
        }
        
//     if(L_Up_cnt)
//    {
//        for (int i = 0; i < L_Up_cnt; i ++)
//        {
//            tft180_draw_point((uint8)(L_Up_line[i].col/2), (uint8)(L_Up_line[i].row/2),RGB565_BLUE);
//        }
//    }
//    if(R_Up_cnt)
//    {
//        for (int i = 0; i < R_Up_cnt; i ++)
//        {
//            tft180_draw_point((uint8)(R_Up_line[i].col/2), (uint8)(R_Up_line[i].row/2), RGB565_BLUE);
//        }
//    }
    }
//    
    for (int i = 0; i < IMAGEH; i ++)
    {
        tft180_draw_point(90/2, i/2,RGB565_BLACK);
    }
    for (int i = 0; i < IMAGEH; i ++)
    {
        tft180_draw_point(60/2, i/2,RGB565_YELLOW);
    }
//     for (int i = 0; i < IMAGEH; i ++)
//    {
//        tft180_draw_point(120/2, i/2,RGB565_YELLOW);
//    }
//    for (int i = 0; i < IMAGEW-8; i ++)
//    {
//        tft180_draw_point(i/2, 40/2,RGB565_PURPLE);
//    }
//     for (int i = 0; i < IMAGEW-8; i ++)
//    {
//        tft180_draw_point(i/2, 20/2,RGB565_YELLOW);
//    }
//    for (int i = 0; i < IMAGEW-8; i ++)
//    {
//        tft180_draw_point(i/2, 10/2,RGB565_GREEN);
//    }
//    
    
//    for (int i = 0; i < IMAGEW-8; i ++)
//    {
//        tft180_draw_point(i/2, 50/2,RGB565_PURPLE);
//    }
//        for (int i = 0; i < IMAGEH; i ++)
//    {
//        tft180_draw_point(180/6, i/2,RGB565_BLACK);
//    }
//    for (int i = 0; i < IMAGEH; i ++)
//    {
//        tft180_draw_point(180/6*5, i/2,RGB565_BLACK);
//    }
//       for (int i = 0; i < IMAGEW; i ++)
//     {
//        tft180_draw_point(i,20,RGB565_BLACK);
//     }
}

/***********************************************************
 * 函数功能：边线处理
 * 返回值：无

 * *********************************************************/
uint8 Road_width=0;  //记录起始点宽度
uint8 L_high=0;
uint8 R_high=0;
uint8 Left_last_count=0; //记录左线上一次点数（用于单边丢线，限制点数，防止突变）
uint8 Right_last_count=0;//记录右线上一次点数（用于单边丢线，限制点数，防止突变）
uint8 Mid_last_count=0;  //记录中线上一次点数（用于单边丢线，限制点数，防止突变）

void sideline_gather(void)//边线采集，对爬取的边线每行只采集一个点
{
  
//     uint8 Left_line_count2=Left_line_count-2;
//    for(int i=Left_line_count;i>0;i--)
//    {
//        if(Left_line[i].row<Left_line[Left_line_count2].row)
//        {
//            Left_line_count2--;
//            Left_line[Left_line_count2].row=Left_line[i].row;
//            Left_line[Left_line_count2].col=Left_line[i].col;
//        }
//    }
//    
    uint8 Left_line_count2=0;
    for(int i=1;i<Left_line_count;i++)
    {
        if(Left_line[i].row<Left_line[Left_line_count2].row)
        {
            Left_line_count2++;
            Left_line[Left_line_count2].row=Left_line[i].row;
            Left_line[Left_line_count2].col=Left_line[i].col;
        }
    }
    for(int i=Left_line_count2;i<Left_line_count;i++)
    {
        Left_line[Left_line_count].row=0;
        Left_line[Left_line_count].col=0;
    }
    Left_line_count=Left_line_count2;

    uint8 Right_line_count2=0;
    for(int i=1;i<Right_line_count;i++)
    {
        if(Right_line[i].row<Right_line[Right_line_count2].row)
        {
            Right_line_count2++;
            Right_line[Right_line_count2].row=Right_line[i].row;
            Right_line[Right_line_count2].col=Right_line[i].col;
        }
    }
    for(int i=Right_line_count2;i<Right_line_count;i++)
    {
        Right_line[Right_line_count].row=0;
        Right_line[Right_line_count].col=0;
    }
    Right_line_count=Right_line_count2;
}


void sideline_process(void)//边线处理
{
    if(Left_findflag)
        L_high=Left_line[0].row - Left_line[Left_line_count-1].row;
    if(Right_findflag)
        R_high=Right_line[0].row- Right_line[Right_line_count-1].row;

    //左边界处理
    if (Left_findflag)//如果左边界存在并找到
    {
        //平滑边界
        for (int i = 1; i < Left_line_count - 1; i++)
        {
            Left_line[i].row = (Left_line[i].row + Left_line[i + 1].row) / 2;
            Left_line[i].col = (Left_line[i].col + Left_line[i + 1].col) / 2;
        }
        for (int i = Left_line_count - 1; i > 1; i--)
        {
            Left_line[i].row = (Left_line[i].row + Left_line[i - 1].row) / 2;
            Left_line[i].col = (Left_line[i].col + Left_line[i - 1].col) / 2;
        }
    }
    if (Right_findflag)//右边界存在且找到时开始处理
    {
        //平滑边界
        for (int i = 1; i < Right_line_count - 1; i++)
        {
            Right_line[i].row = (Right_line[i].row + Right_line[i + 1].row) / 2;
            Right_line[i].col = (Right_line[i].col + Right_line[i + 1].col) / 2;
        }
        for (int i = Right_line_count - 1; i > 1; i--)
        {
            Right_line[i].row = (Right_line[i].row + Right_line[i - 1].row) / 2;
            Right_line[i].col = (Right_line[i].col + Right_line[i - 1].col) / 2;
        }
    }
    //边界点太少，去掉
    if (Left_line_count < 5)
    {
        Left_line_count=0;
        Left_findflag = 0;
    }
    if (Right_line_count < 5)
    {
        Right_line_count=0;
        Right_findflag = 0;
    }
    //左边线起始点(列)大于右边线(左右边线起始处交叉，大概率有条边线出错)，将起始行大的边线去除
    if (Left_findflag && Right_findflag)
    {
        if (Left_line[0].col > Right_line[0].col)
        {
            if (Left_line[0].row > Right_line[0].row)
                Left_findflag = 0;
            else if (Left_line[0].row < Right_line[0].row)
                Right_findflag = 0;
        }
    }
////    如果左/右边界太往上且点数差距较大，删除边线
//        if (Left_line_count - Right_line_count > 50 && Right_findflag && Right_line_count < 30)  //左右边界数量差距太大
//        {
//            Right_findflag = 0;
//            Right_line_count = 0;
//        }
//    
//        if (Right_line_count - Left_line_count > 50 && Left_findflag && Left_line_count < 30)   //右>>左
//        {
//            Left_findflag = 0;
//            Left_line_count = 0;
//        }
//
//    if(Left_findflag && Right_findflag)
//    {
//        if(Left_line[0].row-Right_line[0].row>20)
//        {
//            Right_findflag=0;
//            Right_line_count=0;
//        }
//        else if(Right_line[0].row-Left_line[0].row>20)
//        {
//            Left_findflag=0;
//            Left_line_count=0;
//        }
//    }
    
//    // 边线几乎水平 删除边线
//    if (Left_line_count>20 && (Left_line[1].row - Left_line[Left_line_count - 1].row < 5))  // 左
//        Left_findflag = 0;
//    if (Right_line_count>20 && (Right_line[1].row - Right_line[Right_line_count - 1].row < 5))  //右
//        Right_findflag = 0;

    //记录第一行的赛道宽度
    if (Left_findflag && Right_findflag)
    {
        Road_width = Right_line[0].col - Left_line[0].col;
    }
//    tft180_show_uint(0,48,Right_line[0].col,3);
//    tft180_show_uint(30,48,Left_line[0].col,3);
//    tft180_show_uint(60,48,Right_line[50].col,3);
//    tft180_show_uint(90,48,Left_line[50].col,3);

//    tft180_show_uint(120,112,Road_width,3);
}
/*--------------------------------------------------------------------------
 * 【函数功能】：单边寻线
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：无
 *--------------------------------------------------------------------------*/

void Single_right()   //single_flag=1   
{
  //防止中线出错
  Right_line[0].row = Right_line[1].row;
  Right_line[0].col = Right_line[1].col;
  //初始化中线个数
  Mid_line_count=0;
  //通过边线相邻两个点的列坐标差来延伸已确定的第一个点
  int16 col = (int16)(Right_line[0].col-Road_width/2);
  if (col>IMAGEW-1)    col = IMAGEW-1;
  else if(col<0)  col = 0;
  Mid_line_count = Mid_line_count + 1;
  Mid_line[0].row = Right_line[0].row;
  Mid_line[0].col = (uint8)col;
  for(int i=1; i<Right_line_count;i++)
  {
    if(barrier_right_single==0)
    {
      col=Right_line[i].col-Half_width[i];
    }
    else if(barrier_right_single==1)
    {
      col=Right_line[i].col+20;
    }
   
    if (col>IMAGEW-1) col = IMAGEW-1;
    else if(col<0)  col = 0;  
    Mid_line_count = Mid_line_count + 1;
    Mid_line[i].row = Right_line[i].row;
    Mid_line[i].col = (uint8)col;
  } 
}
void Single_left()  //single_flag=2
{ 
  //防止中线出错
  Left_line[0].row = Left_line[1].row;
  Left_line[0].col = Left_line[1].col;
  //初始化中线个数 
  Mid_line_count=0;
  
  int16 col = (int16)(Left_line[0].col+Road_width/2);

  if (col>IMAGEW-1)    col = IMAGEW-1;
  else if(col<0)  col = 0;
  Mid_line_count = Mid_line_count + 1;
  Mid_line[0].row = Left_line[0].row;
  Mid_line[0].col = (uint8)col;
  
  for(int i=1; i<Left_line_count;i++)
  {
    if(barrier_left_single==0)
    {
     col=Left_line[i].col+Half_width[i];
    }
    else if(barrier_left_single==1)
    {
     col=Left_line[i].col+6;
    }
    if (col>IMAGEW-1)    col = IMAGEW-1;
    else if(col<0)  col = 0;  
    Mid_line_count = Mid_line_count + 1;
    Mid_line[i].row = Left_line[i].row;
    Mid_line[i].col = (uint8)col;
  } 
}



/*--------------------------------------------------------------------------
 * 【函数功能】：获取中线
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：无
 *--------------------------------------------------------------------------*/

void get_mid()
{
  
 if(Single_line==0)
 {
    //保存上一场中线数据
    for(int i = 0;i<Mid_line_count;i++)
    {
        Mid_Last_line[i].row = Mid_line[i].row;
        Mid_Last_line[i].col = Mid_line[i].col;
    }
    Mid_last_count = Mid_line_count;
    //情况1:左右边线均存在
    if(Left_findflag && Right_findflag)
    {
        //初始化中线个数
        Mid_line_count=0;
        //防止中线出错
        Left_line[0].row = Left_line[1].row;
        Left_line[0].col = Left_line[1].col;
        Right_line[0].row = Right_line[1].row;
        Right_line[0].col = Right_line[1].col;
        //左边线比右边线长 将长边第 Kx 个点与短边第 x 个点对应平均求中线
        if(Left_line_count>=Right_line_count)
        {
            float k = 1.0 * Left_line_count / Right_line_count;
            for(int i = 0;i<Right_line_count;i++)//(uint8)((abs(Right_line_count-Mid_last_count)/(Right_line_count+Mid_last_count))*Right_line_count)
            {
                Mid_line_count++;
                Mid_line[i].row = (uint8)((Left_line[(uint8)(k * i)].row + Right_line[i].row)/2);
                Mid_line[i].col = (uint8)((Left_line[(uint8)(k * i)].col + Right_line[i].col)/2);
            }
        }
        //右边线比左边线长  将长边第 Kx 个点与短边第 x 个点对应平均求中线
        else if(Left_line_count<Right_line_count)
        {
            float k = 1.0 * Right_line_count / Left_line_count;
            for(int i = 0;i<Left_line_count;i++)
            {
                Mid_line_count++;
                Mid_line[i].row = (uint8)((Left_line[i].row + Right_line[(uint8)(k * i)].row)/2);
                Mid_line[i].col = (uint8)((Left_line[i].col + Right_line[(uint8)(k * i)].col)/2);
            }
        }
    }
    //情况2:(右边界丢线,左边界不丢线) || 左边线数量 >> 右边线数量
    else if(Left_findflag==1&&Right_findflag==0)
    {
      Left_line[0].row = Left_line[1].row;
      Left_line[0].col = Left_line[1].col;
      //初始化中线个数
      Mid_line_count=0;
      int16 col = (int16)(Left_line[0].col + Road_width/2);
      if (col>IMAGEW-1)    col = IMAGEW-1;
      else if(col<0)  col = 0;
      Mid_line_count = Mid_line_count + 1;
      Mid_line[0].row = Left_line[0].row;
      Mid_line[0].col = (uint8)col;
      for(int i=1;i<Left_line_count;i++)//
      {
        col = (int16)(Mid_line[i-1].col+(Left_line[i].col-Left_line[i-1].col));
        if (col>IMAGEW-1)    col = IMAGEW-1;
        else if(col<0)  col = 0;
        Mid_line_count = Mid_line_count + 1;
        Mid_line[i].row = Left_line[i].row;
        Mid_line[i].col = (uint8)col;
      }
    }
    //情况3: ( 左边线丢线,右边线不丢线 ) || (左边线数量 << 右边线数量)
      else if(Left_findflag==0&&Right_findflag==1)
      {
        //防止中线出错
        Right_line[0].row = Right_line[1].row;
        Right_line[0].col = Right_line[1].col;
        //初始化中线个数
        Mid_line_count=0;
        //通过边线相邻两个点的列坐标差来延伸已确定的第一个点
        int16 col = (int16)(Right_line[0].col-Road_width/2);
        if (col>IMAGEW-1)    col = IMAGEW-1;
        else if(col<0)  col = 0;
        Mid_line_count = Mid_line_count + 1;
        Mid_line[0].row = Right_line[0].row;
        Mid_line[0].col = (uint8)col;
        for(int i=1;i<Right_line_count;i++)//(uint8)((1-abs(Right_line_count-Mid_last_count)*1.0/(Right_line_count+Mid_last_count))
        {
          col = (int16)(Mid_line[i-1].col-(Right_line[i-1].col-Right_line[i].col));
          if (col>IMAGEW-1)    col = IMAGEW-1;
          else if(col<0)  col = 0;
          Mid_line_count = Mid_line_count + 1;
          Mid_line[i].row = Right_line[i].row;
          Mid_line[i].col = (uint8)col;
        }
    }
      //中线平缓
    float k = 0.8;//1.0-abs(Left_line_count-Right_line_count)*1.0/(Left_line_count+Right_line_count)
    float k2;
    for(int i=0;i<Mid_line_count-1;i++)
    {
        Mid_line[i].row = (Mid_line[i].row + Mid_line[i+1].row)/2;
        Mid_line[i].col = (Mid_line[i].col + Mid_line[i+1].col)/2;
    }
    for(int i=Mid_line_count-1;i>1;i--)
    {
        Mid_line[i].row = (Mid_line[i].row + Mid_line[i-1].row)/2;
        Mid_line[i].col = (Mid_line[i].col + Mid_line[i-1].col)/2;
    }
    if(Mid_last_count>=Mid_line_count)
    {
        k2 = 1.0*Mid_last_count/Mid_line_count;
        for(int i=0;i<Mid_line_count;i++)
        {
            Mid_line[i].row = (uint8)((1-k)*Mid_Last_line[(uint8)(k2*i)].row + k*Mid_line[i].row);
            Mid_line[i].col = (uint8)((1-k)*Mid_Last_line[(uint8)(k2*i)].col + k*Mid_line[i].col);
        }
    }
    else if(Mid_last_count<Mid_line_count)
    {
        k2 = 1.0*Mid_line_count/Mid_last_count;
        for(int i=0;i<Mid_last_count;i++)
        {
            Mid_line[i].row = (uint8)((1-k)*Mid_Last_line[i].row + k*Mid_line[(uint8)(k2*i)].row);
            Mid_line[i].col = (uint8)((1-k)*Mid_Last_line[i].col + k*Mid_line[(uint8)(k2*i)].col);
        }
    }
  }
 
 
  else if(Single_line==1)  //任务结束单边巡线出环   
  {
   if(single_flag==1)   //左环  单右
   {
     Single_right();
   }
   else if(single_flag==2) //右环  单左
   {
     Single_left();  
   }
     //中线平缓
    float k = 0.8;//1.0-abs(Left_line_count-Right_line_count)*1.0/(Left_line_count+Right_line_count)
    float k2;
    for(int i=0;i<Mid_line_count-1;i++)
    {
        Mid_line[i].row = (Mid_line[i].row + Mid_line[i+1].row)/2;
        Mid_line[i].col = (Mid_line[i].col + Mid_line[i+1].col)/2;
    }
    for(int i=Mid_line_count-1;i>1;i--)
    {
        Mid_line[i].row = (Mid_line[i].row + Mid_line[i-1].row)/2;
        Mid_line[i].col = (Mid_line[i].col + Mid_line[i-1].col)/2;
    }
    if(Mid_last_count>=Mid_line_count)
    {
        k2 = 1.0*Mid_last_count/Mid_line_count;
        for(int i=0;i<Mid_line_count;i++)
        {
            Mid_line[i].row =(uint8)((1-k)*Mid_Last_line[(uint8)(k2*i)].row + k*Mid_line[i].row);
            Mid_line[i].col =(uint8)((1-k)*Mid_Last_line[(uint8)(k2*i)].col + k*Mid_line[i].col);
        }
    }
    else if(Mid_last_count<Mid_line_count)
    {
        k2 = 1.0*Mid_line_count/Mid_last_count;
        for(int i=0;i<Mid_last_count;i++)
        {
            Mid_line[i].row = (uint8)((1-k)*Mid_Last_line[i].row + k*Mid_line[(uint8)(k2*i)].row);
            Mid_line[i].col = (uint8)((1-k)*Mid_Last_line[i].col + k*Mid_line[(uint8)(k2*i)].col);
        }
    }
  }
 
}

/*--------------------------------------------------------------------------
 * 【函数功能】：最小二乘法
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：无
 *--------------------------------------------------------------------------*/

void least_sqaure(float *kL,float *kR) //左  右
{
    if (Left_findflag)
    {
        int num = Left_line_count;
        int sumX = 0,sumY = 0;
        float sumUP= 0,sumDown=0,avrX=0,avrY=0;
        for(int i = 0;i<num;i++)
        {
            sumX = sumX + Left_line[i].row;
            sumY = sumY + Left_line[i].col;
        }
        avrX = 1.0 * sumX / num;
        avrY = 1.0 * sumY / num;
        for(int i=0;i<num;i++)
        {
            sumUP = sumUP + (Left_line[i].col-avrY)*(Left_line[i].row - avrX);
            sumDown = sumDown + (Left_line[i].row-avrX)*(Left_line[i].row-avrX);
        }
        if(sumDown==0)
        {
            *kL = 0;
        }
        else
        {
            *kL = 1.0*sumUP/sumDown;
        }
    }

    if (Right_findflag)
    {
        int num = Right_line_count;
        int sumX = 0,sumY = 0;
        float sumUP= 0,sumDown=0,avrX=0,avrY=0;
        for(int i = 0;i<num;i++)
        {
            sumX = sumX + Right_line[i].row;
            sumY = sumY + Right_line[i].col;
        }
        avrX = 1.0 * sumX / num;
        avrY = 1.0 * sumY / num;
        for(int i=0;i<num;i++)
        {
            sumUP = sumUP + (Right_line[i].col-avrY)*(Right_line[i].row - avrX);
            sumDown = sumDown + (Right_line[i].row-avrX)*(Right_line[i].row-avrX);
        }
        if(sumDown==0)
        {
            *kR = 0;
        }
        else
        {
            *kR = 1.0*sumUP/sumDown;
        }
    }

}

/*--------------------------------------------------------------------------
 * 【函数功能】：图像初始化
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：无
 *--------------------------------------------------------------------------*/
void image_init()
{ 
  tft180_set_dir(TFT180_PORTAIT);          //TFT方向选择
  tft180_init();              
  tft180_show_string(0, 128, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())              //摄像头初始化
            tft180_show_string(0, 128, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);                                                  // 闪灯表示异常
    }
    tft180_show_string(0, 128, "init success.");
   
}

/*--------------------------------------------------------------------------
 * 【函数功能】：图像处理全流程
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：用于未发车前查看图像及参数
 *--------------------------------------------------------------------------*/

void pre_image(void) 
{
    Get_userimage();//裁剪图像至188X60
    clear_line();//清除左 右 中边线
    Threshold = GetOSTUthreshold();//得到全局阈值  
    Find_MaxLine();//寻找最大白点所在的列数
    if(count_white!=0)//白点数最多行的白点数不为0，防止近处全黑时扫出边线
    {
        find_start();//寻找种子
    }
    
    if (Left_line_count)//左 右迷宫巡线
        findline_lefthand_adaptive(user_image[0] , 7, 4, Left_line[Left_line_count-1].col, Left_line[Left_line_count-1].row, &Left_line_count);
    if (Right_line_count)
      findline_righthand_adaptive(user_image[0], 7, 4, Right_line[Right_line_count-1].col, Right_line[Right_line_count-1].row,&Right_line_count);
   
    sideline_process();//边线平滑，滤除错误边线
    sideline_gather();//边线采集，每行只采集一个数据
    get_mid();        //中线拟合
    
    if(Mid_line_count)//如果存在中线
    {
      get_mid_bias();
    }
    
    tft180_show_gray_image(0, 0, user_image[0], IMAGEW, IMAGEH, IMAGEW / 2, IMAGEH / 2, 0);
    draw_line();
    
    tft180_show_float(0,118,Mid_add,2,2);
    tft180_show_uint(60,112,Threshold,3);

    
    
}
/*--------------------------------------------------------------------------
 * 【函数功能】：图像处理全流程
 * 【参    数】：无
 * 【返 回 值】：无
 * 【备    注】：无
 *--------------------------------------------------------------------------*/

void deal_image(void)
{
    Get_userimage();//裁剪图像至188X60
    clear_line();//清除左 右 中边线
    Threshold = GetOSTUthreshold();//得到全局阈值  
    Find_MaxLine();//寻找最大白点所在的列数
    if(count_white!=0)//白点数最多行的白点数不为0，防止近处全黑时扫出边线
    {
        find_start();//寻找种子
    }
    
    if (Left_line_count)//左 右迷宫巡线
        findline_lefthand_adaptive(user_image[0] , 7, 4, Left_line[Left_line_count-1].col, Left_line[Left_line_count-1].row, &Left_line_count);
    if (Right_line_count)
        findline_righthand_adaptive(user_image[0], 7, 4, Right_line[Right_line_count-1].col, Right_line[Right_line_count-1].row,&Right_line_count);

//    tft180_show_uint(0,64,Right_findflag,1);
//    tft180_show_uint(30,64,Left_findflag,1);
//    tft180_show_uint(60,64,Right_line_count,3);
//    tft180_show_uint(90,64,Left_line_count,3);
     
     sideline_process();//边线平滑，滤除错误边线
     
     
     
//     if(zebra_findflag_start==1)   //发车后先进行斑马线识别，在进行元素识别，防止识别错乱
//     {
//       if(zebra_findflag==0)
//       {
//        zebra_crossing();      
//       }
//     }
//     
//    if(zebra_start==1)
//    {
      if(start_circle==1)     //重置标志位用于再次识别元素
      {
        circle_right.circle_findflag=0;
        circle_left.circle_findflag=0;
        cross.cross_findflag=0;
        start_circle=0;
      }
      if(zebra_circle==1)   //重置标志位用于再次识别斑马线
      {
        zebra_findflag=0;
      }
      if(sprint!=2)          //斑马线区域放完图片后关闭所有元素识别
      {  
          /*****************右环 **********************/      
        if(circle_right.circle_findflag==0 && circle_left.circle_findflag==0)
        {
          circle_right_find();
        }
    
        else if(circle_right.circle_findflag!=0)
        {  
          cross.cross_findflag=0;
          circle_right_pro();
          
        }
        /*****************左环 *********************/
        if(circle_right.circle_findflag==0 && circle_left.circle_findflag==0)
        {
          circle_left_find(); 
        }
        
        else if(circle_left.circle_findflag!=0)
        {
          cross.cross_findflag=0;
          circle_left_pro();
        }
        /*****************十字 *********************/
        if(cross.cross_findflag==0&&circle_right.circle_findflag==0 && circle_left.circle_findflag==0&&point_change==0)//元素互斥
        {
          cross_find();//十字判断
        }
        else if(cross.cross_findflag != 0)
        {
          cross_pro();//十字处理   
        }   
        /****************** 坡道识别****************/
        
        
//        if(tofa_per_flag==1)
//        {
//          straightlong_find();
//          if(straightlong_L_findflag==1&&straightlong_R_findflag==1)
//          {
//            tofa_flag=1;
//            tofa_per_flag=0;
//            tofa_start=0;
//          }
//        }
        /******************路障识别*******************/
        
//        if(cross.cross_findflag==0&&circle_right.circle_findflag==0 && circle_left.circle_findflag==0&&point_change==0)
//        {
//          if(barrier_start==1)
//          {
//            least_sqaure(&kLa,&kRa);
//            straightlong_find();
//            barrier_left_find(); 
//
//            barrier_right_find();    
//          }
//        }  
      }
        /****************** 斑马线识别****************/
      if(zebra_findflag==0)
      {
        zebra_crossing(); 
      }
      /***********************************************/  
//    }
      
      
      //102 //86 //70
//        tft180_show_uint(0,70,circle_left.circle_findflag,1);
//        tft180_show_uint(30,70,circle_left.circle_downcorner_row,3);
//        tft180_show_uint(60,70,circle_left.circle_downcorner_col,3);
//
//        tft180_show_uint(0,86,circle_left.circle_findflag,1);
//        tft180_show_uint(30,86,circle_left.circle_upcorner_row,3);
//        tft180_show_uint(60,86,circle_left.circle_upcorner_col,3);
//        tft180_show_uint(90,86,Max_col,3);

      
//         tft180_show_uint(0,70,circle_right.circle_findflag,1);
//         tft180_show_uint(30,70,circle_right.circle_downcorner_row,3);
//         tft180_show_uint(60,70,circle_right.circle_downcorner_col,3);
//
//         tft180_show_uint(0,86,circle_right.circle_findflag,1);
//         tft180_show_uint(30,86,circle_right.circle_upcorner_row,3);
//         tft180_show_uint(60,86,circle_right.circle_upcorner_col,3);
//         tft180_show_uint(90,86,Max_col,3);

         
//        tft180_show_uint(0,48,cross.cross_findflag,1); 
//        tft180_show_uint(90,48,point_change,1);
//        
//        tft180_show_float(0,64, ka5_cross,3,2);   
//        tft180_show_float(90,64,kb5_cross,3,2); 
//        tft180_show_float(0,48,k_left,3,2);
//        
//        tft180_show_float(0,80,ka6_cross,3,2);
//        tft180_show_float(90,80,kb6_cross,3,2);     
//        tft180_show_float(90,48,k_right,3,2);
//        
//        tft180_show_float(0,64,cov_Leftcircle_R,3,2);
//        tft180_show_float(60,64,cov_Leftcircle_L,3,2);
//        tft180_show_float(0,80,cov_Leftcircle_L_down,3,2);
//  
   
//        tft180_show_uint(0,48,Left_line_count,3);
//        tft180_show_uint(30,48,Right_line_count,3);
//        
//        
//        tft180_show_uint(0,64,cross.cross_L_downfindflag,1);
//        tft180_show_uint(30,64,cross.cross_R_downfindflag,1);
////        tft180_show_uint(90,64,cross.cross_findflag,1);      
////        tft180_show_uint(90,64,element_state,1);
//        
////        tft180_show_float(0,64,ka3_cross,3,2);
////        tft180_show_float(50,64,ka4_cross,3,2);
//        tft180_show_uint(60,64,kkkkkkk,4);
//        tft180_show_uint(100,64,cross.cross_findflag,1);
//////        
//        tft180_show_uint(0,80,cross.Left_downcorner_row,3);
//        tft180_show_uint(30,80,cross.Left_downcorner_col,3);
//        tft180_show_uint(60,80,cross.Right_downcorner_row,3);
//        tft180_show_uint(90,80,cross.Right_downcorner_col,3);
        
        
        
//        tft180_show_uint(0,48,Left_line[cross.Right_downcorner_ordinal-2].col,3);
//        tft180_show_uint(30,48,Left_line[cross.Right_downcorner_ordinal-4].col,3);
//        tft180_show_uint(60,48,Left_line[cross.Right_downcorner_ordinal-6].col,3);
//        tft180_show_uint(90,80,Left_line[cross.Right_downcorner_ordinal].col,3);
//        tft180_show_uint(0,80,Left_line[cross.Right_downcorner_ordinal+2].col,3);
//        tft180_show_uint(30,80,Left_line[cross.Right_downcorner_ordinal+4].col,3);       
//        tft180_show_uint(60,80,Left_line[cross.Right_downcorner_ordinal+6].col,3);
//        tft180_show_uint(90,80,Left_line[cross.Right_downcorner_ordinal+8].col,3);
      
//        tft180_show_float(0,54,kLa,3,2); 
//        tft180_show_float(50,54,kRa,3,2); 
//       
//        tft180_show_uint(0,70,straightlonglong_L_findflag,1);
//        tft180_show_uint(30,70,straightlonglong_R_findflag,1);
//      
//        tft180_show_uint(0,86,barrier_left_single,1);
//        tft180_show_uint(30,86,barrier_right_single,1);
//        
//        tft180_show_uint(60,86,monotone_increase_flag,1); //左
//        tft180_show_uint(90,86,monotone_decrease_flag,1); //右
        
        

        
        sideline_gather();//边线采集，每行只采集一个数据
        get_mid();        //中线拟合

        if(Mid_line_count)//如果存在中线
        {
            get_mid_bias();
        }

//        tft180_show_gray_image(0, 0, user_image[0], IMAGEW, IMAGEH, IMAGEW / 2, IMAGEH / 2, 0);//图像显示
//        draw_line();//画线函数
        
//        tft180_show_float(0,112,Mid_add,2,2);
//        tft180_show_uint(60,112,Threshold,3);
        
       
}