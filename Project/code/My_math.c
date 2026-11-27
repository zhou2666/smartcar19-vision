/*
 * My_math.c
 *
 *  Created on: 2023年12月6日
 *      
 */

#include "My_math.h"
//两点法求斜率
void find_xielv(int start_row, int start_col, int end_row, int end_col, float* ka, float* kb)
{
    *ka = (float)(end_col - start_col) / (end_row - start_row);
    *kb = (float)end_col - (*ka * end_row);
}
float abs_float(float num)
{
    num = num > 0 ? num : -num;
    return num;
}
/*
 * 平面三点共线判定（三阶行列式面积法）
 *
 * S:
 *      1、S>0：边线趋势向右
 *      2、S<0：边线趋势向左
 *
 * */
void find_area(int x1, int y1, int x2, int y2, int x3, int y3, float* S)
{
    *S = 0.5 * (x1 * y2 + x2 * y3 + x3 * y1 - x1 * y3 - x2 * y1 - x3 * y2);
    //    *S = abs_num(*S);
}
//长直线判断
float S_L=0;//左边线斜率
float S_R=0;//右边线斜率
float S_M=0;//中线斜率
uint8 straightlong_L_findflag=0;//左长直道标志位
uint8 straightlong_R_findflag=0;//右长直道标志位
void straightlong_find(void)
{
    straightlong_L_findflag=0;//左长直道标志位
    straightlong_R_findflag=0;//右长直道标志位
    int x1 = 0, x2 = 0, x3 = 0;
    int y1 = 0, y2 = 0, y3 = 0;
    if (Left_findflag&&Left_line[Left_line_count-1].row<=40)//高度大于图像1/3
    {
        x1 = Left_line[2 * Left_line_count / 3].row;
        y1 = Left_line[2 * Left_line_count / 3].col;
        x2 = Left_line[1 * Left_line_count / 3].row;
        y2 = Left_line[1 * Left_line_count / 3].col;
        x3 = Left_line[3].row;
        y3 = Left_line[3].col;
        find_area(x1, y1, x2, y2, x3, y3, &S_L);
    }
    if (Right_findflag&&Right_line[Right_line_count-1].row<=40)//高度大于图像1/3
    {
        x1 = Right_line[2 * Right_line_count / 3].row;
        y1 = Right_line[2 * Right_line_count / 3].col;
        x2 = Right_line[1 * Right_line_count / 3].row;
        y2 = Right_line[1 * Right_line_count / 3].col;
        x3 = Right_line[3].row;
        y3 = Right_line[3].col;
        find_area(x1, y1, x2, y2, x3, y3, &S_R);
    }
    if ((abs_float(S_L) < 60)&&(Left_line_count>20))//140°镜头畸变效果，面积要比正常情况大很多
        straightlong_L_findflag = 1;
    if ((abs_float(S_R) < 60)&&(Right_line_count>20))
        straightlong_R_findflag = 1;

    //二次判断,排除错误情况，左直道必须整体趋势向右，右直道必须整体趋势向左
        if(straightlong_L_findflag)
        {
          uint8 Max_col=Left_line[0].col;
          for(uint8 i=1;i<Left_line_count-1;i++)
          {
            if(Left_line[i].col>Max_col)
            {
                Max_col=Left_line[i].col;
            }
            else if(Max_col-Left_line[i].col>3)
            {
                straightlong_L_findflag=0;
                break;
            }
          }
        }
        if(straightlong_R_findflag)
        {
          uint8 Min_col=Right_line[0].col;
          for(uint8 i=1;i<Right_line_count-1;i++)
          {
            if(Right_line[i].col<Min_col)
            {
                Min_col=Right_line[i].col;
            }
            else if(Right_line[i].col-Min_col>3)
            {
                straightlong_R_findflag=0;
                break;
            }
          }
        }


}

/****************************************
 *函数功能：判断像素点的颜色
 *参数：该像素点的值
 *返回值：1或0
 ***************************************/
uint8 binimage_flag=1;//二值化标志位，默认没有进行二值化
uint8 judge_black(uint8 x)
{
    if (binimage_flag == 1)    //没有二值化
    {
        if (x <= Threshold)     return 1;//黑点
        else                    return 0;
    }
    else if (binimage_flag == 0)                    //进行了二值化
    {
        if (x == 0)     return 1;//黑点
        else            return 0;
    }
    return -1;
}
uint8 judge_white(uint8 x)
{
    if (binimage_flag == 1)
    {
        if (x > Threshold)     return 1;//白点
        else                   return 0;
    }
    else if (binimage_flag == 0)
    {
        if (x == 255)     return 1;//白色
        else            return 0;
    }
    return -1;
}
uint8 block_judge(int start_ROW, int end_ROW, int start_COL, int end_COL,int tmp)
{
  int block_cnt = 0;
  
    for (int i = start_ROW; i < end_ROW; i++) 
    {
      for (int j = start_COL; j < end_COL; j++) 
      {
        if (user_image[i][j]<Threshold)
          block_cnt++;
//        printf("%d \r\n",block_cnt);
      }
    }
    if (block_cnt <= tmp)
      return 0;
    return 1;
}
uint8 white_judge(int start_ROW, int end_ROW, int start_COL, int end_COL,int tmp)
{
  int white_cnt = 0;
  
    for (int i = start_ROW; i < end_ROW; i++) 
    {
      for (int j = start_COL; j < end_COL; j++) 
      {
        if (user_image[i][j]>Threshold)
          white_cnt++;
      }
    }
    if (white_cnt <= tmp)
      return 0;
    return 1;
}