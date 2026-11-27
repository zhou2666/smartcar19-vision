/*
 * Barrier_left.c
 *
 *  Created on: 2023年11月11日
 *
 */
#include "barrier_left.h"
#include "bsp.h"
#include "cross.h"

struct BARRIER barrier_left;
float ka_barrier_left;
float kb_barrier_left;
double ka_left_barrier_lesat;
double kb_left_barrier_lesat;
uint8 barrier_left_single=0;
uint8 barrier_left_count=0;

void barrier_left_find(void)
{
    barrier_left.corner_flag=0;
    barrier_left.find_flag=0;
    monotone_decrease(10,Right_line_count-1); //判断右边边线是否单调递增
    if(straightlong_R_findflag==1&&monotone_decrease_flag==1)
    {
      vector_barrier_corner_left();
      if(barrier_left.corner_flag && barrier_left.corner_col >= 70)
      {
        barrier_left.find_flag=1;
        
      }
      if(barrier_left.find_flag&&barrier_left_count>=2)//识别到两帧黑块
      {                 //开启避障单边巡线
        barrier_right_single=1;
        Single_line=1;
        single_flag=1;    //黑块在左,巡右单边线  （中断中计步停止）
        barrier_left_count=0;
      }  
    }
}

//向量
double barrier_angle=0.0;
void vector_barrier_corner_left(void)
{
  
    float ax=0,ay=0;
    float bx=0,by=0;
    float a_model=0,b_model=0;
    if (Right_findflag)
    {
        for (uint8 i = 15; i < Left_line_count - 5; i++)
        {
            if(Left_line[i].row>15 && Left_line[i].row<70 && Left_line[i].col>20)
            {
               
                ax= Left_line[i-2].col - Left_line[i].col;
                ay= Left_line[i-2].row - Left_line[i].row;

                bx= Left_line[i+2].col - Left_line[i].col;
                by= Left_line[i+2].row - Left_line[i].row;

                if(ABS(ax)>1 && ABS(bx)>1 && ABS(ay)>=1 && ABS(by)>=1)
                {
                    a_model= sqrt(ax*ax+ay*ay);
                    b_model= sqrt(bx*bx+by*by);

                    barrier_angle = acos((ax*bx+ay*by)/(a_model*b_model)) * 180 / PI ;

                    if(barrier_angle > 90&&barrier_angle<170)
                    {                                             //   -0.75    0.49
                       if(kLa>-0.75&&kLa<-0.4&&kRa>0.3&&kRa<0.65)  //L -0.55  R 0.46
                       {  //左右斜率用处不大                                            //  -0.51    0.31
                         if(Right_line_count<=70)
                         {
//                         printf("L %d %d %d %d %d %d %d %f %d\r\n",Left_line[i-1].row,Left_line[i-2].row,Left_line[i-3].row,Left_line[i-4].row,Standard_Road_Wide[i-8]-(Right_line[i-8].col-Left_line[i-8].col),Standard_Road_Wide[i+5]-(Right_line[i+5].col-Left_line[i+5].col),Left_line[i].col,barrier_angle,(Left_line[i-1].col-Left_line[i-4].col));
                           if((Left_line[i-1].row-Left_line[i-2].row)<=1&&(Left_line[i-2].row-Left_line[i-3].row)<=1&&(Left_line[i-3].row-Left_line[i-4].row)<=1
                              &&(Left_line[i-1].col-Left_line[i-4].col)>=3&&(Standard_Road_Wide[i-8]-(Right_line[i-8].col-Left_line[i-8].col))<=3
                                &&(Standard_Road_Wide[i+5]-(Right_line[i+5].col-Left_line[i+5].col))>=4&&(Standard_Road_Wide[i+5]-(Right_line[i+5].col-Left_line[i+5].col))<=15)
                           { //根据摄像头的高度和角度不同需采集不同的Standard_Road_Wide
                             if(Left_line[i].col<=94&&Left_line[i].col>=70)
                             {
                               barrier_left.corner_col=Left_line[i].col;
                               barrier_left.corner_row=Left_line[i].row;
                               barrier_left.corner_flag=1;
                               barrier_left_count++;
                             }
                             break;
                           }
                         }
                       }
                    }
                }
            }
        }
    }
    
//        printf("LL=%f %f %f %f %f %f %f\r\n",ax,ay,bx,by,barrier_angle,a_model,b_model);
//        tft180_show_float(0,64,ax,3,2);
//        tft180_show_float(50,64,ay,3,2);
//        tft180_show_float(100,64,bx,3,2);
//        
//        tft180_show_float(0,80,by,3,2);
//        tft180_show_float(50,80,barrier_angle,3,2);
//        
//        tft180_show_float(100,80,a_model,3,2);
//        tft180_show_float(100,112,b_model,3,2);
//        tft180_show_string(70,112,"L"); //0
//        tft180_show_float(90,64,barrier_angle,3,2);
//        tft180_show_float(100,48,barrier_angle,3,2);
}