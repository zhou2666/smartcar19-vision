/*
 * Barrier_left.c
 *
 *  Created on: 2023年11月11日
 *
 */
#include "barrier_right.h"
#include "bsp.h"
#include "cross.h"
#include "image.h"

struct BARRIER barrier_right;
float ka_barrier_right;
float kb_barrier_right;
double ka_right_barrier_lesat;
double kb_right_barrier_lesat;
uint8 barrier_right_single=0;
uint8 barrier_right_count=0;
uint8 monotone_increase_flag=0;
uint8 monotone_decrease_flag=0;

void barrier_right_find(void)
{
    barrier_right.corner_flag=0;
    barrier_right.find_flag=0;
    monotone_increase(10,Left_line_count-1);//左递减
    if(straightlong_L_findflag==1&&monotone_increase_flag==1)
    {
      vector_barrier_corner_right();
      if(barrier_right.corner_flag && barrier_right.corner_col <130)
      {
        barrier_right.find_flag=1; 
      }
      if(barrier_right.find_flag&&barrier_right_count>=2)//识别到两帧黑块
      {                  //开启避障单边巡线
        barrier_left_single=1;      
        Single_line=1;  
        single_flag=2;   //黑块在右,巡左单边线  （中断中计步停止）
        barrier_right_count=0;   
      }
    }    
}

//向量
double barrier_angle_right=0.0;
void vector_barrier_corner_right(void)
{
  
    float ax_2=0,ay_2=0;
    float bx_2=0,by_2=0;
    float a_model_2=0,b_model_2=0;
    if (Left_findflag)
    {
        for (uint8 i = 15; i < Right_line_count - 5; i++)
        {
            if(Right_line[i].row>15 && Right_line[i].row<70 && Right_line[i].col<160)
            {

                ax_2= Right_line[i-2].col - Right_line[i].col;
                ay_2= Right_line[i-2].row - Right_line[i].row;

                bx_2= Right_line[i+2].col - Right_line[i].col;
                by_2= Right_line[i+2].row - Right_line[i].row;

                if(ABS(ax_2)>1 && ABS(bx_2)>1 && ABS(ay_2)>=1 && ABS(by_2)>=1)
                {
                    a_model_2= sqrt(ax_2*ax_2+ay_2*ay_2);
                    b_model_2= sqrt(bx_2*bx_2+by_2*by_2);

                    barrier_angle_right = acos((ax_2*bx_2+ay_2*by_2)/(a_model_2*b_model_2)) * 180 / PI ;

                    if(barrier_angle_right >90&&barrier_angle_right<170)
                    {   
                      if(kLa>-0.8&&kLa<-0.3&&kRa>0.2&&kRa<0.7)
                      {
                        if(Left_line_count<=70)
                        {
//                       printf("R %d %d %d %d %d %d %d %f\r\n",Right_line[i-1].row,Right_line[i-2].row,Right_line[i-3].row,Right_line[i-4].row,(Right_line[i-8].col-Left_line[i-8].col)-Standard_Road_Wide[i-8],(Right_line[i+5].col-Left_line[i+5].col)-Standard_Road_Wide[i+5],Right_line[i].col,barrier_angle_right);
                          if((Right_line[i-1].row-Right_line[i-2].row)<=1&&(Right_line[i-2].row-Right_line[i-3].row)<=1&&(Right_line[i-3].row-Right_line[i-4].row)<=1&&(Right_line[i-4].col-Right_line[i-1].col)>=3
                             &&(Standard_Road_Wide[i-8]-(Right_line[i-8].col-Left_line[i-8].col))<=4&&(Standard_Road_Wide[i+5]-(Right_line[i+5].col-Left_line[i+5].col))>=4
                               &&(Standard_Road_Wide[i+5]-(Right_line[i+5].col-Left_line[i+5].col))<=15)
                           {
                            if(Right_line[i].col>=94&&Right_line[i].col<=125)
                            {//根据摄像头的高度和角度不同需采集不同的Standard_Road_Wid
                              barrier_right.corner_col=Right_line[i].col;
                              barrier_right.corner_row=Right_line[i].row;
                              barrier_right.corner_flag=1;
                              barrier_right_count++;
                              break;
                            } 
                          }
                        }
                      }
                    }
                }
            }   
        }                                   //2   //1   //-2  //-1    //180            //2.23        //2.23
    }                                       //-1  //2   //0   // -2   //161.56            //0(2.23)    //2
//        printf("RR=%f %f %f %f %f %f %f\r\n",ax_2,ay_2,bx_2,by_2,barrier_angle_right,a_model_2,b_model_2);
//        tft180_show_float(0,64,ax_2,3,2);        
//        tft180_show_float(50,64,ay_2,3,2);
//        tft180_show_float(100,64,bx_2,3,2);       //0
//        
//        tft180_show_float(0,80,by_2,3,2);    
//        tft180_show_float(50,80,barrier_angle_right,3,2);
//        
//        tft180_show_float(100,80,a_model_2,3,2);  //0
//        tft180_show_float(100,112,b_model_2,3,2); //0
//        tft180_show_string(70,112,"R");      
//        tft180_show_float(90,48,barrier_angle_right,3,2);
//        tft180_show_float(100,64,barrier_angle_right,3,2);

}

void monotone_increase(uint8 start_col,uint8 end_col)  //单调 左
{
  uint8 num=0;
  monotone_increase_flag=0;
  for(int i=start_col;i<end_col-1;i++)
  {
    if(Left_line[i].col<=Left_line[i+2].col)
    {
      num++;
//      printf("%d\r\n",num);
    }
    else
    {
      num=0;
      monotone_increase_flag=0;
    }
    if(num>=(end_col-start_col)*4/5)
    {
      monotone_increase_flag=1;
      break;
    }
  }
}

void monotone_decrease(uint8 start_col,uint8 end_col)  //单调  右
{
  uint8 num=0;
  monotone_decrease_flag=0;
  for(int i=start_col;i<end_col-1;i++)
  {
    if(Right_line[i].col>=Right_line[i+2].col)
    {
      num++;
    }
    else
    {
      num=0;
      monotone_decrease_flag=0;
    }
    if(num>=(end_col-start_col)*4/5)
    {
      monotone_decrease_flag=1;
      break;
    }
  }
}