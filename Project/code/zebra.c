#include "zebra.h"
#include "define.h"
uint8_t point_change = 0;      //斑马线符合标志位
uint8 point_change_white_L = 0;//白色斑马线个数
uint8 point_change_black_L = 0;//黑色斑马线个数
uint8 change_count=0;          //斑马线经过次数
uint8 pointchang_row=0;



void zebra_crossing()
{
   static int row_start=69;              //斑马线识别范围
   static int row_end=45;
   
   static int col_start=40;
   static int col_end=150;
   
   static int point_change_white_count=3; //黑白跳变点识别点数
   static int point_change_black_count=3;
//   static  int row_start=65;
//   static  int row_end =25;
//   static  int  col_start=50;
//   static  int  col_end=140;
//   static int point_change_white_count=7;
//   static int point_change_black_count=7;

   
   if(zebra_straight==0)  //有长直道限定  判断位长直道时进行斑马线判断  （常用 默认）
   {
     straightlong_find();
     if (straightlong_L_findflag == 1&&straightlong_R_findflag==1)
     {
      for (int row = row_start; row > row_end; row--)
      {
        for (int col = col_start; col < col_end; col++)
        {
          //由白到黑
          if (judge_white(user_image[row][col]) && judge_white(user_image[row][col+1])&&judge_black(user_image[row][col + 2]) )
          {
            point_change_white_L++;
          }
          //由黑到白
          else if (judge_black(user_image[row][col]) &&judge_black(user_image[row][col+1])&& judge_white(user_image[row][col + 2]))
          {
            point_change_black_L++;
          }

        }     
        //仔细观察图像，白到黑应该至少比黑到白多一个点
        if (point_change_white_L>point_change_white_count&&point_change_black_L>point_change_black_count&&point_change_white_L-point_change_black_L<=2)
        {
 
//           point_change++;
             point_change=1;
//           if(point_change>=3)
//           {
             change_count++;    //自增
             
             zebra_findflag=1;
             zebra_findflag_start=0;
             if(change_count==1)
             {
               zebra_jibu=1;   //第一次斑马线计部标志位
             }     
             
             if(change_count==2)
             {
               sprint++;
             }    
             if(change_count>=60)
             {
               change_count=60;
               
             } 
//            point_change=0;
//           }
          break;
        }
        else
        {
          point_change = 0;
          point_change_white_L = 0;
          point_change_black_L = 0;
        }
      }
     }
   }
   else if(zebra_straight==1)  //无长直道限定   （一般不用）
   {
  
         for (int row = row_start; row > row_end; row--)
         {
           for (int col = 40; col < 150; col++)
           {
             //由白到黑
             if (judge_white(user_image[row][col]) && judge_white(user_image[row][col+1])&&judge_black(user_image[row][col + 2]) )
             {
               point_change_white_L++;
             }
              //由黑到白
              else if (judge_black(user_image[row][col]) &&judge_black(user_image[row][col+1])&& judge_white(user_image[row][col + 2]))
              {
                point_change_black_L++;
              }                                                                                                          
           }                                                                                                             
          
         //仔细观察图像，白到黑应该至少比黑到白多一个点
         if (point_change_white_L>7&&point_change_black_L>7&&point_change_white_L-point_change_black_L<=2)
         {
           point_change = 1;
           change_count++;
           
           zebra_findflag=1;
           zebra_findflag_start=0;
           if(change_count==1)
           {
            zebra_jibu=1;  //第一次斑马线计部标志位
           }  
           if(change_count==2)
           {
             sprint++;
           }      
           break;
         }
         else
         {
           point_change = 0;
           point_change_white_L = 0;
           point_change_black_L = 0;
         }
      }
   }
  
//   if(change_count==1)
//   {
//       element_state=4;
//       point_change=0;
//       point_change = 0;
//       point_change_white_L = 0;
//       point_change_black_L=0;
//     
//   }
   
   if(change_count==1)
   {
//  element_state=4;
     point_change = 0;   //识别一次斑马线后改变识别范围 右近处变远
     point_change_white_L = 0;
     point_change_black_L = 0;     
     row_start=65;
     row_end =25;
     col_start=50;
     col_end=130;
     point_change_white_count=7;
     point_change_black_count=7;
     
   }
   
   else if(change_count==2)
   {
     element_state=4;
     point_change=0;//识别第二次斑马线后改变识别范围 由远处变近
     point_change_white_L = 0;
     point_change_black_L=0;
     point_change_white_count=4;
     point_change_black_count=4;
     row_start=68;
     row_end=45;
     col_start=40;
     col_end=150;
       
   }
   
   else if(change_count>=3)
   {
//     stop_flag=1;
   }
}
