/*
 * Cross.c
 *
 *  Created on: 2024年4月15日
 *      Author: 58367
 */
#include "cross.h"
#include "define.h"
#include "math.h"
#include "My_math.h"

struct CROSS cross;

struct LEFT L_Up_line[100];//十字上方边界线
struct RIGHT R_Up_line[100];
uint8 L_Up_cnt;//十字上方边线的数量
uint8 R_Up_cnt;
float ka_cross;//十字左边补线斜率
float kb_cross;//十字左边补线斜率
float ka2_cross;//十字右边补线斜率
float kb2_cross;//十字右边补线斜率

//最小二乘法
double ka3_cross;//十字右边补线斜率
double kb3_cross;
double ka4_cross;//十字左边补线斜率
double kb4_cross;
int left_downchange_col=0;
int right_downchange_col=0;

int Road_width_70=0;
int Road_width_70_flag=0;

//uint8 mode=0;
uint8 cross_mode_flag=0;
int cross_preview_flag=0;
int cross_preview_count=0;

void cross_find()
{
  
   
    //左下拐角
    cross.cross_L_downfindflag = 0;
    //右下拐角
    cross.cross_R_downfindflag = 0;
    //左上拐点
    cross.cross_L_upcorner_flag = 0;
    //右上拐点
    cross.cross_R_upcorner_flag = 0;

    cross.cross_findflag = 0;
    straightlong_find();
    if(straightlong_L_findflag==0&&straightlong_R_findflag==0)
    {
    /**********************开始检测下拐点***************************/
    if (Left_findflag && Left_line_count>20&& Left_line[0].row>=50 && Left_line[Left_line_count-1].row>=10)
    {
        //左
        for (uint8 i = 4; i < Left_line_count-6; i++)
        {
            //边线先向左后向右
            if (Left_line[i].col-Left_line[i-2].col>=0&&Left_line[i].col-Left_line[i-4].col>=0&&Left_line[i].col-Left_line[i-6].col>=1&&\
                    Left_line[i].col-Left_line[i+2].col>=1&&Left_line[i].col-Left_line[i+4].col>=2&&Left_line[i].col-Left_line[i+6].col>=3)
            {
                cross.Left_downcorner_row = Left_line[i].row;
                cross.Left_downcorner_col = Left_line[i].col;
                cross.Left_downcorner_ordinal = i;
                if (cross.Left_downcorner_row>20&&cross.Left_downcorner_col<Max_col&&cross.Left_downcorner_col>45)
                {
//                  printf("L:hang=%d, lie=%d  \r\n",cross.Left_downcorner_row,cross.Left_downcorner_col);
                  cross.cross_L_downfindflag = 1;
                }
                break;
            }
        }
    }
    if (Right_findflag && Right_line_count>20 && Right_line[0].row>=50 && Right_line[Right_line_count-1].row >= 10)
    {
        //右
        for (uint8 i = 4; i < Right_line_count-6; i++)
        {
            if (Right_line[i-2].col-Right_line[i].col>=0&&Right_line[i-4].col-Right_line[i].col>=0&&Right_line[i-6].col-Right_line[i].col>=0&&\
                    Right_line[i+2].col-Right_line[i].col>=1&&Right_line[i+4].col-Right_line[i].col>=2&&Right_line[i+6].col-Right_line[i].col>=3)
            {
                cross.Right_downcorner_row= Right_line[i].row;
                cross.Right_downcorner_col = Right_line[i].col;
                cross.Right_downcorner_ordinal = i;
                if (cross.Right_downcorner_row>20&&cross.Right_downcorner_col>Max_col&&cross.Left_downcorner_col<145)
                {
//                  printf("R:hang=%d, lie=%d  \r\n",cross.Right_downcorner_row,cross.Right_downcorner_col);
                  cross.cross_R_downfindflag = 1;
                }
                break;
            }
        }
    }
//    if((cross.Right_downcorner_col-cross.Left_downcorner_col)<10)
//    {
//      cross.cross_L_downfindflag =0;
//      cross.cross_R_downfindflag =0;
//
//    }
  /**********************下拐点检测结束****************************/

    /************************丢线检测*******************************/
    /*
     * 判断拐点之后的边线情况
     * 若拐点之后完全空白，则说明是十字拐点
     * */
    //左拐点上方存在 7X7 白框
    if(cross.cross_L_downfindflag)
    {
        uint8_t black_flag = 0;
        for (int i = cross.Left_downcorner_row-2; i > cross.Left_downcorner_row-6; i--)//
        {
            for (int j = cross.Left_downcorner_col;j >= cross.Left_downcorner_col-7 ; j--)
            {
                if(j<=0) j=0;
                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
//                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 1000)) >= 20)
                {
                    black_flag = 1;
                    break;
                }
            }
            if (black_flag == 1)
            {
                break;
            }
        }
        if(black_flag==1)
            cross.cross_L_downfindflag = 0;
    }

    //右边
    if (cross.cross_R_downfindflag)
    {
        uint8_t black_flag = 0;
        for (int i = cross.Right_downcorner_row-2; i > cross.Right_downcorner_row-6; i--)//
        {
            for (int j = cross.Right_downcorner_col;j <= cross.Right_downcorner_col+7 ; j++)
            {
              
                if(j>187) j=187;
                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
//                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 1000)) >= 20)
                {
                    black_flag = 1;
                    break;
                }
            }
            if (black_flag == 1)
            {
                break;
            }
        }
        if(black_flag==1)
            cross.cross_R_downfindflag = 0;
    }
    /************************丢线检测结束****************************/
    /************************判断拐点情况****************************/
    if (cross.cross_L_downfindflag&&cross.cross_R_downfindflag)//拐点均存在
    {
        //直接截断
        if (cross.cross_findflag == 0)
            cross.cross_lost = 1;
        Left_line_count = cross.Left_downcorner_ordinal;
        Right_line_count = cross.Right_downcorner_ordinal;
        cross.cross_findflag = 1;
        kkkkkkk=10;
    }
    else if (cross.cross_L_downfindflag&&!cross.cross_R_downfindflag)//右拐点不存在
    {
        //判断右边 6 X 10是否完全空白
        uint8_t black_flag = 0;
        for (int i = 55; i > 40; i--)//
        {
            for (int j = 165;j <= 185 ; j++)
            {
                
                if(j>187) j=187;
                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
 //                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 1000)) >= 20)
                {
                    black_flag = 1;
                    break;
                }
            }

            if (black_flag == 1)
            {
                break;
            }
        }
        //确定是十字拐点
        if (black_flag==0)
        {
            //从拐点出截断左边线
            if (cross.cross_findflag == 0)
                cross.cross_lost = 2;
            Left_line_count = cross.Left_downcorner_ordinal;
            cross.cross_findflag = 1;
            kkkkkkk=11;

        }
    }
    else if (!cross.cross_L_downfindflag&&cross.cross_R_downfindflag)//左拐点不存在
    {
        //判断左边是否完全空白
        uint8_t black_flag = 0;
        for (int i = 55; i > 40; i--)//
        {
            for (int j = 3;j <= 23 ; j++)
            {
               
                if(j<0) j=0;
                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
//                if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 1000)) >= 20)
                {
                    black_flag = 1;
                    break;
                }
            }
            if (black_flag == 1)
            {
                break;
            }
        }
        //确定是十字拐点
        if (black_flag==0)
        {
            //从拐点出截断右边线
            if (cross.cross_findflag == 0)
                cross.cross_lost = 3;
            Right_line_count = cross.Right_downcorner_ordinal;
            cross.cross_findflag = 1;
            kkkkkkk=12;
        }
    }
   }
}

void cross_pro(void)
{
    if(cross.cross_findflag==1)
    {
        kkkkkkk=1010;
        cross_preview_flag=1;

        cross.Left_downcorner_col = 0;
        cross.Left_downcorner_row = 0;
        cross.cross_L_downfindflag = 0;

        cross.Right_downcorner_row = 0;
        cross.Right_downcorner_col = 0;
        cross.cross_R_downfindflag = 0;

        //如果确定十字状态，不再需要检测拐点上方白色区域，直接截断在拐点处，且不需要清除拐点坐标，防止拐点跳变
        /**********************十字开始检测下拐点***************************/
        if (Left_findflag && Left_line_count>10)
        {
            //左
            for (uint8 i = 4; i < Left_line_count-6; i ++)
            {
                //边线先向左后向右
                if (Left_line[i].col-Left_line[i-2].col>=0 && Left_line[i].col-Left_line[i-3].col>=0
                        &&  Left_line[i].col-Left_line[i+2].col>=1 && Left_line[i].col-Left_line[i+4].col>=2
                        &&  Left_line[i].col-Left_line[i+6].col >= 3 &&  Left_line[i].row > 15)
                {
                    cross.Left_downcorner_row = Left_line[i].row;
                    cross.Left_downcorner_col = Left_line[i].col;
                    cross.Left_downcorner_ordinal = i;
                    cross.cross_L_downfindflag = 1;
                    break;
                }
            }
        }
        if (Right_findflag && Right_line_count>10)
        {
            //右
            for (uint8 i = 4; i < Right_line_count-6; i ++)
            {
                if (Right_line[i-2].col - Right_line[i].col>=0 && Right_line[i-3 ].col-Right_line[i].col>=0
                        && Right_line[i+2].col - Right_line[i].col>=1 && Right_line[i+4].col-Right_line[i].col>=2
                        &&Right_line[i+6].col-Right_line[i].col>=3 && Right_line[i].row > 15)
                {
                    cross.Right_downcorner_row= Right_line[i].row;
                    cross.Right_downcorner_col = Right_line[i].col;
                    cross.Right_downcorner_ordinal = i;
                    cross.cross_R_downfindflag = 1;
//                    cross.cross_findflag = 0;
                    break;
                }
            }
        }
        //******************找左右 上拐点*****************//
        //1.下拐点都存在。最大白列row=15分别向左右找黑白跳变点
        //左右边界的第一个点、下拐点、上拐点;最小二乘法

        if(cross.cross_L_downfindflag && cross.cross_R_downfindflag)
        {
            if(cross.Left_downcorner_row > 20 && cross.Right_downcorner_row > 20)
            {
               kkkkkkk=1011;
               element_flag=1;
                //找左右上拐点
                if(cross.Left_downcorner_row>50 || cross.Right_downcorner_row>50)
                {
                    find_L_up_point(25,15,40);
                    find_R_up_point(25,15,150);
                }
                else
                {
                    find_L_up_point(25,15,40);
                    find_R_up_point(25,15,150);
                }

                least_sqaure_leftline(&ka3_cross,&kb3_cross);
                Left_line_count =  0;
                least_sqaure_rightline(&ka4_cross,&kb4_cross);
                Right_line_count = 0;
                //左边
                if(ka3_cross < 0 && ka3_cross > -1)
                {
                    if(cross.cross_L_upcorner_flag)
                        circle_L_fitting(Right_line[0].row, cross.cross_L_upcorner_row, ka3_cross, kb3_cross);
                    else
                        circle_L_fitting(Right_line[0].row,cross.cross_R_upcorner_row, ka3_cross, kb3_cross);
                   
                }
                else
                {
                    leftdown_change(&left_downchange_col,68);
                    find_xielv(Right_line[0].row,left_downchange_col,cross.Left_downcorner_row,cross.Left_downcorner_col, &ka_cross, &kb_cross);

                    if(ka3_cross <-1 && ka4_cross > 0 && ka4_cross < 1)
                        circle_L_fitting(Right_line[0].row,cross.cross_R_upcorner_row, ka_cross, kb_cross);
                    else
                        circle_L_fitting(Right_line[0].row,20, ka_cross, kb_cross);
                   
                }
                Left_findflag = 1;
                //右边
                if(ka4_cross > 0 && ka4_cross < 1)
                {
                    if(cross.cross_R_upcorner_flag)
                        circle_R_fitting(Left_line[0].row,cross.cross_R_upcorner_row, ka4_cross, kb4_cross);
                    else
                        circle_R_fitting(Left_line[0].row,cross.cross_L_upcorner_row, ka4_cross, kb4_cross);
                   
                }
                else
                {
                    rightdown_change(&right_downchange_col,68);
                    find_xielv(Left_line[0].row,right_downchange_col,cross.Right_downcorner_row,cross.Right_downcorner_col,&ka2_cross,&kb2_cross);

                    if(ka4_cross > 1 && ka3_cross < 0 && ka3_cross > -1)
                        circle_R_fitting(Left_line[0].row,cross.cross_L_upcorner_row,ka2_cross,kb2_cross);
                    else
                        circle_R_fitting(Left_line[0].row,20, ka2_cross, kb2_cross);
                 
                }
                Right_findflag = 1;
            }
        }
        //2.下拐点只存在一个。分别向左右找黑白跳变点
        //左/右边界的第一个点、下拐点、上拐点;最小二乘法

        //(1).左下拐点存在
        else if(cross.cross_L_downfindflag == 1 && cross.cross_R_downfindflag == 0)
        {
            kkkkkkk=1012;
            element_flag=1;
            if(cross.Left_downcorner_row > 20)
            {
                if(cross.Left_downcorner_row>50)
                {
                    find_L_up_point(25,15,40);
                    find_R_up_point(25,15,140);
                }
                else
                {
                    find_L_up_point(20,10,40);
                    find_R_up_point(20,10,140);
                }

                //左边
                least_sqaure_leftline(&ka3_cross,&kb3_cross);
                Left_line_count =  0;
                if(cross.cross_L_upcorner_flag)
                    circle_L_fitting(Right_line[0].row, cross.cross_L_upcorner_row, ka3_cross, kb3_cross);
                else
                    circle_L_fitting(Right_line[0].row,cross.cross_R_upcorner_row, ka3_cross, kb3_cross);
                Left_findflag = 1;

                //右边
                if(cross.cross_R_upcorner_flag)
                {
                    find_xielv(Right_line[0].row,150,cross.cross_R_upcorner_row, cross.cross_R_upcorner_col, &ka2_cross, &kb2_cross);
                    Right_line_count =0;
                    circle_R_fitting(Right_line[0].row,cross.cross_R_upcorner_row, ka2_cross, kb2_cross);
                    Right_findflag = 1;
                }
                else
                {
                    find_xielv(Right_line[0].row,150,cross.cross_L_upcorner_row, 140, &ka2_cross, &kb2_cross);
                    Right_line_count =0;
                    circle_R_fitting(Right_line[0].row,cross.cross_L_upcorner_row, ka2_cross, kb2_cross);
                    Right_findflag = 1;
                }
            }
        }
        //(2).右下拐点存在
        else if(cross.cross_L_downfindflag == 0 && cross.cross_R_downfindflag == 1)
        {
            kkkkkkk=1013;
            element_flag=1;
            if(cross.Right_downcorner_row > 20)
            {
                if(cross.Right_downcorner_row>50)
                {
                    find_L_up_point(25,15,40);
                    find_R_up_point(25,15,140);
                }
                else
                {
                    find_L_up_point(20,10,40);
                    find_R_up_point(20,10,140);
                }
                //左边
                if(cross.cross_L_upcorner_flag)
                {
                    find_xielv(Left_line[0].row,30,cross.cross_L_upcorner_row, cross.cross_L_upcorner_col, &ka_cross, &kb_cross);
                    Left_line_count = 0;
                    circle_L_fitting(Left_line[0].row, cross.cross_L_upcorner_row, ka_cross, kb_cross);
                    Left_findflag = 1;
                }
                else
                {
                    find_xielv(Left_line[0].row,30,cross.cross_R_upcorner_row, cross.cross_R_upcorner_col-20, &ka_cross, &kb_cross);
                    Left_line_count = 0;
                    circle_L_fitting(Left_line[0].row, cross.cross_R_upcorner_row, ka_cross, kb_cross);
                    Left_findflag = 1;
                }
                //右边
                least_sqaure_rightline(&ka4_cross,&kb4_cross);
                Right_line_count = 0;
                if(cross.cross_R_upcorner_flag)
                    circle_R_fitting(Right_line[0].row,cross.cross_R_upcorner_row, ka4_cross, kb4_cross);
                else
                    circle_R_fitting(Right_line[0].row,cross.cross_L_upcorner_row, ka4_cross, kb4_cross);
                Right_findflag = 1;
            }
        }
//
        //3.下拐点都不存在。最大白列row=15分别向左右找黑白跳变点
 //********************************十字结束*************************************//
        if((Road_width > 160 && cross.cross_L_downfindflag == 0 && cross.cross_R_downfindflag == 0)
           ||(Road_width > 100 && cross.cross_L_downfindflag && cross.Left_downcorner_row >= 60 && cross.cross_R_downfindflag && cross.Right_downcorner_row>=60)
           ||(Road_width > 100 && cross.cross_R_downfindflag && cross.cross_L_downfindflag == 0 && cross.Right_downcorner_row >= 55)
           ||(Road_width > 100 && cross.cross_L_downfindflag && cross.cross_R_downfindflag == 0 && cross.Left_downcorner_row >= 55))
        {
            cross.cross_findflag=2;
   
        }
    }
    else if(cross.cross_findflag==2)
    {
        //70行左右黑白跳变点   计算70行的列数
        leftdown_change(&left_downchange_col,60);
        rightdown_change(&right_downchange_col,60);
        Road_width_70 = right_downchange_col - left_downchange_col;
        if(Road_width_70<150)
        {
            Road_width_70_flag = 1;
//            mid_flag1=1;
        }
//       tft180_show_uint(120,48,Road_width_70,3);
       
       
//        if(Road_width < 160 && Left_findflag && Right_findflag&& (Left_line[9].col-Left_line[6].col) > 0 && (Left_line[11].col-Left_line[8].col) > 0 
//            && (Left_line[13].col-Left_line[10].col) > 0 && (Right_line[9].col-Right_line[6].col) < 0&& (Right_line[11].col-Right_line[8].col) < 0 && 
//              (Right_line[13].col-Right_line[10].col)<0&& Road_width_70_flag)
        if(Road_width < 160 && Left_findflag && Right_findflag&& (Left_line[6].col-Left_line[3].col) > 0 && (Left_line[8].col-Left_line[5].col) > 0 
           && (Right_line[6].col-Right_line[3].col) < 0&& (Right_line[8].col-Right_line[5].col) < 0 && Road_width_70_flag)
        {
            least_sqaure(&k_left,&k_right);
            least_sqaure_left(&ka5_cross,&kb5_cross);
            least_sqaure_right(&ka6_cross,&kb6_cross);  //用于判断十字前方走势 判断左右p环
        
            cross_preview_count++;
            cross_preview_flag=cross_preview_count;
            cross.cross_findflag=0;
            element_state=3;                       //圆环状态标志位。 正常循迹不执行任务时可注释
            kkkkkkk=0;
            start_circle=1;
            zebra_circle=1;
        }
        //左边
        find_L_up_point(30,20,60);
        find_xielv(65,40,cross.cross_L_upcorner_row, cross.cross_L_upcorner_col, &ka_cross, &kb_cross);
        Left_line_count = 0;
        circle_L_fitting(65, cross.cross_L_upcorner_row, ka_cross, kb_cross);
        Left_findflag = 1;
        //右边
        find_R_up_point(30,20,120);
        find_xielv(65,145,cross.cross_R_upcorner_row, cross.cross_R_upcorner_col, &ka2_cross, &kb2_cross);
        Right_line_count = 0;
        circle_R_fitting(65, cross.cross_R_upcorner_row, ka2_cross, kb2_cross);
        Right_findflag = 1;
    }
}

//***********************最小二乘法*******************************//
//  col = start_row * ka + kb
//  start_left_three_fitting_col = ka_left_1 * Left_line[0].row + kb_left_1;
//  end_left_three_fitting_col = ka_left_1 * cross.cross_L_upcorner_row + kb_left_1;
void least_sqaure_leftline(double *ka,double *kb)
{
    double left_fitting_three_col[3];
    double left_fitting_three_row[3];
    double k_left_three_fitting;
    double b_left_three_fitting;
    left_fitting_three_col[0] = Left_line[0].col;
    left_fitting_three_row[0] = Left_line[0].row;
    left_fitting_three_col[1] = cross.Left_downcorner_col;
    left_fitting_three_row[1] = cross.Left_downcorner_row;
    left_fitting_three_col[2] = cross.cross_L_upcorner_col;
    left_fitting_three_row[2] = cross.cross_L_upcorner_row;

    least_sqaure_method(left_fitting_three_col,left_fitting_three_row, 3,&k_left_three_fitting, &b_left_three_fitting);
    *ka = 1 / k_left_three_fitting * 1.0;
    *kb = - b_left_three_fitting / k_left_three_fitting * 1.0;

    return;
}
void least_sqaure_rightline(double *ka,double *kb)
{
    double right_fitting_three_col[3];
    double right_fitting_three_row[3];
    double k_right_three_fitting;
    double b_right_three_fitting;
    right_fitting_three_col[0] = Right_line[0].col;
    right_fitting_three_row[0] = Right_line[0].row;
    right_fitting_three_col[1] = cross.Right_downcorner_col;
    right_fitting_three_row[1] = cross.Right_downcorner_row;
    right_fitting_three_col[2] = cross.cross_R_upcorner_col;
    right_fitting_three_row[2] = cross.cross_R_upcorner_row;

    least_sqaure_method(right_fitting_three_col,right_fitting_three_row,3,&k_right_three_fitting, &b_right_three_fitting);
    *ka = 1 / k_right_three_fitting * 1.0;
    *kb = - b_right_three_fitting / k_right_three_fitting * 1.0;

    return;
}


void least_sqaure_left(double *ka,double *kb)
{
    double right_fitting_three_col[3];
    double right_fitting_three_row[3];
    double k_right_three_fitting;
    double b_right_three_fitting;
    right_fitting_three_col[0] = Left_line[50].col;
    right_fitting_three_row[0] = Left_line[50].row;
    right_fitting_three_col[1] = Left_line[55].col;
    right_fitting_three_row[1] = Left_line[55].row;
    right_fitting_three_col[2] = Left_line[60].col;
    right_fitting_three_row[2] = Left_line[60].row;

    least_sqaure_method(right_fitting_three_col,right_fitting_three_row,3,&k_right_three_fitting, &b_right_three_fitting);
    *ka = 1 / k_right_three_fitting * 1.0;
    *kb = - b_right_three_fitting / k_right_three_fitting * 1.0;

    return;
}
void least_sqaure_right(double *ka,double *kb)
{
    double right_fitting_three_col[3];
    double right_fitting_three_row[3];
    double k_right_three_fitting;
    double b_right_three_fitting;
    right_fitting_three_col[0] = Right_line[50].col;
    right_fitting_three_row[0] = Right_line[50].row;
    right_fitting_three_col[1] = Right_line[55].col;
    right_fitting_three_row[1] = Right_line[55].row;
    right_fitting_three_col[2] = Right_line[60].col;
    right_fitting_three_row[2] = Right_line[60].row;

    least_sqaure_method(right_fitting_three_col,right_fitting_three_row,3,&k_right_three_fitting, &b_right_three_fitting);
    *ka = 1 / k_right_three_fitting * 1.0;
    *kb = - b_right_three_fitting / k_right_three_fitting * 1.0;

    return;
}

//最小二乘法
void least_sqaure_method(const double *x, const double *y, int n, double *k, double *b)
{
    int  i;
    double  sumx , sumy, sumx2 ,sumxy ;
    sumx = 0.0;
    sumy = 0.0;
    sumx2 = 0.0;
    sumxy = 0.0;
    //计算N次
    for (i = 0; i < n; i++)
    {
        //将横坐标方向的x值进行累加
        sumx = sumx + x[i];
        //将纵坐标方向的y值进行累加
        sumy = sumy + y[i];
        sumx2 = sumx2 + pow(x[i], 2.0);
        sumxy = sumxy + (x[i] * y[i]);
    }
    // 根据公式求解b1和b0的值
    *k = (sumxy - ((sumx * sumy) / (double)n)) / (sumx2-( pow(sumx,2.0)/(double)n));
    *b = (sumy - ((*k) * sumx)) / (double)n;

    return;
}


void leftdown_change(int *col,int row)
{
    int j;
    for(j=94;j>10;j--)
    {
        if(judge_black(user_image[row][j-1])== 1&&judge_black(user_image[row][j])== 1
                &&judge_white(user_image[row][j+1])== 1&&judge_white(user_image[row][j+2])== 1)
        {
            *col=j;
            break;
        }
    }
}
void rightdown_change(int *col,int row)
{
    int j;
    for(j=94;j<180;j++)
    {
        if(judge_white(user_image[row][j-1])== 1&&judge_white(user_image[row][j])== 1
                &&judge_black(user_image[row][j+1])== 1&&judge_black(user_image[row][j+2])== 1)
        {
            *col=j;
            break;
        }
    }
}



void find_L_up_point(uint8 k1,uint8 k2,uint8 j1)
{
    uint8 flag_find_L_upcorner=0;//找到左上拐点
    if(Max_col>110)
        Max_col=110;//以防误判
    //左边
    for(uint8 k = k1 ; k > k2 ; k-- )
    {
       for (uint8 j = Max_col; j >= j1; j--)
       {
           if ((uint8_t)(judge_black(user_image[k][j])== 1) && (judge_black(user_image[k][j - 1]) == 1)
                  &&(judge_black(user_image[k][j - 2]) == 1)&&(judge_black(user_image[k][j + 1]) == 0)
                  &&(judge_black(user_image[k][j + 2]) == 0)&&(judge_black(user_image[k][j + 3]) == 0))
           {
               cross.cross_L_upcorner_flag = 1;
               cross.cross_L_upcorner_row = k - 1;
               cross.cross_L_upcorner_col = j;
               flag_find_L_upcorner = 1;
               break;
           }
       }
       if(flag_find_L_upcorner == 1)
           break;
    }
}

void find_R_up_point(uint8 k1,uint8 k2,uint8 j1)
{
    uint8 flag_find_R_upcorner=0;//找到右上拐点
    if(Max_col<80)
        Max_col=80;//以防误判
    //右边
    for(uint8 k = k1; k > k2; k--)
    {
        for (uint8 j = Max_col; j <= j1; j++)
        {
            if ((uint8_t)(judge_black(user_image[k][j + 1])== 1) && (judge_black(user_image[k][j + 2]) == 1)
                    &&(judge_black(user_image[k][j + 3]) == 1)&&(judge_black(user_image[k][j]) == 0)
                    &&(judge_black(user_image[k][j - 1]) == 0)&&(judge_black(user_image[k][j - 2]) == 0))
            {
                cross.cross_R_upcorner_flag = 1;
                cross.cross_R_upcorner_row = k - 1;
                cross.cross_R_upcorner_col = j;
                flag_find_R_upcorner=1;
                break;
            }
        }
        if(flag_find_R_upcorner==1)
            break;
    }
}
void circle_R_fitting(uint8_t start_row, uint8_t end_row, float ka, float kb)
{
    for (int i = Right_line_count; i < 100; i++)
    {
        if (start_row * ka + kb < 1)
        {
            Right_line[i].col = 1;
            Right_line[i].row = start_row;
        }
        else if (start_row * ka + kb >= 186)
        {
            Right_line[i].col = 186;
            Right_line[i].row = start_row;
        }
        else if ((start_row * ka + kb >= 0) && (start_row * ka + kb < 187))
        {
            Right_line[i].col = (uint8_t)(start_row * ka + kb);
            Right_line[i].row = start_row;
        }
        Right_line_count++;
        start_row--;
        if (start_row < end_row)
            break;
    }
}

//uint8_t start_col    uint8_t end_col  没有用上!!!!!!!!
void circle_L_fitting(uint8_t start_row, uint8_t end_row, float ka, float kb)
{
    for (int i = Left_line_count; i < 100; i++)
    {
        if (start_row * ka + kb < 1)
        {
            Left_line[i].col = 1;
            Left_line[i].row = start_row;
        }
        else if (start_row * ka + kb >= 187)
        {
            Left_line[i].col = 186;
            Left_line[i].row = start_row;
        }
        else if ((start_row * ka + kb >= 0) && (start_row * ka + kb < 187))
        {
            Left_line[i].col = (uint8_t)(start_row * ka + kb);
            Left_line[i].row = start_row ;
        }
        Left_line_count++;
        start_row--;
        if (start_row < end_row)
            break;
    }
}

//void cross_findpoint_Up (void)
//{
//    uint8_t L_col_start = 5, L_row_start = 55;
//    uint8_t R_col_start = IMAGEW - 5, R_row_start = 55;
//    //初始化
//    L_Up_cnt = 0;
//    R_Up_cnt = 0;
//    for (int i = 0; i < 100; i++)
//    {
//        L_Up_line[i].col = 0;
//        L_Up_line[i].row = 0;
//
//        R_Up_line[i].col = 0;
//        R_Up_line[i].row = 0;
//    }
//
//    //起始点初始化
//    if (cross.cross_R_downfindflag == 1 || cross.cross_L_downfindflag == 1)    //正入十字，从下拐点列往上扫
//    {
//        //从拐点上方三行开始扫线
//        R_col_start = cross.Right_downcorner_col+3;  //从拐点往右移三列
//        R_row_start = cross.Right_downcorner_row-3;
//
//        L_col_start = cross.Left_downcorner_col-3;    //从拐点往左移三列
//        L_row_start = cross.Left_downcorner_row-3;
//
//        //从下往上扫找左右边界，从右往左扫找左右边界
//        for (uint8 j = L_col_start; j <= 90; j++)//扫一半图像
//        {
//            for (uint8 i = L_row_start; i >= 10; i--)
//            {
//                if ((uint8_t) ((abs(user_image[i][j] - user_image[i - 1][j]) * 1.0
//                        / (user_image[i][j] + user_image[i - 1][j]) * 1.0 * 100)) >= 10)
//                {
//                    L_Up_line[L_Up_cnt].row = i - 1;
//                    L_Up_line[L_Up_cnt].col = j;
//                    L_Up_cnt++;
//                    break;
//                }
//            }
//            if (L_Up_cnt && L_Up_line[L_Up_cnt - 1].col >= 5 && L_Up_line[L_Up_cnt - 1].row <= 55 && L_Up_cnt < 100)
//                break;
//        }
//
//        //左手爬线寻找上边线
//        if (L_Up_cnt)
//            findline_lefthand_adaptive_up(user_image[0], 7, 3, L_Up_line[L_Up_cnt - 1].col, L_Up_line[L_Up_cnt - 1].row,&L_Up_cnt);
//
//        //寻找左上拐点
//        if (L_Up_cnt > 5)
//        {
//            for (int i = 3; i < L_Up_cnt - 4; i++)
//            {
//                //边线在该点之后，连续向上
//                if (L_Up_line[i].row - L_Up_line[i + 2].row >= 0 && L_Up_line[i].row - L_Up_line[i + 4].row >= 0
//                        && L_Up_line[i].row - L_Up_line[i + 6].row > 0 && L_Up_line[i + 1].row - L_Up_line[i + 3].row >= 0
//                        && L_Up_line[i + 1].row - L_Up_line[i + 5].row > 0
//                        && L_Up_line[i + 2].row - L_Up_line[i + 4].row >= 0)
//                {
//                    //拐点下方边线可能会误判
//                    if (L_Up_line[i].col > cross.Left_downcorner_col || cross.cross_L_downfindflag == 0)
//                    {
//                        cross.cross_L_upcorner_flag = 1;
//                        cross.cross_L_upcorner_row = L_Up_line[i].row;
//                        cross.cross_L_upcorner_col = L_Up_line[i].col;
//                        break;
//                    }
//                    else
//                    {
//                        cross.cross_L_upcorner_flag = 0;
//                        cross.cross_L_upcorner_row = 0;
//                        cross.cross_L_upcorner_col = 0;
//                    }
//                }
//            }
//        }
//        //寻找右边上边线拐点
//        for (uint8 j = R_col_start; j >= 95; j--)
//        {
//            for (uint8 i = R_row_start; i >= 10; i--)
//            {
//                if ((uint8_t) ((abs(user_image[i][j] - user_image[i - 1][j]) * 1.0
//                        / (user_image[i][j] + user_image[i - 1][j]) * 1.0 * 100)) >= 10)
//                {
//                    R_Up_line[R_Up_cnt].row = i - 1;
//                    R_Up_line[R_Up_cnt].col = j;
//                    R_Up_cnt++;
//                    break;
//                }
//            }
//            if (R_Up_cnt && R_Up_line[R_Up_cnt - 1].col <= 183 && R_Up_line[R_Up_cnt - 1].row <= 55 && R_Up_cnt < 100)
//                break;
//        }
//        //右手爬线寻找上边线
//        if (R_Up_cnt)
//            findline_righthand_adaptive_up(user_image[0], 7, 3, R_Up_line[R_Up_cnt - 1].col, R_Up_line[R_Up_cnt - 1].row,&R_Up_cnt);
//
//        //寻找右上拐点
//        if (R_Up_cnt > 5)
//        {
//            for (int i = 3; i < R_Up_cnt - 4; i++)
//            {
//                if (R_Up_line[i].row - R_Up_line[i + 2].row >= 0 && R_Up_line[i].row - R_Up_line[i + 4].row > 0
//                        && R_Up_line[i].row - R_Up_line[i + 6].row > 0 && R_Up_line[i + 1].row - R_Up_line[i + 3].row >= 0
//                        && R_Up_line[i + 1].row - R_Up_line[i + 5].row > 0
//                        && R_Up_line[i + 2].row - R_Up_line[i + 6].row >= 0)
//                {
//                    if (R_Up_line[i].col < cross.Right_downcorner_col || cross.cross_R_downfindflag == 0)
//                    {
//                        cross.cross_R_upcorner_flag = 1;
//                        cross.cross_R_upcorner_row = R_Up_line[i].row;
//                        cross.cross_R_upcorner_col = R_Up_line[i].col;
//                        break;
//                    }
//                    else
//                    {
//                        cross.cross_R_upcorner_flag = 0;
//                        cross.cross_R_upcorner_row = 0;
//                        cross.cross_R_upcorner_col = 0;
//                    }
//                }
//            }
//        }
//    }
//}
//
//void findline_lefthand_adaptive_up(uint8_t* image, int block_size, int clip_value, int x, int y, uint8_t* num)//左手迷宫巡线(十字左上拐点)
//{
//    int half = block_size / 2;
//    int step = 0, dir = 0, turn = 0;
//
//    while (step + *num < 100 && half < x && x < IMAGEW - half - 1 && half < y && y < IMAGEH - half - 1 && turn < 4) {
//        int local_thres = 0;
//        for (int dy = -half; dy <= half; dy++) {
//            for (int dx = -half; dx <= half; dx++) {
//                local_thres += user_image[y + dy][x + dx];
//            }
//        }
//        local_thres /= block_size * block_size;
//        local_thres -= clip_value;
//
//        int front_value = user_image[y + dir_front[dir][1]][x + dir_front[dir][0]];
//        int frontleft_value = user_image[y + dir_frontleft[dir][1]][x + dir_frontleft[dir][0]];
//        if (front_value < local_thres) {
//            dir = (dir + 1) % 4;
//            turn++;
//        }
//        else if (frontleft_value < local_thres) {
//            x += dir_front[dir][0];
//            y += dir_front[dir][1];
//            L_Up_line[step + *num].row = y;
//            L_Up_line[step + *num].col = x;
//            step++;
//            turn = 0;
//        }
//        else {
//            x += dir_frontleft[dir][0];
//            y += dir_frontleft[dir][1];
//            dir = (dir + 3) % 4;
//            L_Up_line[step + *num].row = y;
//            L_Up_line[step + *num].col = x;
//            step++;
//            turn = 0;
//        }
//    }
//    *num = step + *num;
//}
//void findline_righthand_adaptive_up(uint8_t* image, int block_size, int clip_value, int x, int y, uint8_t* num)//右手迷宫巡线(十字右上拐点)
//{
//    int half = block_size / 2;
//    int step = 0, dir = 0, turn = 0;
//    while (step + *num < 100 && half < x && x < IMAGEW - 1 - half && half < y && y < IMAGEH - 1 - half && turn < 4)
//    {
//        int local_thres = 0;
//        for (int dy = -half; dy <= half; dy++) {
//            for (int dx = -half; dx <= half; dx++) {
//                local_thres += user_image[y + dy][x + dx];
//            }
//        }
//        local_thres /= block_size * block_size;
//        local_thres -= clip_value;
//
//        int front_value = user_image[y + dir_front[dir][1]][x + dir_front[dir][0]];
//        int frontright_value = user_image[y + dir_frontright[dir][1]][x + dir_frontright[dir][0]];
//        if (front_value < local_thres) {
//            dir = (dir + 3) % 4;
//            turn++;
//        }
//        else if (frontright_value < local_thres) {
//            x += dir_front[dir][0];
//            y += dir_front[dir][1];
//            R_Up_line[step + *num].col = x;
//            R_Up_line[step + *num].row = y;
//            step++;
//            turn = 0;
//        }
//        else {
//            x += dir_frontright[dir][0];
//            y += dir_frontright[dir][1];
//            dir = (dir + 1) % 4;
//            R_Up_line[step + *num].row = y;
//            R_Up_line[step + *num].col = x;
//            step++;
//            turn = 0;
//        }
//    }
//    *num = step + *num;
//}
//
//
//void circle_R_fitting(uint8_t start_row, uint8_t start_col, uint8_t end_row, uint8_t end_col, float ka, float kb)
//{
//    for (int i = Right_line_count; i < 100; i++)
//    {
//        if (start_row * ka + kb < 1)
//        {
//            Right_line[i].col = 1;
//            Right_line[i].row = start_row;
//        }
//        else if (start_row * ka + kb >= 186)
//        {
//            Right_line[i].col = 186;
//            Right_line[i].row = start_row;
//        }
//        else if ((start_row * ka + kb >= 0) && (start_row * ka + kb < 187))
//        {
//            Right_line[i].col = (uint8_t)(start_row * ka + kb);
//            Right_line[i].row = start_row;
//        }
//        Right_line_count++;
//        start_row--;
//        if (start_row < end_row)
//            break;
//    }
//}
//
//void circle_L_fitting(uint8_t start_row, uint8_t start_col, uint8_t end_row, uint8_t end_col, float ka, float kb)
//{
//    for (int i = Left_line_count; i < 100; i++)
//    {
//        if (start_row * ka + kb < 1)
//        {
//            Left_line[i].col = 1;
//            Left_line[i].row = start_row;
//        }
//        else if (start_row * ka + kb >= 187)
//        {
//            Left_line[i].col = 186;
//            Left_line[i].row = start_row;
//        }
//        else if ((start_row * ka + kb >= 0) && (start_row * ka + kb < 187))
//        {
//            Left_line[i].col = (uint8_t)(start_row * ka + kb);
//            Left_line[i].row = start_row;
//        }
//        Left_line_count++;
//        start_row--;
//        if (start_row < end_row)
//            break;
//    }
//}
//
//
///*
// * 寻找十字左上拐点
// *
// * */
//void find_Cross_upcorner_Left(void)
//{
//    cross.cross_L_upcorner_flag=0;
////    uint8 row_start=0;
////    uint8 col_start=0;
////    uint8 row_end=0;
////    uint8 col_end=0;
////    if(cross.cross_findflag==2)
////    {
////        if(Left_findflag)
////        {
////            row_start=30;
////            col_start=Max_col;
////            row_end=Right_line[Right_line_count-1].row;
////            col_end=15;
////        }
////    }
////    for (uint8 col = col_start; col > col_end; col--)
////    {
////        for (uint8 row = row_start; row > row_end; row--)
////        {
////            if (judge_white(user_image[row][col])&&judge_white(user_image[row-1][col]) &&judge_black(user_image[row-2][col])&& judge_black(user_image[row-3][col]))
////            {
////                cross.cross_L_upcorner_row = row;
////                cross.cross_L_upcorner_col = col;
////                cross.cross_L_upcorner_flag = 1;
////                break;
////            }
////        }
////        if (circle_left.circle_upcorner_flag == 1)
////            break;
////    }
//       if (Left_findflag)
//       {
//           for (uint8 i = Left_line_count - 6; i > 6; i--)
//           {
//               if (Left_line[i].col - Left_line[i - 2].col >= 0 && Left_line[i].col - Left_line[i - 4].col >= 1 && Left_line[i].col - Left_line[i - 6].col >= 2&&\
//                   Left_line[i].row - Left_line[i + 2].row <= 0 && Left_line[i].col - Left_line[i + 4].row <= 1&& Left_line[i].row - Left_line[i + 6].row <= 2)
//               {
//                   if (Left_line[i].row > 10)
//                   {
//                       cross.cross_L_upcorner_row = Left_line[i].row;
//                       cross.cross_L_upcorner_col = Left_line[i].col;
//                       cross.cross_L_upcorner_ordinal = i;
//                       cross.cross_L_upcorner_flag = 1;
//                       break;
//                   }
//               }
//           }
//       }
//
//}
///*
// * 寻找十字右上拐点
// *
// * */
//void find_Cross_upcorner_Right(void)
//{
//    cross.cross_R_upcorner_flag=0;
//
//       if (Right_findflag)
//       {
//           for (uint8 i = Right_line_count - 6; i > 6; i--)
//           {
//               if (Right_line[i].col - Right_line[i - 2].col <= 0 && Right_line[i].col - Right_line[i - 4].col <= 1 && Right_line[i].col - Right_line[i - 6].col <= 2&&\
//                   Right_line[i].row - Right_line[i + 2].row >= 0 && Right_line[i].col - Right_line[i + 4].row >= 1&& Right_line[i].row - Right_line[i + 6].row >= 2)
//               {
//                   if (Right_line[i].row > 10)
//                   {
//                       cross.cross_R_upcorner_row = Right_line[i].row;
//                       cross.cross_R_upcorner_col = Right_line[i].col;
//                       cross.cross_R_upcorner_ordinal = i;
//                       cross.cross_R_upcorner_flag = 1;
//                       break;
//                   }
//               }
//           }
//       }
//
//}
//void Find_num_corner(void)
//{
//
//}
