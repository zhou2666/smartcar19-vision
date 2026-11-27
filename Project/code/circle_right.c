/*
 * circle_right.c
 *
 *  Created on: 2023年12月8日
 *   
 */
#include "circle_right.h"
#include "define.h"  
#include "bsp.h"
struct CIRCLE circle_right;

//**************识别右圆环****************//
void circle_right_find()
{
    circle_right.circle_left_true = 0;
    circle_right.circle_right_true = 0;
    /****************检测左边情况*********************/
    straightlong_find();
    if(straightlong_L_findflag)
    {
        circle_right.circle_left_true = 1;//左边长直线，符合右圆环特征
        for(int k=3;k<Left_line_count-3;k++)//遍历左线不能有拐点以防误判
        {
            if(Left_line[k].col-Left_line[k+2].col>0 && Left_line[k].col-Left_line[k-4].col>0)
            {
                circle_right.circle_left_true = 0;
                break;
            }
        }
    }
    //****************检测右边情况************************
    if(circle_right.circle_left_true && Right_findflag)
    {
        find_downcorner();//下拐点
        find_whitecnt();//上方空白数量
        cov_Rightcircle();////求右边边线拐点上下两部分以及左边线的协方差
           
        if (circle_right.Length_white > LENGTH_WHITE_THRESHOLD)//下拐点上方空白长度符合
        {
            if (cov_Rightcircle_L > 100 && cov_Rightcircle_R>0 && cov_Rightcircle_R_down < 0)
                circle_right.circle_right_true = 1;//右边符合圆环特征
        }
    }

    if(circle_right.circle_left_true && circle_right.circle_right_true )
    {
        circle_right.circle_findflag=1;
        kkkkkkk=1211;
    }
}
float Circle_right_yaw=0.0;
//*************处理右圆环*****************//
void circle_right_pro()
{
    //变量太多，入环上拐点与出环上拐点用的同一套函数与变量
    //1:识别到圆环 2:到达第一个岔路口白色区域 3：到达黑色区域 4：入环拉线
    if(circle_right.circle_findflag==1||circle_right.circle_findflag==2 ||circle_right.circle_findflag==3||element_state==2)//保持直行,识别入环时刻
    {
        circle_right_in();
        if(Left_findflag)
        {
            find_upcorner();            
            if(circle_right.circle_upcorner_flag)//连接上拐点（大致在上拐点的位置，不精细，但比较稳定可以找到）
            {
                find_xielv(Left_line[0].row,135,circle_right.circle_upcorner_row,circle_right.circle_upcorner_col, &ka2_cross, &kb2_cross);
                Right_line_count =  0;
                circle_R_fitting(Left_line[0].row ,circle_right.circle_upcorner_row,ka2_cross, kb2_cross);//补线到与左边线平齐
                Right_findflag = 1;
            }
        }
    }
   
}
/*
*寻找入环时刻
 * */
void circle_right_in(void)
{
    uint8_t black_flag = 0;
    for (int i = 65; i > 40; i--)//右边近处全白
    {
        for (int j = 180;j >= 170; j--)
        {
            if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
//           if ((uint8_t)((abs(user_image[i][j] - user_image[i-1][j]) * 1.0 / (user_image[i][j] + user_image[i-1][j]) * 1.0 * 1000)) >= 20)
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
    if(black_flag==0)
    {
      if(Max_col<120)
      {
        circle_right.circle_findflag=2;//位于第一个岔路口，直行
        element_flag=1;
//          speed_falg1=1;
        nono_pic_falg=1;
      }
    }
    if(circle_right.circle_findflag==2)
    {
      if(block_judge(40,45,150,155,20)==1)  //识别到圆环中心黑色区域黑像素点大于20 保证状态时机稳定
      {
        if(Right_findflag && Right_line_count>=20)
        {
//            if(Right_line[1].col-Right_line[4].col>0 && Right_line[3].col-Right_line[6].col>0)//边线底部连续向左的趋势
//            {
          if(Right_line[1].col-Right_line[3].col>=1 && Right_line[2].col-Right_line[4].col>=1
             && Right_line[3].col-Right_line[5].col>=1 && Right_line[4].col-Right_line[6].col>=1)//边线底部连续向左的趋势
          {
            circle_right.circle_findflag=3;//到达环岛黑色区域            
          }
        }
      }
    }
    else if(circle_right.circle_findflag==3)
    {
//        circle_right.circle_findflag=0;
        start_circle=1;
        zebra_circle=1;
//        Single_line=1;
//        single_flag=2;
       
        element_state=2;     //右环标志位
    }
}
/*
 * 找圆环下拐点
 *
 * */
void find_downcorner(void)
{
    circle_right.circle_downcorner_flag = 0;
    if (Right_findflag)
    {
        for (uint8 i = 6; i < Right_line_count-6;i++)
        {
            if (Right_line[i - 6].col - Right_line[i].col >= 2 &&
                Right_line[i + 6].col - Right_line[i].col >= 2 &&
                Right_line[i - 6].row - Right_line[i].row >= 2 &&
                Right_line[i + 6].row - Right_line[i].row >= 0 )//边线先向左再向右，且下拐点为行数最小点
            {
                if (Right_line[i].row > 15)
                {
                    circle_right.circle_downcorner_row = Right_line[i].row;
                    circle_right.circle_downcorner_col = Right_line[i].col;
                    circle_right.circle_downcorner_ordinal = i;
                    circle_right.circle_downcorner_flag = 1;
                    break;
                }
            }
            if (Right_line[i].row <= 15)
                break;
        }
    }
}

void find_upcorner(void)
{
    circle_right.circle_upcorner_flag= 0;
    uint8 row_start=0;
    uint8 col_start=0;
    uint8 row_end=0;
    uint8 col_end=0;

   if(circle_right.circle_findflag == 1 && Left_findflag)
    {
        row_start=15;
        col_start=Left_line[Left_line_count-1].col+10;;
        row_end=1;
        col_end=120;
    }
    else if(circle_right.circle_findflag==2 && Left_findflag)
    {
        row_start=20;
        col_start=Left_line[Left_line_count-1].col+15;
        row_end=1;
        col_end=130;
    }
    else if(circle_right.circle_findflag==3 && Left_findflag)
    {
        row_start=30;
        if(Max_col>95)
            col_start=Max_col;
        else
            col_start=110;
        row_end=1;
        col_end=140;
    }
    for(uint8 j= col_start;j<col_end;j++)
    {
        for (uint8 i = row_start; i > row_end; i--)
        {
            if (judge_white(user_image[i][j])&&judge_white(user_image[i-1][j]) &&judge_black(user_image[i - 2][j])&& judge_black(user_image[i - 3][j]))
            {
                circle_right.circle_upcorner_flag=1;
                circle_right.circle_upcorner_col=j;
                circle_right.circle_upcorner_row=i;
                break;
            }
        }
        
        if(circle_right.circle_upcorner_flag)
        {
            break;
        }
    }
}

/*
 * 找出环拐点
 *
 * */
void find_outcorner(void)
{
    circle_right.circle_out_flag = 0;//默认不存在
    if (Left_findflag)
    {
        for (uint8 i = 6; i < Left_line_count - 6; i++)
        {
            if (Left_line[i].col - Left_line[i - 2].col >= 0 && Left_line[i].col - Left_line[i - 4].col >= 1 &&Left_line[i].col - Left_line[i - 6].col >= 2 && \
                Left_line[i].col - Left_line[i + 2].col >= 0 && Left_line[i].col - Left_line[i + 4].col >= 1&& Left_line[i].col - Left_line[i + 6].col >= 2)
            {
                if (Left_line[i].row > 10) //
                {
                    circle_right.circle_outcorner_row = Left_line[i].row;
                    circle_right.circle_outcorner_col = Left_line[i].col;
                    circle_right.circle_outcorner_ordinal = i;
                    circle_right.circle_out_flag = 1;
                }

            }
        }
    }
}

/*
 * 协方差
 *
 * */
float cov_Rightcircle_L = 0;
float cov_Rightcircle_R = 0;
float cov_Rightcircle_R_down = 0;
void cov_Rightcircle(void)
{
    cov_Rightcircle_L = 0;
    cov_Rightcircle_R = 0;
    cov_Rightcircle_R_down = 0;
    if (Left_findflag)
    {
        float L_ordinal_ave = 0, L_col_ave = 0;
        float L_cnt = 0;
        L_cnt = Left_line_count - 3;
        for (int i = 3; i < Left_line_count; i++)
        {
            L_ordinal_ave += (float)i;
            L_col_ave += (float)Left_line[i].col;
        }

        L_ordinal_ave /= L_cnt;
        L_col_ave /= L_cnt;

        for (int i = 3; i < Left_line_count; i++)
        {
            cov_Rightcircle_L += (i - L_ordinal_ave) * (Left_line[i].col - L_col_ave);
        }
        cov_Rightcircle_L /= (L_cnt - 1);
    }

    if (Right_findflag)
    {
        float R_ordinal_ave = 0, R_col_ave = 0;
        uint8_t R_cnt = 0;
        uint8_t start = 0;
        if (circle_right.circle_downcorner_flag)
            start = circle_right.circle_downcorner_ordinal;
        else
            start = 3;
        //拐点上方的协方差
        for (int i = start; i < Right_line_count; i++)
        {
            R_ordinal_ave += (float)i;
            R_col_ave += (float)Right_line[i].col;
            R_cnt++;
        }

        R_ordinal_ave /= R_cnt;
        R_col_ave /= R_cnt;

        for (int i = start; i < Right_line_count; i++)
        {
            cov_Rightcircle_R += (i - R_ordinal_ave) * (Right_line[i].col - R_col_ave);
        }
        cov_Rightcircle_R /= (R_cnt - 1);

        //拐点下方的协方差
        if (circle_right.circle_downcorner_flag)
        {
            start = 3;
            R_cnt = 0;
            R_ordinal_ave = 0;
            R_col_ave = 0;
            for (int i = start; i < circle_right.circle_downcorner_ordinal; i++)
            {
                R_ordinal_ave += (float)i;
                R_col_ave += (float)Right_line[i].col;
                R_cnt++;
            }

            R_ordinal_ave /= R_cnt;
            R_col_ave /= R_cnt;

            for (int i = start; i < circle_right.circle_downcorner_ordinal; i++)
            {
                cov_Rightcircle_R_down += (i - R_ordinal_ave) * (Right_line[i].col - R_col_ave);
            }
            cov_Rightcircle_R_down /= (R_cnt - 1);
        }
    }
}

/*
 * 下拐点上方右边一整行白点的行数
 *
 * */
void find_whitecnt(void)
{
    uint8_t white_flag = 0;//右边一整行均白标志
    circle_right.Length_white = 0;
    if (circle_right.circle_downcorner_flag == 1)//下拐点存在
    {
        for (uint8_t row = circle_right.circle_downcorner_row+3; row > 10; row--)
        {
            for (uint8_t col = circle_right.circle_downcorner_col; col < IMAGEW - 15; col++)
            {
                if (judge_white(user_image[row][col]) && judge_white(user_image[row][col + 1]))

                    white_flag = 1;
                else
                    white_flag = 0;
            }
            if (white_flag == 1)
                circle_right.Length_white++;
             if (white_flag == 0 && (circle_right.circle_downcorner_row + 3) - row > 10)//最多向上寻找十行
                break;
        }
    }
}
