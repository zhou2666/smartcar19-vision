/*
 * circle_left.c
 *
 * Created on: 2023年12月6日
 *    
 */
#include "circle_left.h"
#include "define.h"  
#include "bsp.h"
uint8 yh_run=1;

float angle=0;
struct CIRCLE circle_left;


//**************识别左圆环****************//
void circle_left_find()
{
    circle_left.circle_left_true = 0;
    circle_left.circle_right_true = 0;
    /****************检测右边情况*********************/
    straightlong_find();
    if(straightlong_R_findflag)
    {
        circle_left.circle_right_true = 1;//右边长直线，左边符合左圆环特征
        for(int k=3;k<Right_line_count-3;k++)//遍历右线不能有拐点以防误判
        {
            if(Right_line[k].col-Right_line[k+2].col<0 && Right_line[k].col-Right_line[k-4].col<0)
            {
                circle_left.circle_right_true = 0;
                break;
            }
        }
    } 
    //****************检测左边情况************************
    if(circle_left.circle_right_true && Left_findflag)
    {
        find_downcorner_Left();//下拐点
        find_whitecnt_Left();  //上方空白数量
        cov_Leftcircle();      //求右边边线拐点上下两部分以及左边线的协方差    
        
        if (circle_left.Length_white > LENGTH_WHITE_THRESHOLD)//下拐点上方空白长度符合
        {       
            if (cov_Leftcircle_R < -90 && cov_Leftcircle_L<0 && cov_Leftcircle_L_down > 0)
            {
                circle_left.circle_left_true = 1;//右边符合圆环特征   
            }
        }
    }   
    if(circle_left.circle_left_true && circle_left.circle_right_true )
    {
        circle_left.circle_findflag=1;
        kkkkkkk=1111;
    }
}
float Circle_left_yaw=0.0;

//*************处理左圆环*****************//
void circle_left_pro()
{
    //变量太多，入环上拐点与出环上拐点用的同一套函数与变量
    //1:识别到圆环 2:到达第一个岔路口白色区域 3：到达黑色区域 4：入环拉线
    if(circle_left.circle_findflag==1||circle_left.circle_findflag==2||circle_left.circle_findflag==3 ||element_state==1)//保持直行,识别入环时刻
    {
        circle_left_in();
        if(Right_findflag)
        {
           find_upcorner_Left();
            if(Right_findflag)
            {
                if(circle_left.circle_upcorner_flag)//连接上拐点（大致在上拐点的位置，不精细，但比较稳定可以找到）
                {   
                    find_xielv(Right_line[0].row,45,circle_left.circle_upcorner_row,circle_left.circle_upcorner_col, &ka_cross, &kb_cross);
                    Left_line_count =  0;
                    circle_L_fitting(Right_line[0].row,circle_left.circle_upcorner_row, ka_cross, kb_cross);//补线到与左边线平齐
                    Left_findflag = 1;
                }
            }
        }
    }
  
}

void circle_left_in()
{
    uint8_t black_flag = 0;
    for (int i = 65; i > 40; i--)//判断左边近处是否全白
    {
        for (int j = 18;j >= 8; j--)
        {
            if ((uint8_t)((abs(user_image[i][j] - user_image[i-2][j]) * 1.0 / (user_image[i][j] + user_image[i-2][j]) * 1.0 * 100)) >= 10)
            {
//           if ((uint8_t)((abs(user_image[i][j] - user_image[i-1][j]) * 1.0 / (user_image[i][j] + user_image[i-1][j]) * 1.0 * 1000)) >= 20)
                black_flag = 1;
                break;
            }
        }
        if (black_flag == 1)
        {
            break;
        }
    }
    if(black_flag==0)              //左边近处是否全白
    {
      if(Max_col>60)
      {
        circle_left.circle_findflag=2;//位于第一个岔路口，直行
        element_flag=1;
      }
    }
    if(circle_left.circle_findflag==2)
    {
      if(block_judge(40,45,25,30,20)==1)  //识别到圆环中心黑色区域黑像素点大于20 保证状态时机稳定
      {
        if(Left_findflag && Left_line_count>=20)
        {                                                                                  
//            if(Left_line[4].col-Left_line[1].col>1 && Left_line[6].col-Left_line[3].col>1&&Left_line[8].col-Left_line[5].col>1)//边线底部连续向右的趋势
//            {
//                circle_left.circle_findflag=3;//到达环岛黑色区域
//            }
          if(Left_line[3].col-Left_line[1].col>=1 && Left_line[4].col-Left_line[2].col>=1
             && Left_line[5].col-Left_line[3].col>=1 && Left_line[6].col-Left_line[4].col>=1)//边线底部连续向右的趋势
          {
            circle_left.circle_findflag=3;//到达环岛黑色区域
          }
        }
      }
    }
    else if(circle_left.circle_findflag==3)
    {
//        circle_left.circle_findflag=0;
        start_circle=1;
        zebra_circle=1;
//        Single_line=1;
//        single_flag=1;
        
        element_state=1;   //左环标志位
    }
}
/*
 * 找圆环下拐点
 *
 * */
void find_downcorner_Left(void)
{
    circle_left.circle_downcorner_flag = 0;
    if (Left_findflag)
    {
        for (uint8 i = 6; i < Left_line_count - 6; i++)
        {
            if (Left_line[i].col - Left_line[i-6].col >= 2 &&
                Left_line[i].col - Left_line[i+6].col >= 2 &&
                Left_line[i-6].row - Left_line[i].row >= 2 &&
                Left_line[i+6].row - Left_line[i].row >= 0)//边线先向右再向左，且下拐点为行数最小点
            {
                if (Left_line[i].row > 15)
                {
                    circle_left.circle_downcorner_row = Left_line[i].row;
                    circle_left.circle_downcorner_col = Left_line[i].col;
                    circle_left.circle_downcorner_ordinal = i;
                    circle_left.circle_downcorner_flag = 1;
                    break;
                }
            }
            if (Left_line[i].row < 15)
                break;
        }
    }
}

/*
 * 寻找圆环上拐点
 *
 * */
void find_upcorner_Left(void)
{
    circle_left.circle_upcorner_flag=0;
    uint8 row_start=0;
    uint8 col_start=0;
    uint8 row_end=0;
    uint8 col_end=0;
    
    if(circle_left.circle_findflag==1&&Right_findflag)
    {
        row_start=15;
        col_start=Right_line[Right_line_count-1].col-10;
//        if(Max_col>94)
//          col_start=94;
//        else
//          col_start=Max_col;
        row_end=1;
        col_end=70;
    }
    else if(circle_left.circle_findflag==2 && Right_findflag)
    {
       row_start=20;
       col_start=Right_line[Right_line_count-10].col-15;
       row_end=1;
       col_end=60;
    }
    else if(circle_left.circle_findflag == 3 && Right_findflag)
    {
        row_start=30;
        if(Max_col<95)
            col_start=Max_col;
        else
            col_start=IMAGEW/2-10;
//        col_start=Right_line[Right_line_count-10].col-20;
        row_end=1;
        col_end=50;
    }
        
    for (uint8 col = col_start; col > col_end; col--)
    {
        for (uint8 row = row_start; row > row_end; row--)
        {
            if (judge_white(user_image[row][col])&&judge_white(user_image[row-1][col]) &&judge_black(user_image[row-2][col])&& judge_black(user_image[row-3][col]))
            {
                circle_left.circle_upcorner_row = row;
                circle_left.circle_upcorner_col = col;
                circle_left.circle_upcorner_flag = 1;
                break;
            }
        }
        if (circle_left.circle_upcorner_flag == 1)
            break;
    }
}

/*
 * 找出环拐点
 *
 * */
void find_outcorner_Left(void)
{
    circle_left.circle_out_flag = 0;//默认不存在
    if (Right_findflag)
    {
        for (uint8 i = 6; i < Right_line_count - 6; i++)
        {
            if (Right_line[i].col - Right_line[i - 2].col <= 0 && Right_line[i].col - Right_line[i - 4].col <= 1 && Right_line[i].col - Right_line[i - 6].col <= 2&&\
                Right_line[i].col - Right_line[i + 2].col <= 0 && Right_line[i].col - Right_line[i + 4].col <= 1&& Right_line[i].col - Right_line[i + 6].col <= 2)
            {
                if (Right_line[i].row > 10)
                {
                    circle_left.circle_outcorner_row = Right_line[i].row;
                    circle_left.circle_outcorner_col = Right_line[i].col;
                    circle_left.circle_outcorner_ordinal = i;
                    circle_left.circle_out_flag = 1;
                    break;
                }
            }
        }
    }
}

/*
 * 协方差
 *
 * */
//左圆环协方差
float cov_Leftcircle_L=0;//左线的协方差
float cov_Leftcircle_R=0;//拐点上方的协方差
float cov_Leftcircle_L_down=0;//拐点下方的协方差
void cov_Leftcircle(void)//cov_Leftcircle_R < -200 && cov_Leftcircle_L<0 && cov_Leftcircle_L_down > 0
{
    cov_Leftcircle_L = 0;
    cov_Leftcircle_R = 0;
    cov_Leftcircle_L_down = 0;
    if (Left_findflag)
    {
        float L_ordinal_ave = 0, L_col_ave = 0;
        float L_cnt = 0;
        uint8 start = 0;

        if (circle_left.circle_downcorner_flag == 1)
            start = circle_left.circle_downcorner_ordinal;
        else
            start = 3;

        for (int i = start; i < Left_line_count; i++)
        {
            L_ordinal_ave += (float)i;
            L_col_ave += (float)Left_line[i].col;
            L_cnt++;
        }

        L_ordinal_ave /= L_cnt;
        L_col_ave /= L_cnt;

        for (int i = start; i < Left_line_count; i++)
        {
            cov_Leftcircle_L += (i - L_ordinal_ave) * (Left_line[i].col - L_col_ave);//左线的协方差
        }
        cov_Leftcircle_L /= (L_cnt - 1);

        if (circle_left.circle_downcorner_flag == 1)
        {
            start = 3;
            L_cnt = 0;
            L_ordinal_ave = 0;
            L_col_ave = 0;
            for (int i = start; i < circle_left.circle_downcorner_ordinal; i++)
            {
                L_ordinal_ave += (float)i;
                L_col_ave += (float)Left_line[i].col;
                L_cnt++;
            }

            L_ordinal_ave /= L_cnt;
            L_col_ave /= L_cnt;

            for (int i = start; i < circle_left.circle_downcorner_ordinal; i++)
            {
                cov_Leftcircle_L_down += (i - L_ordinal_ave) * (Left_line[i].col - L_col_ave);//拐点下方的协方差
            }
            cov_Leftcircle_L_down /= (L_cnt - 1);
        }
    }

    if (Right_findflag)
    {
        float R_ordinal_ave = 0, R_col_ave = 0;
        uint8 R_cnt = 0;
        uint8 start = 3;
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
            cov_Leftcircle_R += (i - R_ordinal_ave) * (Right_line[i].col - R_col_ave);//拐点上方的协方差
        }
        cov_Leftcircle_R /= (R_cnt - 1);
    }
}
/*
 * 下拐点上方白点个数
 *
 * */
void find_whitecnt_Left(void)
{
    uint8 white_flag = 0;//右边一整行均白标志
    circle_left.Length_white = 0;
    if (circle_left.circle_downcorner_flag == 1)//左下拐点存在
    {
        for (uint8 row = circle_left.circle_downcorner_row + 3; row > 10; row--)
        {
            for (uint8 col = circle_left.circle_downcorner_col; col > 15; col--)
            {
                if (judge_white(user_image[row][col]) && judge_white(user_image[row][col - 1]))
                    white_flag = 1;
                else
                    white_flag = 0;
            }
            if (white_flag == 1)
                circle_left.Length_white++;
            else if (white_flag == 0 && (circle_left.circle_downcorner_row + 3) - row > 10)//至少扫10行
                break;
        }
    }
}
