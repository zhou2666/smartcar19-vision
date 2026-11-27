/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "define.h"
#include "bsp.h"
extern openart1_param_t openart1={0};
extern openart2_param_t openart2={0};
extern int ab;
extern int abd;
extern uint8 locate_now[2];
uint8 msg[18] = {0};

/*********************************************************************************************************
** Function name:           openart_init
** Descriptions:            openart初始化
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 

void openart_init(void) 
{
  fifo_init(&uart_data_fifo1, FIFO_DATA_8BIT, uart_get_data1, 64);               // 初始化 fifo 挂载缓冲区
  uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);                        // 初始化UART_1模块与引脚 
  uart_rx_interrupt(UART_1, ZF_ENABLE);                                         // 开启 UART_1的接收中断  
//  interrupt_set_priority(LPUART1_IRQn, 3);
                                                                        
  fifo_init(&uart_data_fifo4, FIFO_DATA_8BIT, uart_get_data4, 64);               // 初始化 fifo 挂载缓冲区
  uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);                        // 初始化UART_4模块与引脚 
  uart_rx_interrupt(UART_4, ZF_ENABLE);                                         // 开启 UART_4 的接收中断  
//  interrupt_set_priority(LPUART4_IRQn, 3);
  
//  fifo_init(&uart_data_fifo8, FIFO_DATA_8BIT, uart_get_data8, 64);               // 初始化 fifo 挂载缓冲区
//  uart_init(UART_8, 115200, UART8_RX_D16, UART8_RX_D17);                        // 初始化UART_4模块与引脚 
//  uart_rx_interrupt(UART_8, ZF_ENABLE);                                         // 开启 UART_4 的接收中断
}
/*********************************************************************************************************
** 函数简介     UART_1 的接收中断处理函数 这个函数将在 UART_1 对应的中断调用 详见 isr.c
** 参数说明     void
** 返回参数     void
** 使用示例     uart_rx_interrupt_handler1();      
*********************************************************************************************************/ 
void uart_rx_interrupt_handler1(void)
{ 
//    get_data = uart_read_byte(UART_1);                                          // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_1, &get_data1);                                      // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo1, &get_data1, 1);                     // 将数据写入 fifo 中
    fifo_data_count1 = fifo_used(&uart_data_fifo1);                         // 查看 fifo 是否有数据
    if(fifo_data_count1 != 0)                                                // 读取到数据了
     {
      fifo_read_buffer(&uart_data_fifo1, fifo_get_data1, &fifo_data_count1, FIFO_READ_AND_CLEAN);  // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
      fifo_openart2_rx_buffer=fifo_get_data1[0];
     if(fifo_openart2_rx_buffer == 0XFB)
     {
//       lz_flag=1;
     }
     else if(fifo_openart2_rx_buffer == 0XFC) //识别种类
     {
//       printf("%x\n",fifo_openart2_rx_buffer);
       openart2.pack_order2 = count2_one;
     }
     else if(fifo_openart2_rx_buffer == 0XFE) //识别字母
     {
//       printf("%x\n",fifo_openart2_rx_buffer);
       openart2.pack_order2 = count2_two;
     }
     
      else if(fifo_openart2_rx_buffer == 0XFD)//矫正完成
     {         
  //      openart1_jz_flag==0;
  //      openart2_jz_flag==0;
       pid_Init_A(); 
       wk_y=outputL[0];
       //        wk_x=outputL[1];
       bl_x=outputL[1];
       if(direction==0)
         run_distance=run_distance-bl_x;
       else if(direction==1)
         run_distance=run_distance+bl_x;
       if(ABS(wk_y)<0.05)
       {
         wk_y=SIGN(wk_y)*0.08;
       }
       if(ABS(bl_x)<0.05)
       {
         bl_x=SIGN(wk_y)*0.08;
       }
       outputL[0] = 0;
       outputL[1] = 0;
       speed_output[0]=0;
       speed_output[1]=0;
       speed_output[2]=0;
       time_flag1=0;
       check =7;
       abb=0;
       bbc=0;
       location_state = 1;
       flag_pid_error=false_;
       smartcar_state = 0;
       openart1.pack_order1=FH1;
//       wait_identify_flag=1;
       jz_success=1;
//       kk=0;
     }
     else if(fifo_openart2_rx_buffer==0XF6)//图片堆捡完
     {
       smartcar_state=0;
//       kk=10086;
       pick_complete=1;
       gpio_low(B15);
//       lz_flag=0;
     }
     else
     {
       switch(openart2.pack_order2)
       {
       case count2_one://接受图片的种类
         {
           species_picture = fifo_openart2_rx_buffer;
           wait_identify_picture_flag=1;
           openart2.pack_order2 = FH2; 
           break ;
         }
         //*****************************************************************************//
       case count2_two://接收字母的种类
         {
           species_alphabet =fifo_openart2_rx_buffer;
           openart2.pack_order2 = FH2; 
           wait_identify_letter_flag=1;
           jz_success=0;
           break ;
         }
       default:break;
       }
     }
     }
}
/*********************************************************************************************************
** 函数简介     UART_4 的接收中断处理函数 这个函数将在 UART_4 对应的中断调用 详见 isr.c
** 参数说明     void
** 返回参数     void
** 使用示例     uart_rx_interrupt_handler4();
*********************************************************************************************************/ 
extern int abw;
void uart_rx_interrupt_handler4(void)
{ 
//    get_data4 = uart_read_byte(UART_4);                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_4, &get_data4);                       // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo4, &get_data4, 1);         // 将数据写入 fifo 中                
    fifo_data_count4 = fifo_used(&uart_data_fifo4);             // 查看 fifo 是否有数据
    if(fifo_data_count4 != 0)                                   // 读取到数据了
    {
      fifo_read_buffer(&uart_data_fifo4, fifo_get_data4, &fifo_data_count4, FIFO_READ_AND_CLEAN);  // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
      fifo_openart1_rx_buffer=fifo_get_data4[0];
      //     uart_write_buffer(UART_4,fifo_get_data4,fifo_data_count4);
      static int num =0;
      static int n = 0;
     //      static int m = 0;
     if (fifo_openart1_rx_buffer == 0XFF)
     {                //处于接收帧头状态
       num = 0;
       openart1.pack_order1 = count1_one;
     }
     else if(fifo_openart1_rx_buffer== 0XF1)
     {
       start_flag = 1; //位置环进入摄像头调节状态//初始化完成标志位
     }
//     else if(fifo_openart1_rx_buffer == 0xF2)
//     {                  //看到左边图片
//       dispersive_picture=1;
//       dispersive_angle=90;
//       smartcar_state=13;
//     }
//     else if(fifo_openart1_rx_buffer == 0xF3)
//     {                  //看到右边图片
//       dispersive_picture=2;
//       dispersive_angle=-90;
//       smartcar_state=13;
//     }
     //*************************************************************************      
     else if(fifo_openart1_rx_buffer == 0XFB)
     {
       Initial_yaw=Reduction_yaw;
//       printf("%x\n",fifo_openart1_rx_buffer);
       jz_success=0;
       fake_picture=0;
       openart1.pack_order1 = count1_three; //处于接收偏移量状态，进入摄像头矫正
       pid_Init_A();                        //清空位置环和速度环pid参数
       wk_flag=1;
  //      openart1_jz_flag=1;
//       if(smartcar_state!=5)
//       {
          smartcar_state=2;
//       }
       if(direction==0)
         run_distance=run_distance-outputL[1];
       else if(direction==1)
         run_distance=run_distance+outputL[1];
       outputL[0] = 0;
       outputL[1] = 0;
       output_sc[0]=0;
       output_sc[1]=0;
       jz_y=0;
       jz_x=0;
       sc_y=0;
       sc_x=0;
       sc_finish=0;

       check =9;
       compulsion_monitor = 0;
     }
     //**************************************************************************
//     else if(fifo_openart1_rx_buffer == 0XFC) //识别种类
//     {
//       printf("%x\n",fifo_openart1_rx_buffer);
//       openart1.pack_order1 = count1_four;
//     }
//     else if(fifo_openart1_rx_buffer == 0XFE) //识别字母
//     {
//       printf("%x\n",fifo_openart1_rx_buffer);
//       openart1.pack_order1 = count1_five;
//     }
//           else if(fifo_openart1_rx_buffer==0XF4)//图片在左
//            {
//             
//            }
//           else if(fifo_openart1_rx_buffer ==0XF5)//图片在右
//           {
//            
//           }
     else if(fifo_openart1_rx_buffer==0XF6)//图片堆捡完
     {
       smartcar_state=0;       
//       kk=10086;
       pick_complete=1;
     }
//     else if(fifo_openart1_rx_buffer ==0XF7)//遍历结束
//     {
//       smartcar_state=0;
//       bianli_over=1;
    //     }
     else if(fifo_openart1_rx_buffer ==0XF9)
     {
       if(element_state!=0)
       {
         smartcar_state=0;
         pick_complete=1;
         no_picture_flag=1;
       }
       else if(element_state==0)
       {
         //      openart1_jz_flag==0;
         //      openart2_jz_flag==0;
         pid_Init_A(); 
         wk_y=outputL[0];
         //        wk_x=outputL[1];
         bl_x=outputL[1];
         if(direction==0)
           run_distance=run_distance-bl_x;
         else if(direction==1)
           run_distance=run_distance+bl_x;
         if(ABS(wk_y)<0.05)
         {
           wk_y=SIGN(wk_y)*0.08;
         }
         if(ABS(bl_x)<0.05)
         {
           bl_x=SIGN(wk_y)*0.08;
         }
         outputL[0] = 0;
         outputL[1] = 0;
         speed_output[0]=0;
         speed_output[1]=0;
         speed_output[2]=0;
         dispersive_picture=0;//第一帧看到图片标志位清零
         //       wd_flag=0;//弯道标志位清零
         time_flag1=0;
         check =7;
         abw=0;
         abb=0;
         bbc=0;
         location_state = 1;
         flag_pid_error=false_;
         smartcar_state = 0;
         openart1.pack_order1=FH1;
         //       wait_identify_flag=1;
         jz_success=1;//矫正成功标志位
         my_monitor1=0;//矫正监视位清零
         nosc_flag=0;
         fake_picture=1;
         //       ycsf_flag=0;
         
         if(element_state!=0&&element_state!=4&&smartcar_state==0&&pick_complete!=1)
         {
           spin_finish=1;
         }
         //        smartcar_state=8;
       }
       
     }
     else if(fifo_openart1_rx_buffer == 0XFD)//矫正完成
     {
       //      openart1_jz_flag==0;
       //      openart2_jz_flag==0;
       pid_Init_A(); 
       wk_y=outputL[0];
       //        wk_x=outputL[1];
       bl_x=outputL[1];
       if(direction==0)
         run_distance=run_distance-bl_x;
       else if(direction==1)
         run_distance=run_distance+bl_x;
       if(ABS(wk_y)<0.05)
       {
         wk_y=SIGN(wk_y)*0.08;
       }
       if(ABS(bl_x)<0.05)
       {
         bl_x=SIGN(wk_y)*0.08;
       }
       outputL[0] = 0;
       outputL[1] = 0;
       speed_output[0]=0;
       speed_output[1]=0;
       speed_output[2]=0;
       dispersive_picture=0;//第一帧看到图片标志位清零
       //       wd_flag=0;//弯道标志位清零
       time_flag1=0;
       check =7;
       abw=0;
       abb=0;
       bbc=0;
       location_state = 1;
       flag_pid_error=false_;
       smartcar_state = 0;
       openart1.pack_order1=FH1;
       //       wait_identify_flag=1;
       jz_success=1;//矫正成功标志位
       my_monitor1=0;//矫正监视位清零
       nosc_flag=0;
       //       ycsf_flag=0;
       
       if(element_state!=0&&element_state!=4&&smartcar_state==0&&pick_complete!=1)
       {
         spin_finish=1;
       }
     }
     else 
     {
       switch(openart1.pack_order1)
       {
       case count1_one:
         {
           openart1.picture_num = fifo_openart1_rx_buffer;
           openart1.pack_order1 = count1_two;//处于接收坐标数量状态
           //           printf("%d\n",openart1.picture_num);
           break ; 
         }
       case count1_two://处于接收坐标位置状态 
         {
           if(num<openart1.picture_num){
             openart1.picture_array[num] = fifo_openart1_rx_buffer;
             //             printf("%d\n",openart1.picture_array[num++]);
             if(num == openart1.picture_num){
               openart1.pack_order1 = FH1;                //将状态改为等待接收帧头
               openart1.number = openart1.picture_num/2+1;
               bcc=1;
             }
           }break ; 
         }
         //*****************************************************************************//
         //       case count1_three://接收矫正误差
         //         {
         //           if(n<4)
         //           {
         //             openart1.error_array[n]=fifo_openart1_rx_buffer; 
         //             if(dispersive_picture==0&&n==0)//记下第一帧照片的位置（左右）
         //             {
         //                dispersive_position=openart1.error_array[n];
         //                dispersive_picture=1;
         //             }
//             n++;
//             if(n==4)
//             {
//               flag_pid_error=ture_;  
//               n=0;  
//
//             }
//           }
//           
//           
//           break; 
//         }
         
         case count1_three://接收矫正误差
         {
           if(n<4)
           {
             if((n==0||n==3)&&(openart1.error_array[0]==1||openart1.error_array[0]==3||openart1.error_array[2]==1||openart1.error_array[2]==3))
             {
//               ycsf_flag=1;
//               kl_distanse=40;//小框
               kl_distanse=38;//大框
             }
             else if((n==0||n==3)&&(openart1.error_array[0]==0||openart1.error_array[0]==2||openart1.error_array[2]==0||openart1.error_array[2]==2))
             {
//               ycsf_flag=0;
               kl_distanse=20;
             }
             openart1.error_array[n]=fifo_openart1_rx_buffer; 
             if(dispersive_picture==0&&n==0)//记下第一帧照片的位置（左右）
             {
                dispersive_position=openart1.error_array[n];
                dispersive_picture=1;
             }
             n++;
             if(n==4)
             {
               flag_pid_error=ture_;  
               n=0;  

             }
           }
           
           
           break; 
         }

         //*****************************************************************************//
//       case count1_four://接受图片的种类
//         {
//           species_picture = fifo_openart1_rx_buffer;
//           wait_identify_flag=1;
//           openart1.pack_order1 = FH1; 
//           break ;
//         }
//         //*****************************************************************************//
//       case count1_five://接收字母的种类
//         {
//           species_alphabet =fifo_openart1_rx_buffer;
//           wait_identify_flag=1;
//           openart1.pack_order1 = FH1; 
//           break ;
//         }
       default:break;
       }
     }
     monitor_uint8 = fifo_openart1_rx_buffer;
     

    }
}

///*********************************************************************************************************
//** 函数简介     UART8 的接收中断处理函数 这个函数将在 UART_1 对应的中断调用 详见 isr.c
//** 参数说明     void
//** 返回参数     void
//** 使用示例     uart_rx_interrupt_handler8();      
//*********************************************************************************************************/ 
//void uart_rx_interrupt_handler8(void)
//{ 
//  //    get_data = uart_read_byte(UART_1);                                          // 接收数据 while 等待式 不建议在中断使用
//  uart_query_byte(UART_8, &get_data8);                                      // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
//  fifo_write_buffer(&uart_data_fifo8, &get_data8, 1);                     // 将数据写入 fifo 中
//  fifo_data_count8 = fifo_used(&uart_data_fifo8);                         // 查看 fifo 是否有数据
//  if(fifo_data_count8 != 0)                                                // 读取到数据了
//  {
//    fifo_read_buffer(&uart_data_fifo8, fifo_get_data8, &fifo_data_count8, FIFO_READ_AND_CLEAN);  // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
//    fifo_openart3_rx_buffer=fifo_get_data8[0];
//    if(fifo_openart3_rx_buffer == 0XFB)
//    {
//      //       lz_flag=1;
//    }
//    else if(fifo_openart3_rx_buffer == 0XFC) //识别种类
//    {
//      printf("%x\n",fifo_openart3_rx_buffer);
//      openart2.pack_order3 = count3_one;
//    }
//    else if(fifo_openart3_rx_buffer == 0XFE) //识别字母
//    {
////      printf("%x\n",fifo_openart2_rx_buffer);
//      openart3.pack_order3 = count3_two;
//    }
//    
//    else if(fifo_openart3_rx_buffer == 0XFD)//矫正完成
//    {         
//      //      openart1_jz_flag==0;
//      //      openart2_jz_flag==0;
//      pid_Init_A(); 
//      wk_y=outputL[0];
//      //        wk_x=outputL[1];
//      bl_x=outputL[1];
//      if(direction==0)
//        run_distance=run_distance-bl_x;
//      else if(direction==1)
//        run_distance=run_distance+bl_x;
//      if(ABS(wk_y)<0.05)
//      {
//        wk_y=SIGN(wk_y)*0.08;
//      }
//      if(ABS(bl_x)<0.05)
//      {
//        bl_x=SIGN(wk_y)*0.08;
//      }
//      outputL[0] = 0;
//      outputL[1] = 0;
//      speed_output[0]=0;
//      speed_output[1]=0;
//      speed_output[2]=0;
//      time_flag1=0;
//      check =7;
//      abb=0;
//      bbc=0;
//      location_state = 1;
//      flag_pid_error=false_;
//      smartcar_state = 0;
//      openart3.pack_order3=FH3;
//      //       wait_identify_flag=1;
//      jz_success=1;
//      //       kk=0;
//    }
//    else if(fifo_openart3_rx_buffer==0XF6)//图片堆捡完
//    {
//      smartcar_state=0;
//      //       kk=10086;
//      pick_complete=1;
//      //       lz_flag=0;
//    }
//    else
//    {
//      switch(openart3.pack_order3)
//      {
//      case count3_one://接受图片的种类
//        {
//          species_picture = fifo_openart3_rx_buffer;
//          wait_identify_picture_flag=1;
//          openart3.pack_order3 = FH3; 
//          break ;
//        }
//        //*****************************************************************************//
//      case count3_two://接收字母的种类
//        {
//          species_alphabet =fifo_openart3_rx_buffer;
//          openart3.pack_order3 = FH3; 
//          wait_identify_letter_flag=1;
//          break ;
//        }
//      default:break;
//      }
//    }
//  }
//}
//



/*********************************************************************************************************
** Function name:           openart1_send
** Descriptions:            openart1发送数据
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void openart1_send(void)
{
  static uint8_t openart1_data[1];
  openart1_data[0] = openart_mode;
  uart_write_buffer(UART_4, (uint8_t *) &openart1_data, sizeof(openart1_data));
}


/*********************************************************************************************************
** Function name:           openart2_send
** Descriptions:            openart2发送数据
** input parameters:        无
** output parameters:       无
** Returned value:          无
*********************************************************************************************************/ 
void openart2_send(void)
{
  static uint8_t openart2_data[1];
  openart2_data[0] = openart_mode;
  uart_write_buffer(UART_1, (uint8_t *) &openart2_data, sizeof(openart2_data));
}

///*********************************************************************************************************
//** Function name:           openart2_send
//** Descriptions:            openart2发送数据
//** input parameters:        无
//** output parameters:       无
//** Returned value:          无
//*********************************************************************************************************/ 
//void openart3_send(void)
//{
//  static uint8_t openart3_data[1];
//  openart3_data[0] = openart_mode;
//  uart_write_buffer(UART_8, (uint8_t *) &openart8_data, sizeof(openart8_data));
//}



