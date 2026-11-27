/*******************************************************************************************************************/
/***Include***/
//{
#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"
//}
/*******************************************************************************************************************/
/***Functions***/
//{
//------------------------------------------------------------------------------------------------------------------
// @brief       发送数据
// @para        mode    模式选择
// @return      void
//------------------------------------------------------------------------------------------------------------------
float transmit[4] = {0};
extern int vvv;
extern uint8 openart1_rx_buffer;
extern uint8 openart2_rx_buffer;
void send_massage(int mode)
{
  transmit[0] += encoder[0];
  transmit[1] += encoder[1];
  transmit[2] += encoder[2];
  transmit[3] += encoder[3];
  switch(mode)
  {
    case(1):{printf("%f,%f,%f,%f,%f,%f\r\n",Reduction_yaw,deter,acc.acc_x,acc.acc_y,acc.acc_z,speed_output[2]);break;}
    case(2):{printf("%f,%f,%f,%f,%d,%f\r\n",gory_pid.ExpectedValue,gory_pid.ActualValue,gory_pid.output,(gory_pid.Kp * gory_pid.inc_err[0] + gory_pid.Ki * gory_pid.err[0] + gory_pid.Ud[0]),flag_pid,(gory_pid.inc_err[0] * gory_pid.Kp * 0.65));break;}
    case(3):{printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",dv_w_times_dt[0],dv_w_times_dt[1],dv_w_times_dt[2],dv_w_times_dt[3],encoder_target[0],encoder_target[1],encoder_target[2],encoder_target[3],(pulse_per_meter/PID_RATE));break;}
    case(4):{printf("%d,%d,%d,%d\r\n",moter[0].moter_pwm,moter[1].moter_pwm,moter[2].moter_pwm,moter[3].moter_pwm);break;}
    case(5):{printf("%f,%f,%f,%d,%d\r\n",angle_pid.ExpectedValue,angle_pid.ActualValue,angle_pid.output,flag_pid,pid_fucntion_flag);break;}
    case(6):{printf("%f,%f,%f,%f\r\n",speed_pid[0].ExpectedValue,speed_pid[0].ActualValue,speed_pid[1].ExpectedValue,speed_pid[1].ActualValue);break;}
    case(7):{printf("%f,%f,%f,%f,%f\r\n",encoder_pid[0].output,encoder_pid[1].output,encoder_pid[2].output,encoder_pid[3].output,SIGN(91812.8787445434544));break;}
    case(8):{printf("%f,%f,%f,%f,%f,%f,%d,%d  \r\n",location_pid[0].ExpectedValue,location_pid[0].ActualValue,location_pid[1].ExpectedValue,location_pid[1].ActualValue,speed_pid[0].output,speed_pid[1].output,smartcar_state,locat_state);break;}
    case(10):{printf("%f,%f,%f,%f\r\n",Reduction_yaw,eulerAngle.final_yaw,eulerAngle_static.final_yaw,gory_pid.output);break;}
    case(11):{printf("%d,%f,%f,%d,%f,%f,%d,%f,%f\r\n",smartcar_state,need_grid[0],need_grid[1],openart_mode,location_pid[0].ExpectedValue,location_pid[1].ExpectedValue,correct_state,need_correct[0],need_correct[1]);break;}
    case(12):{printf("%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",smartcar_state,openart_mode,location_pid[0].ExpectedValue,location_pid[0].ActualValue,location_pid[1].ExpectedValue,location_pid[1].ActualValue,location_pid[0].err[0],location_pid[1].err[0],location_pid[0].inc_err[0],location_pid[1].inc_err[0],location_pid[0].output,location_pid[1].output);break;}
    case(13):{printf("%d,%d,%d,%d,%f,%f,%d,%d\n",openart_mode,correct_state,openart2.broad_heading,smartcar_state,need_correct[0],need_correct[1],openart2.correct_posture[0],openart2.correct_posture[1]);break;}
    case(14):{printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",gyrocope_err.ex,gyrocope_err_static.ex,gyrocope_err.ey,gyrocope_err_static.ey,gyrocope_err.ez,gyrocope_err_static.ez,gyrocope_err.I_ex,gyrocope_err_static.I_ex,gyrocope_err.I_ey,gyrocope_err_static.I_ey,gyrocope_err.I_ez,gyrocope_err_static.I_ez);break;}
    case(15):{printf("%f,%f,%d,%d,%d,%d,%d\r\n",need_correct[0],need_correct[1],openart1.remove_distance,remove_state,openart_mode,openart1.remove_blob,smartcar_state);break;}
    case(16):{printf("%f,%f,%d,%d,%d,%d,%d,%d\r\n",need_correct[0],need_correct[1],openart2.back_posture[0],openart2.back_posture[1],openart2.back_posture[2],openart2.back_posture[3],openart2.back_posture[4],smartcar_state);break;}
    case(17):{printf("%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%d \r\n",location_pid[0].ExpectedValue,location_pid[0].ActualValue,location_pid[1].ExpectedValue,location_pid[1].ActualValue,location_pid[0].inc_err,location_pid[1].inc_err,location_pid[0].err[0],location_pid[1].err[0],compulsion_monitor,angle_pid.ExpectedValue,angle_pid.ActualValue,smartcar_state);break;}
    case(18):{printf("%f,%f\r\n",speed_pid[1].ExpectedValue,transmit[0]);break;}
    case(19):{printf("%f,%f\r\n",location_pid[1].ExpectedValue,location_pid[1].ActualValue);break;}
    case(20):{printf("%d,%d,%d,%f,%f,%f,%f,%f,%f\r\n",forced_stop,smartcar_state,self_correct,location_pid[0].ExpectedValue,location_pid[0].ActualValue,location_pid[1].ExpectedValue,location_pid[1].ActualValue,outputL[0],outputL[1]);break;}
    case(21):{printf("%f,%f,%f,%f,%f,%f,%f\r\n",outputL[0],outputL[1],aL[0],aL[1],dv_w_times_dt[2],dv_w_times_dt[3],transmit[0]);};break;
    case(22):{printf("%f,%f,%f,%f,%f,%f\r\n",transmit[0],transmit[1],transmit[2],transmit[3],Reduction_yaw,gory_pid.output);break;}
    case(23):{printf("%f,%f,%d,%f,%f\r\n",outputV[0],outputV[1],1,0.95,1.05);break;}
    case(24):{printf("%f,%f,%f,%f,%f,%f,%d,%d\r\n",outputL[0],outputL[1],outputV[0],outputV[1],location_pid[0].output,location_pid[1].output,kkk,smartcar_state);break;}
    case(25):{printf("%f,%d\r\n",outputL[0],species_picture);break;}
    case(36):{printf("%f,%f,%f,%f,%d\r\n",transmit[0],transmit[1],transmit[2],transmit[3],flag);break;}
    case(37):{printf("%f,%f,%f,%f \r\n",encoder[0],encoder[1],encoder[2],encoder[3]);break;}
    case(38):{printf("%f\n",pulse_per_meter);break;}
    case(39):{printf("%f,%f,%f,%f,%f\r\n",transmit[0],transmit[1],transmit[2],transmit[3],Reduction_yaw);break;}
    case(40):{printf("%f,%f,%f,%f,%f,%f,%f,%f,%d \r\n", location_pid[0].output,location_pid[1].output,outputV[0],outputV[1] ,location_pid[0].ExpectedValue,location_pid[1].ExpectedValue,location_pid[0].ActualValue,location_pid[1].ActualValue,kkk );break;}
    case(41):{printf("%f,%f,%f,%f,%d,%d,%d \r\n",location_pid[0].err[0],location_pid[1].err[0],location_pid[0].output,location_pid[1].output,vvv,openart1.error_array[1],openart1.error_array[3]);break;}
    case(42):{printf("%f,%f,%d,%d,%d,%d,%f,%f,%f,%d,%f \r\n",location_pid[0].err[0],location_pid[1].err[0],openart1.error_array[0],openart1.error_array[1],openart1.error_array[2],openart1.error_array[3],Reduction_yaw,location_pid[0].output,location_pid[1].output,vvv,location_pid[0].err[0]*location_pid[0].err[0]+location_pid[1].err[0]*location_pid[1].err[0]);break;}
    case(43):{printf("%f,%f,%d,%f\r\n",location_pid[0].err[0],location_pid[1].err[0],vvv,outputV[0]+outputV[1]);break;}
    case(44):{printf("%f,%d,%d,%d,%d \r\n",location_pid[1].err[0],openart1.error_array[1],openart2.error_array[1],openart1_jz_flag,smartcar_state);break;}
    case(45):{printf("%f,%f,%f,%f,%d,%d\r\n",location_pid[1].err[0],location_pid[0].err[0],location_pid[1].inc_err[0], location_pid[0].inc_err[0],openart1_jz_flag,smartcar_state);break;}
    case(46):{printf("%f,%d,%f      \r\n",ABS(eulerAngle.cad[0]),smartcar_state,Reduction_yaw);break;}
          
              
    case(66):{printf("%f,%f,%f  ,%d  ,%f\r\n",speed_pid[0].ExpectedValue,speed_pid[0].ActualValue,gory_pid.ExpectedValue,smartcar_state,Reduction_yaw);break;}
    case(67):{printf("%d  ,%d  ,%0.2f,%0.2f  ,%0.2f,%0.2f ,      \r\n",my_state,smartcar_state,location_pid[0].err[0],location_pid[1].err[0],location_pid[0].output,location_pid[1].output);break;}//调矫正环用
    case(68):{printf("%d,%d,%d      \r\n",smartcar_state,element_state,fifo_openart1_rx_buffer);break;}
    case(69):{printf("%d ,%d ,%d ,%d ,%d ,%d  \r\n",element_state,yc1,yc2,pick_complete,smartcar_state,element_flag);break;}//调圆环用
    case(70):{printf("%0.2f ,%0.2f     \r\n",Reduction_yaw,angle_pid.output);break;}
    case(71):{printf("%d  ,%d ，%d \r\n",my_state,compulsion_monitor,my_monitor1);break;}//调矫正环用
    case(72):{printf("%f,%d\r\n",Reduction_yaw,kk);break;}
    case(73):{printf("%d  ,%d  ,%d  %d\r\n",lz_flag,smartcar_state,wait_identify_picture_flag,wait_identify_letter_flag);break;}
    case(74):{printf("%d  %d\r\n",fifo_openart1_rx_buffer,fifo_openart2_rx_buffer);break;}
    case(75):{printf("%f  \r\n",wk_ys);break;}
    case(76):{printf("%d,%0.2f\r\n",compulsion_monitor,ABS(location_pid[1].err[0]));break;}
    case(77):{printf("%d %d %0.2f %0.2f %0.2f\r\n",compulsion_monitor,dd,(float)(openart1.error_array[1]),(float)(openart1.error_array[3]),ABS(location_pid[0].err[0]));break;}//看矫正环卡住
    case(78):{printf("%d %d %d \r\n",wait_identify_picture_flag,my_monitor1,smartcar_state);break;}
    case(79):{printf("%d ,%d ,%d\r\n",pick_complete,fifo_openart1_rx_buffer,fifo_openart2_rx_buffer);break;}
    case(80):{printf("%d ,%d  ,%d  ,%d  ,%d\r\n",element_state,smartcar_state,no_picture_flag,yc2,kkkk);break;}
    case(81):{printf("%0.2f %0.2f %d %d %d\r\n",ramp_y,ramp_x,nnnn,tofa_flag,ramp_kz);break;}
    case(82):{printf("%d %d %d %d\r\n",tofa_flag,kk,dl1b_refresh_freq,dl1b_distance_mm);break;}
    case(83):{printf("%0.3f %d %d\r\n",wk_zebra,smartcar_state,element_state);break;}
    case(84):{printf("%0.2f %0.2f %d\r\n",speed_output[0],speed_output[1],openart1.error_array[3]);break;}
    case(85):{printf("%0.3f  %d %d %f\r\n",wk_zebra,zebra_commutation_flag,smartcar_state,need_angle);break;}
    case(86):{printf("%d %d %d %d %d %d\r\n",species_alphabet,element_state,pick_complete,wait_identify_letter_flag,wait_identify_picture_flag,kk);break;}
    case(87):{printf(" %d %d %d\r\n",change_count,point_change,element_state);break;} 
    case(88):{printf(" %0.2f %0.2f %d\r\n",wk_pick_y,location_pid_hard[0].output,spin_finish);break;} 
    case(89):{printf(" %0.2f %d %d\r\n",wk_single,Single_line,smartcar_state);break;} 
    case(90):{printf(" %d %d %d %d %d\r\n",kk,tofa_flag,tofa_per_flag,dl1b_distance_mm,dl1b_refresh_freq);break;} 
    case(91):{printf(" %d %d %d %d %d %d\r\n",carry_count,lz_flag,wait_identify_picture_flag,pick_complete,fifo_openart2_rx_buffer,dl1b_distance_mm);break;} 
    case(92):{printf(" %d %f %d %d %d %d\r\n",smartcar_state,output_stop[0],zebra_findflag,stop_flag,change_count,kkkkkkk);break;} 
    case(93):{printf(" %d %d %d\r\n",smartcar_state,kkkkkkk,yuzhi);break;} 
    case(94):{printf("%d %d %d %0.2f %0.2f \r\n",smartcar_state,compulsion_monitor,dd,location_pid[0].err[0],location_pid[1].err[0]);break;}//看矫正环卡住
    case(95):{printf("%d %d %d %d %d\r\n",smartcar_state,kkkkkkk,barrier_left_count,barrier_right_count,point_change);break;} 
    case(96):{printf("%d %d %d   %d\r\n",smartcar_state,dd,openart1.error_array[3],openart1.error_array[1]);break;} 
    case(97):{printf(" %d %0.2f %0.2f %d %d\r\n",smartcar_state,output_single[0],wk_single,Single_line,single_flag);break;} 
    case(98):{printf("%d  ,%d %0.3f ,%d ,%0.2f,%0.2f ,      \r\n",my_state,smartcar_state,sc_y,compulsion_monitor,location_pid[0].output,location_pid[1].output);break;}//调矫正环用
    case(99):{printf(" %d %d %0.2f %0.2f %0.2f\r\n",smartcar_state,sc_flag,ABS(location_pid[0].err[0]),speed_output[0],gory_pid.output);break;} 
    case(100):{printf(" %d %d %d %d %d\r\n",smartcar_state,e_p_flag,jz_success,wait_identify_letter_flag,kkk);break;} 
    case(101):{printf(" %d\r\n",dd);break;} 
    case(102):{printf(" %0.2f,%0.2f\r\n", output_zebra[0],zebra_forward);break;} 
    case(103):{printf("%d %d %d %d %d\r\n",smartcar_state,ycsf_flag,kl_distanse,dd,openart1.error_array[0]);break;} 
    case(104):{printf("%d %d %d  \r\n",smartcar_state,fake_picture,fifo_openart1_rx_buffer);break;} 
    case(105):{printf(" %d %d %d\r\n",my_state,my_state_y,dd);break;} 
    case(106):{printf(" %0.3f\r\n",wk_ys);break;} 
    case(107):{printf(" %0.3f %0.3f\r\n",speed_output[1],speed_output[0]);break;} 

  default:break;
  }

} 
//}
/*******************************************************************************************************************/