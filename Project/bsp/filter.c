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

//TARGET_tYPE value_buf_acc[2][N_1];
//TARGET_tYPE value_buf_speed[2][N_2];
//TARGET_tYPE value_buf_encoder[4][N_3];

void Queue_Init()
{
  for(int i=0;i<2;i++)
  {
    for(int j=0;j<N_1;j++)
    {
      value_buf_acc[i][j] = 0;
    }
  }
  
    for(int i=0;i<2;i++)
  {
    for(int j=0;j<N_2;j++)
    {
      value_buf_speed[i][j] = 0;
    }
  }
  
    for(int i=0;i<4;i++)
  {
    for(int j=0;j<N_3;j++)
    {
      value_buf_encoder[i][j] = 0;
    }
  }
  for(int i=0;i<2;i++)
  {
      for(int j=0;j<N_3;j++)
    {
      value_buf_openart[i][j] = 0;
    }
  
  }
}

void Add_Queue(int mode,int n,TARGET_tYPE value)
{
  switch (mode)
  {
    case(1):
      {
        for(int i = N_1-1;i > 0;i--)
        {
          value_buf_acc[n][i] = value_buf_acc[n][i-1];
        }
          value_buf_acc[n][0] = value;
      }break;
      
    case(2):
      {
        for(int i = N_2-1;i > 0;i--)
        {
          value_buf_speed[n][i] = value_buf_speed[n][i-1];
        }
          value_buf_speed[n][0] = value;
      }break;
      
    case(3):
      {
        for(int i = N_3-1;i > 0;i--)
        {
          value_buf_encoder[n][i] = value_buf_encoder[n][i-1];
        }
          value_buf_encoder[n][0] = value;        
      break;}
      case(4):
        {
          for(int i = N_3-1;i > 0;i--)
          {
        value_buf_openart[n][i] = value_buf_openart[n][i-1];
          }
        value_buf_openart[n][0] = value;  
        break;}
    default:break;
  }
}

TARGET_tYPE Sum_Queue(int mode,int n)
{
  TARGET_tYPE adder = 0;
  switch(mode)
  {
    case(1):
      {
        for(i = 0;i < N_1;i++)
        {
          adder += value_buf_acc[n][i];
        }break;
      }
      
    case(2):
      {
        for(i = 0;i < N_2;i++)
        {
          adder += value_buf_speed[n][i];
        }break;
      }
     
    case(3):
      {
        for(i = 0;i < N_3;i++)
        {
          adder += value_buf_encoder[n][i];
        }break;
      }
      case(4):
        {
          for(i =0; i<N_3;i++)
          {
        adder += value_buf_openart[n][i];
          }
        }
    default:adder =  0;break;
  }
  return adder;
}

void filter_function(int mode)
{
  float a_1 = 0.30,a_2 = 0.30,a_3 = 0.30,a_4=0.30;
  switch(mode)
  {
    case(1):
      {
        for(int n=0;n<2;n++)
        {
          acc_filter[n].filter_hander = (a_1 * (float)acc_filter[n].input  + (1.0 - a_1) * (float)acc_filter[n].baroAlt);
          acc_filter[n].baroAlt = acc_filter[n].filter_hander;
          Add_Queue(1,n,acc_filter[n].filter_hander);
          acc_filter[n].output = (Sum_Queue(1,n)/N_1);
        }break;
      }
      
    case(2):
      {
        for(int n=0;n<2;n++)
        {
          speed_filter[n].filter_hander = (a_2 * (float)speed_filter[n].input  + (1.0 - a_2) * (float)speed_filter[n].baroAlt);
          speed_filter[n].baroAlt = speed_filter[n].filter_hander;
          Add_Queue(2,n,speed_filter[n].filter_hander);
          speed_filter[n].output = (Sum_Queue(2,n)/N_2);
        }break;      
      }
      
    case(3):
      {
        for(int n=0;n<4;n++)
        {
          encoder_filter[n].filter_hander = (a_3 * (float)encoder_filter[n].input  + (1.0 - a_3) * (float)encoder_filter[n].baroAlt);
          encoder_filter[n].baroAlt = encoder_filter[n].filter_hander;
          Add_Queue(3,n,encoder_filter[n].filter_hander);
          encoder_filter[n].output = (float)((int)(Sum_Queue(3,n)/N_3));
          if(n==1||n==3)
          {
            encoder_filter[n].output= -encoder_filter[n].output;
          }
        }break;           
      }
  case 4:{
    for(int n =0;n<2;n++)
    {
    openart_filter[n].filter_hander=(a_3*location_pid[n].err[0]+(1.0 - a_3) * openart_filter[n].baroAlt);
    openart_filter[n].baroAlt = openart_filter[n].filter_hander;
    Add_Queue(4,n,openart_filter[n].filter_hander);
    location_pid[n].err[0]=(Sum_Queue(4,n)/N_3);
    
    }
  
  break;}
      default:break;
  }
}
//}
/*******************************************************************************************************************/
#if(0)
Reference Resources:
https://blog.csdn.net/sf646951633/article/details/86596540
#endif
/*******************************************************************************************************************/