#include "zf_common_headfile.h"
#include "define.h"  
#include "bsp.h"

uint8  pit_ms_flag           = 0;
uint16 pit_ms_count          = 0;
uint16 dl1b_refresh_freq    = 0;


void tofa_process(void)
{
    // 如果开启了 INT 的使能 那么会自动触发外部中断进行数据采集 就不需要自行采集了
#if (!DL1B_INT_ENABLE)
    if(0 == pit_ms_count % 12)                                                  // 每 10ms 获取一次测距信息 周期 10 ms 频率 100Hz
    {
        dl1b_get_distance();                                                    // 测距调用频率不应高于 100Hz 周期不应低于 10ms
    }
#endif
    // 通过判断 dl1a_finsh_flag 来确定是否更新了数据 记得获取了数据后就清空标志位
    if(dl1b_finsh_flag)
    {
        dl1b_refresh_freq ++;
        dl1b_finsh_flag = 0;
    }
    pit_ms_count = (pit_ms_count == 294) ? (0) : (pit_ms_count + 6);            // 1000ms 周期计数
    if(0 == pit_ms_count % 300)
    {
        pit_ms_flag = 1;
    }
    if(pit_ms_flag)
    {
//      if(dl1b_distance_mm<=400)
//      {    
//         if(dl1b_refresh_freq>=80)
//        {
//          ramp_lowspeed=1;
//        }
//      }
      if(dl1b_distance_mm<=600)
      {
        if(dl1b_refresh_freq>=20)//30
        {
          tofa_per_flag=1;
        }
      }
      dl1b_refresh_freq = 0;
      pit_ms_flag = 0;
    }   
    
//    tft180_show_uint(0,70,dl1b_distance_mm,5);
//    tft180_show_uint(60,70,dl1b_refresh_freq,5);
}
