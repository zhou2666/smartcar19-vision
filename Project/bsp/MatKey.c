#include "MatKey.h"
#include "define.h"
#include "bsp.h"

uint8_t Key_Value=0;
uint8_t flag_KeyDel=0;

void KEY_Init(void)
{
	
    gpio_init(C26, GPI, 1, GPIO_PIN_CONFIG);  //row
    gpio_init(C30, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(C31, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(D4 , GPI, 1, GPIO_PIN_CONFIG);  //col
    gpio_init(B15, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(B23, GPI, 1, GPIO_PIN_CONFIG);
}
void Key_ColOut(uint8_t x)
{
    gpio_set_level(D4,1);   
    gpio_set_level(B15,1);
    gpio_set_level(B23,1);
  
  switch(x)
  {
    case 0:gpio_set_level(D4,1); break;
    case 1:gpio_set_level(B15,1);break;
    case 2:gpio_set_level(B23,1);break;
  }
	
}
uint8 ScanKey(void)
{
  uint8_t i,j,col=0,row=0,KeyCode=0;
  for(j=0;j<3;j++)
  {
    Key_ColOut(j);
    for(i=0;i<3;i++)
    {
      if(gpio_get_level(C26)==0)
      {
        col=i;
        row=1;
        KeyCode=row*4+col+1;
      }
      if(gpio_get_level(C30)==0)
      {
        col=i;
        row=2;
        KeyCode=row*4+col+1;
      }
      if(gpio_get_level(C31)==0)
      {
        col=i;
        row=3;
        KeyCode=row*4+col+1;
      }
    }
  }
  return KeyCode;
}
void Get_Key(void)
{
  if(ScanKey()!=0)
  {
    system_delay_ms(10);
    if(ScanKey()!=0)
    {
      Key_Value=ScanKey();
      flag_KeyDel=1;
      while(ScanKey());
    }
  }
}