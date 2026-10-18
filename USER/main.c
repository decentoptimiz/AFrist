#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "exti.h"

int main(void)
{ 
	delay_init(168);  //初始化延时函数
	LED_Init();				//初始化LED端口 
	BEEP_Init();      //初始化蜂鸣器端口
	KEY_Init();       //初始化与按键连接的硬件接口
	EXTIX_Init();
	//LED0=0;				  	//先点亮红灯
	while(1)
	{
     if(KEY0==0)
		 {
		  LED0=0;
		  delay_ms(3000);
			LED0=1;
		 }
     if(KEY1==0)
      LED1=0;
	}
}
