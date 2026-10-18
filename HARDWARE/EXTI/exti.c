#include "exti.h"
#include "delay.h" 
#include "led.h" 
#include "beep.h"
#include "key.h"
//外部中断初始化程序
//初始化PE2~4,PA0为中断输入.
//初始化PA0 WJUP
void EXTIX_Init()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	
	KEY_Init(); //按键对应的IO口初始化
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);//PE2 连接到中断线2 KEY2
	/* 配置EXTI_Line2*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	

}



//外部中断2服务程序
//PE2 KEY2
void EXTI2_IRQHandler(void)
{
	delay_ms(50);	//消抖 	

	LED0=1;
	LED1=1;
  
	EXTI_ClearITPendingBit(EXTI_Line2);//清除LINE2上的中断标志位 
}




