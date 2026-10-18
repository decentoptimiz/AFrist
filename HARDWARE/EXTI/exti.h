#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"  	

#define KEY4 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)
#define KEY3 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)                                                                                                                                                                                                                                                                                                                             //PE5
#define KEY0 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4) //PE4
#define KEY1 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)	//PE3 
#define KEY2 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) //PE2
#define WK_UP 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)	//PA0
#define KEY5 		GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_0)

#define KEY0_PRES 	1	//KEY0按下
#define KEY1_PRES	2	//KEY1按下
#define KEY2_PRES	3	//KEY2按下
#define WKUP_PRES   4	//KEY_UP按下(即WK_UP)

void KEY_Init(void);	//IO初始化	
void EXTIX_Init(void);	//外部中断初始化		 					    
#endif

























