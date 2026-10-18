/**
  ******************************************************************************
  * @file    bsp_adc_tim.h
  * @brief   示波器底层: TIM3(50kHz TRGO) + ADC1/ADC2 双同步采样 + USART2 调试
  ******************************************************************************
  */
#ifndef __BSP_ADC_TIM_H
#define __BSP_ADC_TIM_H

#include "scope_types.h"

/*---------------------------------------------------------------------------*/
/* DMA 乒乓缓冲 (双ADC 12位数据打包成32位字: 高16位CH1, 低16位CH2)            */
/*---------------------------------------------------------------------------*/
extern uint32_t adc_buf[ADC_BUF_SIZE];

/* 半满/全满标志, 由 DMA2_Stream0 中断置位, 主循环处理完后清零                 */
extern volatile uint8_t adc_half_flag;
extern volatile uint8_t adc_full_flag;

void BSP_Debug_Init(uint32_t baud);                  /* USART2 调试串口初始化      */
void BSP_Debug_Printf(const char *fmt, ...);         /* 调试打印 (输出到 USART2)   */

void BSP_ADC_TIM_Init(void);                         /* TIM3 + 双ADC + DMA 初始化  */
void BSP_ADC_TIM_Start(void);                        /* 启动 TIM3 触发与 ADC 采样   */

#endif /* __BSP_ADC_TIM_H */
