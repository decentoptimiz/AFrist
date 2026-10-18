/**
  ******************************************************************************
  * @file    bsp_screen.h
  * @brief   大彩串口屏驱动 (USART1, 921600, DMA TX + RX 中断)
  ******************************************************************************
  */
#ifndef __BSP_SCREEN_H
#define __BSP_SCREEN_H

#include <stdint.h>

#define SCREEN_BAUD          921600  /* 大彩屏波特率           */
#define SCREEN_TOUCH_HEADER  0x65    /* 触摸事件帧头            */

/* 波形绘图区参数 */
#define SCREEN_WAVE_X0       0       /* 波形区起点 X            */
#define SCREEN_WAVE_Y0       0       /* 波形区起点 Y            */
#define SCREEN_WAVE_H        560     /* 波形区高度              */

void BSP_Screen_Init(void);                                   /* USART1 初始化           */
void BSP_Screen_Send(const uint8_t *buf, uint16_t len);       /* DMA 异步发送            */

/* 波形绘制 */
int16_t Map_Voltage_To_Y(float V, float Vdiv);                /* 电压 -> 屏幕 Y 坐标     */
void Screen_Set_Vdiv(float vdiv);                             /* 设置垂直档位 (V/div)    */
void Screen_Draw_Wave(float *wave_data, uint16_t len);        /* 0x75 画波形             */
void Screen_Erase_Wave(void);                                 /* 0x76 擦除上次波形       */

/* 触摸事件回调 (弱定义, 应用层可覆盖实现) */
void BSP_Screen_TouchHandler(uint8_t page_id, uint8_t ctrl_id, uint8_t action);

#endif /* __BSP_SCREEN_H */
