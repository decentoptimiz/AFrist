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

void BSP_Screen_Init(void);                                   /* USART1 初始化           */
void BSP_Screen_Send(const uint8_t *buf, uint16_t len);       /* DMA 异步发送            */

/* 触摸事件回调 (弱定义, 应用层可覆盖实现) */
void BSP_Screen_TouchHandler(uint8_t page_id, uint8_t ctrl_id, uint8_t action);

#endif /* __BSP_SCREEN_H */
