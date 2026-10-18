/**
  ******************************************************************************
  * @file    bsp_screen.c
  * @brief   大彩串口屏驱动实现 (USART1, PA9=TX/PA10=RX)
  *          - 921600 波特率
  *          - TX 使用 DMA2 Stream7 Channel4 异步发送
  *          - RX 使用中断, 解析 0x65 触摸事件帧
  * @note    本工程使用 STM32F4 标准外设库 (SPL), 非 HAL。
  ******************************************************************************
  */
#include "bsp_screen.h"
#include "stm32f4xx.h"
#include "bsp_adc_tim.h"

/* 接收缓冲 (触摸事件帧) */
static volatile uint8_t s_rx_buf[16];
static volatile uint8_t s_rx_cnt = 0;

/* TX DMA 静态缓冲 (发送期间不可重入) */
#define SCREEN_TX_MAX     4096
#define SCREEN_WAVE_MAX   4096
#define SCREEN_WAVE_PTS   512
static uint8_t s_tx_buf[SCREEN_TX_MAX];      /* 通用发送缓冲                  */
static uint8_t s_wave_buf[SCREEN_WAVE_MAX];  /* 折线组包缓冲                  */

/* 波形垂直档位 (V/div) 与上一次绘制的 Y 坐标缓存 */
static float    s_vdiv = 1.0f;
static int16_t  s_last_y[SCREEN_WAVE_PTS];
static uint16_t s_last_len = 0;

/*===========================================================================*/
/* 名称: BSP_Screen_Init                                                       */
/* 功能: 初始化 USART1 (921600, DMA TX + RX 中断)                              */
/*===========================================================================*/
void BSP_Screen_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* PA9 = TX, PA10 = RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = SCREEN_BAUD;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* TX DMA: DMA2 Stream7 Channel4 (USART1_TX) */
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_tx_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = 0;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* RX 中断 (触摸事件) */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/*===========================================================================*/
/* 名称: BSP_Screen_Send                                                       */
/* 功能: 通过 DMA 异步发送数据到屏幕                                             */
/* 参数: buf - 数据指针, len - 数据长度                                          */
/*===========================================================================*/
void BSP_Screen_Send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (len > sizeof(s_tx_buf))
        len = sizeof(s_tx_buf);

    for (i = 0; i < len; i++)
        s_tx_buf[i] = buf[i];

    /* 等待上一次发送完成 */
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);

    DMA_Cmd(DMA2_Stream7, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Stream7, len);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

/*===========================================================================*/
/* 名称: USART1_IRQHandler                                                     */
/* 功能: 屏幕 RX 中断, 解析 0x65 触摸事件帧 (0x65 page_id ctrl_id action)        */
/*===========================================================================*/
void USART1_IRQHandler(void)
{
    uint8_t d;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        d = (uint8_t)USART_ReceiveData(USART1);

        if (s_rx_cnt == 0)
        {
            if (d == SCREEN_TOUCH_HEADER)
            {
                s_rx_buf[s_rx_cnt++] = d;
            }
        }
        else if (s_rx_cnt < sizeof(s_rx_buf))
        {
            s_rx_buf[s_rx_cnt++] = d;

            /* 触摸事件帧至少 4 字节: 0x65 page_id ctrl_id action */
            if (s_rx_cnt >= 4)
            {
                BSP_Screen_TouchHandler(s_rx_buf[1], s_rx_buf[2], s_rx_buf[3]);
                s_rx_cnt = 0;
            }
        }
        else
        {
            s_rx_cnt = 0;
        }
    }
}

/*===========================================================================*/
/* 名称: Map_Voltage_To_Y                                                      */
/* 功能: 电压值映射为屏幕 Y 坐标 (波形区起点 Y0, 高度 H, 垂直档位 Vdiv)           */
/*       y = Y0 + H/2 - V / Vdiv * (H/8)                                      */
/*===========================================================================*/
int16_t Map_Voltage_To_Y(float V, float Vdiv)
{
    int y;

    if (Vdiv <= 0.0f)
        Vdiv = 1.0f;

    y = SCREEN_WAVE_Y0 + (SCREEN_WAVE_H / 2)
        - (int)(V / Vdiv * (float)(SCREEN_WAVE_H / 8));

    if (y < SCREEN_WAVE_Y0)
        y = SCREEN_WAVE_Y0;
    if (y > SCREEN_WAVE_Y0 + SCREEN_WAVE_H)
        y = SCREEN_WAVE_Y0 + SCREEN_WAVE_H;

    return (int16_t)y;
}

/*===========================================================================*/
/* 名称: Screen_Set_Vdiv                                                       */
/* 功能: 设置垂直档位 (V/div)                                                   */
/*===========================================================================*/
void Screen_Set_Vdiv(float vdiv)
{
    if (vdiv > 0.0f)
        s_vdiv = vdiv;
}

/*===========================================================================*/
/* 名称: Screen_Send_Polyline (内部)                                            */
/* 功能: 折线指令组包并发送                                                      */
/*       帧格式: EE cmd x0_hi x0_lo y0_hi y0_lo [dx dy]... FF FC FF FF        */
/*       首点绝对坐标, 后续为有符号差值; Y 差值斜率限幅在 -127 ~ +127,           */
/*       超出部分由后续同 X 补点补足                                            */
/*===========================================================================*/
static void Screen_Send_Polyline(uint8_t cmd, const int16_t *ys, uint16_t len)
{
    uint16_t n = 0;
    uint16_t i;
    int16_t  prev;
    uint16_t x = SCREEN_WAVE_X0;
    uint8_t  full = 0;

    if (len == 0)
        return;

    s_wave_buf[n++] = 0xEE;
    s_wave_buf[n++] = cmd;

    /* 首点: 绝对坐标 (16位大端) */
    s_wave_buf[n++] = (uint8_t)(x >> 8);
    s_wave_buf[n++] = (uint8_t)(x & 0xFF);
    s_wave_buf[n++] = (uint8_t)(ys[0] >> 8);
    s_wave_buf[n++] = (uint8_t)(ys[0] & 0xFF);
    prev = ys[0];

    for (i = 1; i < len && !full; i++)
    {
        int16_t target = ys[i];
        int16_t cur    = prev;
        uint8_t dx     = 1;    /* 每个采样点首段前移 1 像素 */

        while (cur != target && !full)
        {
            int32_t diff = (int32_t)target - (int32_t)cur;
            int16_t d;

            /* 斜率限幅: Y 差值限制在 -127 ~ +127 */
            if (diff > 127)
                d = 127;
            else if (diff < -127)
                d = -127;
            else
                d = (int16_t)diff;

            if (n + 2 + 4 > SCREEN_WAVE_MAX)
            {
                full = 1;
                break;
            }

            s_wave_buf[n++] = dx;
            s_wave_buf[n++] = (uint8_t)(int8_t)d;

            cur = (int16_t)(cur + d);
            dx  = 0;            /* 补点保持同一 X 坐标 */
        }

        prev = target;
    }

    s_wave_buf[n++] = 0xFF;
    s_wave_buf[n++] = 0xFC;
    s_wave_buf[n++] = 0xFF;
    s_wave_buf[n++] = 0xFF;

    BSP_Screen_Send(s_wave_buf, n);
}

/*===========================================================================*/
/* 名称: Screen_Draw_Wave                                                      */
/* 功能: 将电压数据映射为 Y 坐标并用 0x75 画线; 缓存 Y 坐标以便后续擦除          */
/*===========================================================================*/
void Screen_Draw_Wave(float *wave_data, uint16_t len)
{
    uint16_t i;

    if (len > SCREEN_WAVE_PTS)
        len = SCREEN_WAVE_PTS;

    for (i = 0; i < len; i++)
        s_last_y[i] = Map_Voltage_To_Y(wave_data[i], s_vdiv);

    s_last_len = len;

    Screen_Send_Polyline(0x75, s_last_y, len);
}

/*===========================================================================*/
/* 名称: Screen_Erase_Wave                                                     */
/* 功能: 用 0x76 擦除上一次绘制的波形                                            */
/*===========================================================================*/
void Screen_Erase_Wave(void)
{
    if (s_last_len == 0)
        return;

    Screen_Send_Polyline(0x76, s_last_y, s_last_len);
}

/*===========================================================================*/
/* 名称: BSP_Screen_TouchHandler (弱定义)                                       */
/* 功能: 触摸事件回调, 打印控件 ID 便于确认触摸是否正常; 应用层可覆盖实现          */
/*===========================================================================*/
__weak void BSP_Screen_TouchHandler(uint8_t page_id, uint8_t ctrl_id, uint8_t action)
{
    SCOPE_DBG("[touch] page=%d ctrl=%d action=%d\r\n",
              page_id, ctrl_id, action);
}
