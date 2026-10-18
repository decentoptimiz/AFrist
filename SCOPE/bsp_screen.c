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

/* 接收缓冲 (触摸事件帧) */
static volatile uint8_t s_rx_buf[16];
static volatile uint8_t s_rx_cnt = 0;

/* TX DMA 静态缓冲 (发送期间不可重入) */
static uint8_t s_tx_buf[1024];

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
/* 名称: BSP_Screen_TouchHandler (弱定义)                                       */
/* 功能: 触摸事件回调, 应用层可覆盖实现                                           */
/*===========================================================================*/
__weak void BSP_Screen_TouchHandler(uint8_t page_id, uint8_t ctrl_id, uint8_t action)
{
    (void)page_id;
    (void)ctrl_id;
    (void)action;
}
