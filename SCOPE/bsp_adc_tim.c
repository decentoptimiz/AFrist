/**
  ******************************************************************************
  * @file    bsp_adc_tim.c
  * @brief   示波器底层实现:
  *          - TIM3 输出 TRGO (Update Event) 作为采样触发源, 50kHz
  *          - ADC1(主, PA0)/ADC2(从, PA1) 规则同步模式, DMA2 Stream0 乒乓搬运
  *          - USART2 调试打印 (BSP_Debug_Printf)
  * @note    本工程使用 STM32F4 标准外设库 (SPL), 非 HAL。
  ******************************************************************************
  */
#include "bsp_adc_tim.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h>

/*---------------------------------------------------------------------------*/
/* 标定表: cal_table[通道号0/1][增益档位0/1/2]                                */
/* 默认 k=1, b=0; 实际使用前用两点法实测覆盖                                   */
/*---------------------------------------------------------------------------*/
CalibParams_t cal_table[CAL_CH_CNT][CAL_GAIN_CNT] = {
    { {1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f} },   /* CH1 */
    { {1.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 0.0f} }    /* CH2 */
};

/* DMA 乒乓缓冲: 双ADC 12位数据打包成32位字(高16位CH1, 低16位CH2) */
uint32_t adc_buf[ADC_BUF_SIZE];

/* 半满/全满标志 */
volatile uint8_t adc_half_flag = 0;
volatile uint8_t adc_full_flag = 0;

/*===========================================================================*/
/* 名称: BSP_Debug_Init                                                        */
/* 功能: 初始化 USART2 (PA2=TX, PA3=RX) 作为调试串口                            */
/* 参数: baud - 波特率                                                          */
/*===========================================================================*/
void BSP_Debug_Init(uint32_t baud)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

/*===========================================================================*/
/* 名称: BSP_Debug_Printf                                                      */
/* 功能: 调试打印, 输出到 USART2 (阻塞轮询发送, 仅调试用)                       */
/* 说明: 原模板 fputc 已重定向到 USART1, 故此处不占用 fputc,                    */
/*       而是用 vsnprintf 格式化后经 USART2 逐字节发送                         */
/*===========================================================================*/
void BSP_Debug_Printf(const char *fmt, ...)
{
    char    buf[256];
    va_list ap;
    int     len;
    int     i;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len < 0)
        return;
    if (len > (int)sizeof(buf) - 1)
        len = (int)sizeof(buf) - 1;

    for (i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, (uint8_t)buf[i]);
    }
}

/*===========================================================================*/
/* 名称: TIM3_Trgo_Config                                                      */
/* 功能: 配置 TIM3 作为采样触发源, TRGO = Update Event, 50kHz                   */
/*===========================================================================*/
static void TIM3_Trgo_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler         = TIM3_TRGO_PSC;      /* 0    */
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = TIM3_TRGO_PERIOD;   /* 1679 */
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* TRGO = Update Event */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
}

/*===========================================================================*/
/* 名称: ADC_Dual_Config                                                       */
/* 功能: 配置 ADC1(主,PA0)/ADC2(从,PA1) 规则同步模式 + DMA2 Stream0 乒乓搬运    */
/*===========================================================================*/
static void ADC_Dual_Config(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    NVIC_InitTypeDef      NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

    /* PA0 = ADC1_IN0, PA1 = ADC2_IN1 模拟输入 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 多模配置: 规则同时模式, 双路12位打包为32位字 */
    ADC_CommonInitStructure.ADC_Mode             = ADC_DualMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;  /* ADC clk = 84M/4 = 21MHz */
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_2; /* 32位打包              */
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* 主/从 ADC 相同配置: 外部触发 TIM3 TRGO 上升沿 */
    ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;  /* 由 TIM3 TRGO 逐次触发 */
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion      = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);

    /* 规则通道: 采样时间 144 Cycles (应对前端多路复用器源阻抗) */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);

    /* DMA2 Stream0 Channel0: 外设(ADC->CDR) -> 内存(adc_buf), 字宽, 循环 */
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)adc_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = ADC_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    /* 传输完成 / 半传输中断 (乒乓缓冲) */
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能主 ADC 的 DMA 请求 */
    ADC_DMACmd(ADC1, ENABLE);
}

/*===========================================================================*/
/* 名称: BSP_ADC_TIM_Init                                                      */
/* 功能: TIM3 + ADC1/ADC2 双同步采样 + DMA 初始化 (不启动)                     */
/*===========================================================================*/
void BSP_ADC_TIM_Init(void)
{
    TIM3_Trgo_Config();
    ADC_Dual_Config();
}

/*===========================================================================*/
/* 名称: BSP_ADC_TIM_Start                                                     */
/* 功能: 启动 ADC 与 DMA, 并使能 TIM3 触发采样                                   */
/*===========================================================================*/
void BSP_ADC_TIM_Start(void)
{
    /* 双 ADC 模式: 先使能从 ADC(ADC2), 再使能主 ADC(ADC1) */
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* 使能 TIM3, 输出 TRGO 触发 ADC */
    TIM_Cmd(TIM3, ENABLE);
}

/*===========================================================================*/
/* 名称: DMA2_Stream0_IRQHandler                                               */
/* 功能: DMA 半满/全满中断 (乒乓缓冲标志)                                        */
/*===========================================================================*/
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
        adc_half_flag = 1;
    }

    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        adc_full_flag = 1;
    }
}
