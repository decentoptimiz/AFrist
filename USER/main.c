/**
  ******************************************************************************
  * @file    main.c
  * @brief   示波器系统整合: 主循环 + AutoScale 状态机 + UI 刷新
  ******************************************************************************
  */
#include "sys.h"
#include "delay.h"
#include "scope_types.h"
#include "bsp_adc_tim.h"
#include "bsp_screen.h"
#include "scope_algo.h"

#define SAMPLE_RATE   50000.0f   /* 50kSa/s */

/*---------------------------------------------------------------------------*/
/* 全局运行状态与测量结果                                                       */
/*---------------------------------------------------------------------------*/
ScopeState_t scope;

/* 测量结果 (仅主循环上下文读写, 无需 volatile) */
static float   g_vpp       = 0.0f;
static float   g_avg       = 0.0f;
static float   g_freq      = 0.0f;
static float   g_phase     = 0.0f;
static uint8_t g_saturated = 0;

/* AUTO 触发标志 (可能由触摸中断置位, 需 volatile) */
static volatile uint8_t auto_trigger_flag = 1;

/* 乒乓处理缓冲 */
static uint16_t ch1_code[ADC_BUF_HALF];
static uint16_t ch2_code[ADC_BUF_HALF];
static float    ch1_v[ADC_BUF_HALF];
static float    ch2_v[ADC_BUF_HALF];

/*===========================================================================*/
/* 名称: CD4051_Init                                                          */
/* 功能: 初始化 PC0~PC3 为推挽输出 (两通道增益选择)                             */
/*===========================================================================*/
static void CD4051_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

/*===========================================================================*/
/* 名称: CD4051_SetGain                                                       */
/* 功能: 切换 CD4051 增益档位                                                  */
/*       CH1: PC0(bit0) PC1(bit1);  CH2: PC2(bit0) PC3(bit1)                */
/*===========================================================================*/
static void CD4051_SetGain(uint8_t ch, uint8_t gain)
{
    if (ch == 0)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);
        if (gain & 0x01) GPIO_SetBits(GPIOC, GPIO_Pin_0);
        if (gain & 0x02) GPIO_SetBits(GPIOC, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
        if (gain & 0x01) GPIO_SetBits(GPIOC, GPIO_Pin_2);
        if (gain & 0x02) GPIO_SetBits(GPIOC, GPIO_Pin_3);
    }
}

/*===========================================================================*/
/* 名称: Process_Buffer                                                       */
/* 功能: 处理乒乓缓冲的某一半: 拆包 -> 饱和检测 -> 触发 -> 测量 -> 绘制          */
/*===========================================================================*/
static void Process_Buffer(const volatile uint32_t *buf)
{
    int i;
    int trig_idx;

    /* 拆包: 高16位CH1, 低16位CH2 */
    for (i = 0; i < ADC_BUF_HALF; i++)
    {
        ch1_code[i] = (uint16_t)((buf[i] >> 16) & 0xFFFF);
        ch2_code[i] = (uint16_t)(buf[i] & 0xFFFF);
    }

    /* 正向饱和检测 (供 AutoScale 使用) */
    g_saturated = 0;
    for (i = 0; i < ADC_BUF_HALF; i++)
    {
        if (ch1_code[i] > ADC_SAT_THRESHOLD || ch2_code[i] > ADC_SAT_THRESHOLD)
        {
            g_saturated = 1;
            break;
        }
    }

    /* 触发查找 */
    trig_idx = Find_Trigger(ch1_code, ADC_BUF_HALF, scope.trig_level, scope.trig_edge);
    if (trig_idx < 0)
        return;

    /* 电压换算 (套用标定 V = code*k + b) */
    for (i = 0; i < ADC_BUF_HALF; i++)
    {
        ch1_v[i] = Scope_Code2Voltage(0, scope.ch1_gain, ch1_code[i]);
        ch2_v[i] = Scope_Code2Voltage(1, scope.ch2_gain, ch2_code[i]);
    }

    /* 测量 */
    Scope_Measure_VppAvg(ch1_v, ADC_BUF_HALF, &g_vpp, &g_avg);
    g_freq  = Scope_Measure_Freq(ch1_v, ADC_BUF_HALF, SAMPLE_RATE);
    g_phase = Scope_Measure_Phase(ch1_v, ch2_v, ADC_BUF_HALF);

    /* 波形绘制 (0x76 擦除 + 0x75 绘制) */
    Screen_Erase_Wave();
    Screen_Draw_Wave(ch1_v, ADC_BUF_HALF);
}

/*===========================================================================*/
/* 名称: AutoScale_Task                                                       */
/* 功能: AUTOSCALE 非阻塞状态机                                                */
/*===========================================================================*/
typedef enum
{
    AUTO_IDLE,
    AUTO_CHECK_SAT,
    AUTO_ADJ_HOR,
    AUTO_DONE
} AutoState_t;

static void AutoScale_Task(void)
{
    static AutoState_t state = AUTO_IDLE;

    switch (state)
    {
        case AUTO_IDLE:
            if (auto_trigger_flag)
                state = AUTO_CHECK_SAT;
            break;

        case AUTO_CHECK_SAT:
        {
            if (g_saturated && scope.ch1_gain > GAIN_1X)
            {
                /* 正向削顶: 降低增益 */
                scope.ch1_gain--;
                CD4051_SetGain(0, scope.ch1_gain);
                delay_ms(5);            /* 等待硬件稳定 */
            }
            else if (!g_saturated && g_vpp < 0.05f && scope.ch1_gain < GAIN_36X)
            {
                /* 信号太小: 提高增益 */
                scope.ch1_gain++;
                CD4051_SetGain(0, scope.ch1_gain);
                delay_ms(5);
            }
            else
            {
                state = AUTO_ADJ_HOR;
            }
            break;
        }

        case AUTO_ADJ_HOR:
            if (g_freq > 1000.0f)
                scope.t_div_idx = 0;    /* 1ms/div */
            else if (g_freq > 100.0f)
                scope.t_div_idx = 1;    /* 10ms/div */
            else
                scope.t_div_idx = 2;    /* 100ms/div */
            state = AUTO_DONE;
            break;

        case AUTO_DONE:
            auto_trigger_flag = 0;
            state = AUTO_IDLE;
            break;
    }
}

/*===========================================================================*/
/* 名称: UI_Update_Task                                                       */
/* 功能: 更新屏幕参数文本 (Vpp/Freq/Phase)                                      */
/* 说明: 骨架阶段暂不发送, 测量值通过 BSP_Debug_Printf 在串口观察               */
/*===========================================================================*/
static void UI_Update_Task(void)
{
    /* TODO: 发送 Vpp/Freq/Phase 文本到大彩屏 */
    /* 当前仅占位引用测量值, 避免未使用告警 */
    (void)g_vpp;
    (void)g_avg;
    (void)g_freq;
    (void)g_phase;
}

/*===========================================================================*/
/* 名称: main                                                                 */
/*===========================================================================*/
int main(void)
{
    delay_init(168);

    /* 中断优先级分组: 2 位抢占 + 2 位子优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 运行状态初始化 */
    scope.ch1_gain   = GAIN_1X;
    scope.ch2_gain   = GAIN_1X;
    scope.t_div_idx  = 0;
    scope.trig_level = 2048;        /* 触发默认取中值, 实际按 2.5V 偏置标定 */
    scope.trig_edge  = 0;           /* 上升沿 */
    scope.run_stop   = 1;

    /* 外设初始化 */
    CD4051_Init();
    CD4051_SetGain(0, scope.ch1_gain);
    CD4051_SetGain(1, scope.ch2_gain);

    BSP_Debug_Init(115200);
    BSP_Screen_Init();
    BSP_ADC_TIM_Init();

    BSP_Debug_Printf("[scope] system init done\r\n");

    /* 启动采集 */
    BSP_ADC_TIM_Start();

    while (1)
    {
        if (adc_half_flag)
        {
            adc_half_flag = 0;
            Process_Buffer(&adc_buf[0]);
        }

        if (adc_full_flag)
        {
            adc_full_flag = 0;
            Process_Buffer(&adc_buf[ADC_BUF_HALF]);
        }

        AutoScale_Task();
        UI_Update_Task();
    }
}
