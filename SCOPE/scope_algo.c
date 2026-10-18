/**
  ******************************************************************************
  * @file    scope_algo.c
  * @brief   示波器核心算法实现
  *          - Find_Trigger: 带迟滞的触发搜索
  *          - Vpp / 平均值 / 频率(过零检测) / 相位差(512点汉宁窗 FFT)
  * @note    本机未安装 CMSIS-DSP (arm_cfft_f32 不可用), 相位差改用自包含的
  *          512 点基-2 FFT, 与 arm_cfft_f32 结果等价。若后续安装 ARM.CMSIS 包,
  *          可在 Keil 中定义 ARM_MATH_CM4 并将 fft_radix2 替换为 arm_cfft_f32。
  ******************************************************************************
  */
#include "scope_algo.h"
#include "scope_types.h"
#include "bsp_adc_tim.h"
#include <math.h>

#define PI_F  3.14159265358979f

/*===========================================================================*/
/* 名称: Find_Trigger                                                         */
/* 功能: 带迟滞的上升/下降沿触发搜索                                            */
/* 参数: data - 采样数据(码值), len - 长度, level - 触发电平,                    */
/*       edge - 0:上升沿 1:下降沿                                               */
/* 返回: 触发点索引, 未找到返回 -1                                              */
/*===========================================================================*/
int Find_Trigger(const uint16_t *data, int len, uint16_t level, int edge)
{
    int i;

    for (i = 1; i < len; i++)
    {
        if (edge == 0)   /* 上升沿 */
        {
            if ((int32_t)data[i - 1] < (int32_t)level - TRIG_HYST &&
                (int32_t)data[i]     > (int32_t)level + TRIG_HYST)
            {
                BSP_Debug_Printf("[trig] edge=RISING idx=%d\r\n", i);
                return i;
            }
        }
        else             /* 下降沿 */
        {
            if ((int32_t)data[i - 1] > (int32_t)level + TRIG_HYST &&
                (int32_t)data[i]     < (int32_t)level - TRIG_HYST)
            {
                BSP_Debug_Printf("[trig] edge=FALLING idx=%d\r\n", i);
                return i;
            }
        }
    }

    BSP_Debug_Printf("[trig] miss\r\n");
    return -1;
}

/*===========================================================================*/
/* 名称: Scope_Measure_VppAvg                                                  */
/* 功能: 计算峰峰值与平均值 (输入为真实电压 float)                                */
/*===========================================================================*/
void Scope_Measure_VppAvg(const float *data, int len, float *vpp, float *avg)
{
    int   i;
    float vmin;
    float vmax;
    float sum;

    if (len <= 0)
    {
        *vpp = 0.0f;
        *avg = 0.0f;
        return;
    }

    vmin = data[0];
    vmax = data[0];
    sum  = 0.0f;

    for (i = 0; i < len; i++)
    {
        if (data[i] < vmin) vmin = data[i];
        if (data[i] > vmax) vmax = data[i];
        sum += data[i];
    }

    *vpp = vmax - vmin;
    *avg = sum / (float)len;

    BSP_Debug_Printf("[vpp] Vpp=%.4f V avg=%.4f V\r\n", *vpp, *avg);
}

/*===========================================================================*/
/* 名称: Scope_Measure_Freq                                                    */
/* 功能: 频率测量 (去直流后的正向过零检测, 线性插值)                              */
/* 参数: fs - 采样率 Hz, 返回频率 Hz                                            */
/*===========================================================================*/
float Scope_Measure_Freq(const float *data, int len, float fs)
{
    int   i;
    int   zc = 0;
    float mean = 0.0f;
    float prev;
    float cur;
    float first_t = 0.0f;
    float last_t  = 0.0f;
    float freq = 0.0f;

    if (len < 3 || fs <= 0.0f)
    {
        BSP_Debug_Printf("[freq] Freq=0.00 Hz\r\n");
        return 0.0f;
    }

    for (i = 0; i < len; i++)
        mean += data[i];
    mean /= (float)len;

    prev = data[0] - mean;
    for (i = 1; i < len; i++)
    {
        cur = data[i] - mean;
        if (prev < 0.0f && cur >= 0.0f)
        {
            float t = (float)i - cur / (cur - prev);   /* 亚采样过零点 */
            if (zc == 0)
                first_t = t;
            last_t = t;
            zc++;
        }
        prev = cur;
    }

    if (zc >= 2)
        freq = (float)(zc - 1) / (last_t - first_t) * fs;

    BSP_Debug_Printf("[freq] Freq=%.2f Hz (zeros=%d)\r\n", freq, zc);
    return freq;
}

/*===========================================================================*/
/* 名称: fft_radix2 (内部)                                                     */
/* 功能: 基-2 时间抽取 FFT (就地, 自然序输出)                                   */
/*===========================================================================*/
static void fft_radix2(float *re, float *im, int n)
{
    int i;
    int j;
    int m;

    /* 位反转重排 */
    j = 0;
    for (i = 1; i < n; i++)
    {
        int bit = n >> 1;
        while (j & bit)
        {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j)
        {
            float tr;
            float ti;

            tr = re[i]; re[i] = re[j]; re[j] = tr;
            ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    for (m = 2; m <= n; m <<= 1)
    {
        int   mh  = m >> 1;
        float ang = -2.0f * PI_F / (float)m;
        float wr  = cosf(ang);
        float wi  = sinf(ang);

        for (i = 0; i < n; i += m)
        {
            float tr = 1.0f;
            float ti = 0.0f;

            for (j = 0; j < mh; j++)
            {
                int   a = i + j;
                int   b = a + mh;
                float xr = re[a];
                float xi = im[a];
                float yr = re[b] * tr - im[b] * ti;
                float yi = re[b] * ti + im[b] * tr;
                float nr;

                re[a] = xr + yr; im[a] = xi + yi;
                re[b] = xr - yr; im[b] = xi - yi;

                nr = tr * wr - ti * wi;
                ti = tr * wi + ti * wr;
                tr = nr;
            }
        }
    }
}

/*===========================================================================*/
/* 名称: Scope_Measure_Phase                                                   */
/* 功能: 512 点汉宁窗 FFT 计算 CH2 相对 CH1 的相位差                            */
/* 返回: 相位差 (度, 0~360)                                                     */
/*===========================================================================*/
float Scope_Measure_Phase(const float *ch1, const float *ch2, int len)
{
    static float re1[FFT_POINTS];
    static float im1[FFT_POINTS];
    static float re2[FFT_POINTS];
    static float im2[FFT_POINTS];
    int    i;
    int    k;
    int    max_bin = 0;
    float  max_mag = 0.0f;
    float  phase1;
    float  phase2;
    float  dphi;

    if (len > FFT_POINTS)
        len = FFT_POINTS;

    /* 加周期汉宁窗 (分母取 FFT_POINTS) */
    for (i = 0; i < len; i++)
    {
        float w = 0.5f * (1.0f - cosf(2.0f * PI_F * (float)i / (float)FFT_POINTS));
        re1[i] = ch1[i] * w; im1[i] = 0.0f;
        re2[i] = ch2[i] * w; im2[i] = 0.0f;
    }
    /* 补零 */
    for (i = len; i < FFT_POINTS; i++)
    {
        re1[i] = 0.0f; im1[i] = 0.0f;
        re2[i] = 0.0f; im2[i] = 0.0f;
    }

    fft_radix2(re1, im1, FFT_POINTS);
    fft_radix2(re2, im2, FFT_POINTS);

    /* 找主频 bin (跳过直流) */
    for (k = 1; k < FFT_POINTS / 2; k++)
    {
        float mag = re1[k] * re1[k] + im1[k] * im1[k];
        if (mag > max_mag)
        {
            max_mag = mag;
            max_bin = k;
        }
    }

    if (max_bin == 0)
    {
        BSP_Debug_Printf("[phase] Phase=0.00 deg (no signal)\r\n");
        return 0.0f;
    }

    phase1 = atan2f(im1[max_bin], re1[max_bin]);
    phase2 = atan2f(im2[max_bin], re2[max_bin]);

    dphi = (phase2 - phase1) * 180.0f / PI_F;
    if (dphi < 0.0f)    dphi += 360.0f;
    if (dphi >= 360.0f) dphi -= 360.0f;

    BSP_Debug_Printf("[phase] Phase=%.2f deg (bin=%d)\r\n", dphi, max_bin);
    return dphi;
}
