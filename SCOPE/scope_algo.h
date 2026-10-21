/**
  ******************************************************************************
  * @file    scope_algo.h
  * @brief   示波器核心算法: 触发搜索 + 信号参数测量 (Vpp/平均值/频率/相位)
  ******************************************************************************
  */
#ifndef __SCOPE_ALGO_H
#define __SCOPE_ALGO_H

#include <stdint.h>

#define FFT_POINTS   512   /* FFT 点数 */

/* 波形类型 (波形识别结果) */
typedef enum {
    WAVE_SINE     = 0,   /* 正弦波   */
    WAVE_SQUARE   = 1,   /* 方波     */
    WAVE_TRIANGLE = 2    /* 三角波   */
} WaveType_t;

/* 触发查找: 带迟滞的上升/下降沿搜索, 返回触发点索引, 未找到返回 -1 */
int Find_Trigger(const uint16_t *data, int len, uint16_t level, int edge);

/* Vpp 与平均值 (输入为真实电压 float), 结果写入 vpp/avg 指针 */
void Scope_Measure_VppAvg(const float *data, int len, float *vpp, float *avg);

/* 有效值 Vrms = sqrt( (1/N) * Σ Vi² ) */
float Scope_Measure_Vrms(const float *data, int len);

/* 波形识别: 依据波峰因数(Vpp/Vrms)区分正弦/方波/三角波 */
WaveType_t Identify_Waveform(float *data, int len);

/* 频率测量 (过零检测法, fs 为采样率 Hz), 返回频率 Hz */
float Scope_Measure_Freq(const float *data, int len, float fs);

/* 相位差测量 (512点汉宁窗 FFT), 返回 CH2 相对 CH1 的相位差 (度, 0~360) */
float Scope_Measure_Phase(const float *ch1, const float *ch2, int len);

#endif /* __SCOPE_ALGO_H */
