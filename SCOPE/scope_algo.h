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

/* 触发查找: 带迟滞的上升/下降沿搜索, 返回触发点索引, 未找到返回 -1 */
int Find_Trigger(const uint16_t *data, int len, uint16_t level, int edge);

/* Vpp 与平均值 (输入为真实电压 float), 结果写入 vpp/avg 指针 */
void Scope_Measure_VppAvg(const float *data, int len, float *vpp, float *avg);

/* 频率测量 (过零检测法, fs 为采样率 Hz), 返回频率 Hz */
float Scope_Measure_Freq(const float *data, int len, float fs);

/* 相位差测量 (512点汉宁窗 FFT), 返回 CH2 相对 CH1 的相位差 (度, 0~360) */
float Scope_Measure_Phase(const float *ch1, const float *ch2, int len);

#endif /* __SCOPE_ALGO_H */
