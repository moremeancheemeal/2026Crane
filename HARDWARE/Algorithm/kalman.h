#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "main.h"


// 卡尔曼滤波器结构体（单轴）
typedef struct {
    float x[2];          // 状态向量: [位置, 速度]
    float P[2][2];       // 误差协方差矩阵
    float Q[2];          // 过程噪声协方差 [位置, 速度]
    float R;             // 观测噪声方差
    float dt;            // 采样时间(8ms=0.008s)
} KalmanFilter;

extern KalmanFilter kf_x, kf_y;

float Kalman_Update(KalmanFilter* kf, float measurement); 
void kalman_Init(void);

#endif
