#include "kalman.h"

#include <math.h>
/*===== 双轴滤波使用示例 =====*/
KalmanFilter kf_x, kf_y;

// 初始化滤波器
void Kalman_Init(KalmanFilter* kf, float pos_noise, float vel_noise, float meas_noise) 
{
    kf->x[0] = kf->x[1] = 0.0f;
    kf->P[0][0] = kf->P[1][1] = 1.0f; // 初始协方差
    kf->P[0][1] = kf->P[1][0] = 0.0f;
    kf->Q[0] = pos_noise * pos_noise;  // 过程噪声方差
    kf->Q[1] = vel_noise * vel_noise;
    kf->R = meas_noise * meas_noise;   // 观测噪声方差
    kf->dt = 0.008391f;                   // 8ms采样周期
}

// 执行卡尔曼预测和更新（单轴）
float Kalman_Update(KalmanFilter* kf, float measurement) 
{
    /*===== 1. 预测阶段 =====*/
    // 状态预测: x = F * x
    kf->x[0] += kf->x[1] * kf->dt;
    // kf->x[1] 保持不变 (速度假设恒定)
    
    // 协方差预测: P = F * P * F^T + Q
    float F[2][2] = {{1, kf->dt}, {0, 1}};
    float P_temp[2][2];
    
    // F * P
    P_temp[0][0] = F[0][0]*kf->P[0][0] + F[0][1]*kf->P[1][0];
    P_temp[0][1] = F[0][0]*kf->P[0][1] + F[0][1]*kf->P[1][1];
    P_temp[1][0] = F[1][0]*kf->P[0][0] + F[1][1]*kf->P[1][0];
    P_temp[1][1] = F[1][0]*kf->P[0][1] + F[1][1]*kf->P[1][1];
    
    // (F * P) * F^T
    kf->P[0][0] = P_temp[0][0]*F[0][0] + P_temp[0][1]*F[0][1] + kf->Q[0];
    kf->P[0][1] = P_temp[0][0]*F[1][0] + P_temp[0][1]*F[1][1];
    kf->P[1][0] = P_temp[1][0]*F[0][0] + P_temp[1][1]*F[0][1];
    kf->P[1][1] = P_temp[1][0]*F[1][0] + P_temp[1][1]*F[1][1] + kf->Q[1];

    /*===== 2. 更新阶段 =====*/
    // 计算卡尔曼增益: K = P * H^T / (H * P * H^T + R)
    float H[2] = {1, 0}; // 观测矩阵
    float S = kf->P[0][0] + kf->R; // 创新协方差
    float K[2] = {kf->P[0][0]/S, kf->P[1][0]/S}; // 卡尔曼增益
    
    // 状态更新: x = x + K * (z - H * x)
    float y = measurement - kf->x[0];
    kf->x[0] += K[0] * y;
    kf->x[1] += K[1] * y;
    
    // 协方差更新: P = (I - K * H) * P
    float I_minus_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    kf->P[0][0] = I_minus_KH[0][0]*kf->P[0][0] + I_minus_KH[0][1]*kf->P[1][0];
    kf->P[0][1] = I_minus_KH[0][0]*kf->P[0][1] + I_minus_KH[0][1]*kf->P[1][1];
    kf->P[1][0] = I_minus_KH[1][0]*kf->P[0][0] + I_minus_KH[1][1]*kf->P[1][0];
    kf->P[1][1] = I_minus_KH[1][0]*kf->P[0][1] + I_minus_KH[1][1]*kf->P[1][1];
    
    return kf->x[0]; // 返回滤波后的位置
}


void kalman_Init(void) 
{
    // 初始化: 过程噪声(位置0.1, 速度0.5), 观测噪声0.2
    Kalman_Init(&kf_x, 0.1f, 0.5f, 0.2f);
    Kalman_Init(&kf_y, 0.1f, 0.5f, 0.2f);
}

