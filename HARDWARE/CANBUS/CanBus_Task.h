#ifndef __CANBUS_TASK_H
#define __CANBUS_TASK_H

#include "mytype.h"
#include "can.h"

#define FILTER_BUF_LEN 5

typedef enum
{
    CAN_3508Moto1_ID = 0x201,
    CAN_3508Moto2_ID = 0x202,
    CAN_3508Moto3_ID = 0x203,
    CAN_3508Moto4_ID = 0x204,
    CAN_3508Moto5_ID = 0x205,
    CAN_3508Moto6_ID = 0x206,
    CAN_3508Moto7_ID = 0x207,
    CAN_3508Moto8_ID = 0x208,
} CAN_Message_ID;

typedef struct
{
    int16_t speed_rpm;
    float real_current;
    int16_t given_current;
    uint8_t hall;
    uint16_t angle;
    uint16_t last_angle;
    uint16_t offset_angle;
    int32_t round_cnt;
    int32_t total_angle;
    u8 buf_idx;
    u16 angle_buf[FILTER_BUF_LEN];
    u16 fited_angle;
    u32 msg_cnt;
} moto_measure_t;

void CANFilterInit(void);
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
void SetMotorCurrent1(CAN_HandleTypeDef *hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void SetMotorCurrent2(CAN_HandleTypeDef *hcan, s16 iq5, s16 iq6, s16 iq7, s16 iq8);

void CharToFloat(float *fNum, unsigned char *strBuf, int nLen);
void FloatToChar(float fNum, unsigned char *strBuf, int nLen);
void Uint16ToChar(uint16_t fNum, unsigned char *strBuf, int nLen);

#endif
