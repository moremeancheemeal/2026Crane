#include "CanBus_Task.h"

#include "MotorControlTask.h"

uint16_t can_cnt0;

void CANFilterInit(void)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t motor_index = 0;
    uint8_t motor_found = 0;

    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    if (_hcan->Instance == CAN1)
    {
        /* 6 个 M3508:0x201~0x206 连续映射到 Motor[0..5](= ID - 0x201) */
        if (rx_header.StdId >= CAN_3508Moto1_ID && rx_header.StdId <= CAN_3508Moto6_ID)
        {
            motor_index = (uint8_t)(rx_header.StdId - CAN_3508Moto1_ID);
            motor_found = 1;
        }

        if (motor_found)
        {
            can_cnt0++;
            Motor[motor_index].msg_cnt++ <= 50 ? GetMotorOffset(&Motor[motor_index], rx_data) : GetMotorMeasure(&Motor[motor_index], rx_data);
        }
    }

    if (can_cnt0 == 500)
    {
        can_cnt0 = 0;
    }
}

void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[])
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
    ptr->speed_rpm = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    ptr->real_current = (float)(can_rx_data[4] << 8 | can_rx_data[5]) * 5.0f / 16384.0f;
    ptr->hall = can_rx_data[6];

    if (ptr->angle - ptr->last_angle > 4096)
    {
        ptr->round_cnt--;
    }
    else if (ptr->angle - ptr->last_angle < -4096)
    {
        ptr->round_cnt++;
    }

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[])
{
    ptr->angle = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
    ptr->offset_angle = ptr->angle;
}

void SetMotorCurrent1(CAN_HandleTypeDef *hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    tx_header.StdId = 0x200;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;

    tx_data[0] = (uint8_t)(iq1 >> 8);
    tx_data[1] = (uint8_t)iq1;
    tx_data[2] = (uint8_t)(iq2 >> 8);
    tx_data[3] = (uint8_t)iq2;
    tx_data[4] = (uint8_t)(iq3 >> 8);
    tx_data[5] = (uint8_t)iq3;
    tx_data[6] = (uint8_t)(iq4 >> 8);
    tx_data[7] = (uint8_t)iq4;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}

void SetMotorCurrent2(CAN_HandleTypeDef *hcan, s16 iq5, s16 iq6, s16 iq7, s16 iq8)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    tx_header.StdId = 0x1FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;

    tx_data[0] = (uint8_t)(iq5 >> 8);
    tx_data[1] = (uint8_t)iq5;
    tx_data[2] = (uint8_t)(iq6 >> 8);
    tx_data[3] = (uint8_t)iq6;
    tx_data[4] = (uint8_t)(iq7 >> 8);
    tx_data[5] = (uint8_t)iq7;
    tx_data[6] = (uint8_t)(iq8 >> 8);
    tx_data[7] = (uint8_t)iq8;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}

void CharToFloat(float *fNum, unsigned char *strBuf, int nLen)
{
    if (nLen < 4)
    {
        return;
    }

    char *p = (char *)fNum;
    for (int i = 0; i < 4; i++)
    {
        p[i] = (char)strBuf[i];
    }
}

void FloatToChar(float fNum, unsigned char *strBuf, int nLen)
{
    if (nLen < 4)
    {
        return;
    }

    char *p = (char *)&fNum;
    for (int i = 0; i < 4; i++)
    {
        strBuf[i] = (unsigned char)p[i];
    }
}

void Uint16ToChar(uint16_t fNum, unsigned char *strBuf, int nLen)
{
    if (nLen < 2)
    {
        return;
    }

    char *p = (char *)&fNum;
    for (int i = 0; i < 2; i++)
    {
        strBuf[i] = (unsigned char)p[i];
    }
}
