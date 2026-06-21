#ifndef __VOFAPLUS_H__
#define __VOFAPLUS_H__

#include "main.h"

typedef struct
{
  float ch_data[2];
	uint8_t  tail[4];
} Vofa_data_m_2;//定义一个结构体，存放包尾和2个浮点数

typedef struct
{
  float ch_data[4];
	uint8_t  tail[4];
} Vofa_data_m_4;//定义一个结构体，存放包尾和4个浮点数


typedef struct
{
  float ch_data[8];
	uint8_t  tail[4];
} Vofa_data_m_8;//定义一个结构体，存放包尾和8个浮点数


extern Vofa_data_m_2 Vofa_data_2;
extern Vofa_data_m_4 Vofa_data_4;
extern Vofa_data_m_8 Vofa_data_8;

void Vofa_Send_Data2(float data1, float data2);
void Vofa_Send_Data4(float data1, float data2,float data3, float data4);
void Vofa_Send_Data8(float data1, float data2,float data3, float data4,float data5, float data6,float data7, float data8);

#endif
