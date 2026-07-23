#ifndef __VISION_COMM_H
#define __VISION_COMM_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void VisionComm_Init(UART_HandleTypeDef *huart);
void VisionComm_RxCpltCallback(UART_HandleTypeDef *huart);
void VisionComm_Poll(void);
uint8_t VisionComm_IsReady(void);

HAL_StatusTypeDef VisionComm_SendLine(const char *line);
HAL_StatusTypeDef VisionComm_SendClassifyTrigger(void);

#endif /* __VISION_COMM_H */
