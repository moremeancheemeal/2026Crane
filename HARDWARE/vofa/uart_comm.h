#ifndef __UART_COMM_H__
#define __UART_COMM_H__


#include "main.h"
 

#define UART_BUFFER_SIZE 100
extern uint8_t uart_rx_buffer[UART_BUFFER_SIZE]; 
extern int buffer_index;    // 当前接收数据的索引
void UART_ParseCommand(uint8_t *data); // 命令解析函数声明

#endif
