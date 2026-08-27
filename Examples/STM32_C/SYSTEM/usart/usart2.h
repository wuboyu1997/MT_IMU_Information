#ifndef _USART2_H
#define _USART2_H
#include "sys.h"
#include "stdio.h"	

#define USART2_REC_LEN 1024

typedef struct
{
	uint32_t data_length;
	uint8_t data_buffer[256];
} data_trans_struct;		

void uart2_init(u32 bound);
void USART2_DMA_Send_data(u8 *puts, u16 size);
void USART2_Send_data(u8 *puts, u16 size);


#endif