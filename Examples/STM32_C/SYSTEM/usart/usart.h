#ifndef _USART_H
#define _USART_H

#include "sys.h"
#include "stdio.h"	

#define USART1_REC_LEN 1024

void uart1_init(u32 bound);
void USART1_DMA_Send_data(u8 *puts, u16 size);
void USART1_Send_data(u8 *puts, u16 size);
#endif
