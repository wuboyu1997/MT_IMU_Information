#include "MAX1486.h"
    
extern UART_HandleTypeDef USART1_Handler;

void MAX1486_init(){
    
		GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();	
    
	  // H//F
    GPIO_Initure.Pin=GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull=GPIO_NOPULL; 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_LOW; 
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);		
	
		// TXEN
    GPIO_Initure.Pin=GPIO_PIN_13; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; 
    GPIO_Initure.Pull=GPIO_NOPULL;      
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);   
		max1486_tx_en(0);   
}

void MAX1486_ctr(uint8_t ctr_num){
	if(ctr_num==1){
		USART1_Handler.Instance->CR1  &= ~(1<<2);
	}
	else{
		USART1_Handler.Instance->CR1  |= (1<<2);
		__HAL_UART_CLEAR_IDLEFLAG(&USART1_Handler);
	}
	max1486_tx_en(ctr_num);
}