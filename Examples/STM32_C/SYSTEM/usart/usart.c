#include "usart.h"
#include "string.h"
#include "check_alg.h"
#include "imu.h"

u8 USART1_RX_BUF[USART1_REC_LEN];
uint32_t len1;
u8 usart1_send_flag=0;
uint8_t USART1_TX_BUF[USART1_REC_LEN];
UART_HandleTypeDef USART1_Handler;
DMA_HandleTypeDef HAL_DMA_USART1_RX_Handle;
DMA_HandleTypeDef HAL_DMA_USART1_TX_Handle;

void uart1_init(u32 bound)
{	
	GPIO_InitTypeDef GPIO_Initure;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	PeriphClkInitStruct.PeriphClockSelection =RCC_PERIPHCLK_USART1;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){}
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE(); 
	
	GPIO_Initure.Pin=GPIO_PIN_14;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;
	GPIO_Initure.Pull=GPIO_PULLUP;
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_Initure.Alternate=GPIO_AF4_USART1;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	GPIO_Initure.Pin=GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	HAL_DMA_USART1_TX_Handle.Instance = DMA1_Stream6;
	HAL_DMA_USART1_TX_Handle.Init.Request=DMA_REQUEST_USART1_TX;
	HAL_DMA_USART1_TX_Handle.Init.Direction=DMA_MEMORY_TO_PERIPH;
	HAL_DMA_USART1_TX_Handle.Init.PeriphInc=DMA_PINC_DISABLE;
	HAL_DMA_USART1_TX_Handle.Init.MemInc=DMA_MINC_ENABLE;
	HAL_DMA_USART1_TX_Handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
	HAL_DMA_USART1_TX_Handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
	HAL_DMA_USART1_TX_Handle.Init.Mode=DMA_NORMAL;
	HAL_DMA_USART1_TX_Handle.Init.Priority=DMA_PRIORITY_MEDIUM;
	HAL_DMA_USART1_TX_Handle.Init.FIFOMode=DMA_FIFOMODE_DISABLE;          
	HAL_DMA_USART1_TX_Handle.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	HAL_DMA_USART1_TX_Handle.Init.MemBurst=DMA_MBURST_SINGLE;
	HAL_DMA_USART1_TX_Handle.Init.PeriphBurst=DMA_PBURST_SINGLE;
	HAL_DMA_DeInit(&HAL_DMA_USART1_TX_Handle); 
  if(HAL_DMA_Init(&HAL_DMA_USART1_TX_Handle)!=HAL_OK){}    	
	__HAL_LINKDMA(&USART1_Handler,hdmatx,HAL_DMA_USART1_TX_Handle);
	HAL_DMA_USART1_RX_Handle.Instance = DMA1_Stream5;
	HAL_DMA_USART1_RX_Handle.Init.Request=DMA_REQUEST_USART1_RX;
	HAL_DMA_USART1_RX_Handle.Init.Direction=DMA_PERIPH_TO_MEMORY;
	HAL_DMA_USART1_RX_Handle.Init.PeriphInc=DMA_PINC_DISABLE;
	HAL_DMA_USART1_RX_Handle.Init.MemInc=DMA_MINC_ENABLE;
	HAL_DMA_USART1_RX_Handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
	HAL_DMA_USART1_RX_Handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
	HAL_DMA_USART1_RX_Handle.Init.Mode=DMA_NORMAL;
	HAL_DMA_USART1_RX_Handle.Init.Priority=DMA_PRIORITY_MEDIUM;
	HAL_DMA_USART1_RX_Handle.Init.FIFOMode=DMA_FIFOMODE_ENABLE;          
	HAL_DMA_USART1_RX_Handle.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	HAL_DMA_USART1_RX_Handle.Init.MemBurst=DMA_MBURST_SINGLE;
	HAL_DMA_USART1_RX_Handle.Init.PeriphBurst=DMA_PBURST_SINGLE;
	if( HAL_DMA_Init(&HAL_DMA_USART1_RX_Handle)!=HAL_OK){}    
  __HAL_LINKDMA(&USART1_Handler,hdmarx,HAL_DMA_USART1_RX_Handle);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn,8,1);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn,9,1);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	USART1_Handler.Instance=USART1;
	USART1_Handler.Init.BaudRate=bound;				
	USART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;  
	USART1_Handler.Init.StopBits=UART_STOPBITS_1;	  
	USART1_Handler.Init.Parity=UART_PARITY_NONE;		  
	USART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   
	USART1_Handler.Init.Mode=UART_MODE_TX_RX;		   
	USART1_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
	USART1_Handler.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	USART1_Handler.Init.Prescaler = UART_PRESCALER_DIV1;
	USART1_Handler.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&USART1_Handler);	

  HAL_NVIC_SetPriority(USART1_IRQn,6,0); 
  HAL_NVIC_EnableIRQ(USART1_IRQn); 
  __HAL_UART_CLEAR_IDLEFLAG(&USART1_Handler); 
	delay_ms(1);
  HAL_UART_Receive_DMA(&USART1_Handler,USART1_RX_BUF,USART1_REC_LEN); 
  __HAL_UART_ENABLE_IT(&USART1_Handler,UART_IT_IDLE);
  delay_ms(1);
}

void USART1_IRQHandler(void)
{
  if(__HAL_UART_GET_FLAG(&USART1_Handler,UART_FLAG_IDLE)!=RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&USART1_Handler);

		HAL_UART_Receive_DMA(&USART1_Handler,USART1_RX_BUF,USART1_REC_LEN);
	}
   else if(__HAL_UART_GET_FLAG(&USART1_Handler,UART_FLAG_ORE)!=RESET)
	{
	  	__HAL_UART_CLEAR_IT(&USART1_Handler,UART_FLAG_ORE);
	}
	else
	{
	   HAL_UART_IRQHandler(&USART1_Handler);
	}
}   
 void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&HAL_DMA_USART1_RX_Handle);  
}
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&HAL_DMA_USART1_TX_Handle);  
}

void USART1_DMA_Send_data(u8 *puts, u16 size)
{
  memcpy(USART1_TX_BUF,puts,size);
  SCB_CleanDCache();
  HAL_UART_Transmit_DMA (&USART1_Handler,(uint8_t *)USART1_TX_BUF,size);
}

void USART1_Send_data(u8 *puts, u16 size)
{
  memcpy(USART1_TX_BUF,puts,size);
  SCB_CleanDCache();
  HAL_UART_Transmit(&USART1_Handler,(uint8_t *)USART1_TX_BUF,size,1000);
}