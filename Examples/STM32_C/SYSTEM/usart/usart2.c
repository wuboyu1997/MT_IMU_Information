#include "usart2.h"
#include "string.h"
#include "check_alg.h"
#include "imu.h"

u8 USART2_RX_BUF[USART2_REC_LEN];  
u32 len2;  
u8 usart2_send_flag=0; 
uint8_t USART2_TX_BUF[USART2_REC_LEN];  
UART_HandleTypeDef USART2_Handler; 
DMA_HandleTypeDef HAL_DMA_USART2_RX_Handle; 
DMA_HandleTypeDef HAL_DMA_USART2_TX_Handle; 
extern data_trans_struct imu_data_trans;
extern uint8_t start_listen_flag;
extern uint8_t DataRDY;

void uart2_init(u32 bound)
{	
	GPIO_InitTypeDef GPIO_Initure;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	PeriphClkInitStruct.PeriphClockSelection =RCC_PERIPHCLK_USART2; 
	PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){}
	__HAL_RCC_GPIOA_CLK_ENABLE();		
	__HAL_RCC_USART2_CLK_ENABLE();		
	__HAL_RCC_DMA1_CLK_ENABLE();          
	
	GPIO_Initure.Pin=GPIO_PIN_2;		
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		
	GPIO_Initure.Pull=GPIO_PULLUP;		
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	  
	GPIO_Initure.Pin=GPIO_PIN_3;			
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   
	
	HAL_DMA_USART2_TX_Handle.Instance = DMA1_Stream4;
	HAL_DMA_USART2_TX_Handle.Init.Request=DMA_REQUEST_USART2_TX;		
	HAL_DMA_USART2_TX_Handle.Init.Direction=DMA_MEMORY_TO_PERIPH;          
	HAL_DMA_USART2_TX_Handle.Init.PeriphInc=DMA_PINC_DISABLE;            
	HAL_DMA_USART2_TX_Handle.Init.MemInc=DMA_MINC_ENABLE;                  
	HAL_DMA_USART2_TX_Handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    
	HAL_DMA_USART2_TX_Handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;     
	HAL_DMA_USART2_TX_Handle.Init.Mode=DMA_NORMAL;                           
	HAL_DMA_USART2_TX_Handle.Init.Priority=DMA_PRIORITY_MEDIUM;            
	HAL_DMA_USART2_TX_Handle.Init.FIFOMode=DMA_FIFOMODE_DISABLE;          
	HAL_DMA_USART2_TX_Handle.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	HAL_DMA_USART2_TX_Handle.Init.MemBurst=DMA_MBURST_SINGLE;                
	HAL_DMA_USART2_TX_Handle.Init.PeriphBurst=DMA_PBURST_SINGLE;          
	HAL_DMA_DeInit(&HAL_DMA_USART2_TX_Handle); 
	if(HAL_DMA_Init(&HAL_DMA_USART2_TX_Handle)!=HAL_OK){}    	
	__HAL_LINKDMA(&USART2_Handler,hdmatx,HAL_DMA_USART2_TX_Handle);   

	HAL_DMA_USART2_RX_Handle.Instance = DMA1_Stream2;
	HAL_DMA_USART2_RX_Handle.Init.Request=DMA_REQUEST_USART2_RX;			
	HAL_DMA_USART2_RX_Handle.Init.Direction=DMA_PERIPH_TO_MEMORY;       
	HAL_DMA_USART2_RX_Handle.Init.PeriphInc=DMA_PINC_DISABLE;            
	HAL_DMA_USART2_RX_Handle.Init.MemInc=DMA_MINC_ENABLE;                  
	HAL_DMA_USART2_RX_Handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;   
	HAL_DMA_USART2_RX_Handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;      
	HAL_DMA_USART2_RX_Handle.Init.Mode=DMA_NORMAL;                            
	HAL_DMA_USART2_RX_Handle.Init.Priority=DMA_PRIORITY_MEDIUM;              
	HAL_DMA_USART2_RX_Handle.Init.FIFOMode=DMA_FIFOMODE_ENABLE;          
	HAL_DMA_USART2_RX_Handle.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	HAL_DMA_USART2_RX_Handle.Init.MemBurst=DMA_MBURST_SINGLE;                 
	HAL_DMA_USART2_RX_Handle.Init.PeriphBurst=DMA_PBURST_SINGLE;            
	if( HAL_DMA_Init(&HAL_DMA_USART2_RX_Handle)!=HAL_OK){}    
	__HAL_LINKDMA(&USART2_Handler,hdmarx,HAL_DMA_USART2_RX_Handle); 

  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn,8,0);   
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);         
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn,9,0);   
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);        

	USART2_Handler.Instance=USART2;		
	USART2_Handler.Init.BaudRate=bound;			
	USART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;  
	USART2_Handler.Init.StopBits=UART_STOPBITS_1;	   
	USART2_Handler.Init.Parity=UART_PARITY_NONE;		
	USART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   
	USART2_Handler.Init.Mode=UART_MODE_TX_RX;		 
	USART2_Handler.Init.OverSampling = UART_OVERSAMPLING_16;
	USART2_Handler.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	USART2_Handler.Init.Prescaler = UART_PRESCALER_DIV1;
	USART2_Handler.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&USART2_Handler);					   

  HAL_NVIC_SetPriority(USART2_IRQn,7,0);  
  HAL_NVIC_EnableIRQ(USART2_IRQn);         
  __HAL_UART_CLEAR_IDLEFLAG(&USART2_Handler); 
  delay_ms(1);
  HAL_UART_Receive_DMA(&USART2_Handler,USART2_RX_BUF,USART2_REC_LEN); 
  __HAL_UART_ENABLE_IT(&USART2_Handler,UART_IT_IDLE);
  delay_ms(1);
}

void USART2_IRQHandler(void)
{
  if(__HAL_UART_GET_FLAG(&USART2_Handler,UART_FLAG_IDLE)!=RESET) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&USART2_Handler); 
		HAL_UART_DMAStop(&USART2_Handler);
		len2= USART2_REC_LEN-__HAL_DMA_GET_COUNTER(&HAL_DMA_USART2_RX_Handle);
    SCB_InvalidateDCache();
		if(start_listen_flag==1 && len2>0){
			imu_data_trans.data_length = len2;
			memcpy(imu_data_trans.data_buffer,USART2_RX_BUF,imu_data_trans.data_length);
			DataRDY = 1;
		}
		HAL_UART_Receive_DMA(&USART2_Handler,USART2_RX_BUF,USART2_REC_LEN);
	}
  else if(__HAL_UART_GET_FLAG(&USART2_Handler,UART_FLAG_ORE)!=RESET)
	{
	  	__HAL_UART_CLEAR_IT(&USART2_Handler,UART_FLAG_ORE);
	}
	else
	{
	   HAL_UART_IRQHandler(&USART2_Handler);
	}
}   
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&HAL_DMA_USART2_RX_Handle);  
}
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&HAL_DMA_USART2_TX_Handle);  
}
void USART2_DMA_Send_data(u8 *puts, u16 size)
{
  memcpy(USART2_TX_BUF,puts,size);
  SCB_CleanDCache();
  HAL_UART_Transmit_DMA(&USART2_Handler,(uint8_t *)USART2_TX_BUF,size);
}
void USART2_Send_data(u8 *puts, u16 size)
{
  memcpy(USART2_TX_BUF,puts,size);
  SCB_CleanDCache();
	HAL_UART_Transmit(&USART2_Handler, (uint8_t *)USART2_TX_BUF, size, 1);
}