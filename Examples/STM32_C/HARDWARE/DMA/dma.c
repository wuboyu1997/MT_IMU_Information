#include "dma.h"
#include "usart.h"
#include "usart2.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//DMA驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/8/14
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern UART_HandleTypeDef USART1_Handler;
extern UART_HandleTypeDef USART2_Handler;
DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA句柄 Tx
DMA_HandleTypeDef  UART1RxDMA_Handler;      //DMA句柄 Rx
volatile uint8_t IMU_rx_buffer[100];   //接收数据的数组

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
void DMA_Config_uart1()
{ 
    __HAL_RCC_DMA2_CLK_ENABLE();//DMA1时钟使能 
	
    
    //Tx DMA配置
    UART1TxDMA_Handler.Instance=DMA2_Stream6;                            //数据流选择
		UART1TxDMA_Handler.Init.Request=DMA_REQUEST_USART1_TX;				//USART1发送DMA
    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	
		HAL_DMA_DeInit(&UART1TxDMA_Handler);
		HAL_DMA_Init(&UART1TxDMA_Handler);
		__HAL_LINKDMA(&USART1_Handler,hdmatx,UART1TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
	
	  //Rx DMA配置
    UART1RxDMA_Handler.Instance=DMA2_Stream7;                            //数据流选择
		UART1RxDMA_Handler.Init.Request=DMA_REQUEST_USART1_RX;				//USART1发送DMA
    UART1RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //存储器到外设
    UART1RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART1RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART1RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART1RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART1RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    UART1RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    UART1RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART1RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
		
		HAL_DMA_DeInit(&UART1RxDMA_Handler);
		HAL_DMA_Init(&UART1RxDMA_Handler);
		__HAL_LINKDMA(&USART1_Handler,hdmarx,UART1RxDMA_Handler);
	
		HAL_UART_Receive_DMA(&USART1_Handler,IMU_rx_buffer,100);  //开启DMA接收中断
}

DMA_HandleTypeDef  UART2TxDMA_Handler;      //DMA句柄 Tx
DMA_HandleTypeDef  UART2RxDMA_Handler;      //DMA句柄 Rx
volatile uint8_t debug_rx_buffer[100];   //接收数据的数组
 
void DMA_Config_uart2()
{ 
    __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	
    //Tx DMA配置
    UART2TxDMA_Handler.Instance=DMA1_Stream2;                            //数据流选择
		UART2TxDMA_Handler.Init.Request=DMA_REQUEST_USART2_TX;				//USART1发送DMA
    UART2TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    UART2TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART2TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART2TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART2TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART2TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    UART2TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    UART2TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART2TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART2TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART2TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
		
		HAL_DMA_DeInit(&UART2TxDMA_Handler);
		HAL_DMA_Init(&UART2TxDMA_Handler);
		__HAL_LINKDMA(&USART2_Handler,hdmatx,UART2TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
	
	  //Rx DMA配置
    UART2RxDMA_Handler.Instance=DMA1_Stream3;                            //数据流选择
		UART2RxDMA_Handler.Init.Request=DMA_REQUEST_USART2_RX;				//USART1发送DMA
    UART2RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //存储器到外设
    UART2RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART2RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART2RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART2RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART2RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    UART2RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    UART2RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART2RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART2RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART2RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	
    
		HAL_DMA_DeInit(&UART2RxDMA_Handler);
		HAL_DMA_Init(&UART2RxDMA_Handler);
		__HAL_LINKDMA(&USART2_Handler,hdmarx,UART2RxDMA_Handler);
		
		HAL_UART_Receive_DMA(&USART2_Handler,debug_rx_buffer,100);  //开启DMA接收中断
}
