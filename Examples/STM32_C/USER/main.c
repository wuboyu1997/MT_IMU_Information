/*
 * ============================================================================
 *  Copyright (C) 2026 [河北美泰电子科技有限公司]. All rights reserved.
 *
 *  程序名   : MT_IMU_STM32H743
 *  作者     : Wu Boyu
 *  创建日期 : 2026-08-27
 *  版本     : 1.0.3
 *  描述     : IMU数据采集、解析、打印程序
 * ============================================================================
 */

#include "includes.h"
#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "usart2.h" 
#include "dma.h"
#include "stmflash.h"
#include "MAX1486.h"
#include "imu.h"

uint8_t DataRDY = 0;
uint8_t start_listen_flag = 0;
data_trans_struct imu_data_trans;
uint8_t ck_check[2] = {0x00};

int main(void)
{
	u16 i;
	u8 t=0;
	u8 j=0;
  DataRDY = 0;
	Cache_Enable();                			
	HAL_Init();				        		
	Stm32_Clock_Init(160,5,2,4);  	
	delay_init(400);					
	MAX1486_init();
	MAX1486_ctr(1);
	
	uart1_init(921600);           // 用作开发板输出
  uart2_init(460800);						// 用作IMU输入
	
	/*--------------- IMU 配置 ------------------*/
	// 此处仅为代码示例，实际使用时相关配置会写入FLASH，无需每次上电配置
	uint8_t data_send_temp[2];
	
	delay_ms(5000);
	data_send_temp[0] = *GET_RateCmd(RateCmd_100Hz);             // 配置刷新率
	data_send_temp[1] = *(GET_RateCmd(RateCmd_100Hz)+1);
	MT_IMU_setRate(data_send_temp);
  delay_ms(5000);
	
	data_send_temp[0] = *GET_ModeCmd(ModeCmd_Mode0);             // 配置数据输出模式
	data_send_temp[1] = *(GET_ModeCmd(ModeCmd_Mode0)+1);
	MT_IMU_setMode(data_send_temp);
  delay_ms(5000);
	
	data_send_temp[0] = *GET_SyncCmd(SyncCmd_OUT);             // 配置时间同步类型
	data_send_temp[1] = *(GET_SyncCmd(SyncCmd_OUT)+1);
	MT_IMU_setSync(data_send_temp);
  delay_ms(5000);
	
  /*--------------- 数据解析和打印 ------------------*/
	start_listen_flag = 1;
	while(1){
		if(DataRDY == 1){
			DataRDY=0;
			if(imu_data_trans.data_buffer[0]==0x59&&imu_data_trans.data_buffer[1]==0x53){
				ck_check[0]=0;
				ck_check[1]=0;
				dataCheck_MT(&(imu_data_trans.data_buffer[2]),imu_data_trans.data_length-4,&ck_check[0],&ck_check[1]);
				if(ck_check[0]==imu_data_trans.data_buffer[imu_data_trans.data_length-2] && ck_check[1]==imu_data_trans.data_buffer[imu_data_trans.data_length-1]){
					switch(imu_data_trans.data_buffer[4]){
							case validDataLength_Mode0:
									decodeAndPrnMode0(imu_data_trans.data_buffer);
									break;
							case validDataLength_Mode1:
									decodeAndPrnMode1(imu_data_trans.data_buffer);
									break;
							case validDataLength_Mode2:
									decodeAndPrnMode2(imu_data_trans.data_buffer);
									break;
							case validDataLength_Mode4:
									decodeAndPrnMode4(imu_data_trans.data_buffer);
									break;
							case validDataLength_Mode5:
									decodeAndPrnMode5(imu_data_trans.data_buffer);
									break;
							default:
									break;
					}
				}
			}
		}
	}
}

