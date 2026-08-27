#include "imu.h"
#include "check_alg.h"
#include "usart2.h"
#include "usart.h"
#include <string.h>

imuData_t imuData;
uint8_t cmdDataSend[16];
uint8_t check_temp[2];
void MT_IMU_setRate(uint8_t* data_in){
	cmdDataSend[0] = 0x59;
  cmdDataSend[1] = 0x53;
	cmdDataSend[2] = 0x03;
  cmdDataSend[3] = 0x0A;
	cmdDataSend[4] = data_in[0];
  cmdDataSend[5] = data_in[1];
	dataCheck_MT(&cmdDataSend[2], 4, &check_temp[0], &check_temp[1]);
	cmdDataSend[6] = check_temp[0];
  cmdDataSend[7] = check_temp[1];
	USART2_Send_data(cmdDataSend, 8);
}
void MT_IMU_setMode(uint8_t* data_in){
	cmdDataSend[0] = 0x59;
  cmdDataSend[1] = 0x53;
	cmdDataSend[2] = 0x04;
  cmdDataSend[3] = 0x12;
	cmdDataSend[4] = 0x00;
	cmdDataSend[5] = data_in[0];
  cmdDataSend[6] = data_in[1];
	dataCheck_MT(&cmdDataSend[2], 5, &check_temp[0], &check_temp[1]);
	cmdDataSend[7] = check_temp[0];
  cmdDataSend[8] = check_temp[1];
	USART2_Send_data(cmdDataSend, 9);
}
void MT_IMU_setSync(uint8_t* data_in){
	cmdDataSend[0] = 0x59;
  cmdDataSend[1] = 0x53;
	cmdDataSend[2] = 0x05;
  cmdDataSend[3] = 0x0A;
	cmdDataSend[4] = data_in[0];
  cmdDataSend[5] = data_in[1];
	dataCheck_MT(&cmdDataSend[2], 4, &check_temp[0], &check_temp[1]);
	cmdDataSend[6] = check_temp[0];
  cmdDataSend[7] = check_temp[1];
	USART2_Send_data(cmdDataSend, 8);
}

uint8_t outputDataSend[16];
char tx_buffer[512];
void decodeAndPrnMode0(uint8_t* data_in){
	
	/*----------------------- decode ---------------------*/
	int162Char temp_int162Char;
	int2Char   temp_int2Char;
  
	// acc
	memcpy(temp_int2Char.charData,&data_in[7],4);
	imuData.Ax = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[11],4);
	imuData.Ay = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[15],4);
	imuData.Az = temp_int2Char.intData*1e-6f;
	
	// gyro
	memcpy(temp_int2Char.charData,&data_in[21],4);
	imuData.Gx = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[25],4);
	imuData.Gy = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[29],4);
	imuData.Gz = temp_int2Char.intData*1e-6f;
	
	// eular
	memcpy(temp_int2Char.charData,&data_in[35],4);
	imuData.pitch = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[39],4);
	imuData.roll = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[43],4);
	imuData.yaw = temp_int2Char.intData*1e-6f;
	
	// quaternion
	memcpy(temp_int2Char.charData,&data_in[49],4);
	imuData.q0 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[53],4);
	imuData.q1 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[57],4);
	imuData.q2 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[61],4);
	imuData.q3 = temp_int2Char.intData*1e-6f;
	
	/*----------------------- prn ---------------------*/
	int charOffset=0;
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Mode0: acc+gyro+eular+quaternion \r\n");
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Ax: %.3f  Ay: %.3f  Az: %.3f \r\n",imuData.Ax,imuData.Ay,imuData.Az);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Gx: %.3f  Gy: %.3f  Gz: %.3f \r\n",imuData.Gx,imuData.Gy,imuData.Gz);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "roll: %.3f  pitch: %.3f  yaw: %.3f \r\n",imuData.roll,imuData.pitch,imuData.yaw);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "q0: %.3f  q1: %.3f  q2: %.3f  q3: %.3f \r\n",imuData.q0,imuData.q1,imuData.q2,imuData.q3);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "\r\n");
	
	USART1_DMA_Send_data((uint8_t*)tx_buffer, strlen(tx_buffer));
}

void decodeAndPrnMode1(uint8_t* data_in){
	
	/*----------------------- decode ---------------------*/
	int162Char temp_int162Char;
	int2Char   temp_int2Char;
	
	// temperature
	memcpy(temp_int162Char.charData,&data_in[7],2);
  imuData.Temperature = temp_int162Char.int16Data*1e-2f;
	
	// acc
	memcpy(temp_int2Char.charData,&data_in[11],4);
	imuData.Ax = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[15],4);
	imuData.Ay = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[19],4);
	imuData.Az = temp_int2Char.intData*1e-6f;
	
	// gyro  //todo
	memcpy(temp_int2Char.charData,&data_in[25],4);
	imuData.Gx = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[29],4);
	imuData.Gy = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[33],4);
	imuData.Gz = temp_int2Char.intData*1e-6f;
	
	/*----------------------- prn ---------------------*/
	int charOffset=0;
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Mode1: temp+acc+gyro \r\n");
  charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Temperature: %.3f \r\n",imuData.Temperature);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Ax: %.3f  Ay: %.3f  Az: %.3f \r\n",imuData.Ax,imuData.Ay,imuData.Az);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Gx: %.3f  Gy: %.3f  Gz: %.3f \r\n",imuData.Gx,imuData.Gy,imuData.Gz);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "\r\n");
	
	USART1_DMA_Send_data((uint8_t*)tx_buffer, strlen(tx_buffer));
}

void decodeAndPrnMode2(uint8_t* data_in){
	
	/*----------------------- decode ---------------------*/
	int162Char temp_int162Char;
	int2Char   temp_int2Char;
  
	// acc
	memcpy(temp_int2Char.charData,&data_in[7],4);
	imuData.Ax = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[11],4);
	imuData.Ay = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[15],4);
	imuData.Az = temp_int2Char.intData*1e-6f;
	
	// gyro
	memcpy(temp_int2Char.charData,&data_in[21],4);
	imuData.Gx = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[25],4);
	imuData.Gy = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[29],4);
	imuData.Gz = temp_int2Char.intData*1e-6f;
	
	/*----------------------- prn ---------------------*/
	int charOffset=0;
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Mode2: acc+gyro \r\n");
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Ax: %.3f  Ay: %.3f  Az: %.3f \r\n",imuData.Ax,imuData.Ay,imuData.Az);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Gx: %.3f  Gy: %.3f  Gz: %.3f \r\n",imuData.Gx,imuData.Gy,imuData.Gz);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "\r\n");
	
	USART1_DMA_Send_data((uint8_t*)tx_buffer, strlen(tx_buffer));
}

void decodeAndPrnMode4(uint8_t* data_in){
	
	/*----------------------- decode ---------------------*/
	int162Char temp_int162Char;
	int2Char   temp_int2Char;
  
	// acc
	memcpy(temp_int2Char.charData,&data_in[7],4);
	imuData.Ax = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[11],4);
	imuData.Ay = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[15],4);
	imuData.Az = temp_int2Char.intData*1e-6f;
	
	// gyro
	memcpy(temp_int2Char.charData,&data_in[21],4);
	imuData.Gx = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[25],4);
	imuData.Gy = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[29],4);
	imuData.Gz = temp_int2Char.intData*1e-6f;
	
	// eular
	memcpy(temp_int2Char.charData,&data_in[35],4);
	imuData.pitch = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[39],4);
	imuData.roll = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[43],4);
	imuData.yaw = temp_int2Char.intData*1e-6f;
	
	/*----------------------- prn ---------------------*/
	int charOffset=0;
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Mode4: acc+gyro+eular \r\n");
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Ax: %.3f  Ay: %.3f  Az: %.3f \r\n",imuData.Ax,imuData.Ay,imuData.Az);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Gx: %.3f  Gy: %.3f  Gz: %.3f \r\n",imuData.Gx,imuData.Gy,imuData.Gz);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "roll: %.3f  pitch: %.3f  yaw: %.3f \r\n",imuData.roll,imuData.pitch,imuData.yaw);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "\r\n");
	
	USART1_DMA_Send_data((uint8_t*)tx_buffer, strlen(tx_buffer));
}

void decodeAndPrnMode5(uint8_t* data_in){
	
	/*----------------------- decode ---------------------*/
	int162Char temp_int162Char;
	int2Char   temp_int2Char;
	U642Char   temp_u642Char;
  
	// acc
	memcpy(temp_int2Char.charData,&data_in[7],4);
	imuData.Ax = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[11],4);
	imuData.Ay = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[15],4);
	imuData.Az = temp_int2Char.intData*1e-6f;
	
	// gyro
	memcpy(temp_int2Char.charData,&data_in[21],4);
	imuData.Gx = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[25],4);
	imuData.Gy = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[29],4);
	imuData.Gz = temp_int2Char.intData*1e-6f;
	
	// eular
	memcpy(temp_int2Char.charData,&data_in[35],4);
	imuData.pitch = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[39],4);
	imuData.roll = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[43],4);
	imuData.yaw = temp_int2Char.intData*1e-6f;
	
	// quaternion
	memcpy(temp_int2Char.charData,&data_in[49],4);
	imuData.q0 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[53],4);
	imuData.q1 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[57],4);
	imuData.q2 = temp_int2Char.intData*1e-6f;
	memcpy(temp_int2Char.charData,&data_in[61],4);
	imuData.q3 = temp_int2Char.intData*1e-6f;
	
	// timeStamp
	memcpy(temp_u642Char.charData,&data_in[67],8);
	imuData.timeStamp = temp_u642Char.u64Data*1e-6f;
	
	/*----------------------- prn ---------------------*/
	int charOffset=0;
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Mode5: acc+gyro+eular+quaternion+timeStamp \r\n");
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Ax: %.3f  Ay: %.3f  Az: %.3f \r\n",imuData.Ax,imuData.Ay,imuData.Az);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "Gx: %.3f  Gy: %.3f  Gz: %.3f \r\n",imuData.Gx,imuData.Gy,imuData.Gz);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "roll: %.3f  pitch: %.3f  yaw: %.3f \r\n",imuData.roll,imuData.pitch,imuData.yaw);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "q0: %.3f  q1: %.3f  q2: %.3f  q3: %.3f \r\n",imuData.q0,imuData.q1,imuData.q2,imuData.q3);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "timeStamp: %.3f \r\n",imuData.timeStamp);
	charOffset += snprintf(tx_buffer + charOffset, sizeof(tx_buffer) - charOffset, "\r\n");
	
	USART1_DMA_Send_data((uint8_t*)tx_buffer, strlen(tx_buffer));
}
