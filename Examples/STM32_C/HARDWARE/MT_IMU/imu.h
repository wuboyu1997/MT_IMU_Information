#ifndef _IMU_H
#define _IMU_H
#include <stdio.h>
#include <stdbool.h>
#include "check_alg.h"

#define validDataLength_Mode0 0x3C
#define validDataLength_Mode1 0x20
#define validDataLength_Mode2 0x1C
#define validDataLength_Mode4 0x2A
#define validDataLength_Mode5 0x46

#define GET_RateCmd(type) configRate[type]
#define GET_ModeCmd(type) configMode[type]
#define GET_SyncCmd(type) configSync[type]

typedef enum {
    RateCmd_1Hz,
    RateCmd_2Hz,
    RateCmd_5Hz,
    RateCmd_10Hz,
    RateCmd_20Hz,
    RateCmd_25Hz,
    RateCmd_50Hz,
    RateCmd_100Hz,
    RateCmd_200Hz,
    RateCmd_400Hz,
    RateType_COUNT
} ConfigType_Rate;

typedef enum {
    ModeCmd_Mode0,
    ModeCmd_Mode1,
    ModeCmd_Mode2,
    ModeCmd_Mode3,
    ModeCmd_Mode4,
    ModeCmd_Mode5,
    ModeType_COUNT
} ConfigType_Mode;

typedef enum {
    SyncCmd_OUT,
    SyncCmd_IN,
    SyncCmd_RMC,
    SyncType_COUNT
} ConfigType_Sync;

static uint8_t configRate[RateType_COUNT][2] = {
    {0x00, 0x01},
		{0x00, 0x02},
		{0x00, 0x03},
		{0x00, 0x04},	
		{0x00, 0x05},
		{0x00, 0x06},
		{0x00, 0x07},
    {0x00, 0x08},
		{0x00, 0x09},
		{0x00, 0x0A}
};

static uint8_t configMode[ModeType_COUNT][2] = {
    {0xD8, 0x00},
		{0xC0, 0x04},
		{0xC0, 0x00},
		{0x00, 0x00},
		{0xD0, 0x01},
		{0xD0, 0x10}
};

static uint8_t configSync[SyncType_COUNT][2] = {
    {0x00, 0x00},
		{0x00, 0x01},
		{0x00, 0x02}
};


typedef struct
{
	float Temperature;
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	float q0;
	float q1;
	float q2;
	float q3;
  float pitch;
	float roll;
	float yaw;
	double timeStamp;
} imuData_t;

typedef union
{
	int16_t int16Data;
	char charData[2];
}int162Char;

typedef union
{
	int intData;
	char charData[4];
}int2Char;

typedef union
{
	uint64_t u64Data;
	char charData[8];
}U642Char;

void MT_IMU_setRate(uint8_t* data_in);
void MT_IMU_setMode(uint8_t* data_in);
void MT_IMU_setSync(uint8_t* data_in);

void decodeAndPrnMode0(uint8_t* data_in);
void decodeAndPrnMode1(uint8_t* data_in);
void decodeAndPrnMode2(uint8_t* data_in);
void decodeAndPrnMode4(uint8_t* data_in);
void decodeAndPrnMode5(uint8_t* data_in);



#endif
