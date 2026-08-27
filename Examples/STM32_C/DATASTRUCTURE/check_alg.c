#include "check_alg.h"

uint16_t CRC16(uint8_t *data, int len)
{
	uint16_t CRC_ori = 0xFFFF;
	uint8_t index;
	for (int i = 0; i < len ; i++)
	{
		index = (CRC_ori & 0xFF)^ data[i];
		CRC_ori >>= 8;
		CRC_ori ^= auchCRC[index];
	}
	return CRC_ori;
}

uint8_t XORCheck(uint8_t* buffer,uint8_t len)
{
	uint8_t xorcheck=0;
	uint8_t i=0;
	if((buffer[0] == 0xBD) && (buffer[1] == 0xDB) && (buffer[2] == 0x0A))
	{
		for(i=0;i<len-1;i++)
		{
			xorcheck^=buffer[i];
		}
		if(xorcheck == buffer[len-1])
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}

void dataCheck_MT(uint8_t* data_in, uint8_t data_length, uint8_t* ck1, uint8_t* ck2){
	*ck1 = 0;
	*ck2 = 0;
	for(uint8_t i=0; i<data_length; ++i){
		*ck1 += data_in[i];
		*ck2 += *ck1;
	}
}


