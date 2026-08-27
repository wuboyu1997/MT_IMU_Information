#ifndef __MAX1486_H
#define __MAX1486_H

#include "sys.h"
#include "includes.h"

#define max1486_tx_en(n)  (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET))//SPIÆ¬Ñ¡ÐÅºÅ	   

void MAX1486_init(void);
void MAX1486_ctr(uint8_t ctr_num);



#endif