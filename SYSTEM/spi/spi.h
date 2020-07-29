#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


extern SPI_HandleTypeDef SPI2_Handler;  //SPI¾ä±ú

void SPI2_Init(void);
void SPI2_SetSpeed(u32 SPI_BaudRatePrescaler);
u8 SPI2_ReadWriteByte(u8 TxData);
#endif
