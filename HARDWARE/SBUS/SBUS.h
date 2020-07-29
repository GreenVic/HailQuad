#ifndef  __SBUS_H
#define  __SBUS_H
#include "sys.h"
#include "usart.h" 
#include "dma.h"
#include "string.h"


extern u8 SBUS_Temp[25];
extern u16 Remoter_Channel[16];
extern u16 Cali_Channel[16];      //标定后的16通道数据初值


extern void ProcessSBUS(void);
extern void Sbus_Recieve(void);
extern void Remoter_Cali(void);





















#endif


