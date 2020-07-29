#ifndef  __SBUS_H
#define  __SBUS_H
#include "sys.h"
#include "usart.h" 
#include "dma.h"
#include "string.h"


extern u8 SBUS_Temp[25];
extern u16 Remoter_Channel[16];
extern u16 Cali_Channel[16];      //�궨���16ͨ�����ݳ�ֵ


extern void ProcessSBUS(void);
extern void Sbus_Recieve(void);
extern void Remoter_Cali(void);





















#endif


