#ifndef __DMA_H
#define __DMA_H
#include "sys.h"


extern DMA_HandleTypeDef  USART3TxDMA_Handler;      //DMA���;��
extern DMA_HandleTypeDef  USART3RxDMA_Handler;      //DMA���ܾ��

extern DMA_HandleTypeDef  UART4TxDMA_Handler;      //DMA���;��
extern DMA_HandleTypeDef  UART4RxDMA_Handler;      //DMA���;��

void MYDMA_Config(void);
 
#endif
