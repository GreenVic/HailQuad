#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	

////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

//DMA接收GPS用
#define GPS_Size 1000
//DMA接收SBUS用
#define SBUS_Size 1000
extern u8 GPS_Buf[GPS_Size];   //通过DMA接收GPS数据
extern u8 ProcessBuffer[GPS_Size];
extern u8 SBUS_Buf[SBUS_Size];    //通过DMA接收SBUS数据
	  
///	  
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef UART1_Handler; //UART句柄
extern UART_HandleTypeDef USART3_Handler; //UART句柄
extern UART_HandleTypeDef UART4_Handler; //UART句柄


void USART3_init(u32 bound);
void uart4_init(u32 bound);
void uart_init(u32 bound);
#endif
