#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"	

////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

//DMA����GPS��
#define GPS_Size 1000
//DMA����SBUS��
#define SBUS_Size 1000
extern u8 GPS_Buf[GPS_Size];   //ͨ��DMA����GPS����
extern u8 ProcessBuffer[GPS_Size];
extern u8 SBUS_Buf[SBUS_Size];    //ͨ��DMA����SBUS����
	  
///	  
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern UART_HandleTypeDef UART1_Handler; //UART���
extern UART_HandleTypeDef USART3_Handler; //UART���
extern UART_HandleTypeDef UART4_Handler; //UART���


void USART3_init(u32 bound);
void uart4_init(u32 bound);
void uart_init(u32 bound);
#endif
