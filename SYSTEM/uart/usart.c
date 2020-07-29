#include "usart.h"
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	

//#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc����                       ѡ���ӡ����
int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0);//ѭ������,ֱ���������   
	USART1->TDR=(u8)ch;      
	return ch;
}




u8 SBUS_Buf[SBUS_Size];       //SBUS�õĽ��ջ���
u8 GPS_Buf[GPS_Size];      //GPS�õĽ��ջ���
u8 ProcessBuffer[GPS_Size];




UART_HandleTypeDef UART1_Handler; //UART���
UART_HandleTypeDef USART3_Handler; //UART���
UART_HandleTypeDef UART4_Handler; //UART4���


extern DMA_HandleTypeDef  USART3TxDMA_Handler;      //DMA���;��
extern DMA_HandleTypeDef  USART3RxDMA_Handler;      //DMA���ܾ��








//��ʼ��IO ����1 
//bound:������
/////////////////////////��ӡ�ô���1////////////////////////////
void uart_init(u32 bound)
{	
	//UART ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
	
  
}
///////////////////////////////////////////////////////////////////



//////////////////////////DMA����GPS�ô���3///////////////////////
void USART3_init(u32 bound)
{	
	//UART ��ʼ������
	USART3_Handler.Instance=USART3;					    //USART3
	USART3_Handler.Init.BaudRate=bound;				    //������
	USART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	USART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	USART3_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	USART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	USART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&USART3_Handler);					    //HAL_UART_Init()��ʹ��USART3
	
	
//	__HAL_UART_ENABLE_IT(&USART3_Handler, UART_IT_IDLE);          //�������н����ж�
	__HAL_UART_CLEAR_IDLEFLAG(&USART3_Handler);  //�����־λ 
	HAL_UART_Receive_DMA(&USART3_Handler,GPS_Buf,GPS_Size);//��DMA���գ�ָ�����ջ������ͽ��մ�С		HAL_UART_Receive_DMA(&USART3_Handler,GPS_Buf,GPS_Size);	//��DMA���գ�ָ�����ջ������ͽ��մ�С

	
//	HAL_UART_Receive_IT(&USART3_Handler, (u8*)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
//    __HAL_UART_ENABLE_IT(&USART3_Handler, UART_IT_IDLE);
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��
/////////////////////////////////////////////////////////////////////

//��ʼ������4������SBUS����
void uart4_init(u32 bound)
{	
	//UART ��ʼ������
	UART4_Handler.Instance=UART4;					    //UART4
	UART4_Handler.Init.BaudRate=bound;				    //������
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_9B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART4_Handler.Init.StopBits=UART_STOPBITS_2;	    //��ֹͣλ
	UART4_Handler.Init.Parity=UART_PARITY_EVEN;		    //żУ��λ
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //����ģʽ
	
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()��ʹ��UART4
	__HAL_UART_CLEAR_IDLEFLAG(&UART4_Handler);  //�����־λ
//	__HAL_UART_ENABLE_IT(&UART4_Handler, UART_IT_IDLE);          //�������н����ж�
//	HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
	HAL_UART_Receive_DMA(&UART4_Handler,SBUS_Buf,SBUS_Size);//��DMA���գ�ָ�����ջ������ͽ��մ�С
			
}




void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
	
	}

	
	else if(huart->Instance==USART3)//����Ǵ���3�����д���3 MSP��ʼ����������������������������������������������else��Ӱ��
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIObʱ��
		__HAL_RCC_USART3_CLK_ENABLE();			//ʹ��USART3ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//����ΪUSART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB11

		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB10
		

		HAL_NVIC_EnableIRQ(USART3_IRQn);				//ʹ��USART3�ж�ͨ��
		HAL_NVIC_SetPriority(USART3_IRQn,2,3);			//��ռ���ȼ�2�������ȼ�3
	}
	
	else if(huart->Instance==UART4)//����Ǵ���4�����д���4 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_UART4_CLK_ENABLE();			//ʹ��UART4ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_11;			//PA11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		GPIO_Initure.Alternate=GPIO_AF6_UART4;	//����ΪUART4
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA11

		GPIO_Initure.Pin=GPIO_PIN_12;			//PA12
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA12
		
		HAL_NVIC_EnableIRQ(UART4_IRQn);				//ʹ��UART4�ж�ͨ��
		HAL_NVIC_SetPriority(UART4_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
		

	}
}
 
//����1�жϷ������
void USART1_IRQHandler(void)                	
{ 
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;

	
	HAL_UART_IRQHandler(&UART1_Handler);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler)!=HAL_UART_STATE_READY)//�ȴ�����
	{
        timeout++;////��ʱ����
        if(timeout>maxDelay) break;		
	}
     
	

} 


//����3�жϷ������

void USART3_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART3_Handler);
}

//����4�жϷ������
void UART4_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&UART4_Handler);	//����HAL���жϴ����ú���
} 





 




