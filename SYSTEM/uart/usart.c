#include "usart.h"
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	

//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数                       选择打印串口
int fputc(int ch, FILE *f)
{ 	
	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART1->TDR=(u8)ch;      
	return ch;
}




u8 SBUS_Buf[SBUS_Size];       //SBUS用的接收缓存
u8 GPS_Buf[GPS_Size];      //GPS用的接收缓存
u8 ProcessBuffer[GPS_Size];




UART_HandleTypeDef UART1_Handler; //UART句柄
UART_HandleTypeDef USART3_Handler; //UART句柄
UART_HandleTypeDef UART4_Handler; //UART4句柄


extern DMA_HandleTypeDef  USART3TxDMA_Handler;      //DMA发送句柄
extern DMA_HandleTypeDef  USART3RxDMA_Handler;      //DMA接受句柄








//初始化IO 串口1 
//bound:波特率
/////////////////////////打印用串口1////////////////////////////
void uart_init(u32 bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
  
}
///////////////////////////////////////////////////////////////////



//////////////////////////DMA接收GPS用串口3///////////////////////
void USART3_init(u32 bound)
{	
	//UART 初始化设置
	USART3_Handler.Instance=USART3;					    //USART3
	USART3_Handler.Init.BaudRate=bound;				    //波特率
	USART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	USART3_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	USART3_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	USART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	USART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&USART3_Handler);					    //HAL_UART_Init()会使能USART3
	
	
//	__HAL_UART_ENABLE_IT(&USART3_Handler, UART_IT_IDLE);          //开启空闲接收中断
	__HAL_UART_CLEAR_IDLEFLAG(&USART3_Handler);  //清除标志位 
	HAL_UART_Receive_DMA(&USART3_Handler,GPS_Buf,GPS_Size);//打开DMA接收，指定接收缓存区和接收大小		HAL_UART_Receive_DMA(&USART3_Handler,GPS_Buf,GPS_Size);	//打开DMA接收，指定接收缓存区和接收大小

	
//	HAL_UART_Receive_IT(&USART3_Handler, (u8*)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
//    __HAL_UART_ENABLE_IT(&USART3_Handler, UART_IT_IDLE);
}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄
/////////////////////////////////////////////////////////////////////

//初始化串口4，接受SBUS数据
void uart4_init(u32 bound)
{	
	//UART 初始化设置
	UART4_Handler.Instance=UART4;					    //UART4
	UART4_Handler.Init.BaudRate=bound;				    //波特率
	UART4_Handler.Init.WordLength=UART_WORDLENGTH_9B;   //字长为8位数据格式
	UART4_Handler.Init.StopBits=UART_STOPBITS_2;	    //个停止位
	UART4_Handler.Init.Parity=UART_PARITY_EVEN;		    //偶校验位
	UART4_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART4_Handler.Init.Mode=UART_MODE_TX_RX;		    //接受模式
	
	HAL_UART_Init(&UART4_Handler);					    //HAL_UART_Init()会使能UART4
	__HAL_UART_CLEAR_IDLEFLAG(&UART4_Handler);  //清除标志位
//	__HAL_UART_ENABLE_IT(&UART4_Handler, UART_IT_IDLE);          //开启空闲接收中断
//	HAL_UART_Receive_IT(&UART4_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
	HAL_UART_Receive_DMA(&UART4_Handler,SBUS_Buf,SBUS_Size);//打开DMA接收，指定接收缓存区和接收大小
			
}




void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
	
	}

	
	else if(huart->Instance==USART3)//如果是串口3，进行串口3 MSP初始化——————————————————————else无影响
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOb时钟
		__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
	
		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART3;	//复用为USART3
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11

		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10
		

		HAL_NVIC_EnableIRQ(USART3_IRQn);				//使能USART3中断通道
		HAL_NVIC_SetPriority(USART3_IRQn,2,3);			//抢占优先级2，子优先级3
	}
	
	else if(huart->Instance==UART4)//如果是串口4，进行串口4 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_UART4_CLK_ENABLE();			//使能UART4时钟
	
		GPIO_Initure.Pin=GPIO_PIN_11;			//PA11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		GPIO_Initure.Alternate=GPIO_AF6_UART4;	//复用为UART4
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA11

		GPIO_Initure.Pin=GPIO_PIN_12;			//PA12
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA12
		
		HAL_NVIC_EnableIRQ(UART4_IRQn);				//使能UART4中断通道
		HAL_NVIC_SetPriority(UART4_IRQn,3,3);			//抢占优先级3，子优先级3

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
		

	}
}
 
//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;

	
	HAL_UART_IRQHandler(&UART1_Handler);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler)!=HAL_UART_STATE_READY)//等待就绪
	{
        timeout++;////超时处理
        if(timeout>maxDelay) break;		
	}
     
	

} 


//串口3中断服务程序

void USART3_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART3_Handler);
}

//串口4中断服务程序
void UART4_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&UART4_Handler);	//调用HAL库中断处理公用函数
} 





 




