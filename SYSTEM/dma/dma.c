#include "dma.h"
#include "usart.h"

DMA_HandleTypeDef  USART3TxDMA_Handler;      //DMA句柄
DMA_HandleTypeDef  USART3RxDMA_Handler;
extern UART_HandleTypeDef USART3_Handler;

DMA_HandleTypeDef  UART4TxDMA_Handler;      //DMA句柄
DMA_HandleTypeDef  UART4RxDMA_Handler;
extern UART_HandleTypeDef UART4_Handler;

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
void MYDMA_Config(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();//DMA2时钟使能	
	
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
	//Tx DMA配置
	USART3TxDMA_Handler.Instance=DMA1_Stream3;                            //数据流选择
	USART3TxDMA_Handler.Init.Request=DMA_REQUEST_USART3_TX;				//USART1发送DMA
	USART3TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
	USART3TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
	USART3TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
	USART3TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
	USART3TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
	USART3TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
	USART3TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	USART3TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	USART3TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	USART3TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
	USART3TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	HAL_DMA_DeInit(&USART3TxDMA_Handler);   
	HAL_DMA_Init(&USART3TxDMA_Handler);
	__HAL_LINKDMA(&USART3_Handler,hdmatx,USART3TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
	
	//Rx DMA配置
	USART3RxDMA_Handler.Instance=DMA1_Stream1;                            //数据流选择
	USART3RxDMA_Handler.Init.Request=DMA_REQUEST_USART3_RX;
	USART3RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //外设到存储器
	USART3RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
	USART3RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
	USART3RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
	USART3RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
	USART3RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
	USART3RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	USART3RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	USART3RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	USART3RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
	USART3RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	HAL_DMA_DeInit(&USART3RxDMA_Handler);   
	HAL_DMA_Init(&USART3RxDMA_Handler);
	__HAL_LINKDMA(&USART3_Handler,hdmarx,USART3RxDMA_Handler);    //将DMA与USART1联系起来(接收DMA)
	
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	//Tx DMA配置
	UART4TxDMA_Handler.Instance=DMA1_Stream4;                            //数据流选择
	UART4TxDMA_Handler.Init.Request=DMA_REQUEST_UART4_TX;				//USART1发送DMA
	UART4TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
	UART4TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
	UART4TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
	UART4TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
	UART4TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
	UART4TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
	UART4TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	UART4TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	UART4TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	UART4TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
	UART4TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	HAL_DMA_DeInit(&UART4TxDMA_Handler);   
	HAL_DMA_Init(&UART4TxDMA_Handler);
	__HAL_LINKDMA(&UART4_Handler,hdmatx,UART4TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
	
	//Rx DMA配置
	UART4RxDMA_Handler.Instance=DMA1_Stream2;                            //数据流选择
	UART4RxDMA_Handler.Init.Request=DMA_REQUEST_UART4_RX;
	UART4RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //外设到存储器
	UART4RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
	UART4RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
	UART4RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
	UART4RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
	UART4RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
	UART4RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	UART4RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	UART4RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	UART4RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
	UART4RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	HAL_DMA_DeInit(&UART4RxDMA_Handler);   
	HAL_DMA_Init(&UART4RxDMA_Handler);
	__HAL_LINKDMA(&UART4_Handler,hdmarx,UART4RxDMA_Handler);    //将DMA与USART1联系起来(接收DMA)
} 

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&USART3TxDMA_Handler);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&USART3RxDMA_Handler);
}
 

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&UART4RxDMA_Handler);
}

void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&UART4TxDMA_Handler);
}
 

