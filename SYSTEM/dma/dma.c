#include "dma.h"
#include "usart.h"

DMA_HandleTypeDef  USART3TxDMA_Handler;      //DMA���
DMA_HandleTypeDef  USART3RxDMA_Handler;
extern UART_HandleTypeDef USART3_Handler;

DMA_HandleTypeDef  UART4TxDMA_Handler;      //DMA���
DMA_HandleTypeDef  UART4RxDMA_Handler;
extern UART_HandleTypeDef UART4_Handler;

//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
void MYDMA_Config(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();//DMA2ʱ��ʹ��	
	
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	
	//Tx DMA����
	USART3TxDMA_Handler.Instance=DMA1_Stream3;                            //������ѡ��
	USART3TxDMA_Handler.Init.Request=DMA_REQUEST_USART3_TX;				//USART1����DMA
	USART3TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
	USART3TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
	USART3TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
	USART3TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
	USART3TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
	USART3TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
	USART3TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
	USART3TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	USART3TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	USART3TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
	USART3TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
	HAL_DMA_DeInit(&USART3TxDMA_Handler);   
	HAL_DMA_Init(&USART3TxDMA_Handler);
	__HAL_LINKDMA(&USART3_Handler,hdmatx,USART3TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
	
	//Rx DMA����
	USART3RxDMA_Handler.Instance=DMA1_Stream1;                            //������ѡ��
	USART3RxDMA_Handler.Init.Request=DMA_REQUEST_USART3_RX;
	USART3RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //���赽�洢��
	USART3RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
	USART3RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
	USART3RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
	USART3RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
	USART3RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
	USART3RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
	USART3RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	USART3RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	USART3RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
	USART3RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
	HAL_DMA_DeInit(&USART3RxDMA_Handler);   
	HAL_DMA_Init(&USART3RxDMA_Handler);
	__HAL_LINKDMA(&USART3_Handler,hdmarx,USART3RxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
	
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn,0,0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	
	//Tx DMA����
	UART4TxDMA_Handler.Instance=DMA1_Stream4;                            //������ѡ��
	UART4TxDMA_Handler.Init.Request=DMA_REQUEST_UART4_TX;				//USART1����DMA
	UART4TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
	UART4TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
	UART4TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
	UART4TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
	UART4TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
	UART4TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
	UART4TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
	UART4TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	UART4TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	UART4TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
	UART4TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
	HAL_DMA_DeInit(&UART4TxDMA_Handler);   
	HAL_DMA_Init(&UART4TxDMA_Handler);
	__HAL_LINKDMA(&UART4_Handler,hdmatx,UART4TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
	
	//Rx DMA����
	UART4RxDMA_Handler.Instance=DMA1_Stream2;                            //������ѡ��
	UART4RxDMA_Handler.Init.Request=DMA_REQUEST_UART4_RX;
	UART4RxDMA_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;             //���赽�洢��
	UART4RxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
	UART4RxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
	UART4RxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
	UART4RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
	UART4RxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
	UART4RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
	UART4RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	UART4RxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
	UART4RxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
	UART4RxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
	HAL_DMA_DeInit(&UART4RxDMA_Handler);   
	HAL_DMA_Init(&UART4RxDMA_Handler);
	__HAL_LINKDMA(&UART4_Handler,hdmarx,UART4RxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
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
 

