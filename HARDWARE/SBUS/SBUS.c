#include "SBUS.h"




u8 SBUS_Temp[25];          //SBUSԭʼ��һ֡�����е�25���ֽ�
u16 Remoter_Channel[16];      //���յ�ң����16ͨ������
u16 Cali_Channel[16];      //�궨���16ͨ�����ݳ�ֵ


//////////////////////SBUSЭ�����/////////////////////////////////////////
void ProcessSBUS(void)
{
	u8 i;
	Remoter_Channel[15]=(SBUS_Temp[22]<<3|SBUS_Temp[21]>>5)&0x07ff;
	Remoter_Channel[14]=(SBUS_Temp[21]<<6|SBUS_Temp[20]>>2)&0x07ff;
	Remoter_Channel[13]=(SBUS_Temp[20]<<9|SBUS_Temp[19]<<1|SBUS_Temp[18]>>7)&0x07ff;
	Remoter_Channel[12]=(SBUS_Temp[18]<<4|SBUS_Temp[17]>>4)&0x07ff;
	Remoter_Channel[11]=(SBUS_Temp[17]<<7|SBUS_Temp[16]>>1)&0x07ff;
	Remoter_Channel[10]=(SBUS_Temp[16]<<10|SBUS_Temp[15]<<2|SBUS_Temp[14]>>6)&0x07ff;
	Remoter_Channel[9]=(SBUS_Temp[14]<<5|SBUS_Temp[13]>>3)&0x07ff;
	Remoter_Channel[8]=(SBUS_Temp[13]<<8|SBUS_Temp[12])&0x07ff;
	Remoter_Channel[7]=(SBUS_Temp[11]<<3|SBUS_Temp[10]>>5)&0x07ff;
	Remoter_Channel[6]=(SBUS_Temp[10]<<6|SBUS_Temp[9]>>2)&0x07ff;
	Remoter_Channel[5]=(SBUS_Temp[9]<<9|SBUS_Temp[8]<<1|SBUS_Temp[7]>>7)&0x07ff;
	Remoter_Channel[4]=(SBUS_Temp[7]<<4|SBUS_Temp[6]>>4)&0x07ff;
	Remoter_Channel[3]=(SBUS_Temp[6]<<7|SBUS_Temp[5]>>1)&0x07ff;
	Remoter_Channel[2]=(SBUS_Temp[5]<<10|SBUS_Temp[4]<<2|SBUS_Temp[3]>>6)&0x07ff;
	Remoter_Channel[1]=(SBUS_Temp[3]<<5|SBUS_Temp[2]>>3)&0x07ff;
	Remoter_Channel[0]=(SBUS_Temp[2]<<8|SBUS_Temp[1])&0x07ff;
//	for (i=0;i<16;i++)
//	{
//		printf("%5d ",Remoter_Channel[i]);
//	}
//	printf("\r\n");
}
/////////////////////////////////////////////////////////////////////////



/////////////////////////////////DMA����SBUS���ݲ�����/////////////////////
void Sbus_Recieve(void)
{
	u8 rx_len;
	__HAL_UART_CLEAR_IDLEFLAG(&UART4_Handler);                       //�����־λ
	SCB_DisableDCache();                                             //ע�⣺ H7ϵ���ڶ�ȡDMAǰ��Ҫ�ȹر�DCache����ȡ���ٿ���DCache��
	HAL_UART_DMAStop(&UART4_Handler); 					  
	rx_len =  SBUS_Size - __HAL_DMA_GET_COUNTER(&UART4RxDMA_Handler); //�õ��ѽ������ݵĸ���
	      
	if(SBUS_Buf[0]==0x0F)
	{
		memcpy(SBUS_Temp, SBUS_Buf, rx_len);                          //�����¿�������DMAͨ��֮ǰ����Rx_Buf��������������ݸ��Ƶ�����һ�������У�Ȼ���ٿ���DMA��Ȼ�����ϴ����Ƴ���������
		ProcessSBUS();
	}

	else if(SBUS_Buf[0]==0x8F)										  //֡ͷ��0x0F��0x8F���������ԭ��δ֪��
	{
		memcpy(SBUS_Temp, SBUS_Buf, rx_len); 
		ProcessSBUS();
	}
	SCB_EnableDCache();	 							
	HAL_UART_Receive_DMA(&UART4_Handler,(uint8_t*)SBUS_Buf,SBUS_Size);
		
		

}
/////////////////////////////////////////////////////////////////////////
void Remoter_Cali(void)
{

	u8 cnt;
	while(1)
	{
		if((__HAL_UART_GET_FLAG(&UART4_Handler,UART_FLAG_IDLE) != RESET))      //ǰʮ�����ݲ��ɼ��������д���
		{
			Sbus_Recieve();     
			cnt++;
			if(cnt>10)
			{
				Cali_Channel[0] += Remoter_Channel[0];
				Cali_Channel[1] += Remoter_Channel[1];
				Cali_Channel[2] += Remoter_Channel[2];
				Cali_Channel[3] += Remoter_Channel[3];
//				printf("%d,%d,%d,%d,%d;\r\n",Cali_Channel[0],Cali_Channel[1],Cali_Channel[2],Cali_Channel[3],cnt);	
			}
			
		}
		if(cnt>=30)																//20��ȡƽ��ֵ��Ϊ��ֵ
		{
			Cali_Channel[0] = Cali_Channel[0]/20;
			Cali_Channel[1] = Cali_Channel[1]/20;
			Cali_Channel[2] = Cali_Channel[2]/20;
			Cali_Channel[3] = Cali_Channel[3]/20;
			break;
		}
		
	
	}



}
