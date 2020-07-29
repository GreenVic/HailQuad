
#include "hmc5983.h"


#define Dummy_Byte   0x80    //�������

//��ʼ��hmc5983
void hmc5983_init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	SPI2_Init();

	__HAL_RCC_GPIOB_CLK_ENABLE();

  

	GPIO_Initure.Pin=GPIO_PIN_12;     //PB12��ΪCS
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //�������
	GPIO_Initure.Pull=GPIO_PULLUP;              //����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
   

	hmc5983_WriteByte(HMC5983_CONF_REG_A,0x18);    //����������1��ƽ�� 75HZ
	hmc5983_WriteByte(HMC5983_CONF_REG_B,0x20);    //��1.3Ga
	hmc5983_WriteByte(HMC5983_MODE_REG,  0x01);    //4��SPI ���β���   ��������Զ�����


}

//����һ�β���
void hmc5983_StartMea(void)
{
  hmc5983_WriteByte(HMC5983_MODE_REG,  0x01);
}

void hmc5983_WriteByte(u8 Address,u8 byte)
{

    HMC5983_CS(0);
    SPI2_ReadWriteByte((Address&0X7F));   //д
    SPI2_ReadWriteByte(byte);
    HMC5983_CS(1);
}


u8 hmc5983_ReadByte(u8 Address)
{
   u8 temp=0;
  // HMC5983_CS(0);
   SPI2_ReadWriteByte(Address|0x80);   //��
   temp=SPI2_ReadWriteByte(0XFF);
  // HMC5983_CS(1);
   return temp;
}

void hmc5983_ReadBuf(u8 Address,u8 *Buf,u8 len)
{
    u8 i=0;
    for(i=0;i<len;i++)
    {
      HMC5983_CS(0);
      SPI2_ReadWriteByte(((Address+i)|0x80));
      Buf[i]=SPI2_ReadWriteByte(0XFF);
      HMC5983_CS(1);
    }
}

void Get_HMC5983(s16 *mx,s16 *my,s16 *mz)
{
	u8 buf[6];  
	
	hmc5983_ReadBuf(0x03,buf,6);
	*mx=((s16)buf[0]<<8)|buf[1];  
	*mz=((s16)buf[2]<<8)|buf[3];         //����˳������Ϊ�Ĵ���˳��Ϊx z y
	*my=((s16)buf[4]<<8)|buf[5];	
	hmc5983_StartMea();	            //��������Ϊ���β���ģʽ
  
}

