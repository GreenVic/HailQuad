#ifndef __HMC5983_H_
#define	__HMC5983_H_

#include "sys.h"
#include "spi.h"

#define HMC5983_CS(n)   (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET))//24L01Æ¬Ñ¡ÐÅºÅ


#define HMC5983_CONF_REG_A              0x00
#define HMC5983_CONF_REG_B              0x01
#define HMC5983_MODE_REG                0x02
#define HMC5983_X_axis_MSB_REG          0x03
#define HMC5983_Status_REG              0x09
#define HMC5983_IDA_REG                 0x0A
#define HMC5983_Temperature_MSB_REG     0x31


void hmc5983_init(void);
void hmc5983_StartMea(void);
void hmc5983_WriteByte(u8 Address,u8 byte);
u8   hmc5983_ReadByte(u8 Address);
void hmc5983_ReadBuf(u8 Address,u8 *Buf,u8 len);
void Get_HMC5983(s16 *mx,s16 *my,s16 *mz);

#endif

