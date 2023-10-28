#ifndef  _I2C_H_
#define  _I2C_H_


 




void Stop_IIC(void);
void Start_IIC(void);
void IIC_Send_NAck(void);
void IIC_Send_Ack(void);
u8 IIC_Wait_Ack_OK(void);
void IIC_Delay(u16 nus);

void IIC_Init(void);
void IIC_SenddByte(u8 data);
u8 IIC_GetByte(void);


#endif
