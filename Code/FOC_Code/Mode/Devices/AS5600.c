#include "DataType.h"
#include "AS5600.h"
#include "stm32f10x.h"
#include <stdio.h>



// 72Mhz = 210Khz
static void EC_IIC_Delay(u16 nus)
{
	u16 i,k;

	for(k=0; k<nus; k++)
	{
		for(i=0; i<5; i++)
		{
			__NOP();
		}
	}
}

static void EC_Start_IIC(void)
{
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
}

static void EC_IIC_Stop(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static void EC_IIC_Send_Ack(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_SDA_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static void EC_IIC_Send_NAck(void)
{
	EC_IIC_SCL_LOW;
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_HIGH;
	EC_IIC_Delay(2);
	EC_IIC_SCL_LOW;
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	EC_IIC_Delay(2);
}

static u8 EC_IIC_Wait_Ack_OK(void)
{
	u8 i=0;

	EC_IIC_SCL_HIGH; 
	EC_IIC_Delay(2);                              
	while( EC_IIC_SDA_STATE == HIGH)
	{
			i++;
			if(i>10)
			{
				EC_IIC_Stop();
				return D_FALSE;
			}
			EC_IIC_Delay(2);	
	}			
	EC_IIC_SCL_LOW;	
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
	return D_TRUE;
}

static void EC_IIC_SenddByte(u8 Data)
{
	u8 i=0;
	for(i=0;i<8;i++)
	{
		if(Data&0x80)	
		{
			EC_IIC_SDA_HIGH;
		}
		else
		{
			EC_IIC_SDA_LOW;
		}
		Data<<=1;
		EC_IIC_Delay(2);
		EC_IIC_SCL_HIGH;
		EC_IIC_Delay(2);
		EC_IIC_SCL_LOW;
	}
	EC_IIC_Delay(2);  
	EC_IIC_SDA_HIGH;               
}

static u8 EC_IIC_GetByte(void)
{
	u8 Data=0;
	u8 i=0;
	for(i=0;i<8;i++)
	{		
		Data<<=1;
		EC_IIC_SCL_LOW;
		EC_IIC_Delay(2); 		
		EC_IIC_SCL_HIGH;	
		EC_IIC_Delay(2);
		if( EC_IIC_SDA_STATE == HIGH)	
		{
			Data|=0x01;
		}
	}
 	EC_IIC_SCL_LOW;	
	EC_IIC_Delay(2);
	EC_IIC_SDA_HIGH;
  return Data;
}

//************************// 
//  ��������: AS5600_ IO ��ʼ������
//  
//  ����: ��
//  
//  ����ֵ: D_TRUE:�ɹ�  
//          D_FALSE:ʧ��
//          0xFF:��ַ����Χ
//			
//  ˵��: 
//
//************************//  
void AS5600_Init(void)
{
	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	EC_IIC_SCL_HIGH;
	EC_IIC_SDA_HIGH;//��Ϊ���߿���

	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_10;	
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
}

//************************// 
//  ��������: AS5600_ ����д�뺯��
//  
//  ����: �����ַ�����ȣ�����ָ��
//  
//  ����ֵ: D_TRUE:�ɹ�  
//          D_FALSE:ʧ��
//          0xFF:��ַ����Χ
//			
//  ˵��: 
//
//************************//  
u8 AS5600_WriteData(u8 addr,u8 length,u8 *data)
{
	u8 i;
	EC_Start_IIC();
	EC_IIC_SenddByte( AS5600_ADDRESS << 1 );
    EC_IIC_Wait_Ack_OK();
	EC_IIC_SenddByte(addr);
	EC_IIC_Wait_Ack_OK();
	for(i=0;i<length;i++)
	{
		EC_IIC_SenddByte(data[i]);
		if(EC_IIC_Wait_Ack_OK() == D_FALSE)
		{
			return D_FALSE;
		}
	}
	EC_IIC_Stop();
	return D_TRUE;
}



//************************// 
//  ��������: AS5600_ ���ݶ�ȡ����
//  
//  ����: �����ַ�����ȣ�����ָ��
//  
//  ����ֵ: D_TRUE:�ɹ�  
//          D_FALSE:ʧ��
//          0xFF:��ַ����Χ
//			
//  ˵��: 
//
//************************//  
void AS5600_ReadData(u8 addr,u8 length,u8 *data)
{
	u8 i;

	EC_Start_IIC();
	EC_IIC_SenddByte( AS5600_ADDRESS << 1 );
    EC_IIC_Wait_Ack_OK();
	EC_IIC_SenddByte(addr);
	EC_IIC_Wait_Ack_OK();

	EC_Start_IIC();
	EC_IIC_SenddByte( (AS5600_ADDRESS << 1) | 0X01 );//��
    EC_IIC_Wait_Ack_OK();

	for(i=0;i<length;i++)
	{
		data[i]=EC_IIC_GetByte();
		if( i == length-1)
		{
			EC_IIC_Send_NAck();//���һ���ֽڷ���N_ACK
		}
		else
		{
			EC_IIC_Send_Ack();
		}
	}
	EC_IIC_Stop();
}




void AS5600_Test()
{
	B16_B08 Angle;
	u16 AngleTmp=0;
	
	memset(Angle.B08,0,sizeof(B16_B08));
	AS5600_ReadData(RAW_ANGLE_L_REG,1,&Angle.B08[0]);
	// Delay_ms(5);
	AS5600_ReadData(RAW_ANGLE_H_REG,1,&Angle.B08[1]);
	// Delay_ms(5);
	// AngleTmp = Angle.B16*360/4096;
	// printf("Angle_ADC:%d    Angle:%d\r\n",Angle.B16,AngleTmp);
	// Delay_ms(10);
}










