#include"IncludeFile.h"




/*
IIC_SCL     GPIO_Pin_4                                       
IIC_SDA     GPIO_Pin_5
*/

void IIC_Init()
{

	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_AHB1PeriphClockCmd(PORT_AHB,ENABLE);
	
	IIC_SCL_HIGH;
	IIC_SDA_HIGH;//÷√Œ™◊‹œﬂø’œ–


	GPIO_Initstructure.GPIO_Pin=IIC_SCL|IIC_SDA;	
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_OUT;          
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_OType=GPIO_OType_OD;                
	GPIO_Init(PORT_GROUP,&GPIO_Initstructure);

}

#if (DELAY_TYPE == TIMER) 
void IIC_Delay(u16 nus)
{
	Delay_us(nus);
}

#else
void IIC_Delay(u16 nus)
{
	u16 i,k;

	for(k=0; k<nus; k++)
	{
		for(i=0; i<10; i++)
		{
			__NOP();
		}
	}
}
#endif


void Start_IIC(void)
{
	IIC_SDA_HIGH;
	IIC_Delay(2);
	IIC_SCL_HIGH;
	IIC_Delay(2);
	IIC_SDA_LOW;
	IIC_Delay(2);
	IIC_SCL_LOW;
	IIC_Delay(2);

}


void Stop_IIC(void)
{
	IIC_SCL_LOW;
	IIC_Delay(2);
	IIC_SDA_LOW;
	IIC_Delay(2);
	IIC_SCL_HIGH;
	IIC_Delay(2);
	IIC_SDA_HIGH;
	IIC_Delay(2);

}



void IIC_Send_Ack(void)
{
	IIC_SCL_LOW;
	IIC_SDA_LOW;
	IIC_Delay(2);
	IIC_SCL_HIGH;
	IIC_Delay(2);
	IIC_SCL_LOW;
	IIC_Delay(2);
	IIC_SDA_HIGH;
	IIC_Delay(2);
}



void IIC_Send_NAck(void)
{
	IIC_SCL_LOW;
	IIC_SDA_HIGH;
	IIC_Delay(2);
	IIC_SCL_HIGH;
	IIC_Delay(2);
	IIC_SCL_LOW;
	IIC_Delay(2);
	IIC_SDA_HIGH;
	IIC_Delay(2);
}


u8 IIC_Wait_Ack_OK(void)
{
	u8 i=0;

	IIC_SCL_HIGH; 
	IIC_Delay(2);                              
	while( IIC_SDA_STATE ==1)
	{
			i++;
			if(i>10)
			{
				Stop_IIC();
				return FALSE;
			}
			IIC_Delay(2);	
	}			
	IIC_SCL_LOW;	
	IIC_Delay(2);
	IIC_SDA_HIGH;
	return TRUE;
}

void IIC_SenddByte(u8 data)
{

	u8 i=0;
	
	for(i=0;i<8;i++)
	{
		//IIC_Delay(2);
		if(data&0x80)	
		{
			IIC_SDA_HIGH;
		}
		else
		{
			IIC_SDA_LOW;
		}
		data<<=1;
		IIC_Delay(2);
		IIC_SCL_HIGH;
		IIC_Delay(2);
		IIC_SCL_LOW;
	}
	IIC_Delay(2);  
	IIC_SDA_HIGH;               
}


u8 IIC_GetByte(void)
{
	u8 data=0;
	u8 i=0;

		
	// IIC_SDA_HIGH;
	// IIC_Delay(2);
	for(i=0;i<8;i++)
	{		
			data<<=1;
			IIC_SCL_LOW;
			IIC_Delay(2); 		
			IIC_SCL_HIGH;	
			IIC_Delay(2);
			if( IIC_SDA_STATE ==1)	
			{
      			data|=0x01;
			}

	}
 	IIC_SCL_LOW;	
	IIC_Delay(2);
	IIC_SDA_HIGH;
  return data;
}



































