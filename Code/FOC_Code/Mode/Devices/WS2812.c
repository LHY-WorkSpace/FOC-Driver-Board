#include "DataType.h"
#include "WS2812.h"
#include "stm32f10x.h"



static u8 DispBuff[24*WS2812_NUM];

const u8 Code0 = 0x0C;
const u8 Code1 = 0xC0;






// PB15   SPI2_MOSI
void DMA_FunInit()
{
	
	u8 i;
	DMA_InitTypeDef DMA_InitType;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	DMA_ClearITPendingBit(DMA1_IT_GL5);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	DMA_InitType.DMA_PeripheralBaseAddr = (u32)&(SPI2->DR); 
	DMA_InitType.DMA_MemoryBaseAddr = (u32)&DispBuff;     
	DMA_InitType.DMA_DIR = DMA_DIR_PeripheralDST;                
	DMA_InitType.DMA_BufferSize = 24*WS2812_NUM;         
	DMA_InitType.DMA_PeripheralInc = DMA_PeripheralInc_Disable ;      
	DMA_InitType.DMA_MemoryInc = DMA_MemoryInc_Enable ;          
	DMA_InitType.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitType.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte ;     
	DMA_InitType.DMA_Mode = DMA_Mode_Normal;               
	DMA_InitType.DMA_Priority = DMA_Priority_High;           
	DMA_InitType.DMA_M2M = DMA_M2M_Disable;                
	DMA_Init(DMA1_Channel5,&DMA_InitType);
	DMA_Cmd(DMA1_Channel5,DISABLE);
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);


	for ( i = 0; i < 24*WS2812_NUM; i++)
	{
		DispBuff[i] = 0x00;
	}
}

void  WS2812_Init(void)
{

	GPIO_InitTypeDef SPI_GPIO_Init;
    SPI_InitTypeDef SPI_InitDef;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_15;
	SPI_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	SPI_GPIO_Init.GPIO_Mode = GPIO_Mode_AF_PP;//WS2812为5v,此处外接上拉至5V,故开漏方式
	GPIO_Init(GPIOB,&SPI_GPIO_Init);

    SPI_InitDef.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitDef.SPI_Mode = SPI_Mode_Master;
    SPI_InitDef.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitDef.SPI_CPOL = SPI_CPOL_High;
    SPI_InitDef.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitDef.SPI_NSS = SPI_NSS_Soft;
    SPI_InitDef.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitDef.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitDef.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2,&SPI_InitDef);
	SPI_I2S_ClearITPendingBit(SPI2,SPI_I2S_IT_TXE);
    SPI_Cmd(SPI2,ENABLE);
	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
	DMA_FunInit();
}





void  WS2812_SetColor(u8 Red, u8 Green, u8 Blue,u8 Num)
{
	u8 i;
	for(i = 0; i < 8; i++)
	{	
		if( (Green&(0x80>>i)) )
		{
			DispBuff[Num*3+i] = Code1;
		}
		else
		{
			DispBuff[Num*3+i] = Code0;
		}

		if( (Red&(0x80>>i)) )
		{
			DispBuff[Num*3+i+8] = Code1;
		}
		else
		{
			DispBuff[Num*3+i+8] = Code0;
		}

		if( (Blue&(0x80>>i)) )
		{
			DispBuff[Num*3+i+16] = Code1;
		}
		else
		{
			DispBuff[Num*3+i+16] = Code0;
		}
	}
			
}



void RGB_SendToLED()
{
	DMA_Cmd(DMA1_Channel5,DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_GL5);
	DMA_SetCurrDataCounter(DMA1_Channel5,24*WS2812_NUM);
	DMA_Cmd(DMA1_Channel5,ENABLE);
}


void DMA1_Channel5_IRQHandler()
{
	u8 i;
	DMA_ClearITPendingBit(DMA1_IT_GL5);
	// 等待SPI传输完成
	i = 0;
	while( (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)&&( i++ < 100 ));
	i = 0;
    while( (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET)&&( i++ < 100 ));

}









































































