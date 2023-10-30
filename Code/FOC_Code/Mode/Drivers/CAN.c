#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "CAN.h"


#define


// USART_Data_t USART1_Data;
// USART_Data_t USART2_Data;

/*
PA11 - CAN_RX == IC RX
PA12 - CAN_TX == IC TX
*/
void CAN_Init()
{
	GPIO_InitTypeDef USART_GPIO_Init;
	CAN_InitTypeDef CAN_InitType;
	NVIC_InitTypeDef  NVIC_Initstr;
	CAN_FilterInitTypeDef CAN_FilterInit;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	USART_GPIO_Init.GPIO_Pin = GPIO_Pin_11;
	USART_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	USART_GPIO_Init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&USART_GPIO_Init);

	USART_GPIO_Init.GPIO_Pin = GPIO_Pin_12;
	USART_GPIO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	USART_GPIO_Init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&USART_GPIO_Init);

	CAN_InitType.CAN_Prescaler = CAN_CLK_DIV;
	CAN_InitType.CAN_Mode = CAN_Mode_Normal;
	CAN_InitType.CAN_SJW = CAN_SJW_1tq;
	CAN_InitType.CAN_BS1 = CAN_BS1_8tq;
	CAN_InitType.CAN_BS2 = CAN_BS2_8tq;
	CAN_InitType.CAN_TTCM = DISABLE;
	CAN_InitType.CAN_ABOM = DISABLE;
	CAN_InitType.CAN_AWUM = DISABLE;
	CAN_InitType.CAN_NART = DISABLE;
	CAN_InitType.CAN_RFLM = DISABLE;
	CAN_InitType.CAN_TXFP = DISABLE;
	CAN_Init(CAN1,&CAN_InitType);

	CAN_FilterInit.CAN_FilterIdHigh = ;
	CAN_FilterInit.CAN_FilterIdLow = ;
	CAN_FilterInit.CAN_FilterMaskIdHigh = ;
	CAN_FilterInit.CAN_FilterMaskIdLow = ;	
	CAN_FilterInit.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;
	CAN_FilterInit.CAN_FilterNumber = ;
	CAN_FilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInit.CAN_FilterScale = CAN_FilterScale_16bit;	
	CAN_FilterInit.CAN_FilterActivation = DISABLE;
	CAN_FilterInit(&CAN_FilterInit);

	NVIC_Initstr.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_Initstr.NVIC_IRQChannelSubPriority = 0;
	NVIC_Initstr.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstr);

	NVIC_Initstr.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_Initstr.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_Initstr.NVIC_IRQChannelSubPriority = 0;
	NVIC_Initstr.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstr);

	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
	CAN_ClearFlag(CAN1,0x3ff);

	CAN_WakeUp(CAN1);
}


void CanTX()
{
	CanTxMsg TxData;
	u8 TXState;

	TxData.StdId = ;
	TxData.ExtId = ;
	TxData.IDE = ;
	TxData.RTR = ;
	TxData.DLC = ;
	TxData.Data[7] = ;	
	TXState = CAN_Transmit(CAN1,&);

	while(CAN_TransmitStatus(CAN1,TXState) == CAN_TxStatus_Failed);
}

u8 CanRX()
{
	CanRxMsg RxData;
	u8 TXState;

	memset((u8 *)&RxData,0,sizeof(CanRxMsg));
	if(CAN_MessagePending(CAN1,CAN_FIFO0) != 0)
	{
		CAN_Receive(CAN1,CAN_FIFO0,&RxData);
	}
	else
	{
		RxData.DLC = 0;
	}

	return RxData.DLC;
}

// int fputc(int ch, FILE* stream)          
// {		
// 	u8 i=0;
// 	USART_ClearFlag(USART1,USART_FLAG_TC);
// 	USART_SendData(USART1, (unsigned char) ch);	
// 	while ((USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET) && ( i < 10))
// 	{
// 		// Delay_us(2);
// 	}
// 	USART_ClearFlag(USART1,USART_FLAG_TC);
//     return ch;
// }

