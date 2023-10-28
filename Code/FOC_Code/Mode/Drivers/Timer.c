
#include "Timer.h"
#include "DataType.h"
#include "stm32f10x.h"

// Timer1-4



#define MAX_VAL (1000) // 10 S

static u32 Count;
static u8 Time_Flag;

void TIM3_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// ��ʱ��TIM3��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr;	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	// �ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM3, ENABLE);
	Time_Flag  = 0;
}

void Time_SetFlag(u8 Flag)
{
	Time_Flag |= (1 << Flag);
}

void Time_ResetFlag(u8 Flag)
{
	Time_Flag &= ~(1 << Flag);
}

u8 Time_GetFlag(u8 Flag)
{
	if( (Time_Flag & (1 << Flag)) )
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

// ��ʱ��3�жϷ������  10ms
void TIM3_IRQHandler(void) // TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		if (Count >= MAX_VAL)
		{
			Count = 0;
		}
		Time_SetFlag(Flag_10ms);

		if (Count % 2 == 0)
		{
			Time_SetFlag(Flag_20ms);
		}

		if (Count % 5 == 0)
		{
			Time_SetFlag(Flag_50ms);
		}

		if (Count % 10 == 0)
		{
			Time_SetFlag(Flag_100ms);
		}

		if (Count % 20 == 0)
		{
			Time_SetFlag(Flag_200ms);
		}

		if (Count % 50 == 0)
		{
			Time_SetFlag(Flag_500ms);
		}
	}
}
