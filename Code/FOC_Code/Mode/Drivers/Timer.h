#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////
// ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
// ALIENTEKս��STM32������
// ��ʱ�� ��������
// ����ԭ��@ALIENTEK
// ������̳:www.openedv.com
// �޸�����:2012/9/3
// �汾��V1.0
// ��Ȩ���У�����ؾ���
// Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    Flag_10ms,
    Flag_20ms,
    Flag_50ms,
    Flag_100ms,
    Flag_200ms,
    Flag_500ms,
    Flag_Max,
} Flag_e;

void TIM3_Int_Init(u16 arr, u16 psc);
void Time_SetFlag( u8 Flag);
void Time_ResetFlag( u8 Flag);
u8 Time_GetFlag( u8 Flag);
void Time_SetAllFlag(u8 Type);
extern u32 Count;
extern u32 AuthenticationTimer;
#endif
