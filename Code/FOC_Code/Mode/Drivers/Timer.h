#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK战舰STM32开发板
// 定时器 驱动代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2012/9/3
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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
