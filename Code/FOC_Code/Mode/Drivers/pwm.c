#include "pwm.h"
#include "DataType.h"
#include "stm32f10x.h"
#include <stdio.h>
// B13
// B14
// B15


void PWM_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);



	GPIO_InitTypeDef PWM_IO;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitsruc;
    TIM_OCInitTypeDef TIM_OCInit;
	
	PWM_IO.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;	
	PWM_IO.GPIO_Speed = GPIO_Speed_50MHz;
	PWM_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&PWM_IO);


	TIM_TimeBaseInitsruc.TIM_Prescaler=72-1;
	TIM_TimeBaseInitsruc.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitsruc.TIM_Period=20000-1;          //周期20ms
    TIM_TimeBaseInitsruc.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitsruc.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitsruc);
	
    TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInit.TIM_Pulse = 0;
    TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInit.TIM_OCNPolarity = TIM_OCPolarity_Low;
    TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInit.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1,&TIM_OCInit);
    TIM_OC2Init(TIM1,&TIM_OCInit);
    TIM_OC3Init(TIM1,&TIM_OCInit);

    TIM_OC1FastConfig(TIM1,TIM_OCFast_Enable);
    TIM_OC2FastConfig(TIM1,TIM_OCFast_Enable);
    TIM_OC3FastConfig(TIM1,TIM_OCFast_Enable);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1,ENABLE);
    TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);

    TIM_SetCompare1(TIM1, 5000);
    TIM_SetCompare2(TIM1, 10000);
    TIM_SetCompare3(TIM1, 15000);

}



// //设置脉宽(%)
// void PWM_SetPluseWdie(u8 Phase ,u8 PluseWide)
// {
//     switch (Phase)
//     {
//         case /* constant-expression */:
//             /* code */
//             break;
        
//         default:
//             break;
//     }
// 	TIMx->CCR1 = PluseWide-1;
// }
// //设置周期(us)
// void PWM_SetPlusePeriod(TIM1,u32 PlusePeriod)
// {
// 	TIMx->ARR = PlusePeriod-1;
// }



















