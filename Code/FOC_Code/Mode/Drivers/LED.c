#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "LED.h"


void LED_Init()
{

	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9;	
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
}



void RGB_Init()
{









}



void RGB_SetColor(u8 Red,u8 Green,u8 Blue,u8 Num)
{




}




































