#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"






int main(void)
{
  LED_Init();
  TIM3_Init(10,72);
  Delay_Init();
  AS5600_Init();
  PWM_Init();
  USART1_Init(460800);

  while (1)
  {
    // if(Time_GetFlag(Flag_50ms) == SET)
    // {
    //   Foc_CTL();
    //   // FOC_TickTask();

    //   Time_ResetFlag(Flag_50ms);
    Foc_CTL();
    }


}

