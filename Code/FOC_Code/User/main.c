#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"









  float Angle  = 1.0f;
int main(void)
{

  // u8 flag;

  LED_Init();
  TIM3_Init(10,72);
  AS5600_Init();
  PWM_Init();
  USART1_Init(115200);

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
