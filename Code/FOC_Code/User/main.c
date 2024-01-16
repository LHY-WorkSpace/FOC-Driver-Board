#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"
#include "u8g2_UserGUI.h"
#include "ADC.h"
#include "WS2812.h"



int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  LED_Init();
  Key_Init();
  EEPROM_Init();
  TIM3_Init(10,72);
  Delay_Init();
  AS5600_Init();
  PWM_Init();
  USART1_Init(460800);
  u8g2_Init();
  WS2812_Init();
  // AD_Init();

  while (1)
  {
    // Foc_CTL();
    // u8g2_Task();
      Delay_ms(10);
      WS2812_SetColor(0x80,0x10,0x01,0);
      RGB_SendToLED();

  }


}

