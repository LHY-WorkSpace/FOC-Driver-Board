#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"
#include "u8g2_UserGUI.h"
#include "ADC.h"
#include "WS2812.h"


// 模式1
// u8 RRR = 240;
// u8 GGG = 50;
// u8 BBB = 0;

// 模式2
// u8 RRR = 10;
// u8 GGG = 10;
// u8 BBB = 240;

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  Delay_Init();
  LED_Init();
  Delay_ms(500);
  Delay_ms(500);
  // Key_Init();
  // EEPROM_Init();
  // AsciiCode_Init();

  // 这3个顺序不能动
  WS2812_Init();
  MorseCode_Init();
  TIM3_Init(10,72);


  // AS5600_Init();
  // PWM_Init();
  // USART1_Init(460800);
  // u8g2_Init();

  // AD_Init();

  while (1)
  {
    // Foc_CTL();
    // u8g2_Task();

      // WS2812_SetColor(10,10,240,1);
      // RGB_SendToLED();
      Delay_ms(500);
      // WS2812_SetColor(240,50,0,1);
      // RGB_SendToLED();
      	// MorseCodeSend("Jack Trust Me");

      // WS2812_SetColor(0x80,0x10,0x01,1);
      // WS2812_SetColor(0x80,0x10,0x01,2);
      // WS2812_SetColor(0x80,0x10,0x01,3);
      // AsciiCodeSend("123456");


  }


}

