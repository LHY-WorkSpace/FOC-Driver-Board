#include "stm32f10x.h"
#include <stdio.h>
#include "DataType.h"
#include "FOC.h"
#include "Timer.h"
#include "AS5600.h"
#include "u8g2_UserGUI.h"
#include "ADC.h"
#include "WS2812.h"


// ģʽ1
// u8 RRR = 240;
// u8 GGG = 50;
// u8 BBB = 0;

// ģʽ2
// u8 RRR = 10;
// u8 GGG = 10;
// u8 BBB = 240;



u8 TimeCnt=0;
// ��ɫ
// u8 RRR = 10;
// u8 GGG = 10;
// u8 BBB = 240;
void WS2812_BlueMode()
{
	if(TimeCnt >= 0)
	{
		WS2812_SetAll(10+TimeCnt*23,10+TimeCnt*23,240);
		RGB_SendToLED();
	}
}

// ��ɫ
// u8 RRR = 240;
// u8 GGG = 50;
// u8 BBB = 0;
void WS2812_OrangeMode()
{
	if(TimeCnt >= 0)
	{
		WS2812_SetAll(240,50+TimeCnt*19,TimeCnt*24);
		RGB_SendToLED();
	}
}



void Task()
{
  static u8 Flag = 0xFF;
  KeyInfo_t KeyState;


  if(Time_GetFlag(Flag_30ms) == SET)
  {
    switch (Flag)
    {
      case 0:
        // WS2812_SetAll(10,10,240);//��ɫ
        WS2812_BlueMode();
        break;
      case 1:
        // WS2812_SetAll(240,50,0);//��ɫ
        WS2812_OrangeMode();
        break;
      case 2:
        WS2812_SetAll(240,0,0);//��ɫ
        RGB_SendToLED();
        // // MorseCode_Init();
        // MorseCodeSend("Jack Trust Me");
        Flag = 0xFF;
        break;
      default:
      break;
    }

    if(TimeCnt != 0)
    {
      TimeCnt--;
    }

    Time_ResetFlag(Flag_30ms);
  }

  if(Time_GetFlag(Flag_10ms) == SET)
  {
      KeyState = GetKeyState();
      if(KeyState.KeyState == SINGLE_CLICK)
      {
          if(Flag == 0)
          {
            Flag = 1;
          }
          else
          {
            Flag = 0;
          }
          TimeCnt = 10;
      }
      else if(KeyState.KeyState == LONG_PRESS_HOLD)
      {
          Flag = 2;
      }

    Time_ResetFlag(Flag_10ms);
  }

}



int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  Delay_Init();
  // LED_Init();
  // Delay_ms(500);
  Key_Init();
  // EEPROM_Init();
  // AsciiCode_Init();

  // ��3��˳���ܶ� ���5Vע������Ϊ����ģʽ��
  WS2812_Init();
  // MorseCode_Init();
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
    //   Delay_ms(500);
	  Task();
      // WS2812_SetColor(240,50,0,1);
      // RGB_SendToLED();
      	// MorseCodeSend("Jack Trust Me");

      // WS2812_SetColor(0x80,0x10,0x01,1);
      // WS2812_SetColor(0x80,0x10,0x01,2);
      // WS2812_SetColor(0x80,0x10,0x01,3);
      // AsciiCodeSend("123456");


  }


}

