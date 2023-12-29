#ifndef LED_H
#define LED_H



// PB9  LED
#define LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED_OFF  GPIO_SetBits(GPIOB,GPIO_Pin_9)


void LED_Init(void);
void LED_RGB_Init(void);
void RGB_SetColor(u8 Red,u8 Green,u8 Blue,u8 Num);

#endif







