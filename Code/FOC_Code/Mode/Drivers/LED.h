#ifndef LED_H
#define LED_H


#define LED_ON      GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED_OFF     GPIO_SetBits(GPIOB,GPIO_Pin_9)

void LED_Init(void);

#endif







