#ifndef  U8G2_USERGUI_H
#define  U8G2_USERGUI_H

#include "u8x8.h"
#include "u8g2.h"
#include "Key.h"

#define  SSD1306    (1)
#define  SH1106     (2)

#define  OLED_TYPE  SH1106


#define  OLED_RST_OFF   GPIO_SetBits(GPIOB,GPIO_Pin_14)	              //复位关
#define  OLED_RST_ON    GPIO_ResetBits(GPIOB,GPIO_Pin_14)	          //复位开
#define  OLED_DATA      GPIO_SetBits(GPIOB,GPIO_Pin_12)	              //数据
#define  OLED_CMD       GPIO_ResetBits(GPIOB,GPIO_Pin_12)	          //命令





typedef enum
{
    Main_ui,
    UI_MAX,
}UI_Index_e;



typedef struct 
{
    KeyInfo_t KeyInfo;
    UI_Index_e Index; 
    void (*UI_List[UI_MAX])(void);
}GUI_t;



void u8g2_Task(void);
void u8g2_Init(void);
void Display_U8g2_Logo(void);


#endif
