#include "u8g2_UserGUI.h"
#include "DataType.h"
#include "stm32f10x.h"
#include "Timer.h"
static u8g2_t u8g2_Data;


u8 flag=0;
static unsigned char FreeRTOS_Logo[] =
{
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XFF,0XFF,0XFF,0X05,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0X00,0X00,0X00,0X00,
0X00,0X20,0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X01,
0X00,0X10,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XE0,0XFF,0XFF,0XFF,0X07,
0X00,0X18,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X18,
0X00,0X18,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XC0,0X9F,0XFF,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XE0,0X8D,0XFF,0X3F,0XF8,0XFF,0X1F,0XC0,0X1F,0X00,0X00,0X00,0X00,0X38,
0X00,0X0C,0XB0,0XCD,0XFF,0X7F,0XF8,0XFF,0X1F,0XF8,0XFF,0X01,0XC0,0XFF,0X01,0X78,
0X00,0X06,0XB0,0XCF,0XFF,0XFF,0XFC,0XFF,0X1F,0XFE,0XFF,0X07,0XF8,0XFF,0X07,0X78,
0X00,0X06,0XF0,0XE7,0X0F,0XFF,0XF8,0XFF,0X0F,0XFF,0XFF,0X0F,0XFC,0XFF,0X07,0X78,
0X00,0X06,0XE0,0XE3,0X07,0X7E,0X00,0X3F,0X80,0XFF,0XF8,0X0F,0XFE,0XFF,0X03,0X38,
0X00,0X03,0X00,0XE0,0X07,0X7E,0X80,0X3F,0XC0,0X3F,0XE0,0X1F,0XFF,0X81,0X03,0X38,
0X00,0X03,0X70,0XF0,0X07,0X3F,0X80,0X3F,0XE0,0X1F,0XC0,0X1F,0XFF,0X00,0X00,0X38,
0X80,0X01,0X7C,0XF3,0XFF,0X1F,0X80,0X1F,0XE0,0X0F,0XC0,0X1F,0XFF,0X00,0X00,0X38,
0X80,0X01,0X2E,0XF3,0XFF,0X0F,0X80,0X1F,0XF0,0X07,0XC0,0X1F,0XFF,0X03,0X00,0X38,
0X80,0X01,0XB6,0XF9,0XFF,0X07,0XC0,0X1F,0XF0,0X07,0XC0,0X1F,0XFF,0X1F,0X00,0X38,
0XC0,0X00,0XFE,0XF9,0XFF,0X07,0XC0,0X1F,0XF0,0X07,0XC0,0X1F,0XFE,0XFF,0X00,0X3C,
0XC0,0X00,0XFE,0XF8,0XF1,0X07,0XC0,0X0F,0XF0,0X07,0XC0,0X1F,0XFC,0XFF,0X01,0X3C,
0XE0,0X00,0X3D,0XFC,0XE1,0X07,0XE0,0X0F,0XF0,0X07,0XE0,0X1F,0XF0,0XFF,0X03,0X3C,
0X60,0X00,0X03,0XFC,0XE0,0X0F,0XE0,0X0F,0XF0,0X07,0XE0,0X1F,0X00,0XFF,0X07,0X3C,
0X60,0X00,0X03,0XFC,0XE0,0X0F,0XE0,0X0F,0XF0,0X07,0XF0,0X0F,0X00,0XFC,0X07,0X3C,
0X70,0X80,0X7F,0XFE,0XE0,0X0F,0XE0,0X07,0XF0,0X0F,0XF8,0X0F,0X00,0XF8,0X07,0X3C,
0X30,0X80,0X7F,0X7E,0XE0,0X0F,0XF0,0X07,0XF0,0X1F,0XFE,0X87,0X00,0XF8,0X07,0X3C,
0X30,0X18,0X3F,0X7E,0XE0,0X0F,0XF0,0X07,0XE0,0XFF,0XFF,0X83,0X07,0XFC,0X07,0X3C,
0X18,0X1C,0X00,0X00,0XC0,0X0F,0XF0,0X07,0XC0,0XFF,0XFF,0X81,0XFF,0XFF,0X03,0X1C,
0X18,0XCC,0X00,0X00,0X00,0X00,0XE0,0X03,0X80,0XFF,0X7F,0XC0,0XFF,0XFF,0X01,0X1C,
0X1C,0XFC,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0XFC,0X1F,0XC0,0XFF,0XFF,0X00,0X1C,
0X0C,0XFC,0X9F,0XFF,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X3F,0X00,0X1E,
0X0C,0XF8,0X9F,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X80,0X01,0X00,0X1E,
0X0E,0X30,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X00,0X00,0X00,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X00,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0X0F,0X00,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0X03,0X1E,
0X06,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0X03,0X1E,
0XFE,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0X03,0X1E,
0XFC,0XFF,0XFF,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1E,
0XFC,0XFF,0XFF,0XFF,0XFF,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,
0XE0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,
0X00,0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X0F,0X00,0X00,0X00,0X00,0X00,0X0E,
0X00,0X00,0X00,0XC0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,0X00,0X00,0X00,0X0F,
0X00,0X00,0X00,0X00,0X00,0XF0,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,0X00,0X0F,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X07,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0X03,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF,0X01,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0XFF,0XFF,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};






void Power_On(void);

GUI_t GUI_Info =
{   
    .KeyInfo.KeyNum = Key_MaxNum,
    .KeyInfo.KeyState = NONE_PRESS,
    .Index = Main_ui,
    .UI_List[Main_ui] = Power_On,
    // .UI_List[Main_ui] = Main_UI,

};



void OLED_Init()
{

}

//***************************************************//
//  功能描述: 
//  
//  参数: 无
//  
//  返回值: TRUE / FALSE
//  
//  说明: 无
//  
//***************************************************//
void u8g2_Init()
{
    OLED_Init();
    #if(OLED_TYPE == SSD1306)
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2_Data, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay); 
    #elif(OLED_TYPE == SH1106)
    u8g2_Setup_sh1106_128x64_noname_f(&u8g2_Data, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay); 
    #else
    #warning "显示设备未适配"
    #endif
	u8g2_InitDisplay(&u8g2_Data);
	u8g2_SetPowerSave(&u8g2_Data, 0);
    u8g2_ClearDisplay(&u8g2_Data);
}





void u8g2_Logo(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1);  // Transparent
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);

    u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(u8g2, 1,54,"Design By LHY");	
}


void Display_FreeRTOS_Logo()
{
    u8g2_ClearBuffer(&u8g2_Data);
    u8g2_DrawXBM(&u8g2_Data,0,8,128,48,FreeRTOS_Logo);
    u8g2_SendBuffer(&u8g2_Data);
    Delay_ms(500);
    u8g2_ClearBuffer(&u8g2_Data);
}


void Display_U8g2_Logo()
{

    u8g2_Logo(&u8g2_Data);
    u8g2_SendBuffer(&u8g2_Data); 
    Delay_ms(500);  
    u8g2_ClearBuffer(&u8g2_Data);
}

void DispalyUTF_8Strings()
{   
    u8g2_ClearBuffer(&u8g2_Data);
    u8g2_SetFontMode(&u8g2_Data,0);
    u8g2_SetFont(&u8g2_Data, u8g2_font_6x12_tr);
    u8g2_DrawUTF8(&u8g2_Data,10,10,"STM32 U8g2");

    u8g2_DrawButtonUTF8(&u8g2_Data,40,40,U8G2_BTN_HCENTER|U8G2_BTN_BW1|U8G2_BTN_SHADOW1,0,1,1,"OK");
    u8g2_SendBuffer(&u8g2_Data); 
     u8g2_ClearBuffer(&u8g2_Data);
    Delay_ms(500);
    u8g2_DrawButtonUTF8(&u8g2_Data,41,41,U8G2_BTN_HCENTER|U8G2_BTN_BW1,0,1,1,"FreeRTOS");
    u8g2_SendBuffer(&u8g2_Data);   
    Delay_ms(500);
}

void Mov(u16 x,u16 y)
{
    char Data[6];

    // u8g2_ClearBuffer(&u8g2_Data);
    // u8g2_DrawButtonUTF8(&u8g2_Data,40,40,U8G2_BTN_HCENTER|U8G2_BTN_BW1|U8G2_BTN_SHADOW1,0,1,1,"OK");
    u8g2_ClearBuffer(&u8g2_Data);
    memset(Data,0x00,sizeof(Data)-1);
    
    sprintf(Data+1,"%d",x);
    Data[0]='x';
    u8g2_DrawUTF8(&u8g2_Data,x,y+8,Data);
    sprintf(Data+1,"%d",y);
    Data[0]='y';
    u8g2_DrawUTF8(&u8g2_Data,x,y+16,Data);
    u8g2_SendBuffer(&u8g2_Data); 


}


void Power_On()
{
    static uint8_t x=1,y=0,z=0;
    static uint8_t px=1,py=0;

    if(x <= 31)
    {
        u8g2_DrawHLine(&u8g2_Data,62-x*2,31,(x+1)*4);
        u8g2_DrawHLine(&u8g2_Data,62-x*2,32,(x+1)*4);
        x++;
    }
    else
    {
        px = 63*FastCos(DEGTORAD(y));
        py = 31*FastSin(DEGTORAD(y));

        if(y<90)
        {
            y++;
            u8g2_DrawLine(&u8g2_Data,63,31,63-px,31+py);
            u8g2_DrawLine(&u8g2_Data,63,31,63-px,31-py);
            u8g2_DrawLine(&u8g2_Data,63,31,63+px,31+py);
            u8g2_DrawLine(&u8g2_Data,63,31,63+px,31-py);
        }
        else
        {
            if(z<63)
            {
                z++;
                u8g2_DrawVLine(&u8g2_Data,63-z,0,63);
                u8g2_DrawVLine(&u8g2_Data,64+z,0,63);

                if( z > 11 )
                {
                    u8g2_SetFont(&u8g2_Data, u8g2_font_streamline_design_t);
                    u8g2_DrawGlyph(&u8g2_Data,51,84-z,0x011F);
                }
            }
            else
            {
                x=1;
                y=0;
                z=0;
                GUI_Info.Index = Main_ui;
            }
        }
    }
}



void u8g2_Task()
{
    u8g2_ClearBuffer(&u8g2_Data);

    GUI_Info.KeyInfo = GetKeyState();

    GUI_Info.UI_List[GUI_Info.Index]();

    u8g2_DrawFrame(&u8g2_Data,0,0,128,64);

    u8g2_SendBuffer(&u8g2_Data);
}



