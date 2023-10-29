#ifndef EEPROM_H
#define EEPROM_H

// SCL - PA6
// SDA - PA7

#define EEPROM_SIZE (1 * 1024)                         // �ֽ�
#define EEPROM_PAGE_SIZE (16)                           // �ֽ�
#define EEPROM_PAGES (EEPROM_SIZE / EEPROM_PAGE_SIZE) // ҳ�� �ڲ�Ram��ҳ(EEPROM_PAGE_SIZE Byte)����
#define EEPROM_ADDRESS (0xA0)                           // AT24C02 �豸��ַ


#define EE_IIC_SCL_LOW  			GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define EE_IIC_SCL_HIGH  		 	GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define EE_IIC_SDA_LOW  			GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define EE_IIC_SDA_HIGH 			GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define EE_IIC_SDA_STATE            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)

// #define EEPROM_ADDR(NUMBER) ((u16)(((uint32_t)&(((EEPROM_MAP *)0)->NUMBER))&0x0000FFFF))

void EEPROM_Init(void);
u8 EE_WriteData(u16 addr, u16 length, u8 *data);
u8 EE_ReadData(u16 addr, u16 length, u8 *data);

#endif