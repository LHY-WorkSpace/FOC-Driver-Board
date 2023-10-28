#ifndef  DATATYPE_H
#define  DATATYPE_H


#define  D_TRUE       0
#define  D_FALSE      1

#define  HIGH   1
#define  LOW    0

#define	Bit_0	0x0001
#define	Bit_1	0x0002
#define	Bit_2	0x0004
#define	Bit_3	0x0008
#define	Bit_4	0x0010
#define	Bit_5	0x0020
#define	Bit_6	0x0040
#define	Bit_7	0x0080
#define	Bit_8	0x0100
#define	Bit_9	0x0200
#define	Bit_10	0x0400
#define	Bit_11	0x0800
#define	Bit_12	0x1000
#define	Bit_13	0x2000
#define	Bit_14	0x4000
#define	Bit_15	0x8000

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

// typedef char s8;
// typedef short s16;
// typedef int s32;


typedef union 
{
    u8  B08[2];
    u16 B16;
}B16_B08;

typedef union 
{
    u8  B08[4];
    u32 B32;
}B32_B08;

typedef union 
{
    u16 B16[2];
    u32 B32;
}B32_B16;

//弧度->角度
#define RADTODEG(x) ((x) * 57.295779513082320876798154814105f)
//角度->弧度
#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)

float FastSin(float x);
float FastCos(float x);



#define  CLOSE_ALL_IRQ      do{  __disable_irq();  }while(0)    
#define  OPEN_ALL_IRQ       do{  __enable_irq();  }while(0)    



#endif






