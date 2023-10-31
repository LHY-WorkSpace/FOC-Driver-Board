#include "DataType.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "FOC.h"
#include "AS5600.h"
#include "PID.h"
#include <math.h>


static PID_t PositionPID;
static PID_t SpeedPID;
static PID_t ForcePID;
static float Ua,Ub,Uc;

#define SQRT_3      (1.7320508075f)//sqrt(3)/
#define SQRT_3_2    (0.8660254037f)//sqrt(3)/2
#define LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED_OF  GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define ValueLimit(Val,Min,Max) ((Val)<(Min)?(Min):((Val)>(Max)?(Max):(Val)))

void LED_Init()
{

	GPIO_InitTypeDef GPIO_Initstructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9;	
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_Initstructure);

}

void PWM_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef PWM_IO;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitsruc;
    TIM_OCInitTypeDef TIM_OCInit;
	
	PWM_IO.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;	
	PWM_IO.GPIO_Speed = GPIO_Speed_50MHz;
	PWM_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&PWM_IO);


	TIM_TimeBaseInitsruc.TIM_Prescaler=CLK_DIV-1;
	TIM_TimeBaseInitsruc.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitsruc.TIM_Period=PWM_PERIOD-1;//周期
    TIM_TimeBaseInitsruc.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitsruc.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitsruc);
	
    TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInit.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInit.TIM_Pulse = 0;
    TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInit.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInit.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInit.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1,&TIM_OCInit);
    TIM_OC2Init(TIM1,&TIM_OCInit);
    TIM_OC3Init(TIM1,&TIM_OCInit);

    TIM_OC1FastConfig(TIM1,TIM_OCFast_Enable);
    TIM_OC2FastConfig(TIM1,TIM_OCFast_Enable);
    TIM_OC3FastConfig(TIM1,TIM_OCFast_Enable);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM1,ENABLE);
    TIM_ARRPreloadConfig(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);

    PID_Init(&PositionPID);

}


//设置占空比(%)
// PluseWide = 10 = 10%
void PWM_SetDuty(u8 Phase ,u8 PluseWide)
{

    switch (Phase)
    {
        case UA_Phase:
            TIM1->CCR1 = PWM_PERIOD/100*PluseWide-1;
            break;
         case UB_Phase:
            TIM1->CCR2 = PWM_PERIOD/100*PluseWide-1;
            break;
        case UC_Phase:
            TIM1->CCR3 = PWM_PERIOD/100*PluseWide-1;
            break;       
        default:
            break;
    }
}

//求电角度 = 物理角度*极对数
float ElectricalAngle(float physics_angle, int pole_pairs) 
{
  return (physics_angle * (float)pole_pairs);
}


// 限制角度[0 - 360]
float AngleLimit(float Input) 
{
    float Tmp;
    Tmp = fmod(Input,360.0);
    if( Tmp < 0.0)
    {
       Tmp += 360.0;
    }
    return Tmp;
}

//逆变换
void N_Transform(float uq, float ud, float Angle)
{
    float Ualpha,Ubeta; 

    Angle = AngleLimit(Angle);
    //帕克逆变换
    Ualpha = ud * FastCos(DEGTORAD(Angle)) - uq * FastSin(DEGTORAD(Angle)); 
    Ubeta =  ud * FastSin(DEGTORAD(Angle)) + uq * FastCos(DEGTORAD(Angle)); 

    // 克拉克逆变换
    Ua = Ualpha + VCC_MOTOR/2;
    Ub = (SQRT_3*Ubeta-Ualpha)/2 + VCC_MOTOR/2;
    Uc = (-Ualpha-SQRT_3*Ubeta)/2 + VCC_MOTOR/2;

    // PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
    // PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
    // PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
    //printf("Angle:%.2f Ia:%d Ib:%d Ic:%d\r\n",Angle,(uint8_t)(Ua*100/VCC_MOTOR),(uint8_t)(Ub*100/VCC_MOTOR),(uint8_t)(Uc*100/VCC_MOTOR));

}

void P_Transform(float Ia, float Ib, float Ic)
{
    // float iq,id; 
    // float ialpha,ibeta;
    // float Angle = 0.0;

    // ialpha = Ia;
    // ibeta = (1/sqrt(3))*Ia + (2/sqrt(3))*Ib;

    // //// 帕克变换
    // iq = ibeta  * FastCos(DEGTORAD(Angle)) - ialpha * FastSin(DEGTORAD(Angle));
    // id = ialpha * FastCos(DEGTORAD(Angle)) + ibeta  * FastSin(DEGTORAD(Angle));

}



float G_P = 0.05;
float G_I = 0.00003;
float G_D = 0.2;
float G_A = 0.0;

void Foc_CTL()
{
	float Angle  = 1.0f;
    float angtmp;
    float UqTmp;   

LED_ON;
        PID_Change_Kp(&PositionPID,G_P);
        PID_Change_Ki(&PositionPID,G_I);
        PID_Change_Kd(&PositionPID,G_D);
        PID_SetTarget(&PositionPID,G_A);
        Angle = AS5600_Angle(ANGLE_TURN_MODE);
        // printf("FOC:%f,%f,%f,%.2f,%.2f\n",G_P,G_I,G_D,G_A,Angle);
        // angtmp = AngleLimit(Angle);     
        angtmp = ElectricalAngle(Angle,POLE_PAIR);
        angtmp = AngleLimit(angtmp);

        UqTmp = PID_Process(&PositionPID,Angle);

        // printf("FOC:%.2f,%.2f\n",UqTmp,Angle);
        // N_Transform(ValueLimit(UqTmp,-6.0,6.0),0,angtmp);
        N_Transform(UqTmp,0,angtmp);
            // N_Transform(0.01,0,10.0);

        // SVPWM_CTL(ValueLimit(UqTmp,-6.0,6.0),0,angtmp);
        FOC_TickTask();
LED_OF;

}
void FOC_TickTask()
{

    PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
    PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
    PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));

    // PWM_SetDuty(UA_Phase,sssss);
    // PWM_SetDuty(UB_Phase,sssss);
    // PWM_SetDuty(UC_Phase,sssss);

    // printf("FOC:%.2f,%.2f,%.2f\n",(Ua*100/VCC_MOTOR),(Ub*100/VCC_MOTOR),(Uc*100/VCC_MOTOR));

}

// =========================================================================================================================
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f
// function approximating the sine calculation by using fixed size array
// uses a 65 element lookup table and interpolation
// thanks to @dekutree for his work on optimizing this
float _sin(float a){
  // 16bit integer array for sine lookup. interpolation is used for better precision
  // 16 bit precision on sine value, 8 bit fractional value for interpolation, 6bit LUT size
  // resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps)
  static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
  unsigned int i = (unsigned int)(a * (64*4*256.0 /_2PI));
  int t1, t2, frac = i & 0xff;
  i = (i >> 8) & 0xff;
  if (i < 64) {
    t1 = sine_array[i]; t2 = sine_array[i+1];
  }
  else if(i < 128) {
    t1 = sine_array[128 - i]; t2 = sine_array[127 - i];
  }
  else if(i < 192) {
    t1 = -sine_array[-128 + i]; t2 = -sine_array[-127 + i];
  }
  else {
    t1 = -sine_array[256 - i]; t2 = -sine_array[255 - i];
  }
  return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}


void _sincos(float a, float* s, float* c){
  *s = _sin(a);
  *c = _cos(a);
}


// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number) {//low in fat
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  float y = number;
  long i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}


void SIN_CTL(float Uq,float Ud, float angle_el) 
{
    float Ua,Ub,Uc;
    float Ualpha,Ubeta;

    // 正弦PWM调制
    // 逆派克+克拉克变换

    // 在0到360°之间的角度归一化
    // 只有在使用 _sin和 _cos 近似函数时才需要
    angle_el = AngleLimit(angle_el);

     // 逆派克变换
    if( Ud == 0.0)
    {
      Ualpha =  -FastSin(DEGTORAD(angle_el)) * Uq;  // -sin(angle) * Uq;
      Ubeta  =   FastCos(DEGTORAD(angle_el)) * Uq;    //  cos(angle) * Uq;
    }
    else
    {
      Ualpha = Ud * FastCos(DEGTORAD(angle_el)) - Uq * FastSin(DEGTORAD(angle_el)); 
      Ubeta =  Ud * FastSin(DEGTORAD(angle_el)) + Uq * FastCos(DEGTORAD(angle_el)); 
    }

    // 克拉克变换
    Ua = Ualpha + VCC_MOTOR/2;
    Ub = -0.5 * Ualpha  + SQRT_3_2 * Ubeta + VCC_MOTOR/2;
    Uc = -0.5 * Ualpha - SQRT_3_2 * Ubeta + VCC_MOTOR/2;

    PWM_SetDuty(UA_Phase,(uint8_t)(Ua*100/VCC_MOTOR));
    PWM_SetDuty(UB_Phase,(uint8_t)(Ub*100/VCC_MOTOR));
    PWM_SetDuty(UC_Phase,(uint8_t)(Uc*100/VCC_MOTOR));
}


void SVPWM_CTL(float Uq, float Ud,float angle_el) 
{
    float Uout;

     if(Ud)
      { 
        Uout = _sqrt(Ud*Ud + Uq*Uq) / VCC_MOTOR;
        angle_el = AngleLimit(angle_el + atan2(Uq, Ud));
      }
      else
      {
        Uout = Uq / VCC_MOTOR;
        angle_el = AngleLimit(angle_el + 90.0f);
      }

      // 找到我们目前所处的象限
      int sector = floor(angle_el / 60.0f) + 1;
      // 计算占空比
      float T1 = SQRT_3*FastSin(DEGTORAD(sector*60.0f - angle_el))* Uout;
      float T2 = SQRT_3*FastSin(DEGTORAD(angle_el - (sector-1.0)*60.0f)) * Uout;
      // 两个版本
      // 以电压电源为中心/2
      float T0 = 1 - T1 - T2;
      // 低电源电压，拉到0
      //float T0 = 0;

      // 计算占空比（时间）
      float Ta,Tb,Tc; 
      switch(sector)
      {
        case 1:
          Ta = T1 + T2 + T0/2;
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // 可能的错误状态
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

    // 计算相电压和中心
    // Ua = Ta*VCC_MOTOR;
    // Ub = Tb*VCC_MOTOR;
    // Uc = Tc*VCC_MOTOR;

    PWM_SetDuty(UA_Phase,(uint8_t)(Ta*100/VCC_MOTOR));
    PWM_SetDuty(UB_Phase,(uint8_t)(Tb*100/VCC_MOTOR));
    PWM_SetDuty(UC_Phase,(uint8_t)(Tc*100/VCC_MOTOR));
}


void Speed_CTL()
{
  float BBBB[10];
  float AAAA[10];
  u8 i;
  float avg = 0.0f;

  LED_ON;
  for ( i = 0; i < sizeof(AAAA); i++)
  {
      AAAA = FastSin(DEGTORAD(avg++));
  }
  for ( i = 0; i < sizeof(AAAA); i++)
  {
      BBBB = FastCos(DEGTORAD(avg++));
  }
  LED_OF;


  LED_ON;
  for ( i = 0; i < sizeof(AAAA); i++)
  {
      AAAA = _sin(DEGTORAD(avg++));
  }
  for ( i = 0; i < sizeof(AAAA); i++)
  {
      BBBB = _cos(DEGTORAD(avg++));
  }
  LED_OF;

}










