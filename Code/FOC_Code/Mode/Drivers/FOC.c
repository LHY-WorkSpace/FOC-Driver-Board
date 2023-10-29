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

#define SQRT_3 (1.7320508f)
#define LED_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED_OF  GPIO_SetBits(GPIOB,GPIO_Pin_9)

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


	TIM_TimeBaseInitsruc.TIM_Prescaler=36-1;
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

float LimitAngle(float Input) 
{
    float Tmp;
    Tmp = fmod(Input,360.0);
    if( Tmp < 0.0)
    {
       Tmp += 360.0;
    }
    return Tmp;
}
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//逆变换
void N_Transform(float uq, float ud, float Angle)
{
    float Ualpha,Ubeta; 

    Angle = LimitAngle(Angle);
    //帕克逆变换
    Ualpha = ud * FastCos(DEGTORAD(Angle)) - uq * FastSin(DEGTORAD(Angle)); 
    Ubeta =  ud * FastSin(DEGTORAD(Angle)) + uq * FastCos(DEGTORAD(Angle)); 

    // 克拉克逆变换
    Ua = Ualpha + VCC_MOTOR/2;
    // Ub = (sqrt(3)*Ubeta-Ualpha)/2 + VCC_MOTOR/2;
    // Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + VCC_MOTOR/2;

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



float G_P = 0.01;
float G_I = 0.0;
float G_D = 0.0;
float G_A = 0.0;

void Foc_CTL()
{
	float Angle  = 1.0f;
    float angtmp;
    float UqTmp;   


        PID_Change_Kp(&PositionPID,G_P);
        PID_Change_Ki(&PositionPID,G_I);
        PID_Change_Kd(&PositionPID,G_D);
        PID_SetTarget(&PositionPID,G_A);
        Angle = AS5600_Angle(ANGLE_TURN_MODE);
        // printf("FOC:%f,%f,%f,%.2f,%.2f\n",G_P,G_I,G_D,G_A,Angle);
        // angtmp = LimitAngle(Angle);     
        angtmp = ElectricalAngle(Angle,POLE_PAIR);
        angtmp = LimitAngle(angtmp);

        UqTmp = PID_Process(&PositionPID,Angle);

        printf("FOC:%.2f,%.2f\n",UqTmp,Angle);
        N_Transform(_constrain(UqTmp,-6.0,6.0),0,angtmp);

        // N_Transform(_constrain(UqTmp,-6.0,6.0),0,angtmp);
            // N_Transform(0.01,0,10.0);

        // SVPWM_CTL(_constrain(UqTmp,-6.0,6.0),0,angtmp);
        FOC_TickTask();


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



// void BLDCMotor::setPhaseVoltage(float Uq, float angle_el) {
//   switch (foc_modulation)
//   {
//     case FOCModulationType::SinePWM :
//       // 正弦PWM调制
//       // 逆派克+克拉克变换

//       // 在0到360°之间的角度归一化
//       // 只有在使用 _sin和 _cos 近似函数时才需要
//       angle_el = normalizeAngle(angle_el + zero_electric_angle);
//       // 逆派克变换
//       Ualpha =  -_sin(angle_el) * Uq;  // -sin(angle) * Uq;
//       Ubeta =  _cos(angle_el) * Uq;    //  cos(angle) * Uq;

//       // 克拉克变换
//       Ua = Ualpha + voltage_power_supply/2;
//       Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta + voltage_power_supply/2;
//       Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta + voltage_power_supply/2;
//       break;

//     case FOCModulationType::SpaceVectorPWM :
//       // 解释空间矢量调制(SVPWM)算法视频
//       // https://www.youtube.com/watch?v=QMSWUMEAejg

//       // 如果负电压的变化与相位相反
//       // 角度+180度
//       if(Uq < 0) angle_el += _PI;
//       Uq = abs(Uq);

//       // 在0到360°之间的角度归一化
//       // 只有在使用 _sin和 _cos 近似函数时才需要
//       angle_el = normalizeAngle(angle_el + zero_electric_angle + _PI_2);

//       // 找到我们目前所处的象限
//       int sector = floor(angle_el / _PI_3) + 1;
//       // 计算占空比
//       float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uq/voltage_power_supply;
//       float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uq/voltage_power_supply;
//       // 两个版本
//       // 以电压电源为中心/2
//       float T0 = 1 - T1 - T2;
//       // 低电源电压，拉到0
//       //float T0 = 0;

//       // 计算占空比（时间）
//       float Ta,Tb,Tc; 
//       switch(sector){
//         case 1:
//           Ta = T1 + T2 + T0/2;
//           Tb = T2 + T0/2;
//           Tc = T0/2;
//           break;
//         case 2:
//           Ta = T1 +  T0/2;
//           Tb = T1 + T2 + T0/2;
//           Tc = T0/2;
//           break;
//         case 3:
//           Ta = T0/2;
//           Tb = T1 + T2 + T0/2;
//           Tc = T2 + T0/2;
//           break;
//         case 4:
//           Ta = T0/2;
//           Tb = T1+ T0/2;
//           Tc = T1 + T2 + T0/2;
//           break;
//         case 5:
//           Ta = T2 + T0/2;
//           Tb = T0/2;
//           Tc = T1 + T2 + T0/2;
//           break;
//         case 6:
//           Ta = T1 + T2 + T0/2;
//           Tb = T0/2;
//           Tc = T1 + T0/2;
//           break;
//         default:
//          // 可能的错误状态
//           Ta = 0;
//           Tb = 0;
//           Tc = 0;
//       }

//       // 计算相电压和中心
//       Ua = Ta*voltage_power_supply;
//       Ub = Tb*voltage_power_supply;
//       Uc = Tc*voltage_power_supply;
//       break;
//   }
  
//   // 设置硬件中的电压
//   setPwm(Ua, Ub, Uc);
// }

