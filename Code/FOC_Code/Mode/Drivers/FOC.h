#ifndef FOC_H
#define FOC_H
// 满周期计数值  = 72*1000/CLK_DIV/PWM_FRQUENCE
// 建议不低于20khz，否则电机噪声大
#define CLK_DIV             (4)//能被72整除,
#define PWM_FRQUENCE        (60)//Khz
//电机极对数
#define POLE_PAIR	(7)
//最大电压
#define VCC_MOTOR	(12.0f)

typedef enum
{
    UA_Phase,
    UB_Phase,
    UC_Phase,
    U_PhaseMax,
}U_Phase_e;

void PWM_Init(void);
void FOC_GPIO_Init(void);
void FOC_main(void);
void PWM_Task(void);
void PWM_SetDuty(uint8_t Phase,uint8_t Value);
void Foc_CTL(void);
void FOC_TickTask(void);
#endif

