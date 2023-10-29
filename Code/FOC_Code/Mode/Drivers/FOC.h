#ifndef FOC_H
#define FOC_H

//PWM���� 20khz ��= PWM_PERIOD(us)/2
// ���鲻����20khz��������������
#define PWM_PERIOD    (100)
//���������
#define POLE_PAIR	(7)
//����ѹ
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

