#ifndef _DCMOTOR_H_
#define _DCMOTOR_H_

/*Ӳ����Դ���Ŷ���
//DCMOTOR_A      PC13           //  ��ת
//DCMOTOR_B      PE2            //  ��ת
//DCMOTOR_S      PE4            //  �����������
*/
//MOTOR_A   PC13
#define  	DCMOTOR_A_L         GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define  	DCMOTOR_A_H         GPIO_SetBits(GPIOB, GPIO_Pin_8)

//MOTOR_B   PE2
#define  	DCMOTOR_B_L         GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define  	DCMOTOR_B_H         GPIO_SetBits(GPIOB, GPIO_Pin_9)


//	��������
void DCMotorGpio_Init(void);
void DCMotorTest(unsigned char mode);

#endif
