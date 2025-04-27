/***
	*******************************************************************************************************************************************************************
	* @file    main.c
	* @version V2.1
	* @date    2024-7-22
	* @author  ����	
	* @brief   MSPM0G3507С��PID����ͨ��ģ��
   *************************************************************************************************
   *  @description
	*	
	*  �ӿ����ÿ���ʹ��Sysconfig��
   *
>>>>> ����˵����δ������������ת�����������״��룬��Ҷ����ҹ�����Ϊ���꣬�뱣�ֺ��Լ��ĳ��ģ��ڴˣ��������ҵĸ�л
	*************************************************************************************************************************************************************
***/

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------ͷ�ļ�����--------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#include "main.h"
#include "key.h"
#include "delay.h"
#include "oled.h"
#include "bmp.h"
#include "usart.h"
#include "Encoder.h"
#include "motor_ctrl.h"
#include "protocol.h"
#include "gw_gray.h"
#include "mpu6050.h"

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------�Զ������--------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint8_t Car_Mode = Angle_Mode;//����ģʽ


float Basic_Speed = 30;									//���Ŀ���ٶ�
uint8_t OLED_View_Select = 1;							//OLEDѡ��������
float Target_Distance = 0;
float Target_Gyro = 0;
float Target_Angle = 0;
/*-------------------------------------------------------------------------------------------*/
/*---------------------------------------������----------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
int main(void)
{
	SYSCFG_DL_init();//оƬ��Դ��ʼ��,��SysConfig��������Զ�����
	//Ӳ����ʼ��
    OLED_Init();//��ʼ��OLED
	mpu6050_init();//��ʼ��MPU6050
	protocol_init();//Ұ����λ����ʼ��
	NVIC_EnableIRQ_Init();//NVIC�ж����ó�ʼ��
	
	while(1)
	{
		KEY_PROC();//����������
		OLED_Show_Proc();//OLED��ʾ����
		Protocol_Datas_Proc();//Ұ����λ��������
	}
}




float  Target_ChaSu;		//Ŀ�����
float Motor1_Target_Speed;	//��ߵ��Ŀ���ٶ�
float Motor2_Target_Speed;	//�ұߵ��Ŀ���ٶ�
	/*---------------------------------------------------------------------------------------*/
/*------------------------------��ʱ��A1��1ms�жϷ�����------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void TIMER_0_INST_IRQHandler(void)//��ʱ���жϷ�����
{
	static uint16_t count_10ms=0;
	static uint16_t count_100ms=0;
	static uint16_t t=0;
	
	switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) 
	{
		 case DL_TIMERA_IIDX_LOAD:
		 {
			 if(++count_10ms>=10)//��ʱ10ms
			 {count_10ms=0;
				 //������ȡ
				 Key_Read();
				 //��ȡ�Ƕȵ�ʱ����Ҫ����ջ��Ϊ400 200���������Ѹģ�
				 AHRS_Geteuler();
				 //��·�Ҷ�ѭ��ģ�����ݶ�ȡ
				 Huidu_Datas = Huidu_Read();//8λ�Ҷ�����
				 Huidu_Proc(Huidu_Datas);//����8λ�Ҷ����ݽ�������ȡ
				 
				 //-------------------------------PID��������---------------------------------//
				 if(Angle_PID_Flag == 0)//������򿪽ǶȻ�
				 {	
					if(Distance_PID_Flag == 1)//����򿪾��뻷
					{
							Basic_Speed = PID_Calculate(&pid_Distance,Measure_Distance,Target_Distance);
					}
					if(Turn_PID_Flag == 1)//�����Ѱ��
					{
						if(Basic_Speed!=0)
						   Target_ChaSu = PID_Calculate(&pid_Turn,Huidu_Error,Huidu_Target)*(Basic_Speed)*0.04;//ת���������
						else
						   Target_ChaSu = 0;
					}
					else Target_ChaSu = 0 ;//��Ѱ��ʱת�����Ϊ0
					
					 //�����ʱ��Ŀ���ٶ�
					 Motor1_Target_Speed = Basic_Speed + Target_ChaSu;
					 Motor2_Target_Speed = Basic_Speed - Target_ChaSu;					
				 }
				 else //����򿪽ǶȻ�
				 {
					if(Gyro_PID_Flag == 0)
							Target_Gyro = PID_Calculate(&pid_Angle,mpu6050.Yaw,Target_Angle);//�ǶȻ�
					float TurnSpeed = PID_Calculate(&pid_Gyro,Gyro_Z_Measeure,Target_Gyro);//���ٶȻ�
					
					Motor1_Target_Speed = -TurnSpeed;
					Motor2_Target_Speed =  TurnSpeed;
				 }
				 MEASURE_MOTORS_SPEED();//��������ٶ�
				 //�����ٶȻ��ٶ�
				 SET_MOTORS_SPEED(PID_Calculate(&pid_Motor1_Speed,Motor1_Speed,Motor1_Target_Speed	),
								  PID_Calculate(&pid_Motor2_Speed,Motor2_Speed,Motor2_Target_Speed	));
			 }
			 //�ϴ��������ݣ�û�õ���
			  if(++count_100ms>=100)//��ʱ100ms
			 {count_100ms=0;
				 
			 }
			 
			if(++t>=1000)//��ʱ1s�������жϳ��������У�
			{t=0;
				LED3_TOGGLE();//��ת LED ��ƽ
			}
		}break;
		 default:break;
	 }
}







/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------OLED��ʾ����------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void OLED_Show_Proc(void)
{
	switch(OLED_View_Select)
	{
		case 1://������
		{
			OLED_ShowString(46,0,(uint8_t *)"Main",16,1);
			OLED_ShowBinNum(0,16,Huidu_Datas,7,16,1);OLED_ShowNum(80,16,Huidu_Sum,1,16,1);OLED_ShowSignedNum(100,16,Huidu_Error,1,16,1);
			OLED_ShowString(0,32,(uint8_t *)"T_Speed:",16,1);OLED_ShowFloatNum(65,32,Basic_Speed,3,2,16,1);
			OLED_ShowFloatNum(0,48,Motor1_Speed,3,2,16,1);OLED_ShowFloatNum(70,48,Motor2_Speed,3,2,16,1);
			
			OLED_Update();
		}break;
		case 2://�Ƕ����ݶ�ȡ����
		{
//			OLED_ShowString(15,0,(uint8_t *)"Acc and Gyro",16,1);
//			OLED_ShowString(0,16,(uint8_t *)"X:",16,1);OLED_ShowSignedNum(15,16,mpu6050.Accel_Original[0],5,16,1); OLED_ShowSignedNum(75,16,mpu6050.Gyro_Original[0],5,16,1);
//			OLED_ShowString(0,32,(uint8_t *)"Y:",16,1);OLED_ShowSignedNum(15,32,mpu6050.Accel_Original[1],5,16,1);OLED_ShowSignedNum(75,32,mpu6050.Gyro_Original[1],5,16,1);
//			OLED_ShowString(0,48,(uint8_t *)"Z:",16,1);OLED_ShowSignedNum(15,48,mpu6050.Accel_Original[2],5,16,1);OLED_ShowSignedNum(75,48,mpu6050.Gyro_Original[2],5,16,1);			
			OLED_ShowString(40,0,(uint8_t *)"Angle",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Pit:",16,1);OLED_ShowFloatNum(35,16,mpu6050.Pitch,3,2,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Rol:",16,1);OLED_ShowFloatNum(35,32,mpu6050.Roll,3,2,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Yaw:",16,1);OLED_ShowFloatNum(35,48,mpu6050.Yaw,3,2,16,1);
			OLED_Update();
		}break;
		case 3://��ߵ��PID��ʾ
		{
			OLED_ShowString(26,0,(uint8_t *)"Motor1_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Motor1_Speed.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Motor1_Speed.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Motor1_Speed.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 4://�ұߵ��PID��ʾ
		{
			OLED_ShowString(26,0,(uint8_t *)"Motor2_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Motor2_Speed.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Motor2_Speed.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Motor2_Speed.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 5://ת��PID��ʾ
		{
			OLED_ShowString(30,0,(uint8_t *)"Turn_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Turn.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Turn.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Turn.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 6://���뻷PID��ʾ
		{   
			OLED_ShowString(20,0,(uint8_t *)"Distance_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Distance.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Distance.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Distance.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 7://���ٶȻ�PID��ʾ
		{   
			OLED_ShowString(30,0,(uint8_t *)"Gyro_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Gyro.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Gyro.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Gyro.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 8://�ǶȻ�PID��ʾ
		{   
			OLED_ShowString(28,0,(uint8_t *)"Angle_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Angle.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Angle.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Angle.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 9://���ٶȻ������ݣ��л�������ʾ���棬û�д˽��棬��Ҫ��ӽ��棬��Ҫȥkey.c��KEY_PROC()���޸�
		{   
			OLED_ShowFloatNum(0,0,pid_Gyro.Kp,3,3,16,1);
			OLED_ShowFloatNum(0,16,pid_Gyro.Ki,3,3,16,1);
			OLED_ShowFloatNum(0,32,Target_Gyro,3,3,16,1);
			OLED_ShowFloatNum(0,48,Gyro_Z_Measeure,3,3,16,1);
			OLED_Update();
		}break;
		default:break;
	}
}
/*-------------------------------------------------------------------------------------------*/
/*-----------------------------------�����жϳ�ʼ��------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void NVIC_EnableIRQ_Init(void)
{
	//�������0�жϱ�־
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //ʹ�ܴ���0�ж�
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	//�����ʱ���жϱ�־
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //ʹ�ܶ�ʱ���ж�
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	//��ʱ��A��ʼ����
	DL_TimerA_startCounter(TIMER_0_INST);
	//�������ж�ʹ��
	NVIC_EnableIRQ(Encoder_INT_IRQN);
	//�������ж�ʹ�ܣ�û���õ�DMP�⣩
	//NVIC_EnableIRQ(MPU6050_INT_IRQN);
	
}
/*-------------------------------------------------------------------------------------------*/
/*-------------------------------Ұ����λ���������ݴ���--------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void Protocol_Datas_Proc(void)
{
	if(Car_Mode != Run_Mode)
	{		
		int PROTOCOL_TEMP=0;//Ұ����λ���ϴ���ʱ����
		if(Car_Mode==Speed_Mode)//�ٶȻ�ģʽ
		{
			Turn_PID_Flag = 0;		//�ر�ѭ��
			Angle_PID_Flag = 0;		//�رսǶȻ�
			Gyro_PID_Flag = 0;		//�رս��ٶȻ�
			Distance_PID_Flag = 0;	//�رվ��뻷
			
			//�ϴ�����ѡ��
			if(Set_Motor_Param_Select==1) PROTOCOL_TEMP = Motor1_Speed;
			else if(Set_Motor_Param_Select==2) PROTOCOL_TEMP = Motor2_Speed;
		}
		else if(Car_Mode == Turn_Mode)//ת��ģʽ
		{
			Turn_PID_Flag = 1;		
			Distance_PID_Flag = 0;
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;	
			
			PROTOCOL_TEMP = Huidu_Error;
		}
		else if(Car_Mode == Distance_Mode)//���뻷ģʽ
		{
			Turn_PID_Flag = 0;	
			Distance_PID_Flag = 1;		
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;		
			
			PROTOCOL_TEMP = Measure_Distance;
		}   
		else if(Car_Mode == Gyro_Mode)//���ٶȻ�ģʽ
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 1;
			Angle_PID_Flag = 1;	
			
			PROTOCOL_TEMP = Gyro_Z_Measeure;
		}
		else if(Car_Mode == Angle_Mode)//�ǶȻ�ģʽ
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 0;
			Angle_PID_Flag = 1;
			
			PROTOCOL_TEMP = mpu6050.Yaw;
		}
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &PROTOCOL_TEMP, 4);//����λ����������	
		receiving_process();//�Խ������ݽ��д���
	}
	else//����Run_Mode�µĴ���	������Ҫ�ϴ����ݣ�
	{
		//����Run_Mode�µĳ����߼�����
		
		
		
		
		
		
		
		
		
		
	}
}
