/***
	*******************************************************************************************************************************************************************
	* @file    main.c
	* @version V2.1
	* @date    2024-7-22
	* @author  御龙	
	* @brief   MSPM0G3507小车PID调试通用模板
   *************************************************************************************************
   *  @description
	*	
	*  接口配置可以使用Sysconfig看
   *
>>>>> 其他说明：未经允许不可擅自转发、售卖本套代码，大家都是我国的有为青年，请保持好自己的初心，在此，向你表达我的感谢
	*************************************************************************************************************************************************************
***/

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------头文件声明--------------------------------------------*/
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
/*-------------------------------------自定义变量--------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint8_t Car_Mode = Angle_Mode;//调试模式


float Basic_Speed = 30;									//电机目标速度
uint8_t OLED_View_Select = 1;							//OLED选择界面变量
float Target_Distance = 0;
float Target_Gyro = 0;
float Target_Angle = 0;
/*-------------------------------------------------------------------------------------------*/
/*---------------------------------------主函数----------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
int main(void)
{
	SYSCFG_DL_init();//芯片资源初始化,由SysConfig配置软件自动生成
	//硬件初始化
    OLED_Init();//初始化OLED
	mpu6050_init();//初始化MPU6050
	protocol_init();//野火上位机初始化
	NVIC_EnableIRQ_Init();//NVIC中断配置初始化
	
	while(1)
	{
		KEY_PROC();//按键处理函数
		OLED_Show_Proc();//OLED显示函数
		Protocol_Datas_Proc();//野火上位机处理函数
	}
}




float  Target_ChaSu;		//目标差速
float Motor1_Target_Speed;	//左边电机目标速度
float Motor2_Target_Speed;	//右边电机目标速度
	/*---------------------------------------------------------------------------------------*/
/*------------------------------定时器A1的1ms中断服务函数------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void TIMER_0_INST_IRQHandler(void)//定时器中断服务函数
{
	static uint16_t count_10ms=0;
	static uint16_t count_100ms=0;
	static uint16_t t=0;
	
	switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) 
	{
		 case DL_TIMERA_IIDX_LOAD:
		 {
			 if(++count_10ms>=10)//定时10ms
			 {count_10ms=0;
				 //按键读取
				 Key_Read();
				 //读取角度的时候需要将堆栈改为400 200（本工程已改）
				 AHRS_Geteuler();
				 //七路灰度循迹模块数据读取
				 Huidu_Datas = Huidu_Read();//8位灰度数据
				 Huidu_Proc(Huidu_Datas);//根据8位灰度数据进行误差读取
				 
				 //-------------------------------PID处理数据---------------------------------//
				 if(Angle_PID_Flag == 0)//如果不打开角度环
				 {	
					if(Distance_PID_Flag == 1)//如果打开距离环
					{
							Basic_Speed = PID_Calculate(&pid_Distance,Measure_Distance,Target_Distance);
					}
					if(Turn_PID_Flag == 1)//如果打开寻迹
					{
						if(Basic_Speed!=0)
						   Target_ChaSu = PID_Calculate(&pid_Turn,Huidu_Error,Huidu_Target)*(Basic_Speed)*0.04;//转向环输出差速
						else
						   Target_ChaSu = 0;
					}
					else Target_ChaSu = 0 ;//不寻迹时转向差速为0
					
					 //计算此时的目标速度
					 Motor1_Target_Speed = Basic_Speed + Target_ChaSu;
					 Motor2_Target_Speed = Basic_Speed - Target_ChaSu;					
				 }
				 else //如果打开角度环
				 {
					if(Gyro_PID_Flag == 0)
							Target_Gyro = PID_Calculate(&pid_Angle,mpu6050.Yaw,Target_Angle);//角度环
					float TurnSpeed = PID_Calculate(&pid_Gyro,Gyro_Z_Measeure,Target_Gyro);//角速度环
					
					Motor1_Target_Speed = -TurnSpeed;
					Motor2_Target_Speed =  TurnSpeed;
				 }
				 MEASURE_MOTORS_SPEED();//测量电机速度
				 //设置速度环速度
				 SET_MOTORS_SPEED(PID_Calculate(&pid_Motor1_Speed,Motor1_Speed,Motor1_Target_Speed	),
								  PID_Calculate(&pid_Motor2_Speed,Motor2_Speed,Motor2_Target_Speed	));
			 }
			 //上传波形数据（没用到）
			  if(++count_100ms>=100)//定时100ms
			 {count_100ms=0;
				 
			 }
			 
			if(++t>=1000)//定时1s（用于判断程序还在运行）
			{t=0;
				LED3_TOGGLE();//翻转 LED 电平
			}
		}break;
		 default:break;
	 }
}







/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------OLED显示界面------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void OLED_Show_Proc(void)
{
	switch(OLED_View_Select)
	{
		case 1://主界面
		{
			OLED_ShowString(46,0,(uint8_t *)"Main",16,1);
			OLED_ShowBinNum(0,16,Huidu_Datas,7,16,1);OLED_ShowNum(80,16,Huidu_Sum,1,16,1);OLED_ShowSignedNum(100,16,Huidu_Error,1,16,1);
			OLED_ShowString(0,32,(uint8_t *)"T_Speed:",16,1);OLED_ShowFloatNum(65,32,Basic_Speed,3,2,16,1);
			OLED_ShowFloatNum(0,48,Motor1_Speed,3,2,16,1);OLED_ShowFloatNum(70,48,Motor2_Speed,3,2,16,1);
			
			OLED_Update();
		}break;
		case 2://角度数据读取界面
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
		case 3://左边电机PID显示
		{
			OLED_ShowString(26,0,(uint8_t *)"Motor1_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Motor1_Speed.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Motor1_Speed.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Motor1_Speed.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 4://右边电机PID显示
		{
			OLED_ShowString(26,0,(uint8_t *)"Motor2_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Motor2_Speed.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Motor2_Speed.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Motor2_Speed.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 5://转向环PID显示
		{
			OLED_ShowString(30,0,(uint8_t *)"Turn_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Turn.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Turn.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Turn.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 6://距离环PID显示
		{   
			OLED_ShowString(20,0,(uint8_t *)"Distance_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Distance.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Distance.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Distance.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 7://角速度环PID显示
		{   
			OLED_ShowString(30,0,(uint8_t *)"Gyro_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Gyro.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Gyro.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Gyro.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 8://角度环PID显示
		{   
			OLED_ShowString(28,0,(uint8_t *)"Angle_PID",16,1);
			OLED_ShowString(0,16,(uint8_t *)"Kp:",16,1);OLED_ShowFloatNum(26,16,pid_Angle.Kp,3,3,16,1);
			OLED_ShowString(0,32,(uint8_t *)"Ki:",16,1);OLED_ShowFloatNum(26,32,pid_Angle.Ki,3,3,16,1);
			OLED_ShowString(0,48,(uint8_t *)"Kd:",16,1);OLED_ShowFloatNum(26,48,pid_Angle.Kd,3,3,16,1);
			OLED_Update();
		}break;
		case 9://角速度环看数据（切换按键显示界面，没有此界面，需要添加界面，需要去key.c中KEY_PROC()中修改
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
/*-----------------------------------所有中断初始化------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void NVIC_EnableIRQ_Init(void)
{
	//清除串口0中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口0中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	//清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	//定时器A开始计数
	DL_TimerA_startCounter(TIMER_0_INST);
	//编码器中断使能
	NVIC_EnableIRQ(Encoder_INT_IRQN);
	//陀螺仪中断使能（没有用到DMP库）
	//NVIC_EnableIRQ(MPU6050_INT_IRQN);
	
}
/*-------------------------------------------------------------------------------------------*/
/*-------------------------------野火上位机助手数据处理--------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
void Protocol_Datas_Proc(void)
{
	if(Car_Mode != Run_Mode)
	{		
		int PROTOCOL_TEMP=0;//野火上位机上传临时数据
		if(Car_Mode==Speed_Mode)//速度环模式
		{
			Turn_PID_Flag = 0;		//关闭循迹
			Angle_PID_Flag = 0;		//关闭角度环
			Gyro_PID_Flag = 0;		//关闭角速度环
			Distance_PID_Flag = 0;	//关闭距离环
			
			//上传数据选择
			if(Set_Motor_Param_Select==1) PROTOCOL_TEMP = Motor1_Speed;
			else if(Set_Motor_Param_Select==2) PROTOCOL_TEMP = Motor2_Speed;
		}
		else if(Car_Mode == Turn_Mode)//转向环模式
		{
			Turn_PID_Flag = 1;		
			Distance_PID_Flag = 0;
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;	
			
			PROTOCOL_TEMP = Huidu_Error;
		}
		else if(Car_Mode == Distance_Mode)//距离环模式
		{
			Turn_PID_Flag = 0;	
			Distance_PID_Flag = 1;		
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;		
			
			PROTOCOL_TEMP = Measure_Distance;
		}   
		else if(Car_Mode == Gyro_Mode)//角速度环模式
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 1;
			Angle_PID_Flag = 1;	
			
			PROTOCOL_TEMP = Gyro_Z_Measeure;
		}
		else if(Car_Mode == Angle_Mode)//角度环模式
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 0;
			Angle_PID_Flag = 1;
			
			PROTOCOL_TEMP = mpu6050.Yaw;
		}
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &PROTOCOL_TEMP, 4);//向上位机发送数据	
		receiving_process();//对接收数据进行处理
	}
	else//处于Run_Mode下的处理	（不需要上传数据）
	{
		//处于Run_Mode下的程序逻辑运行
		
		
		
		
		
		
		
		
		
		
	}
}
