/***
	*******************************************************************************************************************************************************************
	* @file    gw_gray.c
	* @version V2.1
	* @date    2024-7-22
	* @author  ����	
	* @brief   MSPM0G3507С��PID����ͨ��ģ��
   *************************************************************************************************
   *  @description
	*	
	*  һ��ʼ��Ϊ�������Ϊ�Ƽ���8·�Ҷȴ�������֮���´�оƬ�������ã���������ͨ�����·�Ҷȴ�����
   *
>>>>> ����˵����δ������������ת�����������״��룬��Ҷ����ҹ�����Ϊ���꣬�뱣�ֺ��Լ��ĳ��ģ��ڴˣ��������ҵĸ�л
	*************************************************************************************************************************************************************
***/
#include "gw_gray.h"
#include "delay.h"

#define Read_Huidu_IO1	 ((DL_GPIO_readPins(Huidu_IN1_PORT, Huidu_IN1_PIN)==Huidu_IN1_PIN)?1:0)
#define Read_Huidu_IO2	 ((DL_GPIO_readPins(Huidu_IN2_PORT, Huidu_IN2_PIN)==Huidu_IN2_PIN)?1:0)
#define Read_Huidu_IO3	 ((DL_GPIO_readPins(Huidu_IN3_PORT, Huidu_IN3_PIN)==Huidu_IN3_PIN)?1:0)

#define Read_Huidu_IO4	 ((DL_GPIO_readPins(Huidu_IN4_PORT, Huidu_IN4_PIN)==Huidu_IN4_PIN)?1:0)

#define Read_Huidu_IO5	 ((DL_GPIO_readPins(Huidu_IN5_PORT, Huidu_IN5_PIN)==Huidu_IN5_PIN)?1:0)
#define Read_Huidu_IO6	 ((DL_GPIO_readPins(Huidu_IN6_PORT, Huidu_IN6_PIN)==Huidu_IN6_PIN)?1:0)
#define Read_Huidu_IO7	 ((DL_GPIO_readPins(Huidu_IN7_PORT, Huidu_IN7_PIN)==Huidu_IN7_PIN)?1:0)

uint8_t Huidu_Datas;

uint8_t Huidu_Read(void)
{
	uint8_t Huidu_Data;
	
	Huidu_Data = (Read_Huidu_IO1<<6)|(Read_Huidu_IO2<<5)|(Read_Huidu_IO3<<4)
				|(Read_Huidu_IO4<<3)|(Read_Huidu_IO5<<2)|(Read_Huidu_IO6<<1)
				|(Read_Huidu_IO7);
	
	return Huidu_Data;
}
float Huidu_Target = 0;
float Huidu_Error;
int Huidu_Sum;
//��Ҫ�߼�����ϸ��Ҫ�Լ���д�߼�������ת90��ת180
float Huidu_Proc(uint8_t huidu_data)
{
	float huidu_error;
	static float huidu_lasterror;
	//---------------ѭ�����ֿ�ʼ--------------------//
	Huidu_Sum = 0;
	for(int i=0;i<7;i++)
	{
		if((huidu_data>>i)&0x01) Huidu_Sum++;
	}
	if(Huidu_Sum<=2)//��Ѳ��
	{
		switch(huidu_data)
		{
			//�����
			case 0x08:// 000 1 000
			{
				Huidu_Error = 0;
			}break;
			//-------------------------
			case 0x18:// 001 1 000
			{
				Huidu_Error = 1;
			}break;
			case 0x10:// 001 0 000
			{
				Huidu_Error = 2;
			}break;
			case 0x30:// 011 0 000
			{
				Huidu_Error = 3;
			}break;
			case 0x20:// 010 0 000
			{
				Huidu_Error = 4;
			}break;
			case 0x60:// 110 0 000
			{
				Huidu_Error = 5;
			}break;
			case 0x40:// 100 0 000
			{
				Huidu_Error = 6;
			}break;
			//-------------------------
			case 0x0c:// 000 1 100
			{
				Huidu_Error = -1;
			}break;
			case 0x04:// 000 0 100
			{
				Huidu_Error = -2;
			}break;
			case 0x06:// 000 0 110
			{
				Huidu_Error = -3;
			}break;
			case 0x02:// 000 0 010
			{
				Huidu_Error = -4;
			}break;
			case 0x03:// 000 0 011
			{
				Huidu_Error = -5;
			}break;
			case 0x01:// 000 0 001
			{
				Huidu_Error = -6;
			}break;
			default://���û�з��ϵľͱ�����һ�ε����
			{
				Huidu_Error = huidu_lasterror;
			}break;
		}
	}
	else if(Huidu_Sum>=3&&Huidu_Sum<=5)//תֱ����
	{
		Huidu_Error=0;
	}
	else//ͣ����ת180
	{Huidu_Error=0;
		//Basic_Speed = 0;
	}
	
	huidu_error = Huidu_Error;
	//��¼��ǰ���
	huidu_lasterror = huidu_error;
	return huidu_error;
}



