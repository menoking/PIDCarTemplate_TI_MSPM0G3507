/***
	*******************************************************************************************************************************************************************
	* @file    gw_gray.c
	* @version V2.1
	* @date    2024-7-22
	* @author  御龙	
	* @brief   MSPM0G3507小车PID调试通用模板
   *************************************************************************************************
   *  @description
	*	
	*  一开始是为了适配感为科技的8路灰度传感器，之后害怕带芯片，不能用，改用了普通款的七路灰度传感器
   *
>>>>> 其他说明：未经允许不可擅自转发、售卖本套代码，大家都是我国的有为青年，请保持好自己的初心，在此，向你表达我的感谢
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
//重要逻辑，详细需要自己编写逻辑，例如转90和转180
float Huidu_Proc(uint8_t huidu_data)
{
	float huidu_error;
	static float huidu_lasterror;
	//---------------循迹部分开始--------------------//
	Huidu_Sum = 0;
	for(int i=0;i<7;i++)
	{
		if((huidu_data>>i)&0x01) Huidu_Sum++;
	}
	if(Huidu_Sum<=2)//在巡线
	{
		switch(huidu_data)
		{
			//无误差
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
			default://如果没有符合的就保持上一次的误差
			{
				Huidu_Error = huidu_lasterror;
			}break;
		}
	}
	else if(Huidu_Sum>=3&&Huidu_Sum<=5)//转直角弯
	{
		Huidu_Error=0;
	}
	else//停车或转180
	{Huidu_Error=0;
		//Basic_Speed = 0;
	}
	
	huidu_error = Huidu_Error;
	//记录当前误差
	huidu_lasterror = huidu_error;
	return huidu_error;
}



