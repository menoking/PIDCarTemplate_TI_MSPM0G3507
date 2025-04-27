#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

//电机基本参数
#define ENCODE_13X 13 		//编码器线数
#define JIANSUBI 20     	//减速比
#define BEIPIN 8            //倍频
#define SAMPLE_TIME 0.01	//采样时间
#define CC (ENCODE_13X*JIANSUBI*BEIPIN*SAMPLE_TIME)

#define PI  3.1415f
#define RR  42.5f    		//车轮半径单位cm

/*编码器端口读取宏定义*/
#define Read_Encoder_A 	(DL_GPIO_readPins(Encoder_PORT,Encoder_A_PIN)==Encoder_A_PIN)?0:1//左轮 A相
#define Read_Encoder_B  (DL_GPIO_readPins(Encoder_PORT,Encoder_B_PIN)==Encoder_B_PIN)?0:1//左轮 B相
#define Read_Encoder_C 	(DL_GPIO_readPins(Encoder_PORT,Encoder_C_PIN)==Encoder_C_PIN)?0:1//右轮 A相
#define Read_Encoder_D 	(DL_GPIO_readPins(Encoder_PORT,Encoder_D_PIN)==Encoder_D_PIN)?0:1//右轮 B相

extern float Motor1_Speed;
extern float Motor2_Speed;
extern float Measure_Distance;

void Motor1_Get_Speed(void);
void Motor2_Get_Speed(void);
//测量所有电机速度
void MEASURE_MOTORS_SPEED(void);

#endif
