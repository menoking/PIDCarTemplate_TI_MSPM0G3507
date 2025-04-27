#ifndef __GW_GRAY_H__
#define __GW_GRAY_H__

#include "main.h"

uint8_t Huidu_Read(void);
float Huidu_Proc(uint8_t huidu_data);




extern float Huidu_Target;
extern uint8_t Huidu_Datas;
extern float Huidu_Error;
extern int Huidu_Sum;

#endif 
