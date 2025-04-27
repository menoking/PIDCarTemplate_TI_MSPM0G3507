#ifndef __KEY_H__
#define __KEY_H__
#include "main.h"
/*-------------------------------------------------------------------------------------------*/
/*-----------------------------------LED和蜂鸣器操作函数-------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define LED1_ON()		DL_GPIO_clearPins(LED_PORT, LED_LED1_PIN)
#define LED1_OFF()		DL_GPIO_setPins(LED_PORT, LED_LED1_PIN)
#define LED1_TOGGLE()	DL_GPIO_togglePins(LED_PORT, LED_LED1_PIN)

#define LED2_ON()		DL_GPIO_clearPins(LED_PORT, LED_LED2_PIN)
#define LED2_OFF()		DL_GPIO_setPins(LED_PORT, LED_LED2_PIN)
#define LED2_TOGGLE()	DL_GPIO_togglePins(LED_PORT, LED_LED2_PIN)

#define LED3_ON()		DL_GPIO_clearPins(LED_PORT, LED_LED3_PIN)
#define LED3_OFF()		DL_GPIO_setPins(LED_PORT, LED_LED3_PIN)/
#define LED3_TOGGLE()	DL_GPIO_togglePins(LED_PORT, LED_LED3_PIN)

#define BEEP_ON()		DL_GPIO_clearPins(BEEP_PORT,BEEP_PIN_0_PIN)
#define BEEP_OFF()		DL_GPIO_setPins(BEEP_PORT,BEEP_PIN_0_PIN)
#define BEEP_Toggle()	DL_GPIO_togglePins(BEEP_PORT,BEEP_PIN_0_PIN)


/*-------------------------------------------------------------------------------------------*/
/*--------------------------------------按键读取宏定义---------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define KEY_Number 3

#define KEY1 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY1_PIN)==KEY_KEY1_PIN)?1:0
#define KEY2 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY2_PIN)==KEY_KEY2_PIN)?1:0
#define KEY3 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY3_PIN)==KEY_KEY3_PIN)?0:1

typedef struct 
{
	bool Down_State;         // 按键当前的按下状态，true 表示按下，false 表示未按下
    bool Short_Flag;         // 短按标志，当检测到短按时置为 true
    bool Double_Flag;        // 双击标志，当检测到双击时置为 true
    bool Long_Flag;          // 长按标志，当检测到长按时置为 true
    bool Double_Time_EN;     // 双击时间使能标志，双击检测期间置为 true
    uint8_t Double_Time;     // 双击计时器，记录双击检测的时间间隔
    uint8_t Down_Time;       // 按键按下时间计时器，记录按键按下的持续时间
    uint8_t Judge_State;     // 按键状态机的当前状态，用于按键状态判断
}KEY;

void Key_Read(void);
extern KEY Key[KEY_Number];
void KEY_PROC(void);


#endif
