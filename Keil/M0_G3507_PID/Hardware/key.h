#ifndef __KEY_H__
#define __KEY_H__
#include "main.h"
/*-------------------------------------------------------------------------------------------*/
/*-----------------------------------LED�ͷ�������������-------------------------------------*/
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
/*--------------------------------------������ȡ�궨��---------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
#define KEY_Number 3

#define KEY1 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY1_PIN)==KEY_KEY1_PIN)?1:0
#define KEY2 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY2_PIN)==KEY_KEY2_PIN)?1:0
#define KEY3 			(DL_GPIO_readPins(KEY_PORT,KEY_KEY3_PIN)==KEY_KEY3_PIN)?0:1

typedef struct 
{
	bool Down_State;         // ������ǰ�İ���״̬��true ��ʾ���£�false ��ʾδ����
    bool Short_Flag;         // �̰���־������⵽�̰�ʱ��Ϊ true
    bool Double_Flag;        // ˫����־������⵽˫��ʱ��Ϊ true
    bool Long_Flag;          // ������־������⵽����ʱ��Ϊ true
    bool Double_Time_EN;     // ˫��ʱ��ʹ�ܱ�־��˫������ڼ���Ϊ true
    uint8_t Double_Time;     // ˫����ʱ������¼˫������ʱ����
    uint8_t Down_Time;       // ��������ʱ���ʱ������¼�������µĳ���ʱ��
    uint8_t Judge_State;     // ����״̬���ĵ�ǰ״̬�����ڰ���״̬�ж�
}KEY;

void Key_Read(void);
extern KEY Key[KEY_Number];
void KEY_PROC(void);


#endif
