/*
����������ֱ��ע�ͺͽ�ע�ͼ���
ʹ�ñ�����ʱע������һ����ʱ������ɨ��
�ڹ��ܺ����ڲ����ô�����
*/

//ʹ����������ʱҪ���������ж�
/*	
//�����ʱ���жϱ�־
NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
//ʹ�ܶ�ʱ���ж�
NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
//��ʱ��A��ʼ����
DL_TimerA_startCounter(TIMER_0_INST);
*/

#include "key.h"
//�ṹ������
KEY Key[KEY_Number];
/*-------------------------------------------------------------------------------------------*/
/*-----------------------������ȡ��������Ҫ����10ms�ж��ڽ���ɨ�裩--------------------------*/
/*-------------------------------------------------------------------------------------------*/
void Key_Read(void){
	//��Ӷ�ȡ����
	Key[0].Down_State=KEY1;//Ĭ������Ϊ0
	// Key[1].Down_State=KEY2;
	// Key[2].Down_State=KEY3;
	
	for(int i=0;i<KEY_Number;i++){
		switch(Key[i].Judge_State){
			case 0:{if(Key[i].Down_State==0){Key[i].Judge_State=1;Key[i].Down_Time=0;}else{Key[i].Judge_State=0;}}break;
			case 1:{if(Key[i].Down_State==0){Key[i].Judge_State=2;}else{Key[i].Judge_State=0;}}break;
			case 2:{if((Key[i].Down_State==1)&&(Key[i].Down_Time<=70)){if(Key[i].Double_Time_EN==0){Key[i].Double_Time_EN=1;Key[i].Double_Time=0;}else{Key[i].Double_Flag=1;Key[i].Double_Time_EN=0;}Key[i].Judge_State=0;}else if((Key[i].Down_State==1)&&(Key[i].Down_Time>70)) Key[i].Judge_State=0;else{if(Key[i].Down_Time>70) Key[i].Long_Flag=1;Key[i].Down_Time++;}}break;
		}
		if(Key[i].Double_Time_EN==1){Key[i].Double_Time++;if(Key[i].Double_Time>=35){Key[i].Short_Flag=1;Key[i].Double_Time_EN=0;}}
	}
}
/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------����������------------------------------------------*/
//�ʵ����ͷ�ļ�
#include "oled.h"
/*-------------------------------------------------------------------------------------------*/
void KEY_PROC(void)
{
	/*��������*/
	if(Key[0].Short_Flag==1)
	{Key[0].Short_Flag=0;
		static uint8_t count=0;
        count++;
        OLED_ShowNum(0,0,count,2,16,1);
        OLED_Update();
	}
	// else if(Key[1].Short_Flag==1)
	// {Key[1].Short_Flag=0;
		
		
	// }
	// else if(Key[2].Short_Flag==1)
	// {Key[2].Short_Flag=0;
		
		
	//}
	/*˫������*/       //������ʱ��û���õ�����Ҫ����������Ӵ�������
	if(Key[0].Double_Flag==1)
	{Key[0].Double_Flag=0;
		
	}
	// else if(Key[1].Double_Flag==1)
	// {Key[1].Double_Flag=0;
		
	// }
	// else if(Key[2].Double_Flag==1)
	// {Key[2].Double_Flag=0;
		
	//}
	/*��������*/
	if(Key[0].Long_Flag==1)
	{Key[0].Long_Flag=0;
		
	}
	// else if(Key[1].Long_Flag==1)
	// {Key[1].Long_Flag=0;
		
	// }
	// else if(Key[2].Long_Flag==1)
	// {Key[2].Long_Flag=0;
		
	//}
}
