/*
按键的增减直接注释和解注释即可
使用本按键时注意申请一个定时器进行扫描
在功能函数内部调用处理函数
*/

//使用御龙代码时要开启以下中断
/*	
//清除定时器中断标志
NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
//使能定时器中断
NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
//定时器A开始计数
DL_TimerA_startCounter(TIMER_0_INST);
*/

#include "key.h"
//结构体声明
KEY Key[KEY_Number];
/*-------------------------------------------------------------------------------------------*/
/*-----------------------按键读取函数（需要放在10ms中断内进行扫描）--------------------------*/
/*-------------------------------------------------------------------------------------------*/
void Key_Read(void){
	//添加读取按键
	Key[0].Down_State=KEY1;//默认摁下为0
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
/*-------------------------------------按键处理函数------------------------------------------*/
//适当添加头文件
#include "oled.h"
/*-------------------------------------------------------------------------------------------*/
void KEY_PROC(void)
{
	/*短摁处理*/
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
	/*双击处理*/       //下面暂时都没有用到，需要可以自行添加处理任务）
	if(Key[0].Double_Flag==1)
	{Key[0].Double_Flag=0;
		
	}
	// else if(Key[1].Double_Flag==1)
	// {Key[1].Double_Flag=0;
		
	// }
	// else if(Key[2].Double_Flag==1)
	// {Key[2].Double_Flag=0;
		
	//}
	/*长摁处理*/
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
