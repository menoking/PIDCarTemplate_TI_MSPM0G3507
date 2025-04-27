使用前一定要先移植main.h的内容！！！

依赖main.h的文件：
	oled.h


注意事项：
	中断管理：
		//清除定时器中断标志
		NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
		//使能定时器中断
		NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
		//定时器A开始计数
		DL_TimerA_startCounter(TIMER_0_INST);