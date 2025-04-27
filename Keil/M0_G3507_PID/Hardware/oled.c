/***
	*******************************************************************************************************************************************************************
	* @file    oled.c
	* @version V2.1
	* @date    2024-7-22
	* @author  御龙	
	* @brief   MSPM0G3507小车PID调试通用模板
   *************************************************************************************************
   *  @description
	*	
	*  接口配置可以使用Sysconfig看
	*  本套OLED文件借鉴了很多其他大佬的程序，特别是祖师“江科大”，在此班门弄斧了
	*  本人重新写了规范的底层函数，使M0上出现的OLED奇怪的问题并不会出现，请放心
   *
>>>>> 其他说明：未经允许不可擅自转发、售卖本套代码，大家都是我国的有为青年，请保持好自己的初心，在此，向你表达我的感谢
	*************************************************************************************************************************************************************
***/

#include "oled.h"
#include "oledfont.h"
#include "delay.h"

/*-------------------------------------------------------------------------------------------*/
/*-----------------------------------IIC底层操作函数-----------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
static void I2C_Delay(void){
	delay_us(2);
}
//起始信号
static void I2C_Start(void){
	OLED_SDA_Set();
	OLED_SCL_Set();
	I2C_Delay();
	OLED_SDA_Clr();
	I2C_Delay();
	OLED_SCL_Clr();
}
//结束信号
static void I2C_Stop(void){
	OLED_SCL_Clr();
	OLED_SDA_Clr();
	I2C_Delay();
	OLED_SCL_Set();
	I2C_Delay();
	OLED_SDA_Set();
	I2C_Delay();
}
//发送应答信号
static void I2C_ACK(void){
	OLED_SCL_Clr();
	OLED_SDA_Clr();//在SCL为0时准备好数据
	I2C_Delay();
	OLED_SCL_Set(); //SCL的上升沿发送一位数据
	I2C_Delay();//在SCL高电平期间保持不变
	OLED_SCL_Clr();
}
 
//发送非应答信号
static void I2C_NACK(void){
	OLED_SCL_Clr();
	OLED_SDA_Set(); //在SCL为0时准备好数据
	I2C_Delay();
	OLED_SCL_Set();//SCL的上升沿发送一位数据
	I2C_Delay(); //在SCL高电平期间保持不变
	OLED_SCL_Clr();
}
//等待信号响应
static uint8_t I2C_WaitAck(void){
	OLED_SCL_Clr();
	OLED_SDA_Set();
	I2C_Delay();
	OLED_SCL_Set();
	I2C_Delay();
	OLED_SDA_IN();
	if(OLED_READ_SDA()==1)
	{
		OLED_SDA_OUT();I2C_Stop();return 1;
	}		
	else
	{
		OLED_SDA_OUT();OLED_SCL_Clr();return 0; 
	}
}
//写入一个字节
static void I2C_Send_Byte(uint8_t dat){
	uint8_t i;
	OLED_SCL_Clr();
	for(i=0;i<8;i++)
	{
		OLED_SCL_Clr();//将时钟信号设置为低电平
		if(dat&0x80)//将dat的8位从最高位依次写入
		{
			OLED_SDA_Set();
		}
		else
		{
			OLED_SDA_Clr();
		}
		dat<<=1;
		I2C_Delay();
		OLED_SCL_Set();
		I2C_Delay();
		OLED_SCL_Clr();//将时钟信号设置为低电平
		
	}
}
//读取一个字节
static uint8_t I2C_Read_Byte(void){
	uint8_t i;uint8_t receive=0;
	for(i=0;i<8;i++)
	{
		OLED_SCL_Clr();
		I2C_Delay();
		OLED_SDA_Set();
		OLED_SCL_Set();
		receive<<=1;
		OLED_SDA_IN();
		if(OLED_READ_SDA()==1) {receive++;OLED_SDA_OUT();}
	}
	return receive;
}
//模拟完整发送一个字节数据时序 write_addr:设备地址  write_reg:寄存器地址  write_data:要写入的数据
static void IIC_SendOneByte(u8 write_addr,u8 write_reg,u8 write_data){
	I2C_Start();
	I2C_Send_Byte((write_addr<<1)|0);
	I2C_WaitAck();
	I2C_Send_Byte(write_reg);
	I2C_WaitAck();
	I2C_Send_Byte(write_data);
	I2C_WaitAck();
	I2C_Stop();
}
//IIC连续发送数据 addr:设备地址  reg:寄存器地址  len:写入长度  buf:数据区 返回值:0,正常；其他,错误代码  
static u8 IIC_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf){
	u8 i; 
    I2C_Start(); 
	I2C_Send_Byte((addr<<1)|0);//发送设备地址+写命令	
    I2C_WaitAck();
    I2C_Send_Byte(reg);	//写寄存器地址
    I2C_WaitAck();		//等待应答
	for(i=0;i<len;i++)
	{
		I2C_Send_Byte(buf[i]);	//发送数据
		I2C_WaitAck();		
	}    
    I2C_Stop();	 
	return 0;	
}
//模拟完整读取一个数据时序 read_addr：设备地址  read_reg：寄存器地址
static u8 IIC_ReadOneByte(u8 read_addr,u8 read_reg){
	u8 temp;
	I2C_Start();
    I2C_Send_Byte((read_addr<<1)|0);
	I2C_WaitAck();
	I2C_Send_Byte(read_reg);
	I2C_WaitAck();
	I2C_Start();
	I2C_Send_Byte((read_addr<<1)|1);
	I2C_WaitAck();
	temp=I2C_Read_Byte();
	I2C_NACK();
	I2C_Stop();
	return temp;
}
//IIC连续读取数据  addr:设备地址  reg:要读取的寄存器地址  len:要读取的长度  buf:读取到的数据存储区 返回值:0,正常；其他,错误代码
static u8 IIC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){ 
 	I2C_Start(); 
	I2C_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
    I2C_WaitAck();
    I2C_Send_Byte(reg);	//写寄存器地址
    I2C_WaitAck();		//等待应答
    I2C_Start();
	I2C_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    I2C_WaitAck();		//等待应答 
	while(len)
	{
		if(len==1)
        {
            *buf=I2C_Read_Byte();
            I2C_NACK();//读数据,发送nACK 
        }
            
		else 
        {
            *buf=I2C_Read_Byte();		
            I2C_NACK();//读数据,发送ACK
        }
  
		len--;
		buf++; 
	}    
    I2C_Stop();	//产生一个停止条件 
	return 0;	
}


/*-------------------------------------------------------------------------------------------*/
/*------------------------------------OLED操作函数------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
uint8_t OLED_GRAM[144][8];
//反显函数
void OLED_ColorTurn(uint8_t i){
	if(i==0)OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
	if(i==1)OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
}
//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i){
	if(i==0){
		OLED_WR_Byte(0xC8,OLED_CMD);OLED_WR_Byte(0xA1,OLED_CMD);//正常显示
	}
	else if(i==1){
		OLED_WR_Byte(0xC0,OLED_CMD);OLED_WR_Byte(0xA0,OLED_CMD);//反转显示
		}
}


//发送一个字节 mode:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(uint8_t dat,uint8_t mode){
	I2C_Start();
	I2C_Send_Byte(0x78);
	I2C_WaitAck();
	if(mode){I2C_Send_Byte(0x40);}
    else{I2C_Send_Byte(0x00);}
	I2C_WaitAck();
	I2C_Send_Byte(dat);
	I2C_WaitAck();
	I2C_Stop();
}
//OLED写数据
void OLED_WriteData(uint8_t *Data, uint8_t Count){
	uint8_t i;
	
	I2C_Start();				//I2C起始
	I2C_Send_Byte(0x78);		//发送OLED的I2C从机地址
	I2C_Send_Byte(0x40);		//控制字节，给0x40，表示即将写数量
	/*循环Count次，进行连续的数据写入*/
	for (i = 0; i < Count; i ++)
	{
		I2C_Send_Byte(Data[i]);	//依次发送Data的每一个数据
	}
	I2C_Stop();				//I2C终止
}
//开启OLED显示 
void OLED_DisPlay_On(void){
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//点亮屏幕
}
//关闭OLED显示 
void OLED_DisPlay_Off(void){
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭屏幕
}
//更新显存到OLED	
void OLED_Refresh(void){
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte(0xb0+i,OLED_CMD); //设置行起始地址
		OLED_WR_Byte(0x00,OLED_CMD);   //设置低列起始地址
		OLED_WR_Byte(0x10,OLED_CMD);   //设置高列起始地址
		I2C_Start();
		I2C_Send_Byte(0x78);
		I2C_WaitAck();
		I2C_Send_Byte(0x40);
		I2C_WaitAck();
		for(n=0;n<128;n++)
		{
			I2C_Send_Byte(OLED_GRAM[n][i]);
			I2C_WaitAck();
		}
		I2C_Stop();
	}
}
//清屏函数
void OLED_Clear(void){
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		for(n=0;n<128;n++)
		{
			OLED_GRAM[n][i]=0;//清除所有数据
		}
	}
	OLED_Refresh();//更新显示
}
//画点 x:0~127 y:0~63 t:1 填充 0,清空	
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t){
	if (x > 127){return;}if (y > 63){return;}
	uint8_t i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	if(t){OLED_GRAM[x][i]|=n;}
	else
	{
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
		OLED_GRAM[x][i]|=n;
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	}
}
//画线 X0,Y0:起点坐标 X1,Y1:结束坐标
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1){
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
	uint8_t yflag = 0, xyflag = 0;
	
	if (y0 == y1)		//横线单独处理
	{
		/*0号点X坐标大于1号点X坐标，则交换两点X坐标*/
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}
		
		/*遍历X坐标*/
		for (x = x0; x <= x1; x ++)
		{
			OLED_DrawPoint(x, y0,1);	//依次画点
		}
	}
	else if (x0 == x1)	//竖线单独处理
	{
		/*0号点Y坐标大于1号点Y坐标，则交换两点Y坐标*/
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}
		
		/*遍历Y坐标*/
		for (y = y0; y <= y1; y ++)
		{
			OLED_DrawPoint(x0, y,1);	//依次画点
		}
	}
	else				//斜线
	{
		/*使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高*/
		/*参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf*/
		/*参考教程：https://www.bilibili.com/video/BV1364y1d7Lo*/
		
		if (x0 > x1)	//0号点X坐标大于1号点X坐标
		{
			/*交换两点坐标*/
			/*交换后不影响画线，但是画线方向由第一、二、三、四象限变为第一、四象限*/
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}
		
		if (y0 > y1)	//0号点Y坐标大于1号点Y坐标
		{
			/*将Y坐标取负*/
			/*取负后影响画线，但是画线方向由第一、四象限变为第一象限*/
			y0 = -y0;
			y1 = -y1;
			
			/*置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			yflag = 1;
		}
		
		if (y1 - y0 > x1 - x0)	//画线斜率大于1
		{
			/*将X坐标与Y坐标互换*/
			/*互换后影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围*/
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;
			
			/*置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			xyflag = 1;
		}
		
		/*以下为Bresenham算法画直线*/
		/*算法要求，画线方向必须为第一象限0~45度范围*/
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;
		
		/*画起始点，同时判断标志位，将坐标换回来*/
		if (yflag && xyflag){OLED_DrawPoint(y, -x,1);}
		else if (yflag)		{OLED_DrawPoint(x, -y,1);}
		else if (xyflag)	{OLED_DrawPoint(y, x,1);}
		else				{OLED_DrawPoint(x, y,1);}
		
		while (x < x1)		//遍历X轴的每个点
		{
			x ++;
			if (d < 0)		//下一个点在当前点东方
			{
				d += incrE;
			}
			else			//下一个点在当前点东北方
			{
				y ++;
				d += incrNE;
			}
			
			/*画每一个点，同时判断标志位，将坐标换回来*/
			if (yflag && xyflag){OLED_DrawPoint(y, -x,1);}
			else if (yflag)		{OLED_DrawPoint(x, -y,1);}
			else if (xyflag)	{OLED_DrawPoint(y, x,1);}
			else				{OLED_DrawPoint(x, y,1);}
		}	
	}
}
//画矩形
//X矩形左上角的横坐标0~127 Y矩形左上角的纵坐标0~63  Width矩形宽度0~128  Height矩形高度0~64  IsFilled是否填充 0不填充1填充 
void OLED_DrawRectangle(uint8_t X,uint8_t Y,uint8_t Width, uint8_t Height,uint8_t IsFilled){
	uint8_t i, j;
	if (!IsFilled)		//指定矩形不填充
	{
		/*遍历上下X坐标，画矩形上下两条线*/
		for (i = X; i < X + Width; i ++)
		{
			OLED_DrawPoint(i, Y,1);
			OLED_DrawPoint(i, Y + Height - 1,1);
		}
		/*遍历左右Y坐标，画矩形左右两条线*/
		for (i = Y; i < Y + Height; i ++)
		{
			OLED_DrawPoint(X, i,1);
			OLED_DrawPoint(X + Width - 1, i,1);
		}
	}
	else				//指定矩形填充
	{
		/*遍历X坐标*/
		for (i = X; i < X + Width; i ++)
		{
			/*遍历Y坐标*/
			for (j = Y; j < Y + Height; j ++)
			{
				/*在指定区域画点，填充满矩形*/
				OLED_DrawPoint(i, j,1);
			}
		}
	}
}
//画圆
//X圆的圆心横坐标0~127 Y圆的圆心纵坐标0~63 Radius圆的半径0~255 IsFilled是否填充 0不填充1填充 
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t IsFilled){
	int16_t x, y, d, j;
	
	/*使用Bresenham算法画圆，可以避免耗时的浮点运算，效率更高*/
	/*参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf*/
	/*参考教程：https://www.bilibili.com/video/BV1VM4y1u7wJ*/
	
	d = 1 - Radius;
	x = 0;
	y = Radius;
	
	/*画每个八分之一圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y,1);
	OLED_DrawPoint(X - x, Y - y,1);
	OLED_DrawPoint(X + y, Y + x,1);
	OLED_DrawPoint(X - y, Y - x,1);
	
	if (IsFilled)		//指定圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在指定区域画点，填充部分圆*/
			OLED_DrawPoint(X, Y + j,1);
		}
	}
	
	while (x < y)		//遍历X轴的每个点
	{
		x ++;
		if (d < 0)		//下一个点在当前点东方
		{
			d += 2 * x + 1;
		}
		else			//下一个点在当前点东南方
		{
			y --;
			d += 2 * (x - y) + 1;
		}
		
		/*画每个八分之一圆弧的点*/
		OLED_DrawPoint(X + x, Y + y,1);
		OLED_DrawPoint(X + y, Y + x,1);
		OLED_DrawPoint(X - x, Y - y,1);
		OLED_DrawPoint(X - y, Y - x,1);
		OLED_DrawPoint(X + x, Y - y,1);
		OLED_DrawPoint(X + y, Y - x,1);
		OLED_DrawPoint(X - x, Y + y,1);
		OLED_DrawPoint(X - y, Y + x,1);
		
		if (IsFilled)	//指定圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分圆*/
				OLED_DrawPoint(X + x, Y + j,1);
				OLED_DrawPoint(X - x, Y + j,1);
			}
			
			/*遍历两侧部分*/
			for (j = -x; j < x; j ++)
			{
				/*在指定区域画点，填充部分圆*/
				OLED_DrawPoint(X - y, Y + j,1);
				OLED_DrawPoint(X + y, Y + j,1);
			}
		}
	}
}
//画椭圆
//X椭圆的圆心横坐标0~127  Y椭圆的圆心纵坐标0~63  A椭圆横向半轴长度0~255  B椭圆纵向半轴长度0~255 IsFilled是否填充 0不填充1填充 
void OLED_DrawEllipse(uint8_t X, uint8_t Y, uint8_t A, uint8_t B, uint8_t IsFilled){
	int16_t x, y, j;
	int16_t a = A, b = B;
	float d1, d2;
	
	/*使用Bresenham算法画椭圆，可以避免部分耗时的浮点运算，效率更高*/
	/*参考链接：https://blog.csdn.net/myf_666/article/details/128167392*/
	
	x = 0;
	y = b;
	d1 = b * b + a * a * (-b + 0.5);
	
	if (IsFilled)	//指定椭圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在指定区域画点，填充部分椭圆*/
			OLED_DrawPoint(X, Y + j,1);
			OLED_DrawPoint(X, Y + j,1);
		}
	}
	
	/*画椭圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y,1);
	OLED_DrawPoint(X - x, Y - y,1);
	OLED_DrawPoint(X - x, Y + y,1);
	OLED_DrawPoint(X + x, Y - y,1);
	
	/*画椭圆中间部分*/
	while (b * b * (x + 1) < a * a * (y - 0.5))
	{
		if (d1 <= 0)		//下一个点在当前点东方
		{
			d1 += b * b * (2 * x + 3);
		}
		else				//下一个点在当前点东南方
		{
			d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
			y --;
		}
		x ++;
		
		if (IsFilled)	//指定椭圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分椭圆*/
				OLED_DrawPoint(X + x, Y + j,1);
				OLED_DrawPoint(X - x, Y + j,1);
			}
		}
		
		/*画椭圆中间部分圆弧*/
		OLED_DrawPoint(X + x, Y + y,1);
		OLED_DrawPoint(X - x, Y - y,1);
		OLED_DrawPoint(X - x, Y + y,1);
		OLED_DrawPoint(X + x, Y - y,1);
	}
	
	/*画椭圆两侧部分*/
	d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;
	
	while (y > 0)
	{
		if (d2 <= 0)		//下一个点在当前点东方
		{
			d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
			x ++;
			
		}
		else				//下一个点在当前点东南方
		{
			d2 += a * a * (-2 * y + 3);
		}
		y --;
		
		if (IsFilled)	//指定椭圆填充
		{
			/*遍历两侧部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分椭圆*/
				OLED_DrawPoint(X + x, Y + j,1);
				OLED_DrawPoint(X - x, Y + j,1);
			}
		}
		
		/*画椭圆两侧部分圆弧*/
		OLED_DrawPoint(X + x, Y + y,1);
		OLED_DrawPoint(X - x, Y - y,1);
		OLED_DrawPoint(X - x, Y + y,1);
		OLED_DrawPoint(X + x, Y - y,1);
	}
}
//在指定位置显示一个字符,包括部分字符
//x:0~127  y:0~63  chr:显示字符 size1:选择字体  6x8/6x12/8x16/12x24  mode:0,反色显示;1,正常显示
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode){
	uint8_t i,m,temp,size2,chr1;
	uint8_t x0=x,y0=y;
	if(size1==8)size2=6;
	else size2=(size1/8+((size1%8)?1:0))*(size1/2);  //得到字体一个字符对应点阵集所占的字节数
	chr1=chr-' ';  //计算偏移后的值
	for(i=0;i<size2;i++)
	{
		if(size1==16)
        {temp=OLED_F8x16[chr1][i];} //调用1608字体
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((size1!=8)&&((x-x0)==size1/2)){x=x0;y0=y0+8;}
		y=y0;
  }
}
//显示字符串
//x,y:起点坐标  size1:字体大小 *chr:字符串起始地址 mode:0,反色显示;1,正常显示
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode){
	while((*chr>=' ')&&(*chr<='~'))//判断是不是非法字符!
	{
		OLED_ShowChar(x,y,*chr,size1,mode);
		if(size1==8)x+=6;
		else x+=size1/2;
		chr++;
  }
}
//m^n
uint32_t OLED_Pow(uint8_t m,uint8_t n){
	uint32_t result=1;
	while(n--)
	{
	  result*=m;
	}
	return result;
}
//显示正整数数字 x,y :起点坐标 num :要显示的数字 len :数字的位数 size:字体大小 mode:0,反色显示;1,正常显示
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode){
	uint8_t t,temp,m=0;
	if(size1==8)m=2;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0)
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);
		}
		else 
		{
			OLED_ShowChar(x+(size1/2+m)*t,y,temp+'0',size1,mode);
		}
	}
}
//显示有符号整数 x,y :起点坐标 num :要显示的数字 len :数字的位数 size:字体大小 mode:0,反色显示;1,正常显示
void OLED_ShowSignedNum(uint8_t x,uint8_t y,int32_t num,uint8_t len,uint8_t size1,uint8_t mode){
	uint8_t m=0;if(size1==8)m=2;
	if (num > 0) {OLED_ShowChar(x,y,'+',size1,mode);num = num;}
	else if(num == 0) {OLED_ShowChar(x,y,' ',size1,mode);num = num;}
	else {OLED_ShowChar(x,y,'-',size1,mode);num = -num;}
	OLED_ShowNum(x+size1/2+m,y,num,len,size1,mode);
}
//显示十六进制函数 x,y :起点坐标 num :要显示的十六进制数字 len :数字的位数 size:字体大小 mode:0,反色显示;1,正常显示
void OLED_ShowHexNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode){
	uint8_t i, SingleNumber;
	uint8_t m=0;if(size1==8)m=2;
	OLED_ShowChar(x,y,'0',size1,mode);OLED_ShowChar(x+size1/2+m,y,'X',size1,mode);
	for (i = 0; i < len; i++)							
	{
		SingleNumber = num / OLED_Pow(16, len - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(x+(i+2)*(size1/2+m),y, SingleNumber + '0',size1,mode);
		}
		else
		{
			OLED_ShowChar(x+(i+2)*(size1/2+m),y,SingleNumber - 10 + 'A',size1,mode);
		}
	}
}
//显示二进制函数 x,y :起点坐标 num :要显示的二进制数字 len :数字的位数 size:字体大小 mode:0,反色显示;1,正常显示
void OLED_ShowBinNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode){
	uint8_t i;
	uint8_t m=0;if(size1==8)m=2;
	for (i = 0; i < len; i++)		//遍历数字的每一位	
	{
		OLED_ShowChar(x+i*(size1/2+m), y, num / OLED_Pow(2, len - i - 1) % 2 + '0', size1,mode);
	}
}
//显示浮点型数据 x,y :起点坐标 num :要显示的浮点型数字 intlen :整数位数 fralen 小数位数 size:字体大小 mode:0,反色显示;1,正常显示
void OLED_ShowFloatNum(uint8_t x,uint8_t y,double num,uint8_t intlen,uint8_t fralen,uint8_t size1,uint8_t mode){
	uint32_t PowNum, IntNum, FraNum;
	
	uint8_t m=0;if(size1==8)m=2;
	if (num > 0) {OLED_ShowChar(x,y,'+',size1,mode);num = num;}
	else if(num == 0) {OLED_ShowChar(x,y,' ',size1,mode);num = num;}
	else {OLED_ShowChar(x,y,'-',size1,mode);num = -num;}
	
	/*提取整数部分和小数部分*/
	IntNum = num;						//直接赋值给整型变量，提取整数
	num -= IntNum;						//将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	PowNum = OLED_Pow(10, fralen);		//根据指定小数的位数，确定乘数
	FraNum = round(num * PowNum);		//将小数乘到整数，同时四舍五入，避免显示误差
	IntNum += FraNum / PowNum;				//若四舍五入造成了进位，则需要再加给整数
	
	/*显示整数部分*/
	OLED_ShowNum(x+size1/2+m,y,IntNum,intlen,size1,mode);
	
	/*显示小数点*/
	OLED_ShowChar(x+(size1/2+m)*(intlen+1),y, '.', size1,mode);
	
	/*显示小数部分*/
	OLED_ShowNum(x+(size1/2+m)*(intlen+2),y, FraNum, fralen, size1,mode);
}
//显示汉字
//x,y:起点坐标//num:汉字对应的序号 mode:0,反色显示;1,正常显示
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode){
	uint8_t m,temp;
	uint8_t x0=x,y0=y;
	uint16_t i,size3=(size1/8+((size1%8)?1:0))*size1;  //得到字体一个字符对应点阵集所占的字节数
	for(i=0;i<size3;i++)
	{
//		if(size1==16)
//				//{temp=Hzk1[num][i];}//调用16*16字体
//		else if(size1==24)
//				//{temp=Hzk2[num][i];}//调用24*24字体
//		else if(size1==32)       
//				//{temp=Hzk3[num][i];}//调用32*32字体
//		else if(size1==64)
//				//{temp=Hzk4[num][i];}//调用64*64字体
//		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((x-x0)==size1){x=x0;y0=y0+8;}
		y=y0;
	}
}
//num 显示汉字的个数 space 每一遍显示的间隔 mode:0,反色显示;1,正常显示
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode){
	uint8_t i,n,t=0,m=0,r;
	while(1)
	{
		if(m==0)
		{
			OLED_ShowChinese(128,24,t,16,mode); //写入一个汉字保存在OLED_GRAM[][]数组中
			t++;
		}
		if(t==num)
		{
			for(r=0;r<16*space;r++)      //显示间隔
			{
				for(i=1;i<144;i++)
				{
					for(n=0;n<8;n++)
					{
						OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
					}
				}
				OLED_Refresh();
			}
			t=0;
		}
		m++;
		if(m==16){m=0;}
		for(i=1;i<144;i++)   //实现左移
		{
			for(n=0;n<8;n++)
			{
				OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
	}
}
//显示图片
//x,y：起点坐标 sizex,sizey,图片长宽 BMP[]：要写入的图片数组 mode:0,反色显示;1,正常显示
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode){
	uint16_t j=0;
	uint8_t i,n,temp,m;
	uint8_t x0=x,y0=y;
	sizey=sizey/8+((sizey%8)?1:0);
	for(n=0;n<sizey;n++)
	{
		 for(i=0;i<sizex;i++)
		 {
				temp=BMP[j];
				j++;
				for(m=0;m<8;m++)
				{
					if(temp&0x01)OLED_DrawPoint(x,y,mode);
					else OLED_DrawPoint(x,y,!mode);
					temp>>=1;
					y++;
				}
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y0=y0+8;
				}
				y=y0;
     }
	 }
}
/**
  * 函    数：OLED使用printf函数打印格式化字符串
  * 参    数：X 指定格式化字符串左上角的横坐标，范围：0~127
  * 参    数：Y 指定格式化字符串左上角的纵坐标，范围：0~63
  * 参    数：FontSize 指定字体大小
  *           范围：OLED_8X16		宽8像素，高16像素
  *                 OLED_6X8		宽6像素，高8像素
  * 参    数：format 指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
  * 参    数：... 格式化字符串参数列表
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...){
	char String[30];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							//结束变量arg
	OLED_ShowString(X, Y, (uint8_t *)String, FontSize,1);//OLED显示字符数组（字符串）
}
//OLED端口定义函数
void OLED_GPIO_Init(void){
	
	uint32_t i, j;
	/*在初始化前，加入适量延时，待OLED供电稳定*/
	for (i = 0; i < 1000; i ++)
	{
		for (j = 0; j < 1000; j ++);
	}
	
}
//OLED的初始化
void OLED_Init(void){
	OLED_GPIO_Init();//端口初始化
	/*写入一系列的命令，对OLED进行初始化配置*/
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD);// Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset        Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD);
	
	OLED_ColorTurn(0);//0正常显示，1 反色显示
	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示	
	OLED_Clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////用户自添加函数/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * 函    数：将OLED显存数组更新到OLED屏幕
  * 参    数：无
  * 返 回 值：无
  * 说    明：所有的显示函数，都只是对OLED显存数组进行读写
  *           随后调用OLED_Update函数或OLED_UpdateArea函数
  *           才会将显存数组的数据发送到OLED硬件，进行显示
  *           故调用显示函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */
void OLED_Update(void){
	OLED_Refresh();
}
#define Show_Size 16
//显示字符串函数，例子：OLED_Show_String(1,1,string);表示在第一行第一列显示字符串string
void OLED_Show_String(uint8_t han,uint8_t lie,uint8_t *string){
	OLED_ShowString((lie-1)*8,(han-1)*(Show_Size),string,Show_Size,1);
	OLED_Refresh();
}
void OLED_DisplayLine(uint8_t han,uint8_t lie,uint8_t *temp){
	OLED_Show_String(han,lie,temp);
}

//--------------------------------------------------------------------------------------//
//                                用户操作区域                                          //
//--------------------------------------------------------------------------------------//

