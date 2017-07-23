#include "camera_datatransfer.h"
#include "usart.h"
#include "anotc_baro_ctrl.h"
#include "camera_data_calculate.h"

/*

高度数据：

ultra.relative_height					cm		float
sonar.displacement						mm		float
sonar_fusion.fusion_displacement.out	mm		float

*/

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


//=========================================================================================================================
//====================================================发送数据==============================================================
//=========================================================================================================================
//=========================================================================================================================

unsigned char Data_Buffer[20];

//发送数据接口
void Send_to_Camera(unsigned char *DataToSend ,u8 data_num)
{
	Usart3_Send(DataToSend,data_num);
}

//发送高度数据
void Copter_Send_Height(void)
{
	float tmp_f;
	
	u8 cnt = 0;
	
	//帧头
	Data_Buffer[cnt++] = 0xAA;	
	Data_Buffer[cnt++] = 0xAA;
	
	//功能字
	Data_Buffer[cnt++] = 0x01;	
	
	//内容
	tmp_f = ultra.relative_height*10;	//转为mm单位
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar.displacement;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar_fusion.fusion_displacement.out;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	Send_to_Camera(Data_Buffer,cnt);
}

//发送姿态数据
void Copter_Send_Attitude(void)
{
	float tmp_f;
	
	u8 cnt = 0;
	
	//帧头
	Data_Buffer[cnt++] = 0xAA;	
	Data_Buffer[cnt++] = 0xAA;
	
	//功能字
	Data_Buffer[cnt++] = 0x02;	
	
	//内容
	tmp_f = Roll;	//横滚，单位是°
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = Pitch;	//俯仰
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = Yaw;	//航向
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	Send_to_Camera(Data_Buffer,cnt);
}

//定时发送数据函数（scheduler.c中调用）
void Copter_Data_Send(void)
{
	Copter_Send_Height();
	Copter_Send_Attitude();
}

//=========================================================================================================================
//=================================================== 参数表 ==============================================================
//=========================================================================================================================

//可用参数表

//位置数据
float bias = 0;		//偏移
float angle = 0;	//角度
float speed = 0;	//速度

//姿态
float Roll_Image = 0;		//结果对应的角度
float Pitch_Image = 0;		//结果对应的角度
float Yaw_Image = 0;
float Height_Image = 0;		//结果对应的高度

//参数
float fps = 0;
float processing_fps = 0;

//时间间隔
u32 receive_T = 1000000;	//赋初值，防止除零错误

//=========================================================================================================================
//====================================================接收数据==============================================================
//=========================================================================================================================

//数据暂存数组
unsigned char Tmp_Buffer[20];

//接收Camera状态信息
void Get_Camera_Status(void)
{
	fps = *((float*)(&(Tmp_Buffer[0])));
	processing_fps = *((float*)(&(Tmp_Buffer[4])));
	//tmp = *((float*)(&(Tmp_Buffer[8])));
}

/*
接收图像采集时间信息
u8 mode		0：本帧只更新图像数据，上一帧运算还没有结束
			1：本帧更新图像数据的同时，上一帧的运算也已经结束，数据内容开始输出
*/
float Roll_Image_Latest = 0;	//最新的的Roll
float Height_Image_Latest = 0;	//最新的Height
void Get_Camera_Get_Image_Flag(u8 mode)
{
	static float roll_tmp = 0;	//临时保存roll数值
	static float height_tmp = 0;
	
	if(mode == 0)
	{
		//正常采图
		roll_tmp = Roll;
		height_tmp = sonar.displacement;
	}
	else if(mode == 1)
	{
		//采图+运算开始
		Roll_Image_Latest = roll_tmp;	//读取图像采纳时的roll为当前数据的roll
		Height_Image_Latest = height_tmp;
		
		roll_tmp = Roll;	//正常采图
		height_tmp = sonar.displacement;
	}
}

//接收偏移信息
void Get_Position(void)
{
	//******************* 采集数据 **********************
	
	//接收时间间隔
	receive_T = Get_Cycle_T(3);	//以us为单位
	
	//位置数据
	bias  = *((float*)(&(Tmp_Buffer[0])));
	angle    = *((float*)(&(Tmp_Buffer[4])));
	speed = *((float*)(&(Tmp_Buffer[8])));
	
	//读取本次结果对应图像采集时的飞机姿态信息
	Roll_Image = Roll_Image_Latest;
	Height_Image = Height_Image_Latest;
	
	//************** 数据运算 ******************
	
	//数据补偿
	Real_Length_Calculate(receive_T,Roll_Image,Pitch_Image,Yaw_Image,Height_Image);
}

//=========================================================================================================================
//=========================================================================================================================
//=========================================================================================================================
//=========================================================================================================================

u8 counter = 0;
void Copter_Receive_Handle(unsigned char data)
{
	static u8 mode = 0;
	
	switch(mode)
	{
		case 0:
			if(data == 0xAA)
				mode = 1;
			else
				mode = 0;
			break;
			
		case 1:
			if(data == 0xAF)
				mode = 2;
			else
				mode = 0;
			break;
			
		case 2:
			if(data == 0x01)	//进入功能字1解码
			{
				mode = 10;
				counter = 0;
			}
			else if(data == 0x02)	//进入功能字2解码
			{
				mode = 11;
				counter = 0;
			}
			else if(data == 0x03)
			{
				mode = 12;
				counter = 0;
			}
			else				//没有对应功能字
			{
				mode = 0;
			}
			break;
		
		case 10:
			Tmp_Buffer[counter] = data;	//3*4字节，总共占用数组0-11位
			counter++;
			if(counter>=12)
			{
				Get_Position();	//高度数据获取完成
				mode = 0;
			}
		break;
			
		case 11:
			Tmp_Buffer[counter] = data;	//3*4字节，总共占用数组0-11位
			counter++;
			if(counter>=12)
			{
				Get_Camera_Status();	//底板状态获取完成
				mode = 0;
			}
		break;
			
		case 12:
			Get_Camera_Get_Image_Flag(data);
			mode = 0;
		break;
			
		default:
			mode = 0;
		break;
	}
}
