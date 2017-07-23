/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "data_transfer.h"
#include "usart.h"
#include "imu.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "ms5611.h"
#include "rc.h"
#include "ctrl.h"
#include "time.h"
#include "usbd_user_hid.h"
#include "ultrasonic.h"
#include "anotc_baro_ctrl.h"
#include "fly_mode.h"
#include "height_ctrl.h"
#include "fly_ctrl.h"
#include "adc.h"
#include "camera_datatransfer.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;			//需要发送数据的标志
u8 data_to_send[50];	//用于临时存储发送数据帧的字符串

//校验包中数据的临时保存变量，校验包发送标志置1的同时要向这两个变量中写入校验值
u8 checkdata_to_send,checksum_to_send;	//checkdata_to_send取的是当前发送数组的*(data_buf+2)数据，就是第三个数据（从0开始数数到2），相当于随机取一个数用于校验
										//checksum_to_send就是总和校验

//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	#ifdef ANO_DT_USE_USB_HID
		Usb_Hid_Adddata(data_to_send,length);	//将数据存入USB_HID的缓存数组
	#endif

	#ifdef ANO_DT_USE_USART2
		Usart2_Send(data_to_send, length);
	#endif
}

//发送校验数组的函数，主要被PID发送函数调用
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

//这个函数是对于与最新的匿名地面站（2017年4月）新加的函数，用于发送反馈，告知地面站发出的指令是否收到
static void ANO_DT_Send_Msg(u8 id, u8 data)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

//=====================================================================================
//数据发送函数

//VER
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//飞行速度
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}

//GPSDATA
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=state;		//当前位置信息
	data_to_send[_cnt++]=sat_num;	//卫星数量
	
	_temp2 = lon;					//经度，类型是s32，单位是*10000000
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;					//纬度，类型是s32，单位是*10000000
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);	
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (s16)(100 *back_home_angle);//回航角（±180），单位是（度）*10
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}

//STATUS（三轴姿态、气压计高度（cm）、飞行模式、解锁状态）
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//SENSER
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//SENSER2
//气压计高度、超声波高度
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//RCDATA
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//POWER
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//MOTO
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//PID
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

User_Data1 mydata;	//数据发送结构体变量，在data_transfer.h中有引用
void ANO_DT_Send_User()					//此函数发送s16类型的数据
{
	u8 _cnt=0;
	vs16 _temp;
	
	//帧头 AA AA F1
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
	
	_temp = mydata.d1;							 						//1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = mydata.d2;													//2
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d3;													//3
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp = mydata.d4;													//4
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d5;													//5
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d6;								     				//6
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = mydata.d7;					   	           					//7
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d8; 										 			//8
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d9; 										 			//9
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d10; 										 		//10
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d11; 										 		//11
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mydata.d12; 										 		//12
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;											//LEN位，在这里补上（自动计算包长位）
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

//向地面站辅助程序发送信息
void ANO_DT_Send_User2()
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf2; //用户数据2（对地面站辅助程序）
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++] = mode_state;			//飞行模式
			
	data_to_send[_cnt++] = ctrl_command;		//姿态指令编号
	
	data_to_send[_cnt++] = fly_ready;			//解锁状态
	
	data_to_send[_cnt++] = All_Out_Switch;		//输出控制标志（自己加上的，应急停止功能）
	
	data_to_send[_cnt++] = ultra.measure_ok;	//输出控制标志（自己加上的，应急停止功能）
	
	data_to_send[_cnt++] = height_command;		//高度控制指令编号
	
	data_to_send[_cnt++] = (u8)(receive_fps*10);	//接受图像处理板数据的频率*10（提供一位小数点）
	
	data_to_send[3] = _cnt-4;				//LEN位，在这里补上
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

//**************************************************************************************************\
//							对外数据输入输出函数，总调用接口
//**************************************************************************************************\

//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数中数据发送内容都是按照此函数调用周期的倍数运行的，可以理解为每隔多少个此函数调用周期发一次对应内容
extern float ultra_dis_lpf;
void ANO_DT_Data_Exchange(void)	//当前调用周期1ms
{
//================================================================================
//发送时间判断

	static u8 cnt = 0;					//计数器变量
	
	//定时调用周期表
	static u8 senser_cnt 	= 10;	//9轴姿态传感器数值
	static u8 senser2_cnt 	= 100;	//气压计高度、超声高度
	static u8 user_cnt 	  	= 20;	//用户数据
	static u8 status_cnt 	= 50;	//姿态+高度+飞行模式+安全锁
	static u8 rcdata_cnt 	= 40;	//RC接收机数值
	static u8 motopwm_cnt	= 40;	//输出PWM数值
	static u8 power_cnt		= 100;	//电压和电流数据
	static u8 speed_cnt   	= 100;	//当前速度数值
	static u8 location_cnt  = 50;	//位置信息
	
	//调用周期处理（这代码写的很别扭，但是大约能够实现每隔一定周期调用一次的功能）
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location = 1;
	}

	if(++cnt>200) cnt = 0;

//========================================================================================================
//发送内容判断
	
	//指令反馈信息，用于需要返回命令结果的指令（用于传感器校准）
	if(f.msg_id)		//只要msg_id不为0，就好直接发送此编号的返回信息
	{
		ANO_DT_Send_Msg(f.msg_id,f.msg_data);
		f.msg_id = 0;
	}

	//如果有校验包，则先发校验包
	if(f.send_check)
	{
		f.send_check = 0;
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
	else 
	{
		//根据命令返回数据的优先级应该相对较高，索性全部排在前面
		
		//没有校验包时按照优先级先后顺序把到达发送时间的数据存入发送队列
		if(f.send_version)	//读取下位机版本命令，由数传指令置1，当前源码中只有PID指令才会要求发送校验包
		{
			f.send_version = 0;
			ANO_DT_Send_Version(4,300,100,400,0);
		}
		//根据命令要求发送PID
		else if(f.send_pid1)
		{
			f.send_pid1 = 0;
			ANO_DT_Send_PID(1,ctrl_1.PID[PIDROLL].kp,ctrl_1.PID[PIDROLL].ki,ctrl_1.PID[PIDROLL].kd,
												ctrl_1.PID[PIDPITCH].kp,ctrl_1.PID[PIDPITCH].ki,ctrl_1.PID[PIDPITCH].kd,
												ctrl_1.PID[PIDYAW].kp,ctrl_1.PID[PIDYAW].ki,ctrl_1.PID[PIDYAW].kd);
		}
		else if(f.send_pid2)
		{
			f.send_pid2 = 0;
			ANO_DT_Send_PID(2,ctrl_2.PID[PIDROLL].kp,ctrl_2.PID[PIDROLL].ki,ctrl_2.PID[PIDROLL].kd,
												ctrl_2.PID[PIDPITCH].kp,ctrl_2.PID[PIDPITCH].ki,ctrl_2.PID[PIDPITCH].kd,
												ctrl_2.PID[PIDYAW].kp,ctrl_2.PID[PIDYAW].ki,ctrl_2.PID[PIDYAW].kd);
		}
		else if(f.send_pid3)
		{
			f.send_pid3 = 0;
			ANO_DT_Send_PID(3,pid_setup.groups.hc_sp.kp,pid_setup.groups.hc_sp.ki,pid_setup.groups.hc_sp.kd,
												pid_setup.groups.hc_height.kp,pid_setup.groups.hc_height.ki,pid_setup.groups.hc_height.kd,
												pid_setup.groups.ctrl3.kp,pid_setup.groups.ctrl3.ki,pid_setup.groups.ctrl3.kd);
		}
		else if(f.send_pid4)
		{
			f.send_pid4 = 0;
			ANO_DT_Send_PID(4,pid_setup.groups.ctrl4.kp,pid_setup.groups.ctrl4.ki,pid_setup.groups.ctrl4.kd,
							  pid_setup.groups.ctrl5.kp,pid_setup.groups.ctrl5.ki,pid_setup.groups.ctrl5.kd,
							  pid_setup.groups.ctrl6.kp,pid_setup.groups.ctrl6.ki,pid_setup.groups.ctrl6.kd);
		}
		else if(f.send_pid5)
		{
			f.send_pid5 = 0;
			ANO_DT_Send_PID(5,user_parameter.groups.self_def_1.kp , user_parameter.groups.self_def_1.ki , user_parameter.groups.self_def_1.kd,
									user_parameter.groups.self_def_2.kp , user_parameter.groups.self_def_2.ki , user_parameter.groups.self_def_2.kd,	
									user_parameter.groups.param_A , user_parameter.groups.param_B , user_parameter.groups.param_C );
		}
		else if(f.send_pid6)
		{
			f.send_pid6 = 0;
			ANO_DT_Send_PID(6,user_parameter.groups.param_D , user_parameter.groups.param_E , user_parameter.groups.param_F,
												user_parameter.groups.param_G , user_parameter.groups.param_H , user_parameter.groups.param_I,
												user_parameter.groups.param_J , user_parameter.groups.param_K , user_parameter.groups.param_L);
		}
		
	//=================================================================================================================================
	//                               	定时发送数据（9-1=8个）（优先级低于按照指令回传的数据）
	//=================================================================================================================================
		
		else if(f.send_status)	//姿态+高度+飞行模式+安全锁状态
		{
			f.send_status = 0;
			ANO_DT_Send_Status(	Roll,	Pitch,	Yaw,	(0.1f *baro_fusion.fusion_displacement.out),	mode_state+1,	fly_ready);	
			//					Roll	Pitch	Yaw		高度（气压计融合后的高度）						飞行模式			安全锁状态
		}	
		else if(f.send_speed)	//x，y，z轴速度
		{
			f.send_speed = 0;
			ANO_DT_Send_Speed(0,0,wz_speed);	//wz_speed 是气压计数据得出的相对准确的垂直速度
		}
		else if(f.send_user)	//用户数据
		{
			f.send_user = 0;
			ANO_DT_Send_User();
			ANO_DT_Send_User2();
		}
		else if(f.send_senser)	//9轴姿态传感器数值
		{
			f.send_senser = 0;
			ANO_DT_Send_Senser(mpu6050.Acc.x,mpu6050.Acc.y,mpu6050.Acc.z,mpu6050.Gyro.x,mpu6050.Gyro.y,mpu6050.Gyro.z,ak8975.Mag_Val.x,ak8975.Mag_Val.y,ak8975.Mag_Val.z);
		}
		else if(f.send_senser2)	//气压计高度、超声高度
		{
			f.send_senser2 = 0;
			ANO_DT_Send_Senser2(baro.height,ultra.height);//原始数据
		}
		else if(f.send_rcdata)	//限幅、归一化后的输入摇杆值
		{
			f.send_rcdata = 0;
			ANO_DT_Send_RCData(CH[2]+1500,CH[3]+1500,CH[0]+1500,CH[1]+1500,CH[4]+1500,CH[5]+1500,CH[6]+1500,CH[7]+1500,-500 +1500,-500 +1500);
		}
		else if(f.send_location)
		{
			f.send_location = 0;
			ANO_DT_Send_Location(	0,			0,	bias *10000000 ,	0 *10000000,	0		);
			//						定位状态	卫星数量			经度 			纬度 			回航角
		}
		else if(f.send_motopwm)	//当前PWM输出值
		{
			f.send_motopwm = 0;
			#if MAXMOTORS == 8
					ANO_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],motor_out[4],motor_out[5],motor_out[6],motor_out[7]);
			#elif MAXMOTORS == 6
					ANO_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],motor_out[4],motor_out[5],0,0);
			#elif MAXMOTORS == 4
					ANO_DT_Send_MotoPWM(motor_out[0],motor_out[1],motor_out[2],motor_out[3],0,0,0,0);
			#endif
		}
		else if(f.send_power)	//电压电流
		{
			f.send_power = 0;
			ANO_DT_Send_Power( Battry_Voltage ,456);	//传入数据为V*100，输入123则显示为1.23V
		}

		
	}
	
//=================================================================================================================================


#ifdef ANO_DT_USE_USB_HID
	Usb_Hid_Send();			//定时调用USB_HID的发送函数
#endif
	

}

//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 RX_CH[CH_NUM];
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 tmp;
	
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;						//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头	（只接受 0xAA 0xAF 帧头）
	
	if(*(data_buf+2)==0X01)		//命令集合1（功能字为01）
	{
		if(*(data_buf+4)==0X01)				//0x01：ACC校准
		{
			mpu6050.Acc_CALIBRATE = 1;
		}
		else if(*(data_buf+4)==0X02)		//0x02：GYRO校准
		{
			mpu6050.Gyro_CALIBRATE = 1;
		}
		else if(*(data_buf+4)==0X03)		//0x03：无功能（地面站无法发送此指令）
		{
			//mpu6050.Acc_CALIBRATE = 1;
			//mpu6050.Gyro_CALIBRATE = 1;
		}
		else if(*(data_buf+4)==0X04)		//0x04：MAG校准
		{
			Mag_CALIBRATED = 1;	
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))	//0x21-0x26：六面校准第1-6步
		{
			//acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)		//0x20：退出六面校准
		{
			//acc_3d_step = 0; //退出，6面校准步清0
		}
	}
	
	if(*(data_buf+2)==0X02)		//命令集合2（功能字为02）
	{
		if(*(data_buf+4)==0X01)		//读取PID
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)		//读取飞行模式设置
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取下位机版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X03)		//RCDATA（命令字03）
	{
		//如果NS模式不是1（不是接收机模式或接收机已经掉线），则用数传数据喂狗，喂狗时会把模式切换为数传数据模式
		//只有在一开始就没有遥控器的情况下NS才会是0（表示没有收到过遥控器信号）
//		if( NS != 1 )
//		{
//			Feed_Rc_Dog(2);	//数传
//		}

//		RX_CH[THR] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
//		RX_CH[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
//		RX_CH[ROL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
//		RX_CH[PIT] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
//		RX_CH[AUX1] = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
//		RX_CH[AUX2] = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
//		RX_CH[AUX3] = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
//		RX_CH[AUX4] = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
	}

	//PID1 赋值到运算用的结构体
	if(*(data_buf+2)==0X10)			//设置 PID1 组
    {
        ctrl_1.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_1.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_1.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_1.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_1.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_1.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_1.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_1.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_1.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		PID_Para_Init();
		flash_save_en_cnt = 1;	//用于存储到存储用结构体的标志位
    }
	//PID2 赋值到运算用的结构体
    if(*(data_buf+2)==0X11)			//设置 PID2 组
    {
        ctrl_2.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl_2.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl_2.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl_2.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl_2.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl_2.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		PID_Para_Init();
		flash_save_en_cnt = 1;	//用于存储到存储用结构体的标志位
    }
	//直接存储到存储用的结构体
    if(*(data_buf+2)==0X12)			//设置 PID3 组
    {
		//高度速率
        pid_setup.groups.hc_sp.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.hc_sp.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.hc_sp.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		//高度保持
        pid_setup.groups.hc_height.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        pid_setup.groups.hc_height.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        pid_setup.groups.hc_height.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
		//位置速率
        pid_setup.groups.ctrl3.kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        pid_setup.groups.ctrl3.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        pid_setup.groups.ctrl3.kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		PID_Para_Init();
		flash_save_en_cnt = 1;	//用于存储到存储用结构体的标志位
    }
	if(*(data_buf+2)==0X13)			//PID4
	{
		//位置保持
		pid_setup.groups.ctrl4.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        pid_setup.groups.ctrl4.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        pid_setup.groups.ctrl4.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		//PID11
		pid_setup.groups.ctrl5.kp  = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		pid_setup.groups.ctrl5.ki  = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		pid_setup.groups.ctrl5.kd  = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );

		//PID12
		pid_setup.groups.ctrl6.kp  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		pid_setup.groups.ctrl6.ki  = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		pid_setup.groups.ctrl6.kd  = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		PID_Para_Init();
		flash_save_en_cnt = 1;
	}

	/* ********** 用户定义参数 ********** */
	
	if(*(data_buf+2)==0X14)			//设置PID13;PID14;PID15
	{
		/***PID13**/	
		//self_def_1是pid_t类型变量，其中包含kp,ki,kd和damp,使用到的主要是kp,ki,kd
		user_parameter.groups.self_def_1.kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		user_parameter.groups.self_def_1.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		user_parameter.groups.self_def_1.kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		/***PID14***/
		//self_def_2是pid_t类型变量，其中包含kp,ki,kd和damp,使用到的主要是kp,ki,kd
        user_parameter.groups.self_def_2.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        user_parameter.groups.self_def_2.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        user_parameter.groups.self_def_2.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
		/***PID15(PARAM A B C)***/ 			
        user_parameter.groups.param_A	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        user_parameter.groups.param_B 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        user_parameter.groups.param_C 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		flash_save_en_cnt = 1;
	}
	
	if(*(data_buf+2)==0X15)			//设置PID16;PID17;PID18
	{
		
		/***PID15(PARAM D E F)***/ 			
        user_parameter.groups.param_D	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        user_parameter.groups.param_E = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        user_parameter.groups.param_F	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		
		/***PID15(PARAM G H I)***/ 			
        user_parameter.groups.param_G	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        user_parameter.groups.param_H = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        user_parameter.groups.param_I = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		
		/***PID15(PARAM J K L)***/ 			
        user_parameter.groups.param_J	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        user_parameter.groups.param_K = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        user_parameter.groups.param_L = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.send_check == 0)
		{
			f.send_check = 1;
			checkdata_to_send = *(data_buf+2);
			checksum_to_send = sum;
		}
		flash_save_en_cnt = 1;
	}

	if(*(data_buf+2)==0X40)			//急停控制
	{
		tmp = *(data_buf+4);
		set_all_out_switch(tmp);
	}
	
	if(*(data_buf+2)==0X41)			//姿态校准指令
	{
		tmp = *(data_buf+4);
		set_attitude_calibration(tmp);
	}
	
	if(*(data_buf+2)==0X42)			//高度数据
	{
		tmp = *(data_buf+4);
		set_except_height(tmp);
	}
	
}


//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)	//最大帧长度为49字节
	{
		state = 4;
		RxBuffer[3]=data;	//Len存入数组
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)	//按照_data_len长度把数据存入数组
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;		//SUM位
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
