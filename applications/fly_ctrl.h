#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "stm32f4xx.h"
#include "include.h"

//输出的控制类信息
extern float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值
extern u8 All_Out_Switch;
extern u8 my_height_mode;
extern float my_except_height;

//上位机监控变量（上位机监控状态，但不用于其他函数的控制）
extern u8 ctrl_command;			//当前飞行指令
extern u8 height_command;			//当前飞行指令

//定时调用线程
void Fly_Ctrl(void);			//在schedule里调用
void Fly_Ctrl_Camera(float T);		//Camera同频飞行控制函数
void Ctrl_Mode(float *ch_in);	//在fly_mode里调用

//数传输入处理函数
void set_except_height(u8 height);	//接收高度数据
void set_attitude_calibration(u8);
void set_all_out_switch(u8 cmd);


#endif
