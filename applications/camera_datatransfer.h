#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//可用参数表

//位置数据
extern float bias;		//偏移
extern float angle;	//角度
extern float speed;	//速度

//姿态
extern float Roll_Image;		//结果对应的角度
extern float Pitch_Image;		//结果对应的角度
extern float Yaw_Image;		//结果对应的角度
extern float Height_Image;		//结果对应的高度

//参数
extern float fps;
extern float processing_fps;

//时间间隔
extern u32 receive_T;	//赋初值，防止除零错误

//========================================================================

#endif
