#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//可用参数表

//位置信息
extern float bias;		//偏移
extern float angle;		//角度
extern float speed;		//速度

//参数
extern float fps;
extern float processing_fps;
extern u32 receive_T;

//========================================================================

#endif
