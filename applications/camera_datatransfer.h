#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//可用参数表

extern float bias;			//偏移
extern float bias_pitch;	//pitch偏移
extern float angle;			//角度

//参数
extern float fps;
extern float processing_fps;
extern float receive_T;

extern float Roll_Image;		//结果对应的角度
extern float Pitch_Image;
extern float Yaw_Image;
extern float Height_Image;		//结果对应的高度


//========================================================================

#endif
