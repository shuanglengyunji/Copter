#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

extern float bias_lpf;
extern float bias_real;
extern float receive_fps;

void Real_Length_Calculate(float T,float roll,float pitch,float yaw,float height);

#endif

