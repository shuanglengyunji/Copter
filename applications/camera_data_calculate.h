#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

extern float bias_lpf;
extern float bias_real;
extern float speed_d_bias;

extern float speed_d_bias_lpf;
extern float receive_fps;

void Camera_Calculate(void);

#endif

