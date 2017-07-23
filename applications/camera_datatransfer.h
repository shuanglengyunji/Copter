#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//���ò�����

//λ����Ϣ
extern float bias;		//ƫ��
extern float angle;		//�Ƕ�
extern float speed;		//�ٶ�

//����
extern float fps;
extern float processing_fps;
extern u32 receive_T;

//========================================================================

#endif
