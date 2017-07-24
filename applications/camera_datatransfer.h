#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//���ò�����

//λ������
extern float bias;		//ƫ��
extern float angle;	//�Ƕ�
extern float speed;	//�ٶ�

//��̬
extern float Roll_Image;		//�����Ӧ�ĽǶ�
extern float Pitch_Image;		//�����Ӧ�ĽǶ�
extern float Yaw_Image;		//�����Ӧ�ĽǶ�
extern float Height_Image;		//�����Ӧ�ĸ߶�

//����
extern float fps;
extern float processing_fps;

//ʱ����
extern u32 receive_T;	//����ֵ����ֹ�������

//========================================================================

#endif
