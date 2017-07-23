#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"


float bias_lpf = 0;
float bias_real = 0;
float receive_fps = 0;

//=========================================================================================================================
//=================================================== У׼���� =============================================================
//=========================================================================================================================

//��ͨ�˲���
//dt������ʱ��������λus��
//fc����ֹƵ��
float cam_bias_lpf(float bias ,float dt_us ,float fc,float last_bias_lpf)
{
	float q,out,T;
	
	T = dt_us / 1000000.0f;
	q = 6.28f * T * fc;
	
	if(q >0.95f)
		q = 0.95f;
	
	out = q * bias + (1.0f-q) * last_bias_lpf;
	
	return out;
}

//bias����У׼����
float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1,real_bias;
	x1=hight*my_sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}

//=========================================================================================================================
//													Camera�������㺯��
//=========================================================================================================================

void Real_Length_Calculate(float T,float roll,float pitch,float yaw,float height)
{
	static float bias_old;
	
	receive_fps = 1 * 1000000.0f / T;	//ת��Ϊ��HzΪ��λ
	
	//ȫ��ʱ��+-100��ʾ
	if(speed)
	{
		if(bias_old > 0)
			bias = +100;
		else
			bias = -100;
	}
	bias_old = bias;

	//ֻ���ں���Χ�ڲŻ����
	//������ͬʱ���е�ͨ�˲�
	if(ABS(bias)<50)
	{
		//�������
		bias_real = bias_correct(roll,pitch,height/10.0f,bias);	//��̬���У׼
		bias_lpf = cam_bias_lpf(bias_real,T,0.8f,bias_lpf);		//��ͨ�˲���
	}
}

