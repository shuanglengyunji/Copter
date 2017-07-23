#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"


float bias_lpf = 0;
float bias_real = 0;
float receive_fps = 0;

//=========================================================================================================================
//=================================================== 校准运算 =============================================================
//=========================================================================================================================

//低通滤波器
//dt：采样时间间隔（单位us）
//fc：截止频率
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

//bias数据校准函数
float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1,real_bias;
	x1=hight*my_sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}

//=========================================================================================================================
//													Camera数据运算函数
//=========================================================================================================================

void Real_Length_Calculate(float T,float roll,float pitch,float yaw,float height)
{
	static float bias_old;
	
	receive_fps = 1 * 1000000.0f / T;	//转化为以Hz为单位
	
	//全白时用+-100表示
	if(speed)
	{
		if(bias_old > 0)
			bias = +100;
		else
			bias = -100;
	}
	bias_old = bias;

	//只有在合理范围内才会矫正
	//矫正的同时进行低通滤波
	if(ABS(bias)<50)
	{
		//正常情况
		bias_real = bias_correct(roll,pitch,height/10.0f,bias);	//姿态误差校准
		bias_lpf = cam_bias_lpf(bias_real,T,0.8f,bias_lpf);		//低通滤波器
	}
}

