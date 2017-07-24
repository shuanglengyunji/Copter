#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"
#include "data_transfer.h"
#include "math.h"

//=========================================================================================================================
////================================================= 参数表 ==============================================================
//=========================================================================================================================

//方向： + <--- ---> -			单位：cm和cm/s
float bias_real = 0;
float bias_lpf = 0;
float speed_d_bias = 0;	//对bias做微分得到的speed
float speed_d_lpf = 0;

float receive_fps = 0;

//=========================================================================================================================
//=================================================== 数据运算 =============================================================
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

//bias数据校准函数(roll)
float bias_correct_roll(float roll, float pitch, float hight, float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1,real_bias;
	x1=hight*my_sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}

float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1, real_bias, coe_bias;  //coe_bias 为测得的bias 到实际bias的系数
	
	//矫正参数
	x1 = hight * sin(roll*3.141f/180.0f);
	
	//系数
	coe_bias = (0.0021f*hight*hight+0.2444*hight+8.9576) / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

/*
	T：时间间隔（单位us）
	bias：本次水平偏移量（单位cm）
	bias_old：上次水平偏移量（单位cm）
*/
float get_speed(u32 T,float bias,float bias_last)
{
	/*
		bias数值：  + <--- ---> -
		speed方向： + <--- ---> - 向左飞速度为正，向右飞速度为负
	*/
	
	float dx,dt,speed;
	dx = bias - bias_last;
	dt = T / 1000000.0f;
	speed = dx / dt;
	
	return speed;
}

//=========================================================================================================================
//													Camera数据运算函数
//=========================================================================================================================
float bias_lpf_old;	//上一个在可用范围内的bias
void Camera_Data_Calculate(void)
{
	//***************************************
	//计算接收帧率
	receive_fps = safe_div(1000000.0f,receive_T,0);	//转化为以Hz为单位
	
	//***************************************
	//处理bias值
	static float bias_old;
	if(speed)		//全白时用+-100表示
	{
		if(bias_old > 0)
			bias = +100;
		else
			bias = -100;
	}
	bias_old = bias;

	//***************************************
	//对bias进行矫正和滤波
	//只有在合理范围内才会矫正，矫正的同时进行低通滤波
	if(ABS(bias)<50)
	{
		//正常情况
		bias_real = bias_correct(Roll_Image,Pitch_Image,Height_Image/10.0f,bias);	//姿态误差校准
		bias_lpf = cam_bias_lpf(bias_real,receive_T,0.8f,bias_lpf);		//低通滤波器
		speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old);
		speed_d_lpf = cam_bias_lpf(speed_d_bias,receive_T,1.0f,speed_d_lpf);
		
		bias_lpf_old = bias_lpf;
	}
	else
	{
		speed_d_bias = 0;
	}
}

