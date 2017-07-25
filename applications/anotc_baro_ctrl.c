/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：anotc_baro_ctrl.c
 * 描述    ：气压计控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "anotc_baro_ctrl.h"
#include "ms5611.h"
#include "filter.h"
#include "fly_mode.h"

//对 等效重力向量 和 lim 的乘积 做低通滤波，根据其数值做输出com_val			//这段代码毫无道理
float baro_compensate(float dT,float kup,float kdw,float vz,float lim)	//气压补偿
{
	float z_sin;
	static float com_val,com_tar;
	
	z_sin = my_sqrt(1-my_pow(vz));
	
	//((z_sin/0.44f) *lim) 过2Hz低通 得到 com_tar
	
	//com_tar = (z_sin/0.44f) *lim;
	LPF_1_(2.0f,dT,((z_sin/0.44f) *lim),com_tar);	//低通滤波器
	com_tar = LIMIT(com_tar,0,lim);
	
	if(com_val<(com_tar-100))
	{
		com_val += 1000 *dT *kup;
	}
	else if(com_val>(com_tar+100))
	{
		com_val -= 1000 *dT *kdw;
	}
	return (com_val);
}

//设置死区、平均数滤波、单位化为mm、用距离变化计算速度、加速度
void fusion_prepare(float dT,float av_arr[],u16 av_num,u16 *av_cnt,float deadzone,_height_st *data,_fusion_p_st *pre_data)
{
	//av_cnt 取平均次数
	
	//先设置死区，再把单位从厘米转为毫米，最后取多次平均值
	pre_data->dis_deadzone = my_deathzoom(data->relative_height,pre_data->dis_deadzone,deadzone);		//设置死区
	Moving_Average(av_arr,av_num ,(av_cnt),(10 *pre_data->dis_deadzone ),&(pre_data->displacement));	//取平均值，厘米->毫米
	
	pre_data->speed = safe_div(pre_data->displacement - pre_data->displacement_old,dT,0);	//速度 = （本次高度-上次高度）÷时间
	pre_data->acceleration = safe_div(pre_data->speed - pre_data->speed_old,dT,0);			//加速度
	
	//记录旧数据
	pre_data->displacement_old = pre_data->displacement;
	pre_data->speed_old = pre_data->speed;
}

//加速度融合
void acc_fusion(float dT,_f_set_st *set,float est_acc,_fusion_p_st *pre_data,_fusion_st *fusion)	
{
	fusion->fusion_acceleration.out += est_acc - fusion->est_acc_old; //估计
	anotc_filter_1(set->b1,set->g1,dT,pre_data->acceleration,&(fusion->fusion_acceleration));  //pre_data->acceleration //观测、最优
	
	fusion->fusion_speed_m.out += 1.1f *my_deathzoom(fusion->fusion_acceleration.out,0,20) *dT;	//加速度修正速度
	anotc_filter_1(set->b2,set->g2,dT,pre_data->speed,&(fusion->fusion_speed_m));				//pre_data->speed经过带通滤波得到fusion->fusion_speed_m
	anotc_filter_1(set->b2,set->g2,dT,(-pre_data->speed + fusion->fusion_speed_m.out),&(fusion->fusion_speed_me));	//被带通滤掉的速度分量 进行带通滤波 得到 fusion_speed_me
	fusion->fusion_speed_me.out = LIMIT(fusion->fusion_speed_me.out,-200,200);
	fusion->fusion_speed_m.a = LIMIT(fusion->fusion_speed_m.a,-1000,1000);
	
	fusion->fusion_displacement.out += 1.05f *(fusion->fusion_speed_m.out - fusion->fusion_speed_me.out) *dT;
	anotc_filter_1(set->b3,set->g3,dT,pre_data->displacement,&(fusion->fusion_displacement));
	
	fusion->est_acc_old = est_acc;
}

//超声波融合参数
#define SONAR_AV_NUM 50
float sonar_av_arr[SONAR_AV_NUM];
u16 sonar_av_cnt;

_fusion_p_st sonar;
_fusion_st sonar_fusion;
_f_set_st sonar_f_set = {0.2f,0.5f,0.8f,   0.2f,0.5f,0.8f};

//气压计融合参数											
#define BARO_AV_NUM 100
float baro_av_arr[BARO_AV_NUM];
u16 baro_av_cnt;
_fusion_p_st baro_p;
_fusion_st baro_fusion;
_f_set_st baro_f_set = {0.1f,0.2f,0.3f,   0.1f,0.1f,0.2f};

float sonar_weight;		//超声波数据权重
float wz_speed,sonarz_speed;	//wz_speed 是气压计数据得出的相对准确的垂直速度
void baro_ctrl(float dT,_hc_value_st *height_value)		//获取高度数据（调用周期2ms）
{
	static float dtime;
	static float baro_com_val;	//气压计补偿变量
		
//*******************************************************************************************************************************
//											气压计数据采集
//*******************************************************************************************************************************
	
	dtime += dT;	//传入的dT在数学上是△T，单位是s
	if(dtime > 0.01f) //10 ms
	{
		dtime = 0;
 		if( !MS5611_Update() )//更新ms5611气压计数据（10ms调用一次，每调用两次会读取一次气压计）   MS5611_Update()   0：气压   1：温度
		{
			baro.relative_height = baro.relative_height - 0.1f *baro_com_val;
		}
	}

	//baro.h_dt = 0.02f; //气压计读取间隔时间20ms		//（这段没用上）
	
	//气压计补偿		//dT >= 2ms（2ms左右）
	//这段代码有问题，不合逻辑
	baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_v.z,3500);	//气压计补偿是在气压计读取数据失败时根据reference_v.z进行补偿
	
//*******************************************************************************************************************************
//							高度数据融合校准（设置死区、滑动滤波、单位化、融合加速度数据）
//*******************************************************************************************************************************
	
	//气压计数据加速度融合
	fusion_prepare(	dT,			baro_av_arr,	BARO_AV_NUM,		&baro_av_cnt,		2,			&baro,			&baro_p);
	//				时间微分		气压计滑动数组	滑动数组数据个数		滑动滤波计数变量		死区宽度		采集到的数据		
	acc_fusion(	dT,			&baro_f_set,	acc_3d_hg.z,	&baro_p,			&baro_fusion);
	//		   时间微分		设置参数			加速度值			准备好的数据			融合输出
	
	//超声波数据加速度融合
	fusion_prepare(dT,sonar_av_arr,SONAR_AV_NUM,&sonar_av_cnt,0,&ultra,&sonar);	//设置死区、平均数滤波、单位化为mm、用距离变化计算速度、加速度
	acc_fusion(	dT,			&sonar_f_set,	acc_3d_hg.z,	&sonar,				&sonar_fusion);				//acc_3d_hg.z、sonar是输入，sonar_fusion是输出
	//		   时间微分		设置参数			加速度值			准备好的数据			融合输出

//*******************************************************************************************************************************
//	高度数据融合（选择数据来源）
//*******************************************************************************************************************************
	if(ultra.measure_ok == 1)	//超声波数据有效
	{
		sonar_weight += 0.5f *dtime;	//sonar_weight越大，超声波数据的权重越高
	}
	else
	{
		sonar_weight -= 2.0f *dtime;
	}
	sonar_weight = LIMIT(sonar_weight,0,1);
	
	//模式1为纯气压计模式
	if(mode_state == 1)	//通道5选择气压计模式，超声波数据不参与高度数据融合
	{
		sonar_weight = 0;
	}

//==========================================================================
	
	//		       融合了加速度计数据经过带通滤波		 被带通滤掉的气压计速度量经过带通滤波
	wz_speed     = baro_fusion.fusion_speed_m.out  - baro_fusion.fusion_speed_me.out;	//wz_speed 是气压计数据得出的相对准确的垂直速度
	sonarz_speed = sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out;	//超声波传感器计算出的相对精确垂直速度
	
	//气压计、超声波原始数据融合
	//速度融合
	float m_speed,f_speed;
	m_speed = (1 - sonar_weight) * baro_p.speed + sonar_weight * (sonar.speed);					//数据源：经过平均数滤波、单位转换为mm的速度值
	f_speed = (1 - sonar_weight) * (wz_speed)   + sonar_weight * (sonarz_speed);				//数据源：融合了加速度计的数据
	//f_speed = (1 - sonar_weight) * (wz_speed)   + sonar_weight * (sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out);	
	
	
	//加速度采用经过等效重力向量矫正的加速度计数据
	//速度采用超声波+气压计融合的速度数据
	//高度采用气压计+加速度计融合的气压计高度
	height_value->m_acc = acc_3d_hg.z;
	height_value->m_speed = m_speed;  //(1 - sonar_weight) *hf1.ref_speed_lpf + sonar_weight *(sonar.speed);	//m_speed
	height_value->m_height = baro_p.displacement;
	
	//加速度采纳融合了气压计数据、加速度计数据的加速度
	//速度数据采纳设置死区、限幅后的超声波+气压计融合的速度数据
	//高度数据采纳融合后的气压计数据
	height_value->fusion_acc = baro_fusion.fusion_acceleration.out;
	height_value->fusion_speed = my_deathzoom(LIMIT( (f_speed),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,10);	//f_speed限幅+死区（如果变化小，就认为没变化）
	height_value->fusion_height = baro_fusion.fusion_displacement.out;
}



/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
