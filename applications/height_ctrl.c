/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：height_ctrl.c
 * 描述    ：高度控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "height_ctrl.h"
#include "anotc_baro_ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "rc.h"
#include "PID.h"
#include "ctrl.h"
#include "include.h"
#include "fly_mode.h"

float	set_height_e,set_height_em,
			set_speed_t,set_speed,exp_speed,fb_speed,
			exp_acc,fb_acc,fb_speed,fb_speed_old;

_hc_value_st hc_value;


u8 thr_take_off_f = 0;
u8 auto_take_off,auto_land;
float height_ref;

float auto_take_off_land(float dT,u8 ready)
{
	static u8 back_home_old;
	static float thr_auto;
	
	if(ready==0)
	{
		height_ref = hc_value.fusion_height;
		auto_take_off = 0;
	}
	
	if(Thr_Low == 1 && fly_ready == 0)
	{
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //起飞之前，并且解锁之前，非返航模式拨到返航模式
		{
				if(auto_take_off==0)  //第一步，自动起飞标记0->1
				{
					auto_take_off = 1;
				}
		}
	}
	
	switch(auto_take_off)
	{
		case 1:
		{
			if(thr_take_off_f ==1)
			{
				auto_take_off = 2;
			}
			break;
		}
		case 2:
		{
			if(hc_value.fusion_height - height_ref>500)
			{
				if(auto_take_off==2) //已经触发自动起飞
				{
					auto_take_off = 3;
				}
			}
		
		}
		default:break;
	}

	

	
	if(auto_take_off == 2)
	{
		thr_auto = 200;
	}
	else if(auto_take_off == 3)
	{
		thr_auto -= 200 *dT;
	
	}
	
	thr_auto = LIMIT(thr_auto,0,300);
	
	back_home_old = mode_value[BACK_HOME]; //记录模式历史
		
	return (thr_auto);
}
	


_PID_arg_st h_acc_arg;		//加速度
_PID_val_st h_acc_val;

_PID_arg_st h_speed_arg;	//速度
_PID_val_st h_speed_val;

_PID_arg_st h_height_arg;	//高度
_PID_val_st h_height_val;

void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//比例系数
	h_acc_arg.ki = 0.02f  *pid_setup.groups.hc_sp.kp;				//积分系数
	h_acc_arg.kd = 0;				//微分系数
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	h_speed_arg.kd = 0.0f;				//微分系数
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//比例系数
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//积分系数
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//微分系数
	h_height_arg.k_pre_d = 0.01f ;
	h_height_arg.inc_hz = 20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	
}

float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en_f,ex_i_en;

float Height_Ctrl(float T,float thr,u8 ready,float en)	//en	1：定高   0：非定高
{
	//thr：0 -- 1000
	static u8 speed_cnt,height_cnt;
	
//==============================================================================================
//解锁状态判断（如果没有解锁，则把起飞判断、高度pid积分都清零
	
	if(ready == 0)	//没有解锁（已经上锁）
	{
		ex_i_en = ex_i_en_f = 0;	
		en = 0;						//转换为手动模式，此模式直接对外输出传入的油门值
		thr_take_off = 0;			//起飞油门 = 0
		thr_take_off_f = 0;			//起飞指示归零（表示没有起飞）
	}
	
//==============================================================================================
//遥控器输入值处理
	
	//thr_set是经过死区设置的油门控制量输入值，取值范围 -500 -- +500
	thr_set = my_deathzoom_2(my_deathzoom((thr - 500),0,40),0,10);	//±50为死区，零点为±40的位置
	
	//此后使用 thr_set ，范围 -500 -- +500
	
	//==============================================================================================
	
	
	/*飞行中初次进入定高模式切换处理*/
	if( ABS(en - en_old) > 0.5f )	//从非定高切换到定高（官方注释）
									//我认为是模式在飞行中被切换，切换方向不确定
	{
		if(thr_take_off<10)			//未计算起飞油门（官方注释）
		{
			if(thr_set > -150)	//thr_set是经过死区设置的油门控制量输入值，取值范围 -500 -- +500。
								//thr_set > -150 代表油门非低
			{
				thr_take_off = 400;
			}
		}
		en_old = en;	//更新历史模式
	}
	
//==============================================================================================
// 油门控制飞机进行上升\下降（将油门值转化为期望垂直速度值 set_speed_t）
	
	if(thr_set>0)	//上升
	{
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_UP;	//set_speed_t 表示期望上升速度占最大上升速度的比值
		
		if(thr_set>100)	//达到起飞油门
		{
			ex_i_en_f = 1;
			
			if(!thr_take_off_f)	//如果没有起飞（本次解锁后还没有起飞）
			{
				thr_take_off_f = 1; //用户可能想要起飞（切换为已经起飞）
				thr_take_off = 350; //直接赋值 一次
			}
		}
	}
	else			//悬停或下降
	{
		if(ex_i_en_f == 1)	//从上电开始出现过起飞油门（第一次解锁前油门被软件拉到最低，所以把初次解锁前动油门杆不会有影响
		{
			ex_i_en = 1;	//表示曾经到达过起飞油门（已经起飞或上电后曾经过）
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;	//set_speed_t 表示期望上升速度占最大下降速度的比值
	}
	
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//垂直速度期望限幅
	
	//将 set_speed_t 进行曲线化、滤波生成 exp_speed
//	exp_speed =my_pow_2_curve(exp_speed_t,0.45f,MAX_VERTICAL_SPEED);
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);	//LPF_1_是低通滤波器，截至频率是10Hz，输出值是set_speed
																						//my_pow_2_curve把输入数据转换为2阶的曲线，在0附近平缓，在数值较大的部分卸率大
	
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//exp_speed 限幅
	
	//至此完成对输入数据的处理（把输入数据映射到期望速度）
	//set_speed 为期望垂直速度
	
//==============================================================================================	
//高度数据获取：气压计数据
	
	baro_ctrl(T,&hc_value); 
	
//==============================================================================================

	
	
	//计算高度误差（可加滤波）
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);	//ex_i_en = 1 表示已经到达起飞油门
	
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;		//  △h =（期望速度 - 当前速度） * △t
																		//  h(n) = h(n-1) + △h
	set_height_e = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
	
	LPF_1_(0.05f,T,set_height_em,set_height_e);	//频率 时间 输入 输出	//这个不像是低通滤波，而是像数据按照比例融合
	
	
/////////////////////////////////////////////////////////////////////////////////		
/////////////////////////////////////////////////////////////////////////////////
	if(en < 0.1f)							//手动模式
	{
		exp_speed = hc_value.fusion_speed;
		exp_acc = hc_value.fusion_acc;
	}
	
	
//===============================================================================
//	加速度PID
	
	float acc_i_lim;
	acc_i_lim = safe_div(150,h_acc_arg.ki,0);		//计算 acc_i_lim ，加速度ID的ki越大，acc_i_lim 越小（防止积分影响过大）
													//acc_i_lim = 150 / h_acc_arg.ki
													//避免除零错误（如果出现除零情况，就得0）
	
	//计算加速度
	fb_speed_old = fb_speed;						//存储上一次的速度
	fb_speed = hc_value.fusion_speed;				//读取当前速度
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);	//计算得到加速度：a = dy/dt = [ x(n)-x(n-1)]/dt
	
	//fb_acc是当前加速度值
	
	//一种改进的PID算法（和常见的PID算法的用法一直，结果含义等效）
	thr_pid_out = PID_calculate( T,            		//周期
								 exp_acc,			//前馈
								 exp_acc,			//期望值（设定值）
								 fb_acc,			//反馈值
								 &h_acc_arg, 		//PID参数结构体
								 &h_acc_val,		//PID数据结构体
								 acc_i_lim*en		//integration limit，积分限幅     如果在手动模式，en = 0，这个结果就是0了
								);					//输出		

	//step_filter(1000 *T,thr_pid_out,thr_pid_out_dlim);
	
	//起飞油门
	if(h_acc_val.err_i > (acc_i_lim * 0.2f))	//如果积分大于最大积分值的20%（积分值过大）
	{
		if(thr_take_off<THR_TAKE_OFF_LIMIT)
		{
			thr_take_off += 150 *T;
			h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0) *T;	//加速度积分PID的ki减小
		}
	}
	else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))	//如果积分小于最大负积分值的20%（积分值过小）
	{
		if(thr_take_off>0)
		{
			thr_take_off -= 150 *T;
			h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0) *T;	//加速度积分PID的ki增大
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //一半
	
	//油门补偿
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45度内补偿
	
	//油门输出
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );	//由两部分组成：油门PID + 油门补偿 * 起飞油门
															//thr_take_off应该可以理解为油门基准值，或者说是高度保持油门
	
	thr_out = LIMIT(thr_out,0,1000);
	
/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{

		exp_acc = PID_calculate( dT,           				//周期
								exp_speed,					//前馈
								(set_speed + exp_speed),	//期望值（设定值）
								hc_value.fusion_speed,		//反馈值
								&h_speed_arg, 				//PID参数结构体
								&h_speed_val,				//PID数据结构体
								500 *en						//integration limit，积分限幅
								 );							//输出	
		
		exp_acc = LIMIT(exp_acc,-3000,3000);
		
		//integra_fix += (exp_speed - hc_value.m_speed) *dT;
		//integra_fix = LIMIT(integra_fix,-1500 *en,1500 *en);
		
		//LPF_1_(0.5f,dT,integra_fix,h_speed_val.err_i);
		
		dT2 += dT;
		height_cnt++;
		if(height_cnt>=10)  //200ms 
		{
			/////////////////////////////////////

		 exp_speed = PID_calculate( dT2,            //周期
																0,				//前馈
																0,				//期望值（设定值）
																-set_height_e,			//反馈值
																&h_height_arg, //PID参数结构体
																&h_height_val,	//PID数据结构体
																1500 *en			//integration limit，积分限幅
																 );			//输出	
			
			exp_speed = LIMIT(exp_speed,-300,300);
			/////////////////////////////////////
			dT2 = 0;
			height_cnt = 0;
		}
		
		speed_cnt = 0;
		dT = 0;				
	}		
/////////////////////////////////////////////////////////////////////////////////	
	
	if(en < 0.1f)		//手动模式
	{
		return (thr);	//thr是传入的油门值，thr：0 -- 1000
	}
	else
	{
		return (thr_out);
	}
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
