/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：PID.c
 * 描述    ：PID控制器
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "PID.h"

//_PID_arg_st	参数
//_PID_val_st	变量

float PID_calculate(	float T,            //周期（单位：秒）
						float in_ff,				//前馈值
						float expect,				//期望值（设定值）
						float feedback,				//反馈值（上一次的调整结构，也就是当前的状态输入）
						_PID_arg_st *pid_arg, 		//PID参数结构体
						_PID_val_st *pid_val,		//PID数据结构体
						float inte_lim				//integration limit，积分限幅
					)	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);	//k_inc_d_norm 限幅为 0 -- 1
	
	//本次同上次采样值的差分
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);	//求本次feedback和上次feedback的差，相当于微分（d）
	
	//error
	pid_val->err =  (expect - feedback );	//偏差 = 期望 - 反馈
	
	//error的差分
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	//kd * d_err + kd_feedbeck * d_feedbeck		两种 d 的和，看起来有抑制变化过大的作用
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf);	//对 d 的数值 differential 进行低通滤波，得到最终的 d
																//存入 pid_val->err_d_lpf
	
	//I 积分
	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d   * pid_val->feedback_d )*T;//)*T;//
//					   误差量		  反馈误差微分(d)系数 * 反馈微分
	
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);	//对误差的积分进行限幅
	
	//PID公式
	out = pid_arg->k_ff *in_ff			//前馈值
	    + pid_arg->kp *pid_val->err  	//P
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential	//D
    	+ pid_arg->ki *pid_val->err_i;	//I
	
	//保存历史数据
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
	
	return (out);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
