/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��height_ctrl.c
 * ����    ���߶ȿ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
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
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //���֮ǰ�����ҽ���֮ǰ���Ƿ���ģʽ��������ģʽ
		{
				if(auto_take_off==0)  //��һ�����Զ���ɱ��0->1
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
				if(auto_take_off==2) //�Ѿ������Զ����
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
	
	back_home_old = mode_value[BACK_HOME]; //��¼ģʽ��ʷ
		
	return (thr_auto);
}
	


_PID_arg_st h_acc_arg;		//���ٶ�
_PID_val_st h_acc_val;

_PID_arg_st h_speed_arg;	//�ٶ�
_PID_val_st h_speed_val;

_PID_arg_st h_height_arg;	//�߶�
_PID_val_st h_height_val;

void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//����ϵ��
	h_acc_arg.ki = 0.02f  *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_acc_arg.kd = 0;				//΢��ϵ��
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//����ϵ��
	h_speed_arg.kd = 0.0f;				//΢��ϵ��
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//����ϵ��
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//����ϵ��
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//΢��ϵ��
	h_height_arg.k_pre_d = 0.01f ;
	h_height_arg.inc_hz = 20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	
}

float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en_f,ex_i_en;

float Height_Ctrl(float T,float thr,u8 ready,float en)	//en	1������   0���Ƕ���
{
	//thr��0 -- 1000
	static u8 speed_cnt,height_cnt;
	
//==============================================================================================
//����״̬�жϣ����û�н������������жϡ��߶�pid���ֶ�����
	
	if(ready == 0)	//û�н������Ѿ�������
	{
		ex_i_en = ex_i_en_f = 0;	
		en = 0;						//ת��Ϊ�ֶ�ģʽ����ģʽֱ�Ӷ���������������ֵ
		thr_take_off = 0;			//������� = 0
		thr_take_off_f = 0;			//���ָʾ���㣨��ʾû����ɣ�
	}
	
//==============================================================================================
//ң��������ֵ����
	
	//thr_set�Ǿ����������õ����ſ���������ֵ��ȡֵ��Χ -500 -- +500
	thr_set = my_deathzoom_2(my_deathzoom((thr - 500),0,40),0,10);	//��50Ϊ���������Ϊ��40��λ��
	
	//�˺�ʹ�� thr_set ����Χ -500 -- +500
	
	//==============================================================================================
	
	
	/*�����г��ν��붨��ģʽ�л�����*/
	if( ABS(en - en_old) > 0.5f )	//�ӷǶ����л������ߣ��ٷ�ע�ͣ�
									//����Ϊ��ģʽ�ڷ����б��л����л�����ȷ��
	{
		if(thr_take_off<10)			//δ����������ţ��ٷ�ע�ͣ�
		{
			if(thr_set > -150)	//thr_set�Ǿ����������õ����ſ���������ֵ��ȡֵ��Χ -500 -- +500��
								//thr_set > -150 �������ŷǵ�
			{
				thr_take_off = 400;
			}
		}
		en_old = en;	//������ʷģʽ
	}
	
//==============================================================================================
// ���ſ��Ʒɻ���������\�½���������ֵת��Ϊ������ֱ�ٶ�ֵ set_speed_t��
	
	if(thr_set>0)	//����
	{
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_UP;	//set_speed_t ��ʾ���������ٶ�ռ��������ٶȵı�ֵ
		
		if(thr_set>100)	//�ﵽ�������
		{
			ex_i_en_f = 1;
			
			if(!thr_take_off_f)	//���û����ɣ����ν�����û����ɣ�
			{
				thr_take_off_f = 1; //�û�������Ҫ��ɣ��л�Ϊ�Ѿ���ɣ�
				thr_take_off = 350; //ֱ�Ӹ�ֵ һ��
			}
		}
	}
	else			//��ͣ���½�
	{
		if(ex_i_en_f == 1)	//���ϵ翪ʼ���ֹ�������ţ���һ�ν���ǰ���ű����������ͣ����԰ѳ��ν���ǰ�����Ÿ˲�����Ӱ��
		{
			ex_i_en = 1;	//��ʾ���������������ţ��Ѿ���ɻ��ϵ����������
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;	//set_speed_t ��ʾ���������ٶ�ռ����½��ٶȵı�ֵ
	}
	
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//��ֱ�ٶ������޷�
	
	//�� set_speed_t �������߻����˲����� exp_speed
//	exp_speed =my_pow_2_curve(exp_speed_t,0.45f,MAX_VERTICAL_SPEED);
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);	//LPF_1_�ǵ�ͨ�˲���������Ƶ����10Hz�����ֵ��set_speed
																						//my_pow_2_curve����������ת��Ϊ2�׵����ߣ���0����ƽ��������ֵ�ϴ�Ĳ���ж�ʴ�
	
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//exp_speed �޷�
	
	//������ɶ��������ݵĴ�������������ӳ�䵽�����ٶȣ�
	//set_speed Ϊ������ֱ�ٶ�
	
//==============================================================================================	
//�߶����ݻ�ȡ����ѹ������
	
	baro_ctrl(T,&hc_value); 
	
//==============================================================================================

	
	
	//����߶����ɼ��˲���
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);	//ex_i_en = 1 ��ʾ�Ѿ������������
	
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;		//  ��h =�������ٶ� - ��ǰ�ٶȣ� * ��t
																		//  h(n) = h(n-1) + ��h
	set_height_e = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
	
	LPF_1_(0.05f,T,set_height_em,set_height_e);	//Ƶ�� ʱ�� ���� ���	//��������ǵ�ͨ�˲������������ݰ��ձ����ں�
	
	
/////////////////////////////////////////////////////////////////////////////////		
/////////////////////////////////////////////////////////////////////////////////
	if(en < 0.1f)							//�ֶ�ģʽ
	{
		exp_speed = hc_value.fusion_speed;
		exp_acc = hc_value.fusion_acc;
	}
	
	
//===============================================================================
//	���ٶ�PID
	
	float acc_i_lim;
	acc_i_lim = safe_div(150,h_acc_arg.ki,0);		//���� acc_i_lim �����ٶ�ID��kiԽ��acc_i_lim ԽС����ֹ����Ӱ�����
													//acc_i_lim = 150 / h_acc_arg.ki
													//����������������ֳ���������͵�0��
	
	//������ٶ�
	fb_speed_old = fb_speed;						//�洢��һ�ε��ٶ�
	fb_speed = hc_value.fusion_speed;				//��ȡ��ǰ�ٶ�
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);	//����õ����ٶȣ�a = dy/dt = [ x(n)-x(n-1)]/dt
	
	//fb_acc�ǵ�ǰ���ٶ�ֵ
	
	//һ�ָĽ���PID�㷨���ͳ�����PID�㷨���÷�һֱ����������Ч��
	thr_pid_out = PID_calculate( T,            		//����
								 exp_acc,			//ǰ��
								 exp_acc,			//����ֵ���趨ֵ��
								 fb_acc,			//����ֵ
								 &h_acc_arg, 		//PID�����ṹ��
								 &h_acc_val,		//PID���ݽṹ��
								 acc_i_lim*en		//integration limit�������޷�     ������ֶ�ģʽ��en = 0������������0��
								);					//���		

	//step_filter(1000 *T,thr_pid_out,thr_pid_out_dlim);
	
	//�������
	if(h_acc_val.err_i > (acc_i_lim * 0.2f))	//������ִ���������ֵ��20%������ֵ����
	{
		if(thr_take_off<THR_TAKE_OFF_LIMIT)
		{
			thr_take_off += 150 *T;
			h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0) *T;	//���ٶȻ���PID��ki��С
		}
	}
	else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))	//�������С����󸺻���ֵ��20%������ֵ��С��
	{
		if(thr_take_off>0)
		{
			thr_take_off -= 150 *T;
			h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0) *T;	//���ٶȻ���PID��ki����
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //һ��
	
	//���Ų���
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45���ڲ���
	
	//�������
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );	//����������ɣ�����PID + ���Ų��� * �������
															//thr_take_offӦ�ÿ������Ϊ���Ż�׼ֵ������˵�Ǹ߶ȱ�������
	
	thr_out = LIMIT(thr_out,0,1000);
	
/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{

		exp_acc = PID_calculate( dT,           				//����
								exp_speed,					//ǰ��
								(set_speed + exp_speed),	//����ֵ���趨ֵ��
								hc_value.fusion_speed,		//����ֵ
								&h_speed_arg, 				//PID�����ṹ��
								&h_speed_val,				//PID���ݽṹ��
								500 *en						//integration limit�������޷�
								 );							//���	
		
		exp_acc = LIMIT(exp_acc,-3000,3000);
		
		//integra_fix += (exp_speed - hc_value.m_speed) *dT;
		//integra_fix = LIMIT(integra_fix,-1500 *en,1500 *en);
		
		//LPF_1_(0.5f,dT,integra_fix,h_speed_val.err_i);
		
		dT2 += dT;
		height_cnt++;
		if(height_cnt>=10)  //200ms 
		{
			/////////////////////////////////////

		 exp_speed = PID_calculate( dT2,            //����
																0,				//ǰ��
																0,				//����ֵ���趨ֵ��
																-set_height_e,			//����ֵ
																&h_height_arg, //PID�����ṹ��
																&h_height_val,	//PID���ݽṹ��
																1500 *en			//integration limit�������޷�
																 );			//���	
			
			exp_speed = LIMIT(exp_speed,-300,300);
			/////////////////////////////////////
			dT2 = 0;
			height_cnt = 0;
		}
		
		speed_cnt = 0;
		dT = 0;				
	}		
/////////////////////////////////////////////////////////////////////////////////	
	
	if(en < 0.1f)		//�ֶ�ģʽ
	{
		return (thr);	//thr�Ǵ��������ֵ��thr��0 -- 1000
	}
	else
	{
		return (thr_out);
	}
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
