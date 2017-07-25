#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"
#include "camera_datatransfer.h"
#include "camera_data_calculate.h"
#include "ctrl.h"
#include "mymath.h"

float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值
float my_except_height = 0;//期望高度
u8 my_height_mode = 0;	//模式使用的定高模式
						//0：油门
						//1：期望高度
						//2：期望速度

//========================================================================================
//========================================================================================
//										指令接收
//========================================================================================
//========================================================================================

void set_except_height(u8 height)	//传入高度数据，单位cm，取值范围0cm-255cm
{
	my_except_height = height * 10;
}

void set_attitude_calibration(u8 cmd)
{
	switch(cmd)
	{
		//前后 -- 俯仰 -- y轴
		
		case 0x01:	//前
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y + 0.1f;
		break;
		
		case 0x02:	//后
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y - 0.1f;
		break;
		
		//左右 -- 横滚 -- x轴
		
		case 0x03:	//左
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x + 0.1f;
		break;
		
		case 0x04:	//右
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x - 0.1f;
		break;
		
		//存储
		
		case 0x05:	//存储
			flash_save_en_cnt = 1;	//存储使能
		break;
		
		//清零
		
		case 0x06:	//清零（不存储）
			mpu6050.vec_3d_cali.x = 0;
			mpu6050.vec_3d_cali.y = 0;
		break;
		
		default:
			
		break;
	}
}

void set_all_out_switch(u8 cmd)
{
	switch(cmd)	//第一个数据（u8）
	{
		case 0x01:	//急停
			All_Out_Switch = 0;	//禁止输出
		break;
			
		case 0x02:	//解急停
			All_Out_Switch = 1;	//允许输出
		break;
			
		default:
			
		break;
	}
}

//========================================================================================
//========================================================================================
//										高度控制
//========================================================================================
//========================================================================================

//手飞模式
void hand(void)
{
	//=================== filter ===================================
	//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
	//=================== filter ===================================

	//摇杆控高
	my_height_mode = 0;
	CH_ctrl[THR] = CH_filter[THR];	//2：油门 THR
	
	if(NS==0) //丢失信号
	{
		CH_ctrl[THR] = LIMIT(CH_ctrl[THR],-499,0);	//保持当前油门值，但不能超过半油门，500这个数在定高代码里代表悬停
												//也就是说定高模式丢信号时只能悬停或下降（依照丢信号前状态）
	}
	
	//依旧由油门控制Thr_Low位
	//thr取值范围0-1000，改为CH_filter后取值范围+-500
	if( CH_ctrl[THR] < -400 )	//油门低判断（用于 ALL_Out 里的最低转速保护 和 ctrl2 里的Yaw轴起飞前处理）
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
}

//锁定当前高度（进入模式时锁定当前高度，并接受上位机位置控制）
void height_lock(u8 en)	//en -- 模式启用标志位，用于判断此模式是否被使用
{
	static u8 height_lock_flag = 0;
	
	if(en)
	{
		//高度控制模式被启用
		my_height_mode = 1;
		
		Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//设置期望高度
			
			#if (HEIGHT_SOURCE == 1)
				my_except_height = sonar_fusion.fusion_displacement.out;	//读取当前高度
			#elif (HEIGHT_SOURCE == 2)
				my_except_height = sonar.displacement;						//读取当前高度
			#endif
			
			if( my_except_height < 150)		//容错处理，防止期望高度过低
				my_except_height = 150;
		}
		
	}
	else
	{
		height_lock_flag = 0;
	}
}

//下降到15cm
void falling_to_15cm(void)	//这个高度相当于降落了（起落架大约比15cm短一点点）
{
	Thr_Low = 1;	//油门低标志为1，允许螺旋桨停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 150;		//下降到100mm = 10cm
}

//上升到50cm
void rising_to_50cm(void)
{
	Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 500;		//下降到 150cm
}

//=====================================================================================================================================
//=====================================================================================================================================
//
//									                     姿态（位置）控制函数
//
//					可以使用的参数：	float bias、float bias_real、float bias_lpf	左正右负（数值为正则偏左，数值为负则偏右）
//									float angle、
//									float speed
//
//					控制输出：		CH_ctrl[0]	横滚输出							左负右正（负数向左有加速度，正数向右有加速度）
//
//=====================================================================================================================================
//=====================================================================================================================================

//手动控制姿态
void attitude_hand(void)
{
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[YAW] = CH_filter[YAW];	//3：航向 YAW
}



//横滚角乒乓控制
void attitude_pingpong(void)
{
	/*
	
		参数表：
		user_parameter.groups.self_def_1	横滚方向PID		对应地面站PID13
	
	*/
	
	//横滚自动控制
	if(bias_real > 14)	//偏左
	{
		CH_ctrl[0] =  40 * user_parameter.groups.self_def_1.kp;	//横滚方向kp	向右调整（为正）
	}
	else if(bias_real < -14)
	{
		CH_ctrl[0] = -50 * user_parameter.groups.self_def_1.ki;	//向左调整（为负）
	}
	else
	{
		CH_ctrl[0] = 0;
	}
	
	//俯仰和航向手动控制
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
	
	
}

//根据偏移量进行单PID调整
float roll_integration = 0;
void attitude_single_p(u8 en)
{
	float p_out,i_out,d_out,out;
	
	if(en)
	{
		
		/*
		
			bias		原始值					+ <---  ---> -
			bias_real	校正值					+ <---  ---> -
			bias_lpf	校正值过低通滤波器		+ <---  ---> -
		
			CH_ctrl[0]	横滚输出					- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
		
		*/
		
		if( ABS(bias) > 50 )
		{
			//偏移过大
			
			if( bias > 50 )
			{
				//左偏过大
				p_out = 40 * user_parameter.groups.self_def_1.kp;	//右飞
			}
			else if( bias < -50 )
			{
				//右偏过大
				p_out = -40 * user_parameter.groups.self_def_1.ki;	//左飞
			}
			i_out = 0;
			
		}
		else
		{
			//正常值
			
			//p
			p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
			
			//i
			roll_integration += bias_lpf * user_parameter.groups.self_def_2.ki;
			roll_integration = LIMIT(roll_integration,-40,40);
			i_out = roll_integration;
		}
		
		//d
		d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf 左正右负
		d_out = LIMIT(d_out,-70,70);	//限制输出幅度为+-70，允许d引起刹车动作
		
		//输出整合
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150,150);
		
		CH_ctrl[0] = out;

		//俯仰和航向手动控制
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
		CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
	}
	else
	{
		roll_integration = 0;
	}
}

void yaw_pid(void)
{
	float yaw_error,yaw_out;
	
	//计算偏差
	yaw_error = angle - 0.0f;	//偏左 +，偏右 -
	
	//纠正输出
	yaw_out = user_parameter.groups.param_A *yaw_error;
	
	CH_ctrl[YAW] = yaw_out;		//3：航向 YAW
	
	//=======================================
	
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	
}


//========================================================================================
//									飞行自动控制函数（5ms）
//========================================================================================

void Fly_Ctrl(void)		//调用周期5ms
{
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}

//===========================================================
//==================== 飞行控制逻辑 ==========================
//===========================================================
	
	/*
	
	mode_state：
	0：手动				1：气压计
	2：超声波+气压计		3：自动
	
	飞行控制的主要任务有两个：
	
	1.姿态控制，处理横滚、俯仰、航向三轴
		CH_ctrl[0] = CH_filter[0];	//0：横滚
		CH_ctrl[1] = CH_filter[1];	//1：俯仰
		CH_ctrl[3] = CH_filter[3];	//3：航向
	
	2.高度控制
		my_height_mode = 0		油门输入模式
		使用 CH_filter[THR] 输入油门值，取值范围 -500 -- +500
	
		my_height_mode = 1		期望高度模式
		使用 my_except_height 变量控制高度，单位为mm	
	
	*/
	
/* ********************* 高度控制 ********************* */
	
	if(height_command == 0)
	{
		hand();
	}
	
	if(height_command == 1)
	{
		height_lock(1);	//锁定高度
	}
	else
	{
		height_lock(0);	//清除标志位
	}
	
	if(height_command == 2)
	{
		falling_to_15cm();
	}
	
	//意外状况处理
	if(height_command > 2)	//不应该出现的情况
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	}
	
/* ********************* 姿态控制 ********************* */
	
	//指令1
	if(ctrl_command == 0)
	{
		attitude_hand();	//手飞依旧在5ms中处理
	}
	
//	//指令2
//	if(ctrl_command == 1)
//	{
//		//attitude_pingpong();	//自动控制根据数据源频率转移到Camera同频线程中处理
//	}
	
	//指令3
	if(ctrl_command == 2)
	{
		attitude_hand();
	}
	
	//指令4
	if(ctrl_command == 3)
	{
		//attitude_single_p(1);
	}
	else
	{
		//attitude_single_p(0);
	}
	
	//指令5
	if(ctrl_command == 4)
	{
		attitude_hand();
	}
	
	//指令6
	if(ctrl_command == 5)
	{
		attitude_hand();
	}
	
	//********************************************************
	//意外状况处理
	if(ctrl_command > 5)
	{
		attitude_hand();
	}
	
}

//Cam频率调用的飞行控制函数
void Fly_Ctrl_Cam(void)		//调用周期5ms
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
/* ********************* 姿态控制 ********************* */
	
//	//指令1
//	if(ctrl_command == 0)
//	{
//		attitude_hand();
//	}
//	
//	//指令2
//	if(ctrl_command == 1)
//	{
//		attitude_pingpong();	//横滚角乒乓控制
//	}
//	
//	//指令3
//	if(ctrl_command == 2)
//	{
//		attitude_hand();
//	}
	
	//指令4
	if(ctrl_command == 3)
	{
		attitude_single_p(1);
	}
	else
	{
		attitude_single_p(0);
	}
	
//	//指令5
//	if(ctrl_command == 4)
//	{
//		yaw_pid();
//	}
//	
//	//指令6
//	if(ctrl_command == 5)
//	{
//		attitude_hand();
//	}
	
	//意外状况处理
	if(ctrl_command > 5)
	{
		attitude_hand();
	}
}

//========================================================================================
//							飞行自动控制函数（camera同频调用）
//========================================================================================

void Fly_Ctrl_Camera(float T)		//调用周期为Camera数据采样周期
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	/*
		姿态控制，处理横滚、俯仰、航向三轴
		CH_ctrl[0] = CH_filter[0];	//0：横滚
		CH_ctrl[1] = CH_filter[1];	//1：俯仰
		CH_ctrl[3] = CH_filter[3];	//3：航向
	*/
	
	//指令2
	if(ctrl_command == 1)
	{
		attitude_pingpong();	//横滚角乒乓控制
	}
	
	//指令4
	if(ctrl_command == 3)
	{
		attitude_single_p(1);
	}
	else
	{
		attitude_single_p(0);
	}
	
}

//========================================================================================
//========================================================================================
//				   识别控制指令（在mode_check函数中调用，处理辅助通道数值）
//========================================================================================
//========================================================================================
u8 ctrl_command;
u8 height_command;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{
	//根据AUX2通道（第6通道）的数值输入自动控制指令
	if(*(ch_in+AUX2) < -350)			//-499 -- -350
	{
		ctrl_command = 0;
	}
	else if(*(ch_in+AUX2) < -150)		//-350 -- -150
	{
		ctrl_command = 1;
	}
	else if(*(ch_in+AUX2) < 0)			//-150 -- 0
	{
		ctrl_command = 2;
	}
	else if(*(ch_in+AUX2) < 150)		//0 -- 150
	{
		ctrl_command = 3;
	}
	else if(*(ch_in+AUX2) < 350)		//150 -- 350				
	{
		ctrl_command = 4;
	}
	else								//350 -- 499
	{
		ctrl_command = 5;
	}
	
	//根据AUX3通道控制高度
	if(*(ch_in+AUX3) < -150)	
	{
		height_command = 0;
	}
	else if(*(ch_in+AUX3) < 150)
	{
		height_command = 1;
	}
	else
	{
		height_command = 2;
	}
	
	//电机输出使能
	//All_Out_Switch = 0时急停，All_Out_Switch = 1时正常运行
	//*(ch_in+AUX4)>0时开关被扳动到下方，发出禁止输出指令
	static u8 out_flag = 0;
	static u8 stop_flag = 0;
	if(*(ch_in+AUX4) > 0)			//SWA,输出使能开关，只有在开关播到上方时输出才能被输出到PWM
	{
		out_flag = 0;	//清out_flag
		
		if(stop_flag == 0)
		{
			stop_flag = 1;
			All_Out_Switch = 0;	//禁止输出
		}
		
	}
	else
	{
		stop_flag = 0;	//清stop_flag
		
		if(out_flag == 0)
		{
			out_flag = 1;
			All_Out_Switch = 1;	//允许输出
		}
	}
}

