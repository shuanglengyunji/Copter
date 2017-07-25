/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：rc.c
 * 描述    ：遥控器通道数据处理
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "rc.h"
#include "mymath.h"
#include "mpu6050.h"
#include "filter.h"
#include "ak8975.h"
#include "anotc_baro_ctrl.h"
#include "fly_ctrl.h"
#include "fly_mode.h"

s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5,6,7};    //通道映射

void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_filter[CH_NUM];

u8 NS;
u8 CH_Error[CH_NUM];u16 NS_cnt,CLR_CH_Error[CH_NUM];
 
s16 MAX_CH[CH_NUM]  = {1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 ,1900 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 ,1100 };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 500


float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	//按照 NS 的数值选择数据源
	//把 tmp16_CH 或 RX_CH 中所有通道的数值放进 Mapped_CH 数组
	
	//初始值为0；接收机输入数据时被切换为1；接收机没有数据，数传输入数据时被切换为2
	if( NS == 1 )
	{
		CH_Mapping_Fun(tmp16_CH,Mapped_CH);
	}
//	else if( NS == 2 )
//	{
//		CH_Mapping_Fun(RX_CH,Mapped_CH);
//	}

	//数值被存入 Mapped_CH[]
	
	for( i = 0;i < CH_NUM ; i++ )
	{
		//如果数值超出合理范围，CH_Error[i]=1;
		//如果恢复正常后持续一段时间 CLR_CH_Error[i] > 200
		if( (u16)Mapped_CH[i] > 2500 || (u16)Mapped_CH[i] < 500 )
		{
			CH_Error[i]=1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if( CLR_CH_Error[i] > 200 )
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if( NS == 1 || NS == 2 )	//信号都已经传入了，这一个判断没意义，应该是以前有用现在废弃的代码
		{
			if( CH_Error[i] ) //单通道数据错误
			{
				//本通道数据错误处理
				
			}
			else
			{
				//数据拷贝进 CH_TMP[i]
				CH_TMP[i] = ( Mapped_CH[i] ); //映射拷贝数据，大约 1000~2000
				
				//此后使用 CH_TMP[i] 作为遥控器单通道数据
				
				//限幅
				//摇杆方向选择，用于适配遥控器各通道摇杆数值的正反
				if( !CH_DIR[i] )	
				{
					CH[i] =   LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
				}
				else
				{
					CH[i] = - LIMIT ( (s16)( ( CH_TMP[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - CH_OFFSET ), -500, 500); //归一化，输出+-500
				}
				
				//从这里开始，信号数据传入 CH[i]
			}
		}	
		else //未接接收机或无信号（遥控关闭或丢失信号）或处于其他模式
		{
			
		}
		
		//从这里开始调用 CH[i] 获取数据
			
			
		//=================== filter ===================================
		//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
		//=================== filter =================================== 		
		
		//单通道数据滤波
		filter_A = 6.28f *10 *T;
		
		if( ABS(CH_TMP[i] - CH_filter[i]) < 100 )	//差值小于100
		{
			CH_filter[i] +=        filter_A *( CH[i] - CH_filter[i] ) ;
		}
		else
		{
			CH_filter[i] += 0.5f * filter_A *( CH[i] - CH_filter[i] ) ;	//如果变化量过大，就减小此数据对输出值的影响（滤波系数减小）
		}
		
		
		//如果输入信号的通道异常，就在数据输入端把油门值放到最低，保证安全.
		//这里的通道异常判断是来自于一个对NS进行操作的看门狗的，在本文件下方
		if(NS == 0) //无信号
		{
			if(!fly_ready)
			{
				CH_filter[THR] = -500;	//遥控器丢信号时油门输入采纳值强制为最小
			}
		}
	}
	
	//======================================================================
	Fly_Ready(T,wz_speed);		//解锁判断
	//======================================================================
	
	//NS看门狗
	if(++NS_cnt>200)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

//喂狗函数，在pwm_in和data_transfer两个地方调用
void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次
{
	NS = ch_mode;
	NS_cnt = 0;
}

//**************************************************************************************************

u8 fly_ready = 0;	//0：锁定	1：解锁
u8 thr_stick_low;
s16 ready_cnt=0;

s16 mag_cali_cnt;
s16 locked_cnt;
extern u8 acc_ng_cali;
void Fly_Ready(float T,float height_speed_mm)
{
	//对不同摇杆状态持续时间进行计数
	if( CH_filter[2] < -400 )  				//下满足（油门小于10%）
	{
		thr_stick_low = 1;	//油门低标志置1
		if( fly_ready && ready_cnt != -1 )	//解锁完成，且已退出解锁上锁过程
		{
			//ready_cnt += 1000 *T;
		}
		
		if( CH_filter[3] < -400 )				//左下满足		
		{
			if( ready_cnt != -1 && fly_ready )	//判断已经退出解锁上锁过程且已经解锁
			{
				ready_cnt += 1000 *T;
			}
		}
		else if( CH_filter[3] > 400 )      		//右下满足
		{
			if( ready_cnt != -1 && !fly_ready )	//判断已经退出解锁上锁过程且已经上锁
			{
				ready_cnt += 1000 *T;
			}
		}
		else if( ready_cnt == -1 )				//4通道(CH[3])回位
		{
			ready_cnt=0;
		}
	}
	else
	{
		thr_stick_low = 0;	//油门低标志置0
		ready_cnt=0;
	}

	//对计数结果进行判断
	if( ready_cnt > 300 ) // 600ms 
	{
		//满足解锁/上锁条件
		
		ready_cnt = -1;
		
		//切换解锁状态（解锁时上锁，上锁时解锁）
		if( !fly_ready )
		{
			fly_ready = 1;		//允许解锁
			acc_ng_cali = mpu6050.Gyro_CALIBRATE = 2;	//在解锁的一瞬间执行陀螺仪初始化，认为此时陀螺仪数值为静止时的偏移值
		}
		else
		{
			fly_ready = 0;
		}
		
	}

	//左杆左下满足，右杆右上满足，进行地磁计（电子罗盘）校准
	if(CH_filter[2] < -400 && CH_filter[3] < -400 && (CH_filter[0]>400&&CH_filter[1]>400))
	{
		if(mag_cali_cnt<2000)
		{
			mag_cali_cnt += 1000*T;
		}
		else
		{
			Mag_CALIBRATED = 1;
		}
	}
	else
	{
		mag_cali_cnt = 0;
	}
	
	//解锁后一定时间没有起飞（同时满足油门低和高度低），则上锁
	//手动模式下解锁 油门低 垂直速度低
	//自动模式下解锁 油门低 垂直速度低 手动控高度
	//自动模式下用自动控高时不会自动锁定
	if((fly_ready && (thr_stick_low == 1) && (ABS(height_speed_mm)<300) && (mode_state != 3)) || 
		(fly_ready && (CH_ctrl[THR]<-400) && (ABS(height_speed_mm)<300) && (mode_state == 3) && (my_height_mode == 0)))	// 高度控制模式为手动控高（自动控高时不受此约束）
	{
		if(locked_cnt < 4000)	//4000ms = 4s
		{
			locked_cnt  += 1000*T;	//T大约为0.002(T的单位是us)
		}
		else
		{
			fly_ready = 0;
		}
	}
	else
	{
		locked_cnt = 0;
	}
}

//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 	


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

