/**
  ******************************************************************************
  * @file
  * @author  sy
  * @brief
  * @date
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "chassis_config.h"
#include "chassis.h"
#include <string.h>
#include "holder.h"
#include "usart.h"
#include "tim.h"
#include "can.h"
#include "fast_tri_func.h"
#include "misc_func.h"
#include "judge.h"
#include "judge_tx.h"
#include "ui.h"
#include "freertos.h"
#include "task.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define GAIN_I 0.1
#define GAIN_J 0.1
#define GAIN_K 0.2
#define FOLLOW 1

#define CHASSIS_FOLLOW (0)
#define CHASSIS_SPIN   (1)

/* variables -----------------------------------------------------------------*/
Chassis_t Chassis = {0};
int16_t front_temp,right_temp;
/* function ------------------------------------------------------------------*/


/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Init(void)
{
    uint8_t i;
	
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.m3508.PidSpeed,Chassis_Speed_Kp,
                           Chassis_Speed_Ki,Chassis_Speed_Kd,Chassis_Speed_Inc_Limit);
    }
    for(i=0;i<4;i++)
    {
        pid_init_increment(&Chassis.m3508.PidCurrent,Chassis_Current_Kp,
                           Chassis_Current_Ki,Chassis_Current_Kd,Chassis_Current_Inc_Limit);
    }
}

void Remote_Control_GetMoveData(RemoteData_t RDMsg)
{
    right_temp = RDMsg.Ch2;
    front_temp = RDMsg.Ch3;
}
/**
  * @brief  获取底盘控制数据
  * @param  遥控器消息结构体
  * @retval void
  * @attention 
  */
#define FRONT      (0)
#define RIGHT      (1)
#define CLOCK_WISE (2)
#define chassis_ramp_speed  (4)

int16_t chassis_now_speed[3];
int16_t chassis_aim_speed[3];

void Chassis_GetMoveData(RemoteData_t RDMsg)
{
	    uint8_t i;
    switch (RDMsg.S1)
    {
        case SUP:
				case SMID:
        case SDOWN:
            Remote_Control_GetMoveData(RDMsg);
        break;
            
   
        default:
        break;
    }
    

    for(i=0; i<3; i++)
    {
        chassis_now_speed[i] = chassis_aim_speed[i];
    }
    RemoteDataMsg_Process(&RDMsg);
//    CAN1_Transmit(0x200,Chassis.m3508.TarSpeed);
	
/*从遥控器接受指令*/
//	m3508.TarSpeed
}





/**
  * @brief  底盘PID输出
  * @param  void
  * @retval void
  * @attention （通过速度环（外环）所得值再计算电流环，通过电流坏（内环）直接控制输出）（rx,lpf???）
  */
void Chassis_PidRun(void)
{

        Chassis.m3508.LPf.Speed = 0.8 * Chassis.m3508.Rx.Speed + 0.2 * Chassis.m3508.LPf.Speed;
        Chassis.m3508.TarCurrent = pid_increment_update(Chassis.m3508.TarSpeed, Chassis.m3508.LPf.Speed, &Chassis.m3508.PidSpeed);
        Chassis.m3508.LPf.TarCurrent = 0.8 * Chassis.m3508.TarCurrent + 0.2 * Chassis.m3508.LPf.TarCurrent;
        Chassis.m3508.LPf.Current = 0.8 * Chassis.m3508.Rx.Current + 0.2 * Chassis.m3508.LPf.Current;
        Chassis.m3508.Output = pid_increment_update(Chassis.m3508.LPf.TarCurrent, Chassis.m3508.LPf.Current, &Chassis.m3508.PidCurrent);
        Chassis.m3508.LPf.Output = 0.8 * Chassis.m3508.Output + 0.2 * Chassis.m3508.LPf.Output;
}


/**
  * @brief  底盘电机CAN
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Chassis_CanTransmit(void)
{

		/* CAN 赋值 */
		Chassis.CanData[0]=(uint8_t)(Chassis.m3508.LPf.Output>>8);
		Chassis.CanData[1]=(uint8_t)(Chassis.m3508.LPf.Output);
 
//    CAN1_Transmit(0x200,Chassis.CanData);
}

/**
  * @brief  底盘进程
  * @param  控制指令结构体
  * @retval void
* @attention 保护写在CAN发送前，（V）一键复位，PID误差和输出清零
  */
void Chassis_Process(RemoteData_t RDMsg)
{
    Chassis_GetMoveData(RDMsg);
    Chassis_PidRun();
//	  CAN1_Transmit(0x200,Chassis.m3508.TarSpeed);
	//can 发送
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
