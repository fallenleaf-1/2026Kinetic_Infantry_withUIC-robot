#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();


extern DJIMotorInstance *small_yaw_motor;

#endif // GIMBAL_H