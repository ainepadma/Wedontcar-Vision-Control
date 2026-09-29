#ifndef __MOTOR_H
#define __MOTOR_H
#include "zf_common_headfile.h"

#define POWER_MAX	 	 (10)	   //软件极限功率<>  不知道是多少 改
#define PWM_Step   (10)
#define PWM_Brake_Step (20)
void set_speed_pwm(float left_speed,float right_speed);

#endif