#include "zf_common_headfile.h"
#include "my_common.h"
#include "motor.h"

void set_speed_pwm(float left_speed,float right_speed)
{
	//速度限幅
	if(left_speed > MAX_PWM_DUTY)left_speed = MAX_PWM_DUTY;
	if(left_speed < -MAX_PWM_DUTY)left_speed = -MAX_PWM_DUTY;
	if(right_speed > MAX_PWM_DUTY)right_speed = MAX_PWM_DUTY;
	if(right_speed < -MAX_PWM_DUTY)right_speed = -MAX_PWM_DUTY;
	
	//左轮
	if(left_speed>0)//MOTOR1为左轮，MOTOR2_PWM为右轮
	{
	gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR1_PWM, left_speed * (PWM_DUTY_MAX / 100));                   // 计算占空比
	}		
	else
	{
	gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                   // DIR输出高电平
	pwm_set_duty(MOTOR1_PWM, (-left_speed) * (PWM_DUTY_MAX / 100));                   // 计算占空比
	}	
	
	//右轮	
	if(right_speed>0)//MOTOR1为左轮，MOTOR2_PWM为右轮
	{
	gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                   // DIR输出高电平
	pwm_set_duty(MOTOR2_PWM, right_speed * (PWM_DUTY_MAX / 100));                   // 计算占空比
	}		
	else
	{	
	gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                   // DIR输出高电平
	pwm_set_duty(MOTOR2_PWM, (-right_speed) * (PWM_DUTY_MAX / 100));                   // 计算占空比
	}		


}


