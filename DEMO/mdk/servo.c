#include "zf_common_headfile.h"
#include "my_common.h"

//定义舵机打角值
float servo_motor_angle = SERVO_MOTOR_M; 

float servo_motor_duty(float angle)
{
	
    return (float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)angle / 90.0);
}



void set_servo_pwm(float angle)
{
	//根据偏差计算舵机打角
	//servo_motor_angle = SERVO_MOTOR_M- kp * adc_error;
	//servo_motor_angle =  - kp * adc_error;
	//角度限幅
	if(angle > SERVO_MOTOR_R_MAX) angle = SERVO_MOTOR_R_MAX;
	if(angle < SERVO_MOTOR_L_MAX) angle = SERVO_MOTOR_L_MAX;
	
	//设置舵机PWM占空比
    //  SERVO_MOTOR_DUTY 是一个将角度转换为PWM占空比的宏或函数
    uint32_t duty_cycle = servo_motor_duty(angle);
	//设置舵机打角
	pwm_set_duty(SERVO_MOTOR2_PWM, duty_cycle);

}
