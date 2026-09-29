#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H
#include "zf_common_headfile.h"


typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数  
    float kd;           // 微分系数
    float target;       // 目标值（转向角度或速度）
    float actual;       // 实际值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float derivative;   // 微分项
    float output;       // PID输出
    float integral_limit; // 积分限幅
    float output_limit; // 输出限幅
} pid_motor_controller_t;

typedef struct {
    float servo_angle;   // 计算出的舵机角度
    int8_t left_speed;   // 计算出的左轮速度
    int8_t right_speed;  // 计算出的右轮速度
} pid_motor_output_t;

#define PWM_MAX            50     // 你实际的 PWM 最大值
#define PWM_BRAKE_FORCE    13      // 反向制动的力度（不要太大）
#define PWM_RAMP_STEP       5     // 制动后减到 0 的步进
 
#define INTEGRAL_START_THRESHOLD 20.0f  //限定积分误差 需要测量时修改
#define INTEGRAL_DEAD_ZONE 2.0f  // 误差死区误差较小时停止积分

#define D_MAX_CHANGE 50.0f      // 微分最大变化率限制
#define D_FILTER_ALPHA 0.7f     // 低通滤波系数

#define MAX_SPEED					30

void brake_stop(void);
void brake_stop_hard(void);
void pid_motor_init(void);
float pid_motor_calculate( float target, float actual);
float pid_motor_calculate_integral(pid_motor_controller_t* pid, float dt);
float pid_motor_calculate_derivative(pid_motor_controller_t*pid, float dt);
float pid_motor_calculate(float target, float actual);
bool pid_motor_control( float target_speed, float actual_speed ,pid_motor_output_t* output);

#endif