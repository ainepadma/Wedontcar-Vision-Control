#include "zf_common_headfile.h"
#include "my_common.h"
#include "servo_PID.h"
#include "motor.h"
#include "servo.h"
#include "math.h"
#include "adc.h"

// 转弯PID控制器
pid_servo_controller_t pid_servo;

static uint32 last_calc_time = 0;
static int32 last_encoder_1 = 0, last_encoder_2 = 0;

extern uint8 ring_state;
extern uint8 Straight_line_Flag;
extern uint8 Curve_Type_Flag;

//extern uint8 ring_state;
//extern uint8 Straight_line_Flag;
//extern uint8 Curve_Type_Flag;

// 初始化转弯PID
void pid_servo_init(void)
{
    pid_servo.kp = 5.0f;      // 需要根据实际调试 11.8 -6.2
    pid_servo.ki = 0.0f;     // 积分系数较小，避免过冲
    pid_servo.kd = 0.3f; // 微分系数
    
    pid_servo.target = 0;    // 目标转弯角度（0表示直行）
		// 实际转弯角度 需要通过实际参数传过来
    pid_servo.actual = 0;		
    pid_servo.error = 0;
    pid_servo.last_error = 0;
    pid_servo.integral = 0;   //积分项
    pid_servo.derivative = 0;  //微分项
    pid_servo.output = 0;
    
    pid_servo.integral_limit = 100;  // 积分限幅
    pid_servo.output_limit = 80;     // 输出限幅（占空比）
}

// 完整的抗积分饱和处理（包含输出饱和检测）
float pid_servo_calculate_integral(pid_servo_controller_t* pid, float dt)
{
    // 1. 输出饱和检测
    int output_saturated = 0;
    
    if(pid->output >= pid->output_limit && pid->error > 0) {
        // 输出正饱和且误差为正时，停止积分
        output_saturated = 1;
    } else if(pid->output <= -pid->output_limit && pid->error < 0) {
        // 输出负饱和且误差为负时，停止积分
        output_saturated = 1;
    }
    
    // 2. 只有不在饱和状态时才进行积分
    if(!output_saturated) 
			{
       // 积分分离：只有当误差小于阈值时才积分（避免大误差时积分过冲）
        if (fabs(pid->error) < INTEGRAL_START_THRESHOLD) 
					{
            // 死区控制：忽略微小误差，避免在目标点附近抖动
            if (fabs(pid->error) > INTEGRAL_DEAD_ZONE) 
							{
                pid->integral += pid->error * dt;
							}
            // 如果误差在死区内，不进行积分，保持稳定
					}
        // 如果误差大于积分分离阈值，不进行积分（积分分离）
			}
    
    // 3. 积分限幅保护
    if(pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if(pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    return pid->integral;
}

// 完整的微分计算函数（包含限幅和低通滤波）
float pid_servo_calculate_derivative(pid_servo_controller_t* pid, float dt)
{
    // 1. 计算原始微分值
    float derivative_raw = (pid->error - pid->last_error) / dt;
    
    // 2. 限制微分变化率，防止突变
    if(derivative_raw > pid->derivative + D_MAX_CHANGE)
			{
        derivative_raw = pid->derivative + D_MAX_CHANGE;
			} 
			else if(derivative_raw < pid->derivative - D_MAX_CHANGE) 
				{
        derivative_raw = pid->derivative - D_MAX_CHANGE;
				}
    
    // 3. 一阶低通滤波，抑制噪声 系数为0.3
    pid->derivative = D_FILTER_ALPHA_SERVO * derivative_raw + (1 - D_FILTER_ALPHA_SERVO) * pid->derivative;
    
    return pid->derivative;
}

float pid_servo_calculate(float target, float actual)
{		
		// 计算时间间隔（秒）
    uint32 current_time = timer_get(GPT_TIM_1); // 系统时间函数 获取的是ms
    float dt = (current_time - last_calc_time) / 1000.0f;//ms转化为s
    
   
		if(last_calc_time == 0)
			{
					last_calc_time = current_time;
					return 0; // 第一次直接返回0
			}
		if(dt <= 0 || dt > 0.02f){ dt = 0.02f; } // 添加上限
    last_calc_time = current_time;
	
	
    pid_servo.target = target;
    pid_servo.actual = actual;
    pid_servo.error = pid_servo.target - pid_servo.actual;
    
    // 积分函数
		pid_servo.integral = pid_servo_calculate_integral(&pid_servo, dt);
    // 微分项
    pid_servo.derivative = pid_servo_calculate_derivative(&pid_servo, dt);
    
    // PID输出 （kp+ki+kd）
		
		 if(ring_state!=0||Curve_Type_Flag!=0)
		{
		pid_servo.kp =4;
		pid_servo.ki =0;
			
		pid_servo.kd =0.3;
		}
		
		else
		{
		pid_servo.kp =3.6;
		pid_servo.ki =0;
		pid_servo.kd=0.25;
		}
	
		
    pid_servo.output = pid_servo.kp * pid_servo.error + 
                     pid_servo.ki * pid_servo.integral + 
                     pid_servo.kd * pid_servo.derivative;
    
    // 输出限幅
		
    if(pid_servo.output > pid_servo.output_limit) pid_servo.output = pid_servo.output_limit; 
		else if(pid_servo.output < -pid_servo.output_limit) pid_servo.output = -pid_servo.output_limit;
    
    pid_servo.last_error = pid_servo.error;
    
    return pid_servo.output;
}

float get_servo_turn_output(void) 
{
    return pid_servo.output; 
}

bool pid_servo_control( float actual_angle, pid_servo_output_t* output)
{
    float target_angle = 0.0f;
    float turn_output = pid_servo_calculate(target_angle, actual_angle);

    // 计算舵机命令
	
	if(Curve_Type_Flag>=1)		
	output->servo_angle = SERVO_MOTOR_M - turn_output * 0.195f;
  else if(ring_state>0)output->servo_angle = SERVO_MOTOR_M - turn_output * 0.175;
	else	output->servo_angle = SERVO_MOTOR_M - turn_output * 0.1875f;
//    // 计算电机速度命令
//    output->left_speed = base_speed - turn_output * 0.1f;
//    output->right_speed = base_speed + turn_output * 0.1f;

    return true; // 计算成功
}




