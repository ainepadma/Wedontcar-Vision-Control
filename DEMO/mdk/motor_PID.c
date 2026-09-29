#include "zf_common_headfile.h"
#include "my_common.h"
#include "motor_PID.h"
#include "motor.h"
#include "servo.h"
#include "math.h"
#include "adc.h"

extern volatile int8_t g_right_speed_cmd;
extern volatile int8_t g_left_speed_cmd;
int32 last_PWM_Left;
int32 last_PWM_Right;


extern uint8 ring_state;
extern uint8 Straight_line_Flag;
extern uint8 Curve_Type_Flag;

// void brake_stop(void)				//刹车
// {
// 	 while (g_left_speed_cmd != 0 || g_right_speed_cmd != 0)
//     {
//         if (g_left_speed_cmd > 0)
//             g_left_speed_cmd -= PWM_Brake_Step;
//         else if (g_left_speed_cmd < 0)
//             g_left_speed_cmd += PWM_Brake_Step;

//         if (g_right_speed_cmd > 0)
//             g_right_speed_cmd -= PWM_Brake_Step;
//         else if (g_right_speed_cmd < 0)
//             g_right_speed_cmd += PWM_Brake_Step;

//         // 限幅防止越界
//         if (abs(g_left_speed_cmd) < PWM_Brake_Step)
//             g_left_speed_cmd = 0;
//         if (abs(g_right_speed_cmd) < PWM_Brake_Step)
//             g_right_speed_cmd = 0;

//         // 输出当前PWM
//         //        set_speed_pwm(last_PWM_Left, last_PWM_Right);

//         system_delay_us(500);

// 		}
// }

// 转弯PID控制器
pid_motor_controller_t pid_motor;

static uint32 last_calc_time = 0;
static int32 last_encoder_1 = 0, last_encoder_2 = 0;

// 初始化转弯PID
void pid_motor_init(void)
{
    pid_motor.kp = 3.0f;      // 需要根据实际调试
    pid_motor.ki = 0.0f;     // 积分系数较小，避免过冲
    pid_motor.kd = 0.6f;      // 微分系数
    
    pid_motor.target = 0;    // 目标转弯角度（0表示直行）
		// 实际转弯角度 需要通过实际参数传过来
    pid_motor.actual = 0;		
    pid_motor.error = 0;
    pid_motor.last_error = 0;
    pid_motor.integral = 0;   //积分项
    pid_motor.derivative = 0;  //微分项
    pid_motor.output = 0;
    
    pid_motor.integral_limit = 100;  // 积分限幅
    pid_motor.output_limit = 80;     // 输出限幅（占空比）
}

// 完整的抗积分饱和处理（包含输出饱和检测）
float pid_motor_calculate_integral(pid_motor_controller_t* pid, float dt)
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
float pid_motor_calculate_derivative(pid_motor_controller_t* pid, float dt)
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
    
    // 3. 一阶低通滤波，抑制噪声 系数为0.3 不稳定就进一步削弱
    pid->derivative = D_FILTER_ALPHA * derivative_raw + (1 - D_FILTER_ALPHA) * pid->derivative;
    
    return pid->derivative;
}

float pid_motor_calculate(float target, float actual)
{		
		// 计算时间间隔（秒）
    uint32 current_time = timer_get(GPT_TIM_1); // 系统时间函数 获取的是ms
    float dt = (current_time - last_calc_time) / 1000.0f;//ms转化为s
    
   
		if(last_calc_time == 0)
			{
					last_calc_time = current_time;
					return 0; // 第一次直接返回0
			}
		if(dt <= 0 || dt > 0.01f){ dt = 0.01f; } // 添加上限
    last_calc_time = current_time;
	if(ring_state>0||Curve_Type_Flag>0)
		
	{ 
		 pid_motor.kp = 6.0f;
		
	} 
		else
	{
		 pid_motor.kp = 2.4f;
	}
	
    pid_motor.target = target;
    pid_motor.actual = actual;
    pid_motor.error = pid_motor.target - pid_motor.actual;
    
    // 积分函数
		pid_motor.integral = pid_motor_calculate_integral(&pid_motor, dt);
    // 微分项
    pid_motor.derivative = pid_motor_calculate_derivative(&pid_motor, dt);
    
    // PID输出 （kp+ki+kd）
    pid_motor.output = pid_motor.kp * pid_motor.error + 
                     pid_motor.ki * pid_motor.integral + 
                     pid_motor.kd * pid_motor.derivative;
    
    // 输出限幅
    if(pid_motor.output > pid_motor.output_limit) pid_motor.output = pid_motor.output_limit; 
		else if(pid_motor.output < -pid_motor.output_limit) pid_motor.output = -pid_motor.output_limit;
    
    pid_motor.last_error = pid_motor.error;
    
    return pid_motor.output;
}

bool pid_motor_control( float target_speed, float actual_speed ,pid_motor_output_t* output)
{

    float output_speed = 0.15;
	// 计算电机命令
	if(ring_state>0||Curve_Type_Flag>0)	
	{ 
		 pid_motor.kp = 5.0f;
	} //限幅度80 调整1/4 20
		else
	{
		 pid_motor.kp = 2.4f;	 
	}
	
	
    float turn_output = pid_motor_calculate(target_speed, actual_speed);
    output_speed=target_speed+ turn_output * 0.1025f;
	
	if(output_speed<=30)
	{
	output->left_speed = output_speed;
    output->right_speed = output_speed;
	}
	
	else
	{
		output->left_speed = 30;
		output->right_speed = 30;

	}

    return true; // 计算成功
//		float turn_output = pid_motor_calculate(target_speed, actual_speed);

//	// 计算电机命令
//		float output_speed=target_speed+ turn_output * 0.1025f; //限幅度80 调整1/4 20
//		output->left_speed = output_speed;
//		output->right_speed = output_speed;

//		return true; // 计算成功
}


void brake_stop_hard(void)
{
    int16_t brake_left  = 0;
    int16_t brake_right = 0;

    // 1. 根据当前运行方向，生成反向制动力指令
    if (g_left_speed_cmd > 0)
        brake_left = -PWM_BRAKE_FORCE;      // 向前跑 → 给反向 PWM
    else if (g_left_speed_cmd < 0)
        brake_left =  PWM_BRAKE_FORCE;      // 向后跑 → 给正向 PWM

    if (g_right_speed_cmd > 0)
        brake_right = -PWM_BRAKE_FORCE;
    else if (g_right_speed_cmd < 0)
        brake_right =  PWM_BRAKE_FORCE;

    // 如果当前速度已经很小，就没必要反向了，直接清零
    if (abs(g_left_speed_cmd) < 0.5)
        brake_left = 0;
    if (abs(g_right_speed_cmd) < 0.5)
        brake_right = 0;

    // 限幅，避免超过 PWM 最大值
    if (brake_left > PWM_MAX)   brake_left =  PWM_MAX;
    if (brake_left < -PWM_MAX)  brake_left = -PWM_MAX;
    if (brake_right > PWM_MAX)  brake_right =  PWM_MAX;
    if (brake_right < -PWM_MAX) brake_right = -PWM_MAX;

    // 2. 一段时间内施加反向制动力（硬刹这一下）
    const uint16_t brake_time_ms = 60;  // 反向制动持续时间，可调
    for (uint16_t t = 0; t < brake_time_ms; t++)
    {
        g_left_speed_cmd  = brake_left;
        g_right_speed_cmd = brake_right;

        set_speed_pwm(g_left_speed_cmd, g_right_speed_cmd);

        system_delay_us(2000);  // 1ms
    }

    // 3. 制动结束后，把 PWM 平滑减到 0，防止抖动
    while (g_left_speed_cmd != 0 || g_right_speed_cmd != 0)
    {
        if (g_left_speed_cmd > 0)
            g_left_speed_cmd -= PWM_RAMP_STEP;
        else if (g_left_speed_cmd < 0)
            g_left_speed_cmd += PWM_RAMP_STEP;

        if (g_right_speed_cmd > 0)
            g_right_speed_cmd -= PWM_RAMP_STEP;
        else if (g_right_speed_cmd < 0)
            g_right_speed_cmd += PWM_RAMP_STEP;

        if (abs(g_left_speed_cmd) < 0.5)
            g_left_speed_cmd = 0;
        if (abs(g_right_speed_cmd) < 0.5)
            g_right_speed_cmd = 0;

        set_speed_pwm(g_left_speed_cmd, g_right_speed_cmd);
        system_delay_us(1000);  // 1ms
    }
		
}

