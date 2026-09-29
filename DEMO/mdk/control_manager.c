#include "zf_common_headfile.h"
#include "my_common.h"
#include "control_manager.h"
#include "adc.h"
#include "servo.h"
#include "servo_pid.h"
#include "motor.h"
#include "motor_PID.h"
#include "hall_sensor.h"
#include "tft180.h"
#include "ring.h"
#include "ghg.h"

static control_state_t g_control_state = {0};
static int8 g_base_speed = 10;

// 声明全局缓冲区变量
extern volatile float g_servo_angle_cmd;
extern volatile int8_t g_left_speed_cmd;
extern volatile int8_t g_right_speed_cmd;
extern volatile uint8 ring_state;

extern int16 encoder_data_1;
extern int16 encoder_data_2;

   pid_servo_output_t pid_servo_result;
	 pid_motor_output_t pid_motor_left_result;
	 pid_motor_output_t pid_motor_right_result;  // 添加右电机PID输出变量声明
    float chasur1=0;
	  float chasul1=0;
		float k1=1;

void control_system_init(void)
{		
		clock_init(SYSTEM_CLOCK_600M);  // 不可删除
		debug_init();                   // 调试端口初始化
	
		//时间初始化
		timer_init(GPT_TIM_1, TIMER_MS);
    // 启动定时器1
    timer_start(GPT_TIM_1);
	
		//初始化电机驱动DRV8701
		gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0

		//根据编码器类型和个数选择初始化函数
	
		//初始化编码器（正交）
		//左轮编码器
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
		//右轮编码器
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式
	
		//初始化adc引脚
		adc_init(Left_ADC_Pin, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(Left_Shu_Pin, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(Right_Shu_Pin, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(Right_ADC_Pin, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
		
		//初始化舵机引脚
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);

		//初始化屏幕
		tft180_init();
		tft180_set_dir(TFT180_CROSSWISE);	
		
		//初始化霍尔传感器引脚
		gpio_init(HALL_SENSOR_PIN, GPI, GPIO_HIGH, GPI_PULL_DOWN);
		gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
		
//		//初始化干簧管引脚
//		gpio_init(REED_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_DOWN);
		
		
		//pid初始化
		pid_servo_init();
		pid_motor_init();

     ring_init();
}

void control_set_base_speed(int8 speed)
{
    g_base_speed = speed;
 
}
// 获取控制状态
control_state_t* control_get_state(void)
{
    return &g_control_state;
}


extern volatile uint8_t race_state;
uint8 stop_car_count=0;
 static stop_car=0;
void control_system_run(void)
{ 		
	
	hall();
	int8_t current_base_speed = BASE_SPEED_DEFAULT;
    
//	//干簧管
//	reed_switch_read();
//	car_stop();
//	 system_delay_ms(1);
	if (race_state == 2){control_set_base_speed(0);}
   else{
     control_set_base_speed(10); // 这行可以保留或移动到别处
	 }
	if (race_state == 2)//stop_car_count++;
	//if(stop_car_count>=20)
    {
       
        //brake_stop_hard();
			if(stop_car<=4){current_base_speed =-30;chasul1=0;chasur1=0;}
			else  {current_base_speed =0;chasul1=0;chasur1=0;}
				stop_car++;
			

         // 直接退出函数，后面的 PID 计算、循迹逻辑全部不执行
    }
			// 可以在这里根据需要调整基础速度
		
	//获取传感器数据
    float adc_bias = ADC_GetAngle();

	//float adc_bias = 0;
		float current_leftspeed=GetLeftSpeed();
		float current_rightspeed=GetRightSpeed();
	//获取所有赛道特征状态
    uint8_t adc_valid = ADC_IsValid();//是否有效
    // 各种状态
    uint8_t is_round = Get_ROUND_Flag();
		uint8_t is_cooldown =Get_Change_ROUND();//冷却期计数器作为标志
		uint8_t round_type = Get_Round_Type_Flag(); 
	//0， 1:普通圆环, 2:十字圆环
    uint8_t is_straight = Get_Straight_line_Flag();
    uint8_t curve_type = Get_Curve_Type_Flag(); 
	// 0, 1小左, 2小右, 3大左, 4大右
	
	// 用于存储当前赛道模式的基础速度
	if (race_state == 2){current_base_speed=0;}
   else{  current_base_speed = BASE_SPEED_DEFAULT;}
		
		static uint8_t straight_count=0;
		
    static uint8 lose_line_count = 0;
		// 信号有效性判断
		float turn_output = get_servo_turn_output();
     if (!adc_valid)
    {
        lose_line_count++;
        if (lose_line_count >= 15)     // 连续 15 帧无信号才停车（约 15ms）
        {
            g_left_speed_cmd  = 0;
            g_right_speed_cmd = 0;
            brake_stop_hard();
            g_control_state.system_ready = 0;
						g_servo_angle_cmd=85.6;
            return;
        }
    }
    else lose_line_count = 0;
		   
		if(fabs(adc_bias)<3)
		{
				chasul1=0;
				chasur1=0;
			k1=1;
		}
		
		else
		{
				if (ring_state)
				{
						straight_count=0;
						if (race_state == 2){current_base_speed=0;}
							else {current_base_speed = BASE_SPEED_ROUND_SHIZI;}
							
					
						if (adc_bias>0)
						{

							if(fabs(turn_output*0.5f)<5)chasul1=-fabs(turn_output*1.20f),chasur1=0;
							else chasul1=-5,chasur1=0;
						}
						else if(adc_bias<0)
						{

								if(fabs(turn_output*0.5f)<5)chasur1=-fabs(turn_output*1.20f),chasul1=0;
								else chasur1=-5,chasul1=0;
						}

								// TODO: [未来可在此处添加环岛专用的特殊转向逻辑，例如在入环和出环时强制修正舵机角度]
				}

				else if (is_straight)
				{
						// B. 优先级 2: 直道策略
					straight_count++;
					if (race_state == 2){current_base_speed=0;}
					else current_base_speed = BASE_SPEED_STRAIGHT; // + straight_count*0.2;
					chasur1=0;
					chasul1=0;
						
				}
				else // 不是环岛也不是直道，则检查弯道
				{		

						if (curve_type == 1 || curve_type == 2) // 弯道
						{		
							if (race_state == 2){current_base_speed=0;}
								else {		current_base_speed = BASE_SPEED_SMALL_CURVE;}
							k1=1.2;
									if (adc_bias>0)
								{

									if(fabs(turn_output*1.0f)<7)chasul1=-fabs(turn_output*1.50f),chasur1=fabs(turn_output*0.04f);
									else chasul1=-10,chasur1=0.4;
								}
								else if(adc_bias<0)
								{

										if(fabs(turn_output*1.0f)<7)chasur1=-fabs(turn_output*1.10f),chasul1=fabs(turn_output* 0.595f);
										else chasur1=-5,chasul1=2.4;
								}
						}
									
						else
						{							
//								current_base_speed = BASE_SPEED_DEFAULT; 
//								if (adc_bias>0)
//								{

//									if(fabs(turn_output*1.0f)<5)chasul1=-fabs(turn_output*1.0f),chasur1=0;
//									else chasul1=-5,chasur1=0;
//								}
//								else if(adc_bias<0)
//								{

//										if(fabs(turn_output*1.0f)<5)chasur1=-fabs(turn_output*1.0f),chasul1=0;
//										else chasur1=-5,chasul1=0;
//								}
							if (race_state == 2){current_base_speed=0;}
						else{	current_base_speed = BASE_SPEED_SMALL_CURVE;}
						
									if (adc_bias>0)
								{

									if(fabs(turn_output*1.0f)<7)chasul1=-fabs(turn_output*1.50f),chasur1=fabs(turn_output*0.04f);
									else chasul1=-10,chasur1=0.4;
								}
								else if(adc_bias<0)
								{

										if(fabs(turn_output*1.0f)<7)chasur1=-fabs(turn_output*1.10f),chasul1=fabs(turn_output* 0.595f);
										else chasur1=-5,chasul1=2.4;
								}
						}
				}
			}

    // 5. 更新全局基础速度
			if (race_state == 2){current_base_speed=0;}
			else{
			control_set_base_speed(current_base_speed); // 更新 g_base_speed
			}
    // 6. 执行转向控制（统一的 PID 计算）
    // 注意：该函数会使用 adc_bias (无论是弯道、环岛还是直道计算出来的偏差) 
    // 来计算舵机角度和左右轮差速。
//		float turn_output = get_servo_turn_output();
//		if (adc_bias>0)chasur1=-3.5,chasul1=0;
//		else if(adc_bias<0)chasul1=-3.5,chasur1=0;
    
    if (pid_servo_control( adc_bias, &pid_servo_result))
    {  
        // 结果写入全局缓冲区
        g_servo_angle_cmd = pid_servo_result.servo_angle;

        g_control_state.system_servo_ready = 1;
			if(g_control_state.system_servo_ready&&g_control_state.system_motor_ready)g_control_state.system_ready=1;
    }
// 		if (pid_motor_control( current_base_speed,current_leftspeed, &pid_motor_left_result))
//     {  
//         // 结果写入全局缓冲区
//         //左轮右轮都一样 直接写成相等了
// //        g_left_speed_cmd = pid_motor_left_result.left_speed+chasur1;
// //        g_right_speed_cmd =pid_motor_left_result.left_speed+chasul1-1;
// 				g_right_speed_cmd = pid_motor_left_result.left_speed+chasur1;
//         g_left_speed_cmd =k1*pid_motor_left_result.left_speed+chasul1;
//         g_control_state.system_motor_ready = 1;
// 			if(g_control_state.system_servo_ready&&g_control_state.system_motor_ready)g_control_state.system_ready=1;
//     }	
 	
    uint8_t left_motor_success = pid_motor_control(current_base_speed + chasul1, current_leftspeed, &pid_motor_left_result);
    uint8_t right_motor_success = pid_motor_control(current_base_speed + chasur1, current_rightspeed, &pid_motor_right_result);
    
    if (left_motor_success)
    {
        g_left_speed_cmd = k1 * pid_motor_left_result.left_speed ;
    }
    
    if (right_motor_success)
    {
        g_right_speed_cmd = pid_motor_right_result.right_speed ;
    }
    
    if (left_motor_success && right_motor_success)
    {
        g_control_state.system_motor_ready = 1;
        if(g_control_state.system_servo_ready && g_control_state.system_motor_ready)
            g_control_state.system_ready = 1;
    }
}
float getchasur1()
{
    return chasur1;
}
float getchasul1()
{
    return chasul1;
}
