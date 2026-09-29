#ifndef _mycommon_h
#define _mycommon_h

#include "zf_common_headfile.h"

 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_CH                      (PIT_CH0)                                
#define LED1 (B9)

//占空比限幅
#define MAX_PWM_DUTY            (50) 

//定义电机驱动引脚
#define MOTOR1_DIR               (C9)
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR               (C7)
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR               (D2)
#define MOTOR3_PWM               (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR               (C10)
#define MOTOR4_PWM               (PWM2_MODULE2_CHB_C11)

//定义编码器引脚
#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

//定义电磁信号adc引脚和相关数据
#define ADC_CHANNEL_NUMBER          (4) //使用电感个数

#define Left_ADC_Pin            (ADC1_CH3_B14)
#define Left_Shu_Pin            (ADC1_CH4_B15)
#define Right_Shu_Pin            (ADC1_CH10_B21)
#define Right_ADC_Pin           (ADC1_CH12_B23)

//定义舵机引脚
#define SERVO_MOTOR1_PWM             (PWM4_MODULE2_CHA_C30)                         // 定义主板上舵机对应引脚
#define SERVO_MOTOR2_PWM             (PWM1_MODULE3_CHA_D0)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR3_PWM             (PWM1_MODULE3_CHB_D1)                          // 定义主板上舵机对应引脚

#define SERVO_MOTOR_FREQ            (50)                                           // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (70.6 )                                           // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (102.6)                                           // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_M          		  (85.6)																						// 定义主板上舵机活动中值 角度

// 定义霍尔传感器引脚
#define HALL_SENSOR_PIN       		  (D4)    
#define LED1        								(B9)    // LED指示灯引脚
	
////干簧管引脚
//#define REED_SWITCH_PIN    (D4)    // 干簧管连接的GPIO引脚


// 1. 舵机角度命令
extern volatile float g_servo_angle_cmd;

// 2. 电机速度命令
extern volatile int8_t g_left_speed_cmd;
extern volatile int8_t g_right_speed_cmd;

// 3. 更新标志
extern volatile bool g_cmd_update_flag;



#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif


#endif
