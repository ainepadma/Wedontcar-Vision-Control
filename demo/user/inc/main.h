#include "zf_common_headfile.h"
#include "img_process.h"
#include "track_vision.h"
#include "track_control.h"

// 主板 C12 / KEY_4，低电平按下；不要按住此键上电或复位。
#define SCREEN_TOGGLE_KEY C12
// C13 / KEY_3 控制两个驱动电机；上电默认停止，按住上电不会启动。
#define MOTOR_TOGGLE_KEY C13
// C14 / KEY_2 切换丢线自动停机；上电默认开启，关闭不停止图像循迹。
#define TRACK_LOSS_STOP_TOGGLE_KEY C14
extern volatile uint8_t motor_enabled;
extern volatile uint8_t motor_feedback_fault; // 1=左、2=右、3=两侧；原因见 track_stop_reason
extern volatile uint8_t track_loss_stop_enabled;
extern volatile uint8_t track_ready;
extern volatile uint8_t track_stop_reason;
extern volatile int16_t track_servo_tenths;
void assistant_uart_tx_irq_handler(void);
void track_runtime_init(void);
void track_frame_publish(const track_vision_result_t *result);

// 编码器引脚信息
// 前进反馈须为正；DIR=LOW 实车正转触发 ENC ERR:3，修正两路反馈极性。
// 电机方向与编码器符号分别标定，不要用 abs() 掩盖反号。
// 左轮编码器
#define ENCODER_1 (QTIMER1_ENCODER1)
#define ENCODER_1_A (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B (QTIMER1_ENCODER1_CH2_C1)
#define ENCODER_1_FORWARD_SIGN (1)
// 右轮编码器
#define ENCODER_2 (QTIMER1_ENCODER2)
#define ENCODER_2_A (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B (QTIMER1_ENCODER2_CH2_C24)
#define ENCODER_2_FORWARD_SIGN (-1)

// 电机参数设置
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!
// 左轮电机
#define MOTOR1_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR1_DIR (C7)
// 根据当前实车反转反馈修正：前进使用低电平。
#define MOTOR1_FORWARD_DIR_LEVEL (GPIO_LOW)
// 右轮电机
#define MOTOR2_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C9)
// 根据当前实车反转反馈修正：前进使用低电平。
#define MOTOR2_FORWARD_DIR_LEVEL (GPIO_LOW)


// 舵机参数设置
#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)                  // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (100)  // 定义主板上舵机活动范围(左打方向的极限值) 角度 自行标定
#define SERVO_MOTOR_R_MAX (85) // 定义主板上舵机活动范围(右打方向的极限值) 角度 自行标定
// 独立中位，当前保持原来的 92.5 度；实际直行位置需架空标定，不能照搬参考车数值。
#define SERVO_MOTOR_CENTER (92.5f)
#define SERVO_DIR (SERVO_MOTOR_L_MAX>SERVO_MOTOR_R_MAX?-1.f:1.f) // 根据左右duty的大小自动决定舵机方向

// 速度目标位于 track_control.h，单位是编码器计数/10ms，不是 RPM 或 m/s。
// 舵机左右端点仍使用本车原配置；必须架空验证方向和机械限位后才能落地。
#define TRACK_STEER_KP 0.020f
#define TRACK_STEER_KD 0.0005f
#define TRACK_SERVO_RATE_DEG_S 120.0f

// 以下宏在初步测试例程时不可更改，后面若需更改舵机算法可以自行更改
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif
