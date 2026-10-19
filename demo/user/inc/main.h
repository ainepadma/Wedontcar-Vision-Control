#include "zf_common_headfile.h"
#include "img_process.h"
#include "track_vision.h"
#include "track_control.h"
#include "imu_navigation.h"
#include "ring_control.h"

// 临时端点标定模式已关闭；设为1时电机强制为零，C12/C15以0.5度调节舵机。
#define SERVO_CENTER_TEST_MODE 0
#define SERVO_CALIBRATION_MIN (70.0f)
#define SERVO_CALIBRATION_MAX (110.0f)

// C12 同时控制屏幕与 USB 图传；默认开启，关闭后不刷新屏幕、不复制/发送预览帧。
#define SCREEN_TOGGLE_KEY C12
// C13 / KEY_3 控制两个驱动电机；上电默认停止，按住上电不会启动。
#define MOTOR_TOGGLE_KEY C13
// C14 / KEY_2 控制全部自动保护；默认开启，关闭后仅保留 C13 手动停机和 PWM 硬限幅。
#define PROTECTION_TOGGLE_KEY C14
// C15 / KEY_1 切换低速/全速；上电默认低速。
#define SPEED_MODE_TOGGLE_KEY C15
extern volatile uint8_t motor_enabled;
extern volatile uint8_t motor_feedback_fault; // 1=左、2=右、3=两侧；原因见 track_stop_reason
extern volatile uint8_t protection_enabled;
extern volatile uint8_t speed_full_enabled;
extern volatile uint8_t track_ready;
extern volatile uint8_t track_stop_reason;
extern volatile int16_t track_servo_tenths;
extern imu_navigation_state_t imu_navigation;
extern ring_control_state_t ring_control;
extern volatile uint8_t imu_device_status;
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
#define SERVO_MOTOR_L_MAX (98.5f)  // 实车标定的左侧安全端点
#define SERVO_MOTOR_R_MAX (75.5f)  // 实车标定的右侧安全端点
// 实车按键标定确认 87.0 度为机械正中。
#define SERVO_MOTOR_CENTER (87.0f)
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
