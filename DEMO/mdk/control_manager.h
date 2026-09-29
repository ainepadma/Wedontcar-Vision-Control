#ifndef _contrlo_manager_h
#define _contrlo_manager_h

#include "zf_common_headfile.h"

// 控制系统状态结构
typedef struct {
    int8 left_speed;
    int8 right_speed;
    float servo_angle;
    uint8 system_ready;
		uint8 system_motor_ready;
		uint8 system_servo_ready;
	
} control_state_t;

#define BASE_SPEED_STRAIGHT          (25)   // 直道速度
#define BASE_SPEED_SMALL_CURVE       (22)   // 小弯道速度
#define BASE_SPEED_ROUND_SHIZI       (22)   // 十字圆环速度 
#define BASE_SPEED_DEFAULT           (22)   // 默认/未知速度


// 系统初始化
void control_system_init(void);
// 系统运行主函数
void control_system_run(void);
// 设置基础速度
void control_set_base_speed(int8 speed);
// 获取当前控制状态
control_state_t* control_get_state(void);


float getchasul1(void);
float getchasur(void);
#endif