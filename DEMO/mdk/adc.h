#ifndef __ADC_H
#define __ADC_H
#include "zf_common_headfile.h"
#include "math.h"

#define FILTER_N     (7)//收集数据量
//定义数组
#define LEFT					(0)
#define	LEFT_SHU			    (1)
#define	RIGHT_SHU			    (2)
#define	RIGHT					(3)


#define ADC_MAX				(4096)			//最大值
#define ADC_MIN				(0)				//最小值

#define ALPHA               (0.85f)  // 旁边电感权重，从1开始
#define BETA                (0.425f)   // 中间电感权重，约为alpha的一半
#define OMEGA               (0.1f)  // 阻尼权重，一个较小的值开始
#define K_CS                (10.0f) // 差比和差的比例系
//赛道特征相关设定
#define SUM_HENG_STRAIGHT_LIMIT  (80.0f)  
#define SUM_SHU_STRAIGHT_LIMIT  (10.0f)  
//直道特征
#define SUM_SHU_STRAIGHT_MAX   10.0f   // 竖向很小
#define DIFF_STRAIGHT_MAX      10.0f   // 左右差不大
#define SUM_HENG_MIN           10.0f   // 有线（防止丢线也被当直道
//弯道特征
#define SUM_SHU_CURVE_MIN        (12.0f)   // 竖向能量达到12，才具备“弯道可能”
#define DIFF_HENG_CURVE_MIN_WEAK (40.0f)   // 横着的差值 时，只要diff>=6就算弯（入弯情况）
#define DIFF_SHU_CURVE_MIN_WEAK (30.0f)   // 竖着的差值 时，只要diff>=6就算弯（入弯情况）
#define DIFF_HENG_CURVE_MIN_STRONG (10.0f) // 左右横着的电感的最小差值



//环岛相关
#define ROUND_SUM_TOTAL_ENTER_LIMIT    (150.0f) // 环岛进入时的总信号
#define NORMAL_ROUND_SUM_HENG_ENTER_LIMIT     (130.0f)
#define SHIZI_ROUND_SUM_SHU_ENTER_LIMIT    (65.0f)
#define HORIZON_SUM_TOTAL_EXIT_LIMIT   (140.0f) // 环岛内部或退出时的横向信号低阈值

#define ROUND_ENTRY_THRESHOLD    (3)      // 环岛进入的防抖次数
#define ROUND_MIN_HOLD_CYCLES    (30)    // 环岛内保持的最小时间
#define ROUND_COOLDOWN_CYCLES    (30)    // 退出后的冷却时间
#define ROUND_PREPARE_DELAY_CYCLES         (50)     // 探头看到圆环→车身真正进环 的延时
#define ROUND_EXIT_DELAY_CYCLES            (50)     // 探头先离开圆环→车身真正出环 的延时

float ADC_GetLeft(void);
float ADC_GetLeftShu(void);
float ADC_GetRightShu(void);
float ADC_GetRight(void);
float ADC_GetAngle(void);

uint8 ADC_IsValid(void);
uint8_t ADC_IsStraight(void);
uint8_t ADC_IsCurve(void);
void Is_Large_Curve(float signed_diff, float abs_diff);
void Is_Small_Curve(float signed_diff, float abs_diff);
uint8_t ADC_IsRound(void);
void ADC_GetAllData(float *left, float *left_shu, float *right_shu, float *right, float *bias);


void get_encoder();
float GetLeftSpeed(void);
float GetRightSpeed(void);
// 直道相关
uint8 Get_Straight_line_Flag(void);

// 弯道相关
uint8 Get_Cruve_Flag(void);
uint8 Get_Curve_Type_Flag(void);

// 环岛相关
uint8 Get_ROUND_Flag(void);
uint8 Get_Change_ROUND(void);
uint8 Get_Round_Type_Flag(void);

//AD bias 计算相关
#define HENG_DIFF_ZERO            (0.0f) // 横向差值零点偏移(就是直道刚刚好的时候有左右heng差别)
#define HENG_GAIN                 (10.20f)   
#define SHU_GAIN                  (10.35f)   
#define AD_BIAS_LIMIT_GLOBAL      (80.0f) // 全局限幅：最多认为偏 
#define SHU_STRONG_LEVEL          (20.0f)



float clamp_value(float val, float min_val, float max_val);
#define MAX_SHU_FOR_WEIGHT           (120.0f)
// 纵向传感器总和达到此阈值时，纵向偏差权重 W_shu 达到 1.0，
//用于标定弯道和直道平滑过渡的上限。


#endif
