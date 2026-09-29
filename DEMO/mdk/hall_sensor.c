
// 备注信息
// 定义霍尔传感器引脚
//#define HALL_SENSOR_PIN       		  (D4)    
//#define LED1        								(B9)    // LED指示灯引脚 板子上自带

#include "zf_common_headfile.h"
#include "my_common.h"
// 定义全局变量 (记得在头文件里 extern 出去，让 control_manager 能看到)
// 0: 等待发车 (Waiting)
// 1: 正在比赛 (Running)
// 2: 比赛结束 (Finished)
volatile uint8_t race_state = 0; // 0:等待, 1:跑动, 2:结束
uint32_t race_timer = 0;         // 比赛计时器/安全计时器

void hall(void)
{
    static uint8_t hall_lock = 0; 
    
    // 如果已经在比赛中，增加计时器
    // 假设这个函数每10ms调用一次，那么200就代表2秒
    // 如果调用频率很高，请适当加大这个数字
    if (race_state == 1)
    {
        race_timer++; 
    }

    // --- 霍尔检测逻辑 ---
    if (gpio_get_level(HALL_SENSOR_PIN) == 0) // 检测到磁铁
    {
        gpio_set_level(LED1, GPIO_HIGH); // 亮灯
        
        if (hall_lock == 0) // 必须是新触发的边缘
        {
            hall_lock = 1; // 锁住
            
            // === 状态 0 -> 1 : 起跑 ===
            if (race_state == 0)
            {
                race_state = 1; 
                race_timer = 0; // 清零计时器，开始记录安全时间
            }
            
            // === 状态 1 -> 2 : 停车 ===
            else if (race_state == 1)
            {
                // 【核心保护逻辑】
                // 只有当跑出去的时间超过 200 个周期（比如2秒）后，才允许检测停车信号
                // 这样就完美避开了起跑线上的重复触发
                if (race_timer > 20) 
                {
                    race_state = 2; // 停车
                }
            }
        }
    }
    else // 磁铁离开
    {
        gpio_set_level(LED1, GPIO_LOW); // 灭灯
        hall_lock = 0; // 解锁
    }
}



//uint32 trigger_count = 0;//等待时长
//uint32 trigger_state = 0;
//void hall(void)
//{
//	if (gpio_get_level(HALL_SENSOR_PIN))  // 检测霍尔引脚是否为高电平
//    {
//        // 磁铁离开：停止计时并记录时间
//       // timer_stop(GPT_TIM_1);
//       // trigger_count = timer_get(GPT_TIM_1);  // 获取磁铁停留时间
//        //timer_clear(GPT_TIM_1);                // 清除定时器计数
//        
//				trigger_state = 0;                     	// 重置触发状态
//        gpio_set_level(LED1, GPIO_LOW); 				// LED灭
//      
//    }
//    else  // 霍尔引脚为低电平（磁铁靠近）
//    {
//        gpio_set_level(LED1, GPIO_HIGH);      // LED亮
//        
//        if (0 == trigger_state)               // 如果是新的一次触发
//        {
//            trigger_state = 1;                // 标记已触发
//          //冲突  timer_start(GPT_TIM_1);           // 开始计时
//        }
//    }
//    
////    if (trigger_count)  // 如果有计时结果需要输出
////    {
////        printf("\r\nHall trigger: %d ms.", trigger_count);  // 串口输出停留时间
////        trigger_count = 0;                    // 清除计时结果
////    }
//}