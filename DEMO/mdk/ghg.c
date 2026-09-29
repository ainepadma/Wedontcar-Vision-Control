//#include "zf_common_headfile.h"
//#include "my_common.h"
//#include "motor_PID.h"
//#include "control_manager.h"
//// 全局变量（停车状态标记）
//volatile uint8_t reed_status = 1;        // 干簧管状态：1=断开（无磁铁），0=吸合（检测到磁铁）
//volatile uint8_t stop_flag = 0;   // 停车锁存标志：0=未停车，1=已停车（锁存后不恢复）
//volatile uint8_t ghg_flag =0;//标志位
//volatile uint8_t last_reed_status = 1;   // 上一次干簧管状态（用于检测“从断开→吸合”）
//void reed_switch_read(void)
//{
////    uint8_t temp1 = gpio_get_level(REED_SWITCH_PIN);  // 第一次读取
////    system_delay_ms(5);                                      // 防抖延时
////    uint8_t temp2 = gpio_get_level(REED_SWITCH_PIN);  // 第二次读取
////    
////    // 两次读取电平一致，才更新状态（防抖核心）
////    if(temp1 == temp2)
////    {
////        reed_status = (temp2 == 0) ? 0 : 1;  // 更新当前状态
////			
////        if(last_reed_status == 1 && reed_status == 0 && stop_flag == 0)
////        {
////            ghg_flag++;  // 仅在“吸合”时计数+1
////        }
////        
////        last_reed_status = reed_status;  // 更新上一次状态
////    }
//    reed_status = (gpio_get_level(REED_SWITCH_PIN) == 0) ? 0 : 1;
//    
//	  system_delay_ms(10);
//    if(last_reed_status == 1 && reed_status == 0 && stop_flag == 0 && ghg_flag < 2)
//    {
//        ghg_flag++; 
//    }

//    // 更新上一次状态（必须保留，否则无法检测跳变）
//    last_reed_status = reed_status;
//}
//void car_stop(void)
//{

//		 if(ghg_flag == 2)
//    {
//        stop_flag = 1;  // 锁存停车状态（避免重复触发）
//        
//			  g_left_speed_cmd  = 0;
//            g_right_speed_cmd = 0;
//            brake_stop_hard();
//            //g_control_state.system_ready = 0;
//						g_servo_angle_cmd=85.6;
//    }
//}