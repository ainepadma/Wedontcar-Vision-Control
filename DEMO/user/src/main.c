/*********************************************************************************************************************
* 本例程基于RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）开发
* 
* 修改记录
* 日期              作者                备注
* 2025-09-30        wjm            first version
********************************************************************************************************************/

// *************************** 例程硬件连接说明 ***************************
/*
				连接好电机、主板、驱动板
				将电磁前瞻与运放、运放与主板连接好
			（如果有编码器，将编码器安装好并与主板连接）
			
*/
// *************************** 例程测试说明 ***************************
/*
			本例程包含功能：
			1.采集电磁信号并经过ADC传回MCU
			2.电磁信号直接做差计算偏差
			3.计算舵机打角
			4.设置电机pwm使电机旋转
			
			
			注意！注意！注意！注意！
			注意！注意！注意！注意！
			注意！注意！注意！注意！

			1.测试舵机最左，最右，中值，并在mycommon.h中修改，也可以拆下舵盘，先单独将舵机值设置为90.0，后再将舵盘安装上，则舵机对应值与本例程一致
			2.若担心电机旋转方向与与期望方向相反，可调换电机正负两根线在驱动板上的连接位置（注意不是电池！），也可以直接在speed_control.c中修改DIR的电平
			3.将小车正放在电磁线上，电磁线在小车中轴线上，调节运放的值，使得最左边与最右边电感值相等
			4.用电磁信号计算偏差时，最左电感值和最右电感值默认存放在adc_buffer[0]和adc_buffer[3]中，若接线不同请自行修改
			

*/

// **************************** 代码区域 ****************************

#include "zf_common_headfile.h"
#include "my_common.h"
#include "control_manager.h"
#include "tft180.h"
#include "hall_sensor.h"
#include "motor.h"
#include "servo.h"
#include "adc.h"

#include <stdio.h>
#include <string.h> // 建议加上，以防万一



// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern float adbia;

extern volatile uint8 ring_state;


float angle;

// 全局命令缓冲区 定义
// 初始化为合理的默认值
volatile float g_servo_angle_cmd = 90.0f-4.4; // 舵机中位
volatile int8_t g_left_speed_cmd = 30;
volatile int8_t g_right_speed_cmd = 30;

extern float left_adc;
extern float leftshu_adc;
extern float rightshu_adc;
extern float right_adc;

extern uint8 race_state;
void Init_All(void);
uint16_t a1=90;


//调试相关
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART1_TX_A9
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART1_RX_A10

#define UART_PRIORITY           (LPUART1_IRQn)                                  // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体




uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区



// 定义一个缓冲区，大小要足够装下你要发的字符串
    char vofa_buffer[256];

uint8 get_data1 = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;

static uint8 uart_count=0;

uint8 gpio_status;

float b11=0;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化
    system_delay_ms(300);           //等待主板其他外设上电完成
		
		control_system_init();						//初始化所有外设
		pit_ms_init(PIT_CH, 10);//开启10ms中断
		
		tft180_test_show1();//tft180	
	
	 // 此处编写用户代码 例如外设初始化代码等
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

    uart_write_string(UART_INDEX, "UART Text.");                                // 输出测试信息
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');                                          // 输出换行
	
	
	b11=GetLeftSpeed();
    while(1)
    {
        tft180_test_show2();
       //hall(); 
			//调试显示 (保留你的调试代码，可根据需要调整显示内容)

    tft180_show_float(70, 60, left_adc, 4,3);
    tft180_show_float(70, 75, leftshu_adc, 4, 3);
    tft180_show_float(70, 90, rightshu_adc, 4 , 3 );
		tft180_show_float(70, 105, right_adc, 4, 3 );
			//tft180_show_float(70, 105, adbia, 4, 3 );
			
			// "\r\n" 代表回车换行，必须加，否则数据会挤在一行
//         sprintf(vofa_buffer, "L:%.3f, LS:%.3f, RS:%.3f, R:%.3f\n", 
//             left_adc, leftshu_adc, rightshu_adc, right_adc);
     //   sprintf(vofa_buffer, "ringstate  :%d\n", 
//ring_state);
			// 2. 串口发送逻辑
			    float ls = b11 + getchasul1();
				float rs = b11 + getchasur1();
        uart_count++; // 每次循环计数加 1
//        sprintf(vofa_buffer, "L:%.3f,L:%.3f\r\n", 
//                     ls,rs);
        sprintf(vofa_buffer, "L:%d,R:%d\r\n", 
                     encoder_data_1,encoder_data_2);
        if(uart_count >= 10) // 积攒到 10 次 (用 >= 比 == 更安全)
        {
            uart_count = 0; // 清零计数器

            // 【优化】: 只有要发送的时候，才进行格式化，节省CPU
            // 注意：我在最后加了 \r\n，这样兼容性最好
			sprintf(vofa_buffer, "L:%d,R:%d\r\n", 
                     encoder_data_1,encoder_data_2);
        //sprintf(vofa_buffer, "ringstate:%d\n", 
         //           ring_state);
            // 发送数据
            uart_write_string(UART_INDEX, vofa_buffer);
        }

//        // 3. 接收回显逻辑 (如果不需要接收命令，这段可以删掉，避免干扰)
//        fifo_data_count = fifo_used(&uart_data_fifo);
//        if(fifo_data_count != 0)
//        {
//            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
//            // 只是为了调试，可以不回显，直接处理数据
//            // uart_write_string(UART_INDEX, "Get:"); 
//            // uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);
       // }

//						set_servo_pwm(75);
//						system_delay_ms(500);
//						set_servo_pwm(85.6);
//						system_delay_ms(500);
//						set_servo_pwm(95);
//						system_delay_ms(500)
			
	}
}


//中断函数，在主函数中初始化为每10ms执行一次
void pit_handler ()//handler 自定义任务
{
		//读取和计算
      control_system_run();
			get_encoder();
     // 读取全局缓冲区的值并应用PWM
      set_servo_pwm(g_servo_angle_cmd);
      set_speed_pwm(g_left_speed_cmd, g_right_speed_cmd);
	
			//set_servo_pwm(75);
      //set_speed_pwm(18,18);
}
