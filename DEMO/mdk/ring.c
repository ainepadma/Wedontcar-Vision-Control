// ring_handler.c
#include "ring.h"
#include "adc.h"        // 用于获取 left_adc, right_adc, leftshu_adc, rightshu_adc
#include "my_common.h"  // 假设有全局标志位 flag

// 全局变量定义
float ADC_Compensate = 0;
volatile int16 zt4=0;
volatile int16 zt3=0;
volatile uint8 ring_state = RING_IDLE;
float qzz=0.0;
// 添加环岛冷却计数器
static uint16 ring_cool_down_counter = 0;
static const uint16 RING_COOL_DOWN_PERIOD = 100 ; // 冷却周期，可根据实际需要调整


// 辅助函数：获取四路电感值（需你实现）
static void get_adc_channels(float *left_adc,  float *leftshu_adc, float *rightshu_adc , float *right_adc)
 {
    *left_adc = ADC_GetLeft();      
    *right_adc = ADC_GetRight();    
    *leftshu_adc = ADC_GetLeftShu();   
    *rightshu_adc = ADC_GetRightShu();  
}

void ring_handle(void) {
    static uint16 Ring_Delay = 0;
    static uint8 is_left_ring = 0;
    
    float left_adc,  leftshu_adc, rightshu_adc, right_adc;
    get_adc_channels(&left_adc, &leftshu_adc, &rightshu_adc ,&right_adc);

    // 每周期递减延时（假设主循环周期≈10ms）
    if (Ring_Delay > 0||ring_cool_down_counter > 0)
    {
        if(Ring_Delay>0) Ring_Delay--;
        if(ring_cool_down_counter > 0) ring_cool_down_counter--;
    }
		else
		{
    switch (ring_state) {
        case RING_IDLE:
            // 右环检测：right_adc强 + left_adc弱 + rightshu_adc有效
            if ((rightshu_adc- leftshu_adc> 45)&&
							(left_adc+right_adc>120)&&
							(left_adc>65||right_adc>65)	)
						{//
                ring_state = RING_DETECTED;
                is_left_ring = 1,qzz=1;
								Ring_Delay = 40;
                // 设置标志位（如果需要）
            }
            // 左环检测：left_adc强 + right_adc弱 + leftshu_adc有效
            else if ((leftshu_adc- rightshu_adc> 45)&&
							(left_adc+right_adc>120)&&
							(left_adc>65||right_adc>65)) //
							{
                ring_state = RING_DETECTED;
                is_left_ring = 0,qzz=-1;
								Ring_Delay =40;
								;
            }
            break;

        case RING_DETECTED:
            if (is_left_ring) {
                ADC_Compensate =-qzz*10;   // 
                Ring_Delay = 30;      // 300ms 引导时间
            } else {
                ADC_Compensate =  -qzz*10;  // 
                Ring_Delay = 30;
            }
            ring_state = RING_ENTER_GUIDE;
            break;

        case RING_ENTER_GUIDE: 
           if (( (left_adc + right_adc > 120 && fabsf(right_adc - left_adc) > 30) )
						 ||(fabsf(rightshu_adc-leftshu_adc)<15)&&(rightshu_adc+leftshu_adc<30))
						 //|| Ring_Delay == 0 )
				//if(((fabsf(rightshu_adc-leftshu_adc)<15)&&rightshu_adc+leftshu_adc<30)|| Ring_Delay == 0)
								{
						Ring_Delay = 30;
						ADC_Compensate = -qzz * 15;
						ring_state = RING_INSIDE;
								}
            break;

        case RING_INSIDE:
            // 出口检测：垂直信号突增（需实测阈值）
				ADC_Compensate=0;//-qzz*1;
				if(is_left_ring)
					{
				
						
            if ( (rightshu_adc > 60) &&
                 ((leftshu_adc + rightshu_adc) > 80.0f ))  //||
                // Ring_Delay == 0) 
						{
                Ring_Delay = 30;
                ADC_Compensate =-qzz*10;//*3;//检测到出口命令他转
								
                ring_state = RING_EXIT_DETECT;
							
						}
				
				}
					else
					{
				
            if ( (leftshu_adc > 60) &&
                 ((leftshu_adc + rightshu_adc) > 80.0f ))  //||
                // Ring_Delay == 0) 
						{
                Ring_Delay = 30;
                ADC_Compensate =-qzz*10;//*3;//检测到出口命令他转
								
                ring_state = RING_EXIT_DETECT;
							
						}
				
				}
						// zt3++;
						//	if(zt3>15)
						//	{
							//	zt3=0;
							//	ring_state=RING_EXIT_DETECT;
							//	ADC_Compensate=0;
							//}
            break;

        case RING_EXIT_DETECT:
            if (((fabsf(left_adc - right_adc) < 40) && 
							(left_adc > 50 || right_adc > 50)&&(rightshu_adc+leftshu_adc)>=60)||Ring_Delay==0)  //||
							//ring_state==RING_EXIT_DETECT) 
						{
							ADC_Compensate=-5*qzz;//8*qzz;
                Ring_Delay = 10;
                ring_cool_down_counter=RING_COOL_DOWN_PERIOD; 
                ring_state =RING_EXITING;
            }
//							zt4++;
//							if(zt4>10)
//							{zt4=0;
//								ADC_Compensate=8*qzz;
//                Ring_Delay = 70;
//								ring_state=RING_IDLE;
//							}
            break;

        case RING_EXITING:
            // 出环反向补偿（防冲出）
            ADC_Compensate = 5*qzz;// 
						
            if (left_adc + right_adc < 50 || Ring_Delay == 0) {
				
        //if((rightshu_adc+leftshu_adc<15))  
					ring_state = RING_DONE;
           }
            break;

        case RING_DONE:
            ADC_Compensate = 0;
            ring_state = RING_IDLE;
            break;
    }
	}
}

void ring_init(void) {
    ADC_Compensate = 0;
    ring_state = RING_IDLE;
}