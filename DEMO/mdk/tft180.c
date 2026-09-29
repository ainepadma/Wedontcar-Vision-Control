#include "zf_common_headfile.h"
#include "my_common.h"
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern int16 encoder_data_2;
extern float adc_error;
extern int8 right_speed;
extern float ADC_Compensate;

extern uint8 Straight_line_Flag;	
extern uint8 race_state;
extern uint8 Curve_Type_Flag;
extern volatile uint8 ring_state;
extern volatile uint8 ghg_flag;	
void tft180_test_show1(void)
{
		tft180_show_string(20, 0, "ChengCar_demo");
//		tft180_show_string(0, 15, "ADC1:");
//		tft180_show_string(0, 30, "ADC2:");
//		tft180_show_string(0, 45, "ADC3:");
//		tft180_show_string(0, 60, "ADC4:");
//		tft180_show_string(0, 75, "ERROR");
		tft180_show_string(0, 15, "extra:");
		tft180_show_string(0, 30, "Curve:");
		tft180_show_string(0, 45, "Ring:");
		tft180_show_string(0, 60, "left_adc:");
		tft180_show_string(0, 75, "leftshu_adc:");
		tft180_show_string(0, 90, "rightshu_adc:");
		tft180_show_string(0, 105, "adbia:");
		//tft180_show_string(0, 120, "adbias:");
	//	tft180_show_string(0, 135, "extra:");
	//tft180_show_string(0,120,"ghg_flag:");

	
}

void tft180_test_show2(void)
{
//			tft180_show_int(50, 15, adc_buffer[0], 4);
//      tft180_show_int(50, 30, adc_buffer[1], 4);
//			tft180_show_int(50, 45, adc_buffer[2], 4);
//			tft180_show_int(50, 60, adc_buffer[3], 4);
//			tft180_show_float(50, 75, adc_error, 4, 1);
			tft180_show_float(70, 15, ADC_Compensate, 4,3);
			tft180_show_int(70, 30, encoder_data_1, 4);
			tft180_show_int(50, 45, encoder_data_2, 3);
	//tft180_show_int(50,60,ghg_flag,3);
}