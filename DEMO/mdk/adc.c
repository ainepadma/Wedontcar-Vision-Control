#include "zf_common_headfile.h"
#include "my_common.h"
#include "adc.h"
#include "math.h"
#include "ring.h"

// 全局变量：用于存储编码器计数值 
int16 encoder_data_1 = 0; // 左轮编码器数据 (平均值)
int16 encoder_data_2 = 0; // 右轮编码器数据 (平均值)

float adc_max[4];
float adc_min[4];

// 存储四个ADC通道经过滤波和归一化后的最终值
float adc_deal_1[4];
float adc_deal_2[4];

// 存储最终归一化到0-100范围的ADC值，用于后续控制逻辑
float left_adc;     // 左侧横向传感器
float leftshu_adc;  // 左侧纵向传感器
float rightshu_adc; // 右侧纵向传感器
float right_adc;    // 右侧横向传感器

// 全局状态标志位
uint8 ADC_C_Flag = 0;     // ADC数据是否有效的标志位。1=有效，0=无效（所有传感器都无信号）

//直道相关
uint8 Straight_line_Flag =0;				//走直线标志位

//弯道相关
//uint8 cruve_Flag =0;					//弯道标志位
//uint8 Small_Cruve_Count = 0;	//小弯道计数器
//uint8 Large_Cruve_Count = 0;	//大弯道计数器
uint8 Curve_Type_Flag = 0;

static uint8 Straight_Stable_Count = 0; // 稳定计数器
static uint8 Curve_Stable_Count = 0;		//区分弯道类型的标志：0:无，1:右弯2:左弯


//环岛相关
extern volatile uint8 ring_state;
extern float ADC_Compensate;

uint8 change_ROUND = 0;           // 冷却期（即将进去/未完全出来）标志 (0:不在, 1:在)
uint8 round_cooldown = 0;         // 冷却期计数器
uint8 round_entry_count = 0;      // 环岛进入确认计数器
uint8 round_inside_count = 0;     // 环岛内部持续计时器
uint8 ROUND_Flag = 0;             // 环岛状态主标志 (0:不在, 1:在)
uint8 round_Type_Flag = 0;				//环的类型 1直线圆环 2十字圆环
uint8  round_prepare_flag = 0;          // 预备进环：探头看到环，但车身还没到
uint16 round_prepare_delay_count = 0;   // 预备进环延时计数

uint8  round_exit_pending = 0;          // 准备退出环岛：探头先离开，但车身还在环内
uint16 round_exit_delay_count = 0;      // 退出延时计数

float adbia;

/**
 * @brief 对ADC采样数组进行滤波：去除最大值和最小值后，计算剩余值的平均值。
 * @param ADC 指向包含N个ADC原始采样值的数组
 * @return 返回滤波后的平均值
 * @details 这种滤波方法可以有效去除采样过程中的毛刺和噪声。
 */
float ADC_Del_MaxMin_Average_Filter(float *ADC)
{
    uint8 i;
    uint16 max, min;  // 用于记录数组中的最大值和最小值
    uint16 sum = 0;   // 用于累加所有采样值
    float average = 0.0;

    // 初始化最大值和最小值
    max = 0;
    min = 0xFFFF; // 初始化为一个非常大的数，确保第一个值能被正确比较

    for(i = 0; i < FILTER_N; i++) // 循环遍历所有采样点
    {
        if(max < ADC[i]) max = ADC[i];  // 更新最大值
        if(min > ADC[i]) min = ADC[i];  // 更新最小值
        sum += ADC[i];  // 累加所有值
    }

    // 计算平均值：总和减去最大值和最小值，再除以 (总点数-2)
    average = 1.0 * (sum - max - min) / (FILTER_N - 2);
    return average;
}


/**
 * @brief 将一个浮点数限制在指定的最大值和最小值范围内。
 * @param adc 待限制的数值
 * @param adc_max 允许的最大值
 * @param adc_min 允许的最小值
 * @return 限制后的数值
 * @details 防止数据溢出，保证后续计算的稳定性。
 */
float ADC_Limit(float adc, float adc_max, float adc_min)
{
    if(adc >= adc_max) adc = adc_max;
    else if(adc <= adc_min) adc = adc_min;
    return adc;
}

static float limit_ad_bias(float bias)
{
    if (bias >  AD_BIAS_LIMIT_GLOBAL)  return  AD_BIAS_LIMIT_GLOBAL;
    if (bias < -AD_BIAS_LIMIT_GLOBAL)  return -AD_BIAS_LIMIT_GLOBAL;
    return bias;
}

// 以下函数是供外部模块（如PID控制）调用的接口，用于获取处理后的ADC值
float ADC_GetLeft(void) 		{ return left_adc; }
float ADC_GetRight(void) 		{ return right_adc; }
float ADC_GetLeftShu(void)  { return leftshu_adc; }
float ADC_GetRightShu(void) { return rightshu_adc; }
// 获取ADC数据有效性标志
uint8 ADC_IsValid(void)       { return ADC_C_Flag; }

/**
 * @brief 判断是否为直线/窄弯道
 * @details 基于横向电感信号总和是否低于阈值
 */
uint8_t ADC_IsStraight(void)
{
    float sum_heng = left_adc + right_adc;
    float sum_shu = leftshu_adc + rightshu_adc;
	
	float abs_heng_Difference = (left_adc > right_adc) ? 
                              (left_adc - right_adc) : 
                              (right_adc - left_adc);
    float abs_shu_Difference = (leftshu_adc > rightshu_adc) ? 
                              (leftshu_adc - rightshu_adc) : 
                              (rightshu_adc - leftshu_adc);
    // 如果已经在环岛，则不能是直线
    if (ROUND_Flag == 1)
    {
        Straight_line_Flag = 0;
        Straight_Stable_Count = 0;
        return 0;
    }


   if (sum_heng > SUM_HENG_MIN &&
    sum_shu  < SUM_SHU_STRAIGHT_MAX &&
    abs_heng_Difference < DIFF_STRAIGHT_MAX)
    {
        Straight_Stable_Count++;
        if (Straight_Stable_Count >=2 ) // 连续3帧稳定才确认
        {
            Straight_line_Flag = 1;
            //cruve_Flag = 0;
            Curve_Type_Flag = 0;
            Curve_Stable_Count = 0;
            return 1;
        }
    }
    else
    {
        Straight_Stable_Count = 0;
        Straight_line_Flag = 0;
    }
    return 0;
}

uint8_t ADC_IsCurve(void)
{
    // 数值处理 
		float sum_heng = left_adc + right_adc;
        float sum_shu = leftshu_adc + rightshu_adc;
	
		// 判断左右标志 一般右弯右边大
    float signed_curve_Difference = rightshu_adc -leftshu_adc ;
    
	//绝对值判断弯道剧烈程度
		float abs_shu_Difference = (leftshu_adc > rightshu_adc) ? 
                              (leftshu_adc - rightshu_adc) : 
                              (rightshu_adc - leftshu_adc);
		
		float abs_heng_Difference = (left_adc > right_adc) ? 
                              (left_adc - right_adc) : 
                              (right_adc - left_adc);
		
    // 1. 如果是环岛或直线，全部清零并退出
    if (ring_state != 0 || Straight_line_Flag == 1) 
    {

        Curve_Type_Flag = 0;

        return 0; // 不是弯道
		
    }
    
    //    条件：横向有线、竖向明显抬高、左右差足够大    
     // sum_shu 大 → 入弯、弯中、出弯：要灵敏一些    
	 if (((abs_heng_Difference >= DIFF_HENG_CURVE_MIN_WEAK||abs_shu_Difference >= DIFF_SHU_CURVE_MIN_WEAK))&&sum_heng<100)
    {
         Curve_Type_Flag = 0; 
          Curve_Stable_Count++;
          if (Curve_Stable_Count >=1) // 稍快响应，2帧确认弯道
         {
             Straight_line_Flag = 0;
             // 判断左右方向
             if (signed_curve_Difference > 0)      Curve_Type_Flag = 2; // 右弯
             else if (signed_curve_Difference < 0) Curve_Type_Flag = 1; // 左弯
              else  Curve_Type_Flag = 0;

          }  
         else
         { 
//            // 退出弯道需滞后几帧，防止“回正抖动”
//            if (Curve_Stable_Count > 0)
//               {
//                Curve_Stable_Count--;
//              }
//              else
//              {
              // cruve_Flag = 0;
                  Curve_Type_Flag = 0;
           // }
         }           
    }
    
				
        // 3. 最终确认
        if (Curve_Type_Flag > 0) 
        {
//            cruve_Flag = 1;
            return 1; // 是弯道
        }
       else 
        {
//            cruve_Flag = 0;
            return 0;
        }
    }
   


/**
 * @brief 判断车辆是否处于环岛状态 (ROUND)
 * @details 本函数负责检测车辆进入、维持在环岛内以及退出环岛的过程。
 *          它使用多个状态标志和计数器来实现防抖、延迟和冷却逻辑，
 *          以提高环岛检测的鲁棒性和准确性。
 * @return uint8_t 返回 1 表示车辆当前在环岛内 (IN_ROUND)，返回 0 表示不在环岛内 (NOT_IN_ROUND)。
 */
//环岛相关
// uint8 change_ROUND = 0;           // 冷却期（即将进去/未完全出来）标志 (0:不在, 1:在)
// uint8 round_cooldown = 0;         // 冷却期计数器
// uint8 round_entry_count = 0;      // 环岛进入确认计数器
// uint8 round_inside_count = 0;     // 环岛内部持续计时器
// uint8 ROUND_Flag = 0;             // 环岛状态主标志 (0:不在, 1:在)
// uint8 round_Type_Flag = 0;				//环的类型 1直线圆环 2十字圆环
// uint8  round_prepare_flag = 0;          // 预备进环：探头看到环，但车身还没到
// uint16 round_prepare_delay_count = 0;   // 预备进环延时计数


uint8_t ADC_IsRound(void)
{		
		//相关参数
    float sum_heng = left_adc + right_adc;
	float sum_shu = leftshu_adc + rightshu_adc;
    float sum_total = sum_heng + sum_shu;

    // 冷却期防止判断进/出环岛错误
    if (change_ROUND == 1)
    {
        round_cooldown++;// 增加冷却计数器 
        if (round_cooldown >= ROUND_COOLDOWN_CYCLES)//150
        {
            change_ROUND = 0;
            round_cooldown = 0;
			round_Type_Flag=0;
        }
       // 冷却期间强制认为不在环内 & 清理内部状态
        ROUND_Flag = 0;
        round_prepare_flag = 0;
        round_prepare_delay_count = 0;
        round_exit_pending = 0;
        round_exit_delay_count = 0;
        round_inside_count = 0;
        return 0;
    }
 /* ===================== 1. 已经在环内：IN_ROUND 状态 ===================== */
     if (ROUND_Flag == 1)
    {
        round_inside_count++;

        // 1-1. 如果已经进入“退出延时”阶段
        if (round_exit_pending)
        {
            round_exit_delay_count++;// 增加退出延迟计数器

            // 检查是否已达到预设的退出延迟周期数
            if (round_exit_delay_count >= ROUND_EXIT_DELAY_CYCLES)
            {
                // 延迟结束，确认退出环岛
                ROUND_Flag = 0;                    // 更新主状态：不在环岛
                round_exit_pending = 0;            // 清除退出待定标志
                round_exit_delay_count = 0;        // 清除退出延迟计数器
                round_inside_count = 0;            // 清除在环岛内计数器

                change_ROUND = 1;                  // 启动冷却期，防止立刻重新进入判断
                round_cooldown = 0;                // 初始化冷却计数器
                return 0;                          // 返回不在环岛
            }

            // 延时阶段外部仍认为“在环里”
            return 1;
        }

        // 1-2.正常在环岛内运行
        // 检查是否满足准备退出的条件：
        // 1. 在环岛内停留的时间足够长 (防止刚进去就出来)
        // 2. 总体传感器信号强度降低到某个阈值以下 (表示可能快要离开环岛了)
        if ((round_inside_count > ROUND_MIN_HOLD_CYCLES) &&
            (sum_total < HORIZON_SUM_TOTAL_EXIT_LIMIT))
        {
             // 探测到离开环岛的迹象，启动退出待定状态和延迟计时
            round_exit_pending = 1;
            round_exit_delay_count = 0;
        }

        return 1;   // IN_ROUND 状态
    }

    /* ===================== 2. 预备进环：PREPARE_ROUND 状态 ===================== */
    // 如果尚未确认进入环岛，但检测到了可能是进入环岛的信号，正在进行预判
    if (round_prepare_flag && ROUND_Flag == 0)
    {
        round_prepare_delay_count++;// 增加预进入延迟计数器

        // 如果在预备阶段信号完全不像环了，认为是误判，直接取消
        bool is_still_round_like = false;
         // 判断标准：水平信号强 或者 竖直信号强，并且总信号也足够强
     if ((sum_shu  >= SHIZI_ROUND_SUM_SHU_ENTER_LIMIT && 
         sum_total >  ROUND_SUM_TOTAL_ENTER_LIMIT) ||
        (sum_heng >= NORMAL_ROUND_SUM_HENG_ENTER_LIMIT && 
         sum_total >  ROUND_SUM_TOTAL_ENTER_LIMIT))
        {
            is_still_round_like = true;// 信号仍然符合环岛特征
        }

        if (!is_still_round_like)
        {
            // 预备失败，回到普通
            round_prepare_flag = 0;
            round_prepare_delay_count = 0;
            round_entry_count = 0;
            round_Type_Flag = 0;
            return 0;
        }

         // 如果信号一直很强，并且预进入延迟时间足够长 (确认不是偶然信号)
        if (round_prepare_delay_count >= ROUND_PREPARE_DELAY_CYCLES)
        {
            ROUND_Flag = 1;                     // 更新主状态：已在环岛
            round_inside_count = 0;             // 初始化在环岛内计时器
            round_prepare_flag = 0;             // 清除预进入标志
            round_prepare_delay_count = 0;      // 清除预进入延迟计数器
            return 1;                           // 报告已在环岛
        }

        // 预备阶段：探头已看到环，但车身还没到，外部仍认为“不在环内”
        return 0;
    }

    /* ===================== 3. NORMAL：尝试检测新的圆环入口 ===================== */
    if (ROUND_Flag == 0 && round_prepare_flag == 0)
    {
        uint8 candidate_type = 0;

        // 3-1. 十字圆环
        // 判断是否满足进入环岛的初始信号条件
        // 十字圆环入口特征：竖直方向信号非常强 (sum_shu)，并且总信号也很强 (sum_total)
        if ((sum_shu >= SHIZI_ROUND_SUM_SHU_ENTER_LIMIT) &&
            (sum_total > ROUND_SUM_TOTAL_ENTER_LIMIT))
        {
            candidate_type = 2;    // 十字圆环
        }
        // 3-2. 普通圆环（横向比较强）
         // 普通圆环入口特征：水平方向信号较强 (sum_heng)，并且总信号也很强 (sum_total)
        else if ((sum_heng >= NORMAL_ROUND_SUM_HENG_ENTER_LIMIT) &&
                 (sum_total > ROUND_SUM_TOTAL_ENTER_LIMIT))
        {
            candidate_type = 1;    // 直线圆环
        }
        else
        {
            // 完全不像环，清空进入计数器
            round_entry_count = 0;
            return 0;
        }

        // 满足某种圆环的条件，开始入口防抖计数
        round_entry_count++;
        if (round_entry_count >= ROUND_ENTRY_THRESHOLD)
        {
            // 不是立刻进入 IN_ROUND，而是先进入“预备进环”
            round_entry_count = 0;
            round_prepare_flag = 1;
            round_prepare_delay_count = 0;
            round_Type_Flag = candidate_type;  // 记录圆环类型
        }

        return 0;   // 此时外部仍认为“不在环里”
    }

    return 0;
}

/**
 * @brief  判断赛道类型并计算最终偏差 AD_Bias
 * @param  adc_sum_last 最后一个采样点的总和（用于防丢线辅助判断）
 * @return float 赛道偏差值 AD_Bias
 */
float ADC_TrackTypeJudgment(float adc_sum_last)
{
    float sum_heng = left_adc + right_adc;
    float sum_shu  = leftshu_adc + rightshu_adc;
    float heng_diff = left_adc - right_adc;
    float shu_diff  = leftshu_adc - rightshu_adc;
    float fenzi=0.0f;
    float fenmu=0.1f;
    float current_AD_Bias = 0.0f;
    static float last_valid_bias = 0.0f;   // 上一次有效偏差
    static uint8 no_signal_count = 0;      // 丢线计数器

    // --- 0. 状态更新 ---
   // ADC_IsRound();           // 环岛状态机更新
    ring_handle();
    ADC_IsCurve();           // 弯道状态机更新
    ADC_IsStraight();        // 直道状态机更新



    // ① 环岛状态（最高优先级）
    if (ring_state)
    {
        
//       if(ring_state == 3)
//        {
//            current_AD_Bias=10.0*(0.7 * (heng_diff/sum_heng) + 
//                            0.3 * (shu_diff/sum_shu));
//        }
//        else
//        {
             fenzi = ALPHA* heng_diff + BETA * shu_diff ;
            fenmu = ALPHA * sum_heng + OMEGA * abs(shu_diff) ;
        
            current_AD_Bias =1.1* (K_CS * (fenzi/fenmu));
       
         
            last_valid_bias = limit_ad_bias(last_valid_bias);
            current_AD_Bias = ADC_Compensate+current_AD_Bias;


            // current_AD_Bias=0.7 * (heng_diff/sum_heng) + 
            //                 0.3 * (shu_diff/sum_shu);
//        }    
        last_valid_bias = last_valid_bias + 
                          0.8f * (current_AD_Bias - last_valid_bias) ;
        last_valid_bias = limit_ad_bias(last_valid_bias);
        return last_valid_bias;
    }
        
   
   // 差比和差算法
       if (Curve_Type_Flag > 0)
    {
        fenzi = ALPHA* heng_diff + BETA * shu_diff ;
        fenmu = ALPHA * sum_heng + OMEGA * abs(shu_diff) ;
        
        current_AD_Bias =1.2* K_CS * (fenzi/fenmu);
			
	
    //    current_AD_Bias=0.7 * (heng_diff/sum_heng) + 
    //                         0.3 * (shu_diff/sum_shu);

        last_valid_bias =last_valid_bias + 
                          1.0f * (current_AD_Bias - last_valid_bias);
        last_valid_bias = limit_ad_bias(last_valid_bias);
        return last_valid_bias;
    }

	else if(Straight_line_Flag == 1)
		
	{
	 fenzi = ALPHA* heng_diff + BETA * shu_diff ;
        fenmu = ALPHA * sum_heng + OMEGA * abs(shu_diff) ;
        
        current_AD_Bias = K_CS * (fenzi/fenmu);
			
		


    //    current_AD_Bias=0.7 * (heng_diff/sum_heng) + 
    //                         0.3 * (shu_diff/sum_shu);

        last_valid_bias = last_valid_bias + 
                          0.7f * (current_AD_Bias - last_valid_bias) ;
        last_valid_bias = limit_ad_bias(last_valid_bias);
        return last_valid_bias;
	}

    else //默认区（比如弯后回正、直线前摇摆w
    {
        fenzi = ALPHA* heng_diff + BETA * shu_diff ;
        fenmu = ALPHA * sum_heng + OMEGA * abs(shu_diff) ;
        
        current_AD_Bias = K_CS * (fenzi/fenmu);
       
        last_valid_bias = last_valid_bias + 
                          0.8f * (current_AD_Bias - last_valid_bias) ;
        last_valid_bias = limit_ad_bias(last_valid_bias);
        return last_valid_bias;
        // current_AD_Bias=0.7 * (heng_diff/sum_heng) + 
        //                     0.3 * (shu_diff/sum_shu);

        // last_valid_bias = last_valid_bias + 
        //                   0.3f * (current_AD_Bias - last_valid_bias) ;
        // last_valid_bias = limit_ad_bias(last_valid_bias);
        // return last_valid_bias;
    }
}


float MaxIndex_f(float *arr, int len)
{
    float maxIndex = 0;
    float maxValue = arr[0];

    for(int i = 1; i < len; i++)
    {
        if(arr[i] > maxValue)
        {
            maxValue = arr[i];
            maxIndex = i;
        }
    }
    return maxValue;
}

float MinIndex_f(float *arr, int len)
{
    float minIndex = 0;
    float minValue = arr[0];

    for(int i = 1; i < len; i++)
    {
        if(arr[i] < minValue)
        {
            minValue = arr[i];
            minIndex = i;
        }
    }
    return minValue;
}
/**
 * @brief ADC数据处理的核心函数，负责从原始采样到最终偏差值的整个流程。
 * @return 返回一个[-1, 1]范围内的偏差值（AD_Bias），用于PID控制
 * @details 处理流程如下：
 *  1. 采集原始数据 -> 2. 滤波 -> 3. 归一化 -> 4. 限幅 -> 5. 环岛检测 -> 6. 有效性判断 -> 7. 计算偏差
 */
float ADC_Final_Read_Deal()
{
    float AD_Bias = 0.0;      // 最终输出的偏差值
    float minus = 0.0;        // 左右传感器的差值
    float sum = 0.0;          // 左右传感器的和
    float num_Horizon_buff_Float = 0.0;
		float num_Vertical_buff_Float =0.0;
		
	
    uint16 num_buff = 0;
    uint8 i;

    // 为四个ADC通道分别创建采样缓冲区
    float filter_buf_L[FILTER_N];    // 左侧横向
    float filter_buf_LS[FILTER_N];   // 左侧纵向
    float filter_buf_RS[FILTER_N];   // 右侧纵向
    float filter_buf_R[FILTER_N];    // 右侧横向

    // -------- 1. 采集原始ADC数据 --------
    for(i = 0; i < FILTER_N; i++)
    {
        filter_buf_L[i]  = adc_convert(Left_ADC_Pin);   // 采集左侧横向ADC
        filter_buf_LS[i] = adc_convert(Left_Shu_Pin);   // 采集左侧纵向ADC
        filter_buf_RS[i] = adc_convert(Right_Shu_Pin);  // 采集右侧纵向ADC
        filter_buf_R[i]  = adc_convert(Right_ADC_Pin);  // 采集右侧横向ADC

//        // 去除微弱信号：
        if(filter_buf_L[i] < 240)  filter_buf_L[i] = 0;
        if(filter_buf_LS[i] < 500) filter_buf_LS[i] = 0;
        if(filter_buf_RS[i] < 500) filter_buf_RS[i] = 0;
        if(filter_buf_R[i] < 240)  filter_buf_R[i] = 0;
   }

    // -------- 2. 对采集到的数据进行滤波 --------
    // 使用“去极值平均滤波”来平滑数据，减少噪声
    adc_deal_1[LEFT]      = ADC_Del_MaxMin_Average_Filter(filter_buf_L);
    adc_deal_1[LEFT_SHU]  = ADC_Del_MaxMin_Average_Filter(filter_buf_LS);
    adc_deal_1[RIGHT_SHU] = ADC_Del_MaxMin_Average_Filter(filter_buf_RS);
    adc_deal_1[RIGHT]     = ADC_Del_MaxMin_Average_Filter(filter_buf_R);

    // adc_min[LEFT]      = MinIndex_f(filter_buf_L,FILTER_N);
    // adc_min[LEFT_SHU]  = MinIndex_f(filter_buf_LS,FILTER_N);
    // adc_min[RIGHT_SHU] = MinIndex_f(filter_buf_RS,FILTER_N);
    // adc_min[RIGHT]     = MinIndex_f(filter_buf_R,FILTER_N);
        
    // adc_max[LEFT]      = MaxIndex_f(filter_buf_L,FILTER_N);
    // adc_max[LEFT_SHU]  = MaxIndex_f(filter_buf_LS,FILTER_N);
    // adc_max[RIGHT_SHU] = MaxIndex_f(filter_buf_RS,FILTER_N);
    // adc_max[RIGHT]     = MaxIndex_f(filter_buf_R,FILTER_N);
    // // -------- 3.归一化处理 --------
    // left_adc    =  100.0f*(adc_deal_1[LEFT]-adc_min[LEFT])/(adc_max[LEFT] -adc_min[LEFT]+0.1f);
    // leftshu_adc =  100.0f*(adc_deal_1[LEFT_SHU]-adc_min[LEFT_SHU])/(adc_max[LEFT_SHU] -adc_min[LEFT_SHU]+0.1f);
    // rightshu_adc=  100.0f*(adc_deal_1[RIGHT_SHU]-adc_min[RIGHT_SHU])/(adc_max[RIGHT_SHU] -adc_min[RIGHT_SHU]+0.1f);
    // right_adc   =  100.0f*(adc_deal_1[RIGHT]-adc_min[RIGHT])/(adc_max[RIGHT] -adc_min[RIGHT]+0.1f);

    left_adc    =  100.0 * adc_deal_1[LEFT]/ADC_MAX;
    leftshu_adc =  100.0 * adc_deal_1[LEFT_SHU]/ADC_MAX;
    rightshu_adc=  100.0 * adc_deal_1[RIGHT_SHU]/ADC_MAX;
    right_adc   =  100.0 * adc_deal_1[RIGHT]/ADC_MAX;
    // -------- 4. 数据限幅 --------
    left_adc    = ADC_Limit(left_adc,    ADC_MAX, ADC_MIN);
    leftshu_adc = ADC_Limit(leftshu_adc, ADC_MAX, ADC_MIN);
    rightshu_adc= ADC_Limit(rightshu_adc, ADC_MAX, ADC_MIN);
    right_adc   = ADC_Limit(right_adc,   ADC_MAX, ADC_MIN);


    // -------- 8. 判断ADC信号是否有效 --------
    // 检查最后一个采样点的总和
    num_buff = filter_buf_L[FILTER_N-1] + filter_buf_LS[FILTER_N-1] +
               filter_buf_RS[FILTER_N-1] + filter_buf_R[FILTER_N-1];

    if(num_buff >10.0f)ADC_C_Flag = 1; // 有信号，数据有效
	else ADC_C_Flag = 0;
		AD_Bias=ADC_TrackTypeJudgment(num_buff);
		
    return AD_Bias; // 返回最终的偏差值
}


// 获取最终的偏差值，供PID控制器使用
float ADC_GetAngle(void)
{
	adbia=ADC_Final_Read_Deal();
    return ADC_Final_Read_Deal();
}

//是否是直道
uint8 Get_Straight_line_Flag(void)
{
    return Straight_line_Flag;
}

//是否是弯道
//uint8 Get_Cruve_Flag(void)
//{
//    return cruve_Flag;
//}

//弯道类型
uint8 Get_Curve_Type_Flag(void)
{
    return Curve_Type_Flag;
}

//环岛状态
uint8 Get_ROUND_Flag(void)
{
    return ROUND_Flag;
}

//环岛冷却期标志
uint8 Get_Change_ROUND(void)
{
    return change_ROUND;
}


//环岛类型标志（(1:普通圆环, 2:十字圆环)）
uint8 Get_Round_Type_Flag(void)
{
    return round_Type_Flag;
}

void get_encoder()
{	

		encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = -encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);   
}

float GetLeftSpeed(void)
{
	return 0.095f* encoder_data_1;

}
float GetRightSpeed(void)
{
	return 0.095f* encoder_data_2;

}

float clamp_value(float val, float min_val, float max_val)
{
    // 如果 val 小于最小值，返回最小值
    if (val < min_val)
    {
        return min_val;
    }
    // 如果 val 大于最大值，返回最大值
    else if (val > max_val)
    {
        return max_val;
    }
    // 否则，返回原值
    else
    {
        return val;
    }
}
