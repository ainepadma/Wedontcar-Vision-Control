/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "main.h"
#include "motor_switch.h"
#include "steering_log.h"
#include "zf_device_imu660rb.h"

int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;
volatile int32_t encoder_total_l = 0;
volatile int32_t encoder_total_r = 0;
volatile uint8_t motor_enabled = 0;
volatile uint8_t motor_feedback_fault = 0;
volatile uint8_t protection_enabled = AUTOMATIC_PROTECTION_DEFAULT;
volatile uint8_t speed_full_enabled = 0;
volatile uint8_t track_ready = 0;
volatile uint8_t track_stop_reason = TRACK_STOP_MANUAL;
volatile track_stop_snapshot_t track_last_stop;
volatile int16_t track_servo_tenths = 0;
volatile uint8_t imu_device_status = IMU660RB_RESULT_ERROR;
imu_navigation_state_t imu_navigation;
ring_control_state_t ring_control;
static motor_switch_t motor_switch_state;
static motor_switch_t protection_switch_state;
static motor_switch_t speed_mode_switch_state;
#if SERVO_CENTER_TEST_MODE
static motor_switch_t servo_decrease_switch_state;
static float servo_center_test_angle = SERVO_MOTOR_CENTER;
#endif
static motor_feedback_watchdog_t motor_feedback_watchdog;
static motor_overspeed_watchdog_t motor_overspeed_watchdog;
static track_control_state_t track_control;
volatile uint8_t track_stripe_brake_status;
static stripe_brake_state_t stripe_brake;
static float stripe_brake_servo_angle;

/* Only the stripe stop uses signed drive. Zero PWM before changing DIR. */
static void stripe_brake_apply(int16_t left, int16_t right)
{
    pwm_set_duty(MOTOR1_PWM, 0U);
    pwm_set_duty(MOTOR2_PWM, 0U);
    gpio_set_level(MOTOR1_DIR, left < 0 ? !MOTOR1_FORWARD_DIR_LEVEL : MOTOR1_FORWARD_DIR_LEVEL);
    gpio_set_level(MOTOR2_DIR, right < 0 ? !MOTOR2_FORWARD_DIR_LEVEL : MOTOR2_FORWARD_DIR_LEVEL);
    pwm_set_duty(MOTOR1_PWM, (uint32_t)(left < 0 ? -(int32_t)left : left));
    pwm_set_duty(MOTOR2_PWM, (uint32_t)(right < 0 ? -(int32_t)right : right));
    /* Existing telemetry uses unsigned PWM magnitudes; DIR carries polarity. */
    speed_pwm_l = (float)(left < 0 ? -(int32_t)left : left);
    speed_pwm_r = speed_pwm = (float)(right < 0 ? -(int32_t)right : right);
    /* Keep the last commanded angle energized; do not release the servo. */
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32_t)SERVO_MOTOR_DUTY(stripe_brake_servo_angle));
}
static volatile uint32_t control_clock_ms;
static uint32_t consumed_frame_generation;
static int16_t log_path_error, log_path_heading;
static uint8_t log_path_confidence;

// 主循环只发布观测，中断独占控制器、PID 和 PWM，防止停机后被旧帧重新写入。
static volatile struct
{
    int16_t error, heading;
    int16_t near_error, near_heading;
    uint8_t near_geometry_valid;
    int16_t bottom_error;
    uint8_t bottom_valid;
    uint8_t confidence, valid, ring_candidate;
    int8_t ring_direction;
    track_ring_observation_t ring;
    stripe_brake_observation_t stripe;
    uint32_t observed_ms, generation, frame_number;
} track_mailbox;

/* Boot is the only caller before PIT starts; PIT owns samples thereafter. */
static uint8_t track_imu_poll(uint8_t stationary)
{
    if (!imu_navigation.device_ready)
        return 0U;
    imu660rb_sample_t sample;
    imu660rb_result_t status = imu660rb_read_sample(&sample);
    imu_device_status = status;
    if (status == IMU660RB_RESULT_ERROR)
        imu_navigation.device_ready = 0U;
    if (status != IMU660RB_RESULT_OK)
        return 0U;
    imu_navigation_observe(&imu_navigation, sample.acc_x, sample.acc_y, sample.acc_z,
                           sample.gyro_x, sample.gyro_y, sample.gyro_z, stationary);
    return 1U;
}

void track_imu_init(void)
{
    imu_device_status = imu660rb_init();
    imu_navigation_init(&imu_navigation, imu_device_status == IMU660RB_RESULT_OK);
    /* Use the original 200-sample GY average while drive PWM is still zero.
     * Allow up to one extra second of NOT_READY; vision also runs without an IMU. */
    for (unsigned elapsed = 0U;
         elapsed < 3000U && imu_navigation.device_ready && !imu_navigation.bias_ready;
         elapsed += SPEED_CONTROL_PERIOD_MS)
    {
        system_delay_ms(SPEED_CONTROL_PERIOD_MS);
        track_imu_poll(1U);
    }
}

void track_runtime_init(void)
{
    const track_control_config_t config = {
        .servo_center = SERVO_MOTOR_CENTER,
        .servo_left = SERVO_MOTOR_L_MAX,
        .servo_right = SERVO_MOTOR_R_MAX,
        .speed_straight = TRACK_SPEED_STRAIGHT,
        .speed_curve_min = TRACK_SPEED_CURVE_MIN,
        .kp = TRACK_STEER_KP,
        .kd = TRACK_STEER_KD,
        .max_servo_rate_deg_s = TRACK_SERVO_RATE_DEG_S
    };
    track_control_init(&track_control, &config);
    stripe_brake_init(&stripe_brake);
    track_stripe_brake_status = 0U;
    stripe_brake_servo_angle = SERVO_MOTOR_CENTER;
    ring_control_init(&ring_control);
    protection_switch_state.enabled = AUTOMATIC_PROTECTION_DEFAULT;
    speed_mode_switch_state.enabled = 0U;
    protection_enabled = AUTOMATIC_PROTECTION_DEFAULT;
    track_control_set_loss_stop(&track_control, protection_enabled);
    ring_control_set_protection(&ring_control, protection_enabled);
    speed_full_enabled = 0U;
#if SERVO_CENTER_TEST_MODE
    // 中值测试不依赖赛道就绪；C13 只用于开关舵机输出。
    track_control_set_loss_stop(&track_control, 0U);
    protection_switch_state.enabled = 0U;
    protection_enabled = 0U;
#endif
    motor_feedback_watchdog_reset(&motor_feedback_watchdog);
    motor_overspeed_watchdog_reset(&motor_overspeed_watchdog);
    speed_target = 0.0f;
    motor_enabled = 0;
    track_servo_tenths = (int16_t)(track_control.servo_angle * 10.0f);
}

void track_frame_publish(const track_vision_result_t *result,
                         const stripe_brake_observation_t *stripe)
{
    // 临界区复制结构化观测和候选路径；不复制像素、不计算几何或执行IO。
    uint32_t primask = interrupt_global_disable();
    track_mailbox.stripe = *stripe;
    track_mailbox.error = result->error;
    track_mailbox.frame_number = result->frame_number;
    track_mailbox.heading = result->heading;
    track_mailbox.near_error = result->near_error;
    track_mailbox.near_heading = result->near_heading;
    track_mailbox.near_geometry_valid = result->near_geometry_valid;
    track_mailbox.bottom_error = result->bottom_error;
    track_mailbox.bottom_valid = result->bottom_valid;
    track_mailbox.confidence = result->confidence;
    track_mailbox.valid = result->valid;
    track_mailbox.ring_candidate = result->ring_candidate;
    track_mailbox.ring_direction = result->ring_direction;
    track_mailbox.ring = result->ring;
    track_mailbox.observed_ms = control_clock_ms;
    ++track_mailbox.generation;
    __DMB();
    interrupt_global_enable(primask);
}



void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // 调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    // 数据同步隔离
}

void SysTick_Handler(void)
{

}

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        control_clock_ms += SPEED_CONTROL_PERIOD_MS;
        uint8_t was_running = motor_enabled;
        uint8_t yaw_fresh = track_imu_poll((uint8_t)!was_running);
        // 独立于图像主循环：30ms 去抖，长按不连发，上电按住不启动。
        uint8_t button_pressed = motor_switch_sample(&motor_switch_state,
                                                     gpio_get_level(MOTOR_TOGGLE_KEY) == GPIO_LOW);
        uint8_t protection_pressed = motor_switch_sample(&protection_switch_state,
            gpio_get_level(PROTECTION_TOGGLE_KEY) == GPIO_LOW);
        uint8_t speed_mode_pressed = motor_switch_sample(&speed_mode_switch_state,
            gpio_get_level(SPEED_MODE_TOGGLE_KEY) == GPIO_LOW);
#if SERVO_CENTER_TEST_MODE
        uint8_t servo_decrease_pressed = motor_switch_sample(&servo_decrease_switch_state,
            gpio_get_level(SCREEN_TOGGLE_KEY) == GPIO_LOW);
        if (servo_decrease_pressed)
            servo_center_test_angle = constrain_float(servo_center_test_angle - 0.5f,
                                                       SERVO_CALIBRATION_MIN,
                                                       SERVO_CALIBRATION_MAX);
#endif
        if (protection_pressed)
        {
            track_control_set_loss_stop(&track_control,
                                        protection_switch_state.enabled);
            protection_enabled = track_control.loss_stop_enabled;
            motor_feedback_fault = 0U;
            motor_feedback_watchdog_reset(&motor_feedback_watchdog);
            motor_overspeed_watchdog_reset(&motor_overspeed_watchdog);
        }
        if (speed_mode_pressed)
        {
#if SERVO_CENTER_TEST_MODE
            servo_center_test_angle = constrain_float(servo_center_test_angle + 0.5f,
                                                       SERVO_CALIBRATION_MIN,
                                                       SERVO_CALIBRATION_MAX);
#else
            /* C15 is a one-shot RAM adjustment, no longer a LOW/FULL toggle.
             * Init reloads TRACK_SPEED_STRAIGHT after power cycling. */
            track_control.config.speed_straight += 10.0f;
            speed_full_enabled = 0U;
#endif
        }
        if (button_pressed && !stripe_brake.stopped)
        {
            motor_feedback_fault = 0;
            /* C13 intent is based on the state at tick entry. If a protection
             * already stopped the car in this tick, the same press must not
             * toggle it back on. */
            if (was_running)
                track_control_stop(&track_control, TRACK_STOP_MANUAL);
            else
            {
                ring_control_init(&ring_control); // C13 starts a new run: clear faults and 270/360 completion records
                track_control_toggle(&track_control, control_clock_ms);
                steering_log_start();
            }
        }

        // 获取编码器读数
        encoder_data_1 = ENCODER_1_FORWARD_SIGN * encoder_get_count(ENCODER_1);
        encoder_clear_count(ENCODER_1);                // 清空编码器计数
        encoder_data_2 = ENCODER_2_FORWARD_SIGN * encoder_get_count(ENCODER_2);
        encoder_clear_count(ENCODER_2);
        // 停机时也累计，供手转整圈标定每轮的实际正交计数。
        encoder_total_l += encoder_data_1;
        encoder_total_r += encoder_data_2;
        // 分别保留两轮测量值；停机后的滑行也是真实反馈，不伪装成零速。
        speed_real_l = encoder_data_1;
        speed_real_r = encoder_data_2;
        speed_real = speed_real_r; // 兼容旧调试变量，明确仅代表右轮。

        // 编码器极性要按实车标定，不能随 DIR 盲目取反或用绝对值掩盖。
        // 起步未施加有效PWM时的单个负计数可能是编码器抖动；
        // 有驱动且连续30ms明显反转才锁停，真正反转仍先于无反馈保护处理。
        static uint8_t reverse_ticks_l, reverse_ticks_r;
        if (track_control.running)
        {
            reverse_ticks_l = ((uint32_t)speed_pwm_l >= 500U && encoder_data_1 <= -3)
                ? (uint8_t)(reverse_ticks_l < 3U ? reverse_ticks_l + 1U : 3U) : 0U;
            reverse_ticks_r = ((uint32_t)speed_pwm_r >= 500U && encoder_data_2 <= -3)
                ? (uint8_t)(reverse_ticks_r < 3U ? reverse_ticks_r + 1U : 3U) : 0U;
        }
        else
            reverse_ticks_l = reverse_ticks_r = 0U;
        uint8_t reverse_fault = (uint8_t)((reverse_ticks_l >= 3U ? 1U : 0U) |
                                          (reverse_ticks_r >= 3U ? 2U : 0U));
        if (reverse_fault)
        {
            motor_feedback_fault = reverse_fault;
            track_control_stop(&track_control, TRACK_STOP_ENCODER);
        }

        // 编码器反号优先；用上一周期实际 PWM 对应本周期读到的反馈。
        // 单轮连续 500ms 有驱动却几乎无正计数，锁停两轮，防止 PI 堵转加力。
        uint8_t no_feedback = motor_feedback_watchdog_sample(&motor_feedback_watchdog,
            (uint8_t)(was_running && track_control.running),
            encoder_data_1, encoder_data_2, (uint32_t)speed_pwm_l, (uint32_t)speed_pwm_r);
        if (no_feedback)
        {
            motor_feedback_fault = no_feedback;
            track_control_stop(&track_control, TRACK_STOP_NO_FEEDBACK);
        }

        uint8_t overspeed = motor_overspeed_watchdog_sample(&motor_overspeed_watchdog,
            (uint8_t)(was_running && track_control.running),
            encoder_data_1, encoder_data_2, speed_target);
        if (overspeed)
        {
            motor_feedback_fault = overspeed;
            track_control_stop(&track_control, TRACK_STOP_OVERSPEED);
        }

        // Ring decision selects the path BEFORE ordinary steering observes it.
        ring_control_set_protection(&ring_control, protection_enabled);
        ring_control_tick(&ring_control, track_control.running,
            (uint8_t)(yaw_fresh && imu_navigation.bias_ready),
            imu_navigation.yaw_rate_dps,
            0.5f * ((float)(encoder_data_1 > 0 ? encoder_data_1 : 0) +
                    (float)(encoder_data_2 > 0 ? encoder_data_2 : 0)), control_clock_ms);
        uint8_t fresh_ring_frame = (uint8_t)(track_mailbox.generation != consumed_frame_generation);
        if (fresh_ring_frame || ring_control.control_refresh)
        {
            consumed_frame_generation = track_mailbox.generation;
            /* PIT exclusively owns this scratch. Keep the 2.5KB observation
             * off the shared 8KB main/IRQ stack; mailbox publication is atomic. */
            static track_ring_observation_t vision;
            vision = track_mailbox.ring;
#if !SERVO_CENTER_TEST_MODE
            if (fresh_ring_frame)
                ring_control_observe(&ring_control, &vision, track_mailbox.observed_ms,
                                     track_control.running);
            const track_path_observation_t path = ring_control_select_path(&ring_control, &vision);
#else
            const track_path_observation_t path = vision.path[TRACK_PATH_NORMAL];
#endif
            if (ring_control.reset_tracking_history)
            {
                /* Ring handoff only: no ordinary-frame reset or parameter change. */
                if (path.valid)
                {
                    track_control.filtered_error = (float)path.error;
                    track_control.filtered_heading = (float)path.heading;
                }
                track_control.derivative = 0.0f;
                track_control.derivative_ready = 0U;
                ring_control.reset_tracking_history = 0U;
            }
            track_control_observe_near(&track_control, path.error, path.heading,
                                  track_mailbox.near_error, track_mailbox.near_heading,
                                  (uint8_t)(track_mailbox.near_geometry_valid &&
                                    ring_control.path_mode == TRACK_PATH_NORMAL && !ring_control.line_active),
                                  path.confidence, path.valid,
                                  fresh_ring_frame ? track_mailbox.observed_ms : control_clock_ms);
            log_path_error = path.error;
            log_path_heading = path.heading;
            log_path_confidence = path.confidence;
            track_control_observe_bottom(&track_control, track_mailbox.bottom_error,
                (uint8_t)(fresh_ring_frame && track_mailbox.bottom_valid &&
                          ring_control.path_mode == TRACK_PATH_NORMAL && !ring_control.line_active),
                fresh_ring_frame ? track_mailbox.observed_ms : control_clock_ms);
        }
        /* C14 enables stripe arming; a completed stop remains latched. */
        uint8_t stripe_just_latched = 0U;
#if !SERVO_CENTER_TEST_MODE && !SPEED_BENCH_MODE
        const stripe_brake_observation_t stripe_observation = track_mailbox.stripe;
        if (!protection_enabled && !stripe_brake.stopped)
            stripe_brake_init(&stripe_brake);
        if (protection_enabled && fresh_ring_frame && !stripe_brake.stopped &&
            stripe_brake_observe(&stripe_brake, &stripe_observation,
                                 track_mailbox.observed_ms, control_clock_ms))
        {
            stripe_just_latched = 1U;
            stripe_brake_servo_angle = track_control.servo_angle;
            Pid_Reset(&speed_pid_l);
            Pid_Reset(&speed_pid_r);
        }
#endif
        track_stripe_brake_status = stripe_brake.stopped ? 2U : (stripe_brake.armed ? 1U : 0U);
        if (protection_enabled && ring_control.fault)
            track_control_stop(&track_control, TRACK_STOP_RING);
        /* This runs every control tick, even with no camera frame or C13. */
        if (stripe_brake.stopped)
            track_control_stop(&track_control, TRACK_STOP_STRIPE);
        track_control_output_t output = track_control_tick(&track_control, control_clock_ms);
        if (ring_control_guidance_active(&ring_control) &&
            output.target_speed > TRACK_SPEED_CURVE_MIN)
            output.target_speed = TRACK_SPEED_CURVE_MIN;
        // v0.6: corner-repaired paths use the same PD, calibration and slew
        // limits as ordinary following. No legacy 360 output may overwrite it.
        if (ring_control.line_active)
            output.target_speed *= constrain_float(RING_STRAIGHT_SPEED_SCALE, 0.1f, 1.0f);
#if SPEED_BENCH_MODE
        // 架空时不依赖摄像头识别，也不让视觉转向改变两轮目标。
        output.servo_angle = SERVO_MOTOR_CENTER;
#endif
        /* Capture BEFORE PWM/targets are cleared. Does not change stop policy. */
        track_stop_record(&track_last_stop,(uint8_t)(was_running || stripe_just_latched),output.running,output.stop_reason,
            motor_feedback_fault,protection_enabled,control_clock_ms,
            encoder_data_1,encoder_data_2,speed_target,speed_pwm_l,speed_pwm_r);
        motor_enabled = output.running;
        motor_switch_state.enabled = output.running;
        track_ready = output.ready;
        track_stop_reason = output.stop_reason;
#if SERVO_CENTER_TEST_MODE
        track_servo_tenths = (int16_t)(servo_center_test_angle * 10.0f);
#else
        track_servo_tenths = (int16_t)(output.servo_angle * 10.0f);
#endif
        const float requested_counts =
#if SERVO_CENTER_TEST_MODE
            0.0f;
#elif SPEED_BENCH_MODE
            (speed_full_enabled ? TRACK_SPEED_FULL_STRAIGHT : 80.0f);
#else
            (speed_full_enabled
                ? TRACK_SPEED_FULL_CURVE_MIN +
                    constrain_float((output.target_speed - TRACK_SPEED_CURVE_MIN) /
                        (TRACK_SPEED_STRAIGHT - TRACK_SPEED_CURVE_MIN), 0.0f, 1.0f) *
                    (TRACK_SPEED_FULL_STRAIGHT - TRACK_SPEED_FULL_CURVE_MIN)
                : output.target_speed);
#endif
        // 每10ms最多增加3个计数目标；减速立即更新目标，保持弯道降速响应。
        static float ramped_counts;
        if (!motor_enabled)
            ramped_counts = 0.0f;
        else if (requested_counts > ramped_counts + 3.0f)
            ramped_counts += 3.0f;
        else
            ramped_counts = requested_counts;
        const float target_counts = ramped_counts;
        speed_target = target_counts;
        if (button_pressed || was_running != motor_enabled)
        {
            // 用户停机、丢线、停帧和编码器故障都通过同一条归零路径。
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            // 只清历史，不重新加载参数，保留将来分别标定的左右轮增益。
            Pid_Reset(&speed_pid_l);
            Pid_Reset(&speed_pid_r);
            motor_feedback_watchdog_reset(&motor_feedback_watchdog);
            motor_overspeed_watchdog_reset(&motor_overspeed_watchdog);
            speed_pwm_l = speed_pwm_r = speed_pwm = 0.0f;
        }

        float log_target_left = 0.0f, log_target_right = 0.0f;
        if (stripe_brake.stopped)
        {
            /* Capture the stop position now; use current speed for immediate
             * damping, without counting pre-trigger motion as displacement. */
            const int16_t left = stripe_brake_hold(&stripe_brake, 0U,
                encoder_data_1, stripe_just_latched);
            const int16_t right = stripe_brake_hold(&stripe_brake, 1U,
                encoder_data_2, stripe_just_latched);
            speed_target = 0.0f;
            track_servo_tenths = (int16_t)(stripe_brake_servo_angle * 10.0f);
            stripe_brake_apply(left, right);
        }
        else if (motor_enabled)
        {
#if SERVO_CENTER_TEST_MODE
            // 标定期间只给舵机中值脉冲，禁止任何电机驱动。
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_center_test_angle));
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            Pid_Reset(&speed_pid_l);
            Pid_Reset(&speed_pid_r);
            speed_pwm_l = speed_pwm_r = speed_pwm = 0.0f;
#else
            // 舵机只在用户启动且视觉有效时输出，控制器已经完成角度限幅与限速。
            // SPEED_BENCH_MODE 下锁死中位：架空时前轮不受力，但落地测试必须让前轮
            // 保持直行，否则车身会被前轮带偏、扯紧线缆，测到的速度环数据全部失真。
            pwm_set_duty(SERVO_MOTOR_PWM,
                SPEED_BENCH_MODE ? SERVO_MOTOR_DUTY(SERVO_MOTOR_CENTER) : SERVO_MOTOR_DUTY(output.servo_angle));
            // 保留已实测极性与曲率降速关系；有界 PI 增加低速负载补偿。
            // t=1 表示一个 10ms 采样周期，不是 1 秒；P=25、I=0.3、D=0。
            // 差速仅拆分轮速目标：右转左轮快，左转右轮快；公共速度不变。
            const float differential = SPEED_BENCH_MODE || (ring_control.action_locked && ring_control.kind == TRACK_RING_360) ? 0.0f : Speed_Differential_Delta(target_counts,
                (output.servo_angle - SERVO_MOTOR_CENTER) * SERVO_DIR);
            const float target_left = target_counts + differential;
            const float target_right = target_counts - differential;
            log_target_left = target_left;
            log_target_right = target_right;
            speed_pwm_l = PidLocCtrl(&speed_pid_l, target_left - speed_real_l, 1.f);
            // 最终驱动防线仍保留非负/硬件满量程限制，并同步内部输出状态。
            speed_pwm_l = constrain_float(speed_pwm_l, 0.0f, speed_OUTMAX);
            speed_pid_l.out = speed_pwm_l;
            gpio_set_level(MOTOR1_DIR, MOTOR1_FORWARD_DIR_LEVEL);
            pwm_set_duty(MOTOR1_PWM, (uint32_t)speed_pwm_l);

            speed_pwm_r = PidLocCtrl(&speed_pid_r, target_right - speed_real_r, 1.f);
            speed_pwm_r = constrain_float(speed_pwm_r, 0.0f, speed_OUTMAX);
            speed_pid_r.out = speed_pwm_r;
            gpio_set_level(MOTOR2_DIR, MOTOR2_FORWARD_DIR_LEVEL);
            pwm_set_duty(MOTOR2_PWM, (uint32_t)speed_pwm_r);
            speed_pwm = speed_pwm_r;
#endif
        }
        else
        {
            // 停机是撤销驱动 PWM（非主动制动），不计算 PID、不积累积分。
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            pwm_set_duty(SERVO_MOTOR_PWM, 0);
            speed_pwm_l = speed_pwm_r = speed_pwm = 0.0f;
        }

        /* Capture applied outputs only; numeric snapshot, no serial/file IO. */
        const steering_log_record_t log_record = {
            control_clock_ms, track_mailbox.frame_number,
            (uint32_t)(control_clock_ms - track_mailbox.observed_ms),
            log_path_error, log_path_heading, track_mailbox.near_error,
            track_mailbox.near_heading, track_mailbox.bottom_error,
            (int16_t)track_control.filtered_error, (int16_t)track_control.filtered_heading,
            track_servo_tenths, (int16_t)(log_target_left * 10.0f),
            (int16_t)(log_target_right * 10.0f), encoder_data_1, encoder_data_2,
            (int16_t)speed_pwm_l, (int16_t)speed_pwm_r,
            motor_enabled, track_ready, track_stop_reason, log_path_confidence,
            track_control.frame_valid, ring_control.phase, ring_control.kind, protection_enabled,
            ring_control.corner_stage, ring_control.corner_guide_id,
            ring_control_path_source(&ring_control), ring_control.direction,
            ring_control.bridge_seen, ring_control.bridge_passed,
            ring_control.exit_armed, ring_control.transition_frames,
            imu_navigation.acc_forward, imu_navigation.acc_left, imu_navigation.acc_up,
            imu_navigation.gyro_x_raw, imu_navigation.gyro_y_raw, imu_navigation.gyro_z_raw,
            (int16_t)(imu_navigation.yaw_rate_dps * 10.0f), yaw_fresh, imu_navigation.bias_ready
        };
        steering_log_tick(&log_record);
        pit_flag_clear(PIT_CH0);
    }

    if(pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    assistant_uart_tx_irq_handler();
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
    #if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                      // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
    #endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
        
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // 不允许删除
    __DSB();
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断 
        if(NULL != flexio_camera_uart_handler)
        {
            flexio_camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        if(NULL != camera_uart_handler)
        {
            camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    // 勿删除此IF语句
    if(NULL != flexio_camera_vsync_handler)
    {
        flexio_camera_vsync_handler();
    }
    
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    //tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}









/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



