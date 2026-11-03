#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "zf_device_imu660rb.h"
#include <math.h>

// 暗部增强及地面区域阈值；上方灯光与底部车体不参与阈值估计。
// C13 控制启停；赛道版 C14 默认关闭视觉保护，编码器保护始终有效。
#define CAMERA_SEND_RAW 1
#define CAMERA_TUNING_EXPOSURE 1024U
// 台架调速度环时的遥测方式：1=逐飞助手虚拟示波器（二进制），0=#SPD 文本。
// 两者不能同时发：ASCII 文本会打断示波器的二进制帧解析。
#define SPEED_SCOPE_MODE 0
// Auto black-level calibration uses dark reference pixels; exposure stays manual.
// 0 keeps the driver default manual black-level calibration.
#define CAMERA_TUNING_AUTO_BLACK_LEVEL 1U
#define TRACK_DISPLAY_WIDTH 125U
#define TRACK_DISPLAY_HEIGHT 100U
#define SCREEN_FRAME_PERIOD_MS 100U
#define IMU_AXIS_TEST_MODE 0
static uint8_t camera_exposure_status;
static uint8_t screen_enabled = 1;
static uint8_t screen_stats_dirty = 1;
static uint32_t processed_frames;
static uint32_t displayed_frames;
static uint32_t capture_fps, process_fps, display_fps;
static uint32_t invalid_vision_frames;
static uint8_t camera_threshold;

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 与运行时方向配置保持一致
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 与运行时方向配置保持一致

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // C13 每 10ms 独立扫描，停机键不依赖摄像头或屏幕刷新。
    // 电机上电保持零输出，必须先松开按键再有效按下才启动。
    gpio_init(MOTOR_TOGGLE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(PROTECTION_TOGGLE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SPEED_MODE_TOGGLE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    track_runtime_init();
    pit_ms_init(PIT_CH0, SPEED_CONTROL_PERIOD_MS);

    // IMU 失败只记录状态，不阻塞摄像头、按键或循迹控制。
    imu_device_status = imu660rb_init();
    imu_navigation_init(&imu_navigation,
                        imu_device_status == IMU660RB_RESULT_OK);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
uint32_t hist_gram[256];
static uint8_t image_filter_scratch[5U * MT9V03X_W];
static image_track_preprocess_state_t image_preprocess_state;
static track_vision_result_t track_result;
static track_vision_result_t screen_track_result;
static ring_control_state_t screen_ring_snapshot;
static uint8_t screen_guide_id,screen_path_preview,screen_approach_source;
static char screen_path_source='N';

// 冻结一帧快照，由 UART 中断慢慢发送。传输中不覆盖，忙时跳过新预览帧。
// 数据头及诊断文本也纳入同一队列，禁止 printf 插入正在发送的图像。
#define CAMERA_DIAGNOSTIC_SIZE 2048U
static uint8_t camera_tx_packet[CAMERA_DIAGNOSTIC_SIZE + 12U + sizeof(image)];
static volatile uint32_t camera_tx_position;
static volatile uint32_t camera_tx_length;
#if CAMERA_SEND_RAW
static uint8_t camera_snapshot_ready;
static void assistant_prepare_raw_image(void)
{
    if (camera_tx_length == 0U && !camera_snapshot_ready)
    {
        memcpy(camera_tx_packet + CAMERA_DIAGNOSTIC_SIZE + 12U, image, sizeof(image));
        camera_snapshot_ready = 1;
    }
}
#endif

static void assistant_transport_cancel(void)
{
    uint32_t primask = interrupt_global_disable();
    LPUART_DisableInterrupts(LPUART1, kLPUART_TxDataRegEmptyInterruptEnable);
    camera_tx_position = 0U;
    camera_tx_length = 0U;
#if CAMERA_SEND_RAW
    camera_snapshot_ready = 0U;
#endif
    interrupt_global_enable(primask);
}

void assistant_uart_tx_irq_handler(void)
{
    if (!(LPUART_GetEnabledInterrupts(LPUART1) & kLPUART_TxDataRegEmptyInterruptEnable))
        return;
    // 有限次填充 FIFO，绝不在中断中等待 UART 就绪。
    for (uint32_t budget = 0; budget < 32U; ++budget)
    {
        if (camera_tx_position >= camera_tx_length)
        {
            LPUART_DisableInterrupts(LPUART1, kLPUART_TxDataRegEmptyInterruptEnable);
            camera_tx_length = 0;
            break;
        }
        if (!(LPUART_GetStatusFlags(LPUART1) & kLPUART_TxDataRegEmptyFlag))
            break;
        LPUART_WriteByte(LPUART1, camera_tx_packet[camera_tx_position++]);
    }
}

// 逐飞助手 V2：12 字节图像配置头，后接逐像素灰度数据。
// 与官方 seekfree_assistant_camera_config / camera_send 的线格式一致。
static void assistant_send_image(void)
{
    if (camera_tx_length != 0U)
        return;
#if CAMERA_SEND_RAW
    if (!camera_snapshot_ready)
        return;
#endif
    uint8_t header[12] = {
        0xAA, 0, 0x02, 0x02,
        MT9V03X_W & 0xFF, (MT9V03X_W >> 8) & 0xFF,
        MT9V03X_H & 0xFF, (MT9V03X_H >> 8) & 0xFF,
        0x01, 0, 0, 0
    };
    for (uint8_t i = 0; i < sizeof(header); ++i)
    {
        if (i != 1)
            header[1] += header[i];
    }
    // 只在临界区读取控制状态；格式化和串口传输仍在开中断后进行。
    // LSPD/RSPD 为计数/10ms，TARGET10 为目标计数的10倍，LPWM/RPWM 为PWM指令。
    uint32_t primask = interrupt_global_disable();
    uint8_t snapshot_running = motor_enabled, snapshot_fault = motor_feedback_fault;
    uint8_t snapshot_protection = protection_enabled;
    uint8_t snapshot_full_speed = speed_full_enabled;
    uint8_t snapshot_ready = track_ready, snapshot_stop = track_stop_reason;
    int16_t snapshot_servo = track_servo_tenths;
    float snapshot_target = speed_target;
    float snapshot_left_speed = speed_real_l, snapshot_right_speed = speed_real_r;
    float snapshot_left_pwm = speed_pwm_l, snapshot_right_pwm = speed_pwm_r;
    imu_navigation_state_t snapshot_imu = imu_navigation;
    track_stop_snapshot_t snapshot_last_stop = track_last_stop;
    static ring_control_state_t snapshot_ring; /* Main-loop owned; keep ISR stack headroom. */
    snapshot_ring = ring_control;
    uint8_t snapshot_imu_status = imu_device_status;
    interrupt_global_enable(primask);
    int prefix_length = snprintf((char *)camera_tx_packet, CAMERA_DIAGNOSTIC_SIZE,
        "#CAM FW=0.21.0 T=%lu CAP=%lu VF=%lu PROC=%lu DISP=%lu CFPS=%lu PFPS=%lu DFPS=%lu IO=%u MOTOR=%u MFAULT=%u PROTECT=%u FULL=%u EXP=%u STATUS=%u TRACK=%u Q=%u VR=%u/%u N=%u F=%u NW=%u FW=%u WG=%u RC=%u RD=%d REJ=%02X NC=%u AM=%u DC=%u ERR=%d HEAD=%d READY=%u STOP=%u SERVO10=%d TARGET10=%d LSPD=%d RSPD=%d LPWM=%u RPWM=%u THR=%u BAD=%lu IMU=%u ID=%02X IS=%lu IB=%u IBS=%u AF=%d AL=%d AU=%d GY=%d YR10=%d RS=%u RDIR=%d RY10=%d RT=%u ENT=%u PASS=%u EXT=%u PATH=%u RF=%u JN=%u LO=%u RO=%u FV=%u LQ=%u RQ=%u CE=%d CH=%d ES=%u XS=%u NP=%u NO=%u TL=%u TR=%u WB=%u SS=%u LA=%u CL=%u CR=%u CS=%u CG=%u CVF=%lu CGF=%lu HOLD=%u AP=%u AR=%u CT=%u NE=%d NH=%d NV=%u NRO=%u ER=%u BE=%d BV=%u\r\n",
        (unsigned long)timer_get(GPT_TIM_1), (unsigned long)mt9v03x_frame_count,
        (unsigned long)track_result.frame_number, (unsigned long)processed_frames,
        (unsigned long)displayed_frames,
        (unsigned long)capture_fps, (unsigned long)process_fps, (unsigned long)display_fps,
        (unsigned int)screen_enabled, (unsigned int)snapshot_running, (unsigned int)snapshot_fault,
        (unsigned int)snapshot_protection, (unsigned int)snapshot_full_speed,
        CAMERA_TUNING_EXPOSURE, (unsigned int)camera_exposure_status,
        (unsigned int)track_result.valid, (unsigned int)track_result.confidence,
        (unsigned int)track_result.valid_rows, (unsigned int)track_result.roi_rows,
        (unsigned int)track_result.near_valid_rows, (unsigned int)track_result.far_valid_rows,
        (unsigned int)track_result.near_width, (unsigned int)track_result.far_width,
        (unsigned int)track_result.width_growth_percent,
        (unsigned int)track_result.ring_candidate, (int)track_result.ring_direction,
        (unsigned int)track_result.reject_flags, (unsigned int)track_result.no_candidate_rows,
        (unsigned int)track_result.ambiguous_rows, (unsigned int)track_result.disconnected_rows,
        (int)track_result.error, (int)track_result.heading, (unsigned int)snapshot_ready,
        (unsigned int)snapshot_stop, (int)snapshot_servo, (int)(snapshot_target * 10.0f),
        (int)snapshot_left_speed, (int)snapshot_right_speed,
        (unsigned int)snapshot_left_pwm, (unsigned int)snapshot_right_pwm,
        (unsigned int)camera_threshold,
        (unsigned long)invalid_vision_frames,
        (unsigned int)snapshot_imu_status, (unsigned int)imu660rb_last_identity(),
        (unsigned long)snapshot_imu.sample_count,
        (unsigned int)snapshot_imu.bias_ready, (unsigned int)snapshot_imu.bias_samples,
        (int)snapshot_imu.acc_forward, (int)snapshot_imu.acc_left,
        (int)snapshot_imu.acc_up, (int)snapshot_imu.gyro_y_raw,
        (int)(snapshot_imu.yaw_rate_dps * 10.0f),
        (unsigned int)snapshot_ring.phase, (int)snapshot_ring.direction,
        (int)(snapshot_ring.yaw_progress_deg * 10.0f),
        (unsigned int)(snapshot_ring.kind == TRACK_RING_270 ? 270U :
                       (snapshot_ring.kind == TRACK_RING_360 ? 360U : 0U)),
        (unsigned int)snapshot_ring.entry_seen, (unsigned int)snapshot_ring.entry_passed,
        (unsigned int)snapshot_ring.exit_seen, (unsigned int)snapshot_ring.path_mode,
        (unsigned int)snapshot_ring.fault, (unsigned int)track_result.ring.junction_near,
        (unsigned int)track_result.ring.left_open, (unsigned int)track_result.ring.right_open,
        (unsigned int)track_result.ring.forward_visible,
        (unsigned int)track_result.ring.left_quality, (unsigned int)track_result.ring.right_quality,
        (int)snapshot_ring.selected_path.error, (int)snapshot_ring.selected_path.heading,
        (unsigned int)track_result.ring.entry_score, (unsigned int)track_result.ring.exit_score,
        (unsigned int)track_result.ring.near_paired, (unsigned int)track_result.ring.near_open,
        (unsigned int)track_result.ring.topology[0], (unsigned int)track_result.ring.topology[1],
        (unsigned int)track_result.ring.white_band, (unsigned int)track_result.ring.straight_support,
        (unsigned int)snapshot_ring.line_active,
        (unsigned int)track_result.ring.corners.count[0],(unsigned int)track_result.ring.corners.count[1],
        (unsigned int)snapshot_ring.corner_stage,(unsigned int)snapshot_ring.corner_guide_id,
        (unsigned long)snapshot_ring.corner_frame_number,(unsigned long)snapshot_ring.corner_guide_frame_number,
        (unsigned int)snapshot_ring.corner_guide_held,
        (unsigned int)track_result.ring.approach_source,(unsigned int)track_result.ring.approach_reject,
        (unsigned int)track_result.ring.corners.continued_mask,
        (int)track_result.near_error, (int)track_result.near_heading,
        (unsigned int)track_result.near_geometry_valid,
        (unsigned int)track_result.near_only,
        (unsigned int)track_result.edge_rescue,
        (int)track_result.bottom_error, (unsigned int)track_result.bottom_valid);
    if (prefix_length <= 0 || (uint32_t)prefix_length >= CAMERA_DIAGNOSTIC_SIZE)
        return;
    /* Last automatic-stop evidence survives C13 restart; keep one #CAM line. */
    int last_length=snprintf((char *)camera_tx_packet+prefix_length-2,
        CAMERA_DIAGNOSTIC_SIZE-(uint32_t)prefix_length+2U,
        " LSSEQ=%lu LST=%lu LSREASON=%u LSW=%u LSP=%u LSL=%d LSR=%d LSTARGET10=%d LSLPWM=%u LSRPWM=%u\r\n",
        (unsigned long)snapshot_last_stop.sequence,(unsigned long)snapshot_last_stop.ms,
        (unsigned)snapshot_last_stop.reason,(unsigned)snapshot_last_stop.wheels,
        (unsigned)snapshot_last_stop.protection,(int)snapshot_last_stop.left,(int)snapshot_last_stop.right,
        (int)(snapshot_last_stop.target*10.f),(unsigned)snapshot_last_stop.left_pwm,(unsigned)snapshot_last_stop.right_pwm);
    if(last_length<=0 || (uint32_t)last_length>=CAMERA_DIAGNOSTIC_SIZE-(uint32_t)prefix_length+2U)
        return;
    prefix_length=prefix_length-2+last_length;
    memcpy(camera_tx_packet + prefix_length, header, sizeof(header));
#if CAMERA_SEND_RAW
    // 到识别完成后才发包，使灰度快照和 TRACK/ERR/Q 诊断来自同一帧。
    memmove(camera_tx_packet + prefix_length + sizeof(header),
            camera_tx_packet + CAMERA_DIAGNOSTIC_SIZE + 12U, sizeof(image));
    camera_snapshot_ready = 0;
#else
    memcpy(camera_tx_packet + prefix_length + sizeof(header), image, sizeof(image));
#endif
    camera_tx_position = 0;
    __DMB();
    camera_tx_length = (uint32_t)prefix_length + sizeof(header) + sizeof(image);
    LPUART_EnableInterrupts(LPUART1, kLPUART_TxDataRegEmptyInterruptEnable);
}

#if SPEED_BENCH_MODE && !SPEED_SCOPE_MODE
// 架空调速只发短文本，避免灰度图把10ms速度环采样拖成约4秒一条。
static void assistant_send_speed(uint32_t now)
{
    static uint32_t last_ms;
    if ((uint32_t)(now - last_ms) < 50U || camera_tx_length != 0U)
        return;
    last_ms = now;
    uint32_t primask = interrupt_global_disable();
    uint8_t running = motor_enabled, fault = motor_feedback_fault;
    uint8_t full = speed_full_enabled, protection = protection_enabled;
    uint8_t stop = track_stop_reason;
    float target = speed_target, left = speed_real_l, right = speed_real_r;
    float lpwm = speed_pwm_l, rpwm = speed_pwm_r;
    int32_t left_total = encoder_total_l, right_total = encoder_total_r;
    interrupt_global_enable(primask);
    int length = snprintf((char *)camera_tx_packet, CAMERA_DIAGNOSTIC_SIZE,
        "#SPD FW=0.21.0 T=%lu MOTOR=%u PROTECT=%u FULL=%u STOP=%u MFAULT=%u TARGET10=%d LSPD=%d RSPD=%d LPWM=%u RPWM=%u LTOT=%ld RTOT=%ld\r\n",
        (unsigned long)now, (unsigned int)running, (unsigned int)protection,
        (unsigned int)full, (unsigned int)stop, (unsigned int)fault,
        (int)(target * 10.0f), (int)left, (int)right,
        (unsigned int)lpwm, (unsigned int)rpwm,
        (long)left_total, (long)right_total);
    if (length <= 0 || length >= CAMERA_DIAGNOSTIC_SIZE)
        return;
    camera_tx_position = 0U;
    __DMB();
    camera_tx_length = (uint32_t)length;
    LPUART_EnableInterrupts(LPUART1, kLPUART_TxDataRegEmptyInterruptEnable);
}
#endif

#if SPEED_SCOPE_MODE
// 逐飞助手 V2 虚拟示波器：4 通道浮点包，50 Hz。
// 帧格式（默认，与本工程 #CAM 头一致）：0xAA | 和校验 | 0x10|通道数 | 包长 | float[4] 小端
// 老版 zf_components 的顺序是 0xAA | 功能 | 校验 | 包长。示波器窗口一直为空时，
// 把下面的 SCOPE_FUNC_AT_1 改为 1 再烧录一次即可验证。
#ifndef SCOPE_FUNC_AT_1
#define SCOPE_FUNC_AT_1 0
#endif
static void assistant_send_scope(uint32_t now)
{
    static uint32_t last_ms;
    if ((uint32_t)(now - last_ms) < 20U || camera_tx_length != 0U)
        return;
    last_ms = now;

    uint32_t primask = interrupt_global_disable();
    float channel[4];
#if SPEED_BENCH_MODE
    channel[0] = speed_target;  // 通道1：目标速度（计数/10ms）
    channel[1] = speed_real_l;  // 通道2：左轮实际
    channel[2] = speed_real_r;  // 通道3：右轮实际
    channel[3] = speed_pwm_l;   // 通道4：左轮 PWM
#else
    /* 赛道模式：TFT 硬件故障期间，把视觉链路搬到示波器上看。
     * 通道1 横向偏差 ERR（偏右为正）/ 通道2 远端航向 HEAD /
     * 通道3 舵机角度×10（870=中位）/ 通道4 目标速度（计数/10ms）。 */
    channel[0] = (float)track_result.error;
    channel[1] = (float)track_result.heading;
    channel[2] = (float)track_servo_tenths;
    channel[3] = speed_target;
#endif
    interrupt_global_enable(primask);

    uint8_t packet[20];
    uint8_t sum = 0U;
    memset(packet, 0, sizeof(packet));
    packet[0] = 0xAA;
    packet[3] = (uint8_t)sizeof(packet);
    memcpy(&packet[4], channel, sizeof(channel));
#if SCOPE_FUNC_AT_1
    packet[1] = (uint8_t)(0x10 | 4);
    for (uint32_t i = 0U; i < sizeof(packet); ++i)
        if (i != 2U) sum += packet[i];
    packet[2] = sum;
#else
    packet[2] = (uint8_t)(0x10 | 4);
    for (uint32_t i = 0U; i < sizeof(packet); ++i)
        if (i != 1U) sum += packet[i];
    packet[1] = sum;
#endif

    memcpy(camera_tx_packet, packet, sizeof(packet));
    camera_tx_position = 0U;
    __DMB();
    camera_tx_length = (uint32_t)sizeof(packet);
    LPUART_EnableInterrupts(LPUART1, kLPUART_TxDataRegEmptyInterruptEnable);
}
#endif

static void screen_button_poll(uint32_t now)
{
#if SERVO_CENTER_TEST_MODE
    (void)now;
    return;
#else
    static uint8_t last_level = GPIO_LOW;
    static uint8_t stable_level = GPIO_LOW;
    static uint32_t changed_at;
    uint8_t level = gpio_get_level(SCREEN_TOGGLE_KEY);
    if (level != last_level)
    {
        last_level = level;
        changed_at = now;
    }
    if (level != stable_level && (uint32_t)(now - changed_at) >= 30U)
    {
        stable_level = level;
        if (level == GPIO_LOW)
        {
            screen_enabled = !screen_enabled;
            TFT180_BLK(screen_enabled);
            if (!screen_enabled)
                assistant_transport_cancel();
            screen_stats_dirty = 1;
        }
    }
#endif
}

static void update_frame_rates(uint32_t now)
{
    static uint32_t previous_time, previous_capture, previous_process, previous_display;
    static uint8_t initialized;
    if (!initialized)
    {
        previous_time = now;
        previous_capture = mt9v03x_frame_count;
        previous_process = processed_frames;
        previous_display = displayed_frames;
        initialized = 1;
        return;
    }
    uint32_t elapsed = now - previous_time;
    if (elapsed >= 1000U)
    {
        uint32_t captured = mt9v03x_frame_count;
        capture_fps = (captured - previous_capture) * 1000U / elapsed;
        process_fps = (processed_frames - previous_process) * 1000U / elapsed;
        display_fps = (displayed_frames - previous_display) * 1000U / elapsed;
        previous_time = now;
        previous_capture = captured;
        previous_process = processed_frames;
        previous_display = displayed_frames;
        screen_stats_dirty = 1;
    }
}

static void screen_draw_stats(void)
{
    char line[17];
#if IMU_AXIS_TEST_MODE
    if (imu_device_status != IMU660RB_RESULT_OK)
    {
        tft180_show_string(0, 102, "IMU660RB ERROR  ");
        snprintf(line, sizeof(line), "ID:%02X STATUS:%u ",
                 (unsigned int)imu660rb_last_identity(),
                 (unsigned int)imu_device_status);
        tft180_show_string(0, 118, line);
        tft180_show_string(0, 134, "CHECK IMU WIRE  ");
    }
    else
    {
        snprintf(line, sizeof(line), "X A%5d G%5d", (int)imu_navigation.acc_left,
                 (int)imu_navigation.gyro_x_raw);
        tft180_show_string(0, 102, line);
        snprintf(line, sizeof(line), "Y A%5d G%5d", (int)-imu_navigation.acc_up,
                 (int)imu_navigation.gyro_y_raw);
        tft180_show_string(0, 118, line);
        snprintf(line, sizeof(line), "Z A%5d G%5d", (int)-imu_navigation.acc_forward,
                 (int)imu_navigation.gyro_z_raw);
        tft180_show_string(0, 134, line);
    }
#elif SERVO_CENTER_TEST_MODE
    const int16_t middle = track_servo_tenths;
    snprintf(line, sizeof(line), "MID:%3d.%1d DEG", (int)(middle / 10),
             (int)(middle >= 0 ? middle % 10 : -(middle % 10)));
    tft180_show_string(0, 102, line);
    tft180_show_string(0, 118, "C12-  C15+     ");
    tft180_show_string(0, 134, motor_enabled ? "C13 SERVO ON  " : "C13 SERVO OFF ");
#else
    uint32_t stop_lock=interrupt_global_disable();
    track_stop_snapshot_t last_stop=track_last_stop;
    uint8_t stopped=(uint8_t)!motor_enabled;
    interrupt_global_enable(stop_lock);
    if(stopped && last_stop.sequence) {
        const char *name=last_stop.reason==TRACK_STOP_NO_FEEDBACK ? "NOFB" :
            last_stop.reason==TRACK_STOP_OVERSPEED ? "FAST" :
            last_stop.reason==TRACK_STOP_ENCODER ? "ENC" :
            last_stop.reason==TRACK_STOP_LOST ? "LOST" :
            last_stop.reason==TRACK_STOP_STALE ? "STALE" :
            last_stop.reason==TRACK_STOP_RING ? "RING" : "READY";
        snprintf(line,sizeof(line),"LAST %s:%u P%u ",name,(unsigned)last_stop.wheels,(unsigned)last_stop.protection);
        tft180_show_string(0,102,line);
        snprintf(line,sizeof(line),"L%d R%d       ",(int)last_stop.left,(int)last_stop.right);
        tft180_show_string(0,118,line);
        snprintf(line,sizeof(line),"T%d C13:RUN    ",(int)last_stop.target);
        tft180_show_string(0,134,line);
        screen_stats_dirty=0;
        return;
    }
    const ring_control_state_t *ring_display=&screen_ring_snapshot;
    track_path_observation_t display_path=screen_track_result.ring.path[TRACK_PATH_NORMAL];
    if(screen_approach_source!=TRACK_PATH_NORMAL) display_path=screen_track_result.ring.approach_path;
    else if(screen_guide_id!=RING_GUIDE_NONE) {
        const ring_corner_guide_t *g=&screen_track_result.ring.corners.guide[screen_guide_id];
        display_path=(track_path_observation_t){g->error,g->heading,g->valid,g->confidence};
    }
    else if(ring_display->action_locked || screen_path_source=='?') display_path=ring_display->selected_path;
    snprintf(line, sizeof(line), "%cE%4d H%4d   ", screen_path_preview ? 'P' :
        (screen_path_source=='H' || screen_path_source=='?' ? screen_path_source : ' '),
        /* H is a historical held path; ? is a control/image frame mismatch. */
        (int)display_path.error,(int)display_path.heading);
    tft180_show_string(0, 102, line);
    snprintf(line, sizeof(line), "L%u R%u S%u G%u %c  ",
        (unsigned int)screen_track_result.ring.corners.count[0],(unsigned int)screen_track_result.ring.corners.count[1],
        (unsigned int)ring_display->corner_stage,(unsigned int)screen_guide_id,screen_path_source);
    tft180_show_string(0, 118, line);
    if (motor_feedback_fault)
    {
        const char *fault_format = track_stop_reason == TRACK_STOP_NO_FEEDBACK
            ? "NO FB:%u      " : (track_stop_reason == TRACK_STOP_OVERSPEED
            ? "AIR/FAST:%u   " : "ENC ERR:%u    ");
        snprintf(line, sizeof(line), fault_format, (unsigned int)motor_feedback_fault);
        tft180_show_string(0, 134, line);
    }
    else if (track_stop_reason == TRACK_STOP_RING)
        tft180_show_string(0, 134, "RING FAULT     ");
    else if (motor_enabled && ring_display->action_locked)
    {
        snprintf(line, sizeof(line), "%u %s ",ring_display->kind==TRACK_RING_270 ? 270U : 360U,
            ring_display->corner_guide_held ? "HOLD    " :
            (ring_display->phase==RING_STATE_INSIDE ? "INSIDE  " :
             (ring_display->corner_guide_id==RING_GUIDE_270_CONNECT ? "CONNECT " : "GUIDE   ")));
        tft180_show_string(0, 134, line);
    }
    else if (ring_display->completed_mask != 0U && (!ring_display->entry_seen || ring_display->phase == RING_STATE_COOLDOWN))
    {
        snprintf(line, sizeof(line), "D360:%u D270:%u ",
                 (unsigned int)ring_display->completed_360, (unsigned int)ring_display->completed_270);
        tft180_show_string(0, 134, line);
    }
    else if (!motor_enabled && screen_path_preview)
        tft180_show_string(0,134,"PREVIEW STOP  ");
    else if (!protection_enabled)
        tft180_show_string(0, 134, motor_enabled
            ? (speed_full_enabled ? "FREE FULL     " : "FREE LOW      ")
            : "C13 FREE RUN  ");
    else if (motor_enabled)
        tft180_show_string(0, 134, speed_full_enabled ? "TRACK FULL    " : "TRACK LOW     ");
    else if (track_stop_reason == TRACK_STOP_LOST)
        tft180_show_string(0, 134, "TRACK LOST    ");
    else if (track_stop_reason == TRACK_STOP_STALE)
        tft180_show_string(0, 134, "CAM TIMEOUT   ");
    else
        tft180_show_string(0, 134, track_ready ? "READY C13:GO  " : "WAIT TRACK    ");
#endif
    screen_stats_dirty = 0;
}

// Overlay only: red real boundaries/corners, yellow virtual boundaries, blue path.
static void screen_point(float x,float y,uint16_t color)
{
    if(!isfinite(x) || !isfinite(y) || x<0.f || y<0.f || x>=(float)MT9V03X_W || y>=(float)MT9V03X_H) return;
    uint16_t sx=(uint16_t)(x*(float)TRACK_DISPLAY_WIDTH/(float)MT9V03X_W);
    uint16_t sy=(uint16_t)(y*(float)TRACK_DISPLAY_HEIGHT/(float)MT9V03X_H);
    tft180_draw_point(sx,sy,color);
}
static uint8_t screen_preview_guide(const ring_corner_frame_t *v)
{
    if(v->paired_mouth && v->guide[RING_GUIDE_270_CONNECT].valid) return RING_GUIDE_270_CONNECT;
    if(v->cross_entry && v->guide[RING_GUIDE_270_EXTEND].valid) return RING_GUIDE_270_EXTEND;
    for(unsigned side=0;side<2;++side) if(v->entry[side]) {
        unsigned id=ring_pdf_guide(1,side);
        if(v->guide[id].valid) return (uint8_t)id;
    }
    return RING_GUIDE_NONE;
}
/* Display-only state. No control mailbox, actuator or ISR writes. */
static ring_control_state_t screen_preview_state;
static uint8_t screen_preview_ready;
static void screen_preview_observe(uint32_t now)
{
    if(motor_enabled) {screen_preview_ready=0U;return;}
    if(!screen_preview_ready) {ring_control_init(&screen_preview_state);screen_preview_ready=1U;}
    /* Hand-push preview has no motion deadline; real driving still calls tick
     * with the original timeout and stale-frame protection in the ISR. */
    ring_control_observe(&screen_preview_state,&track_result.ring,now,1U);
    (void)ring_control_select_path(&screen_preview_state,&track_result.ring);
}
static void screen_draw_track_overlay(void)
{
    uint32_t primask=interrupt_global_disable();
    screen_ring_snapshot=ring_control;
    uint8_t running=motor_enabled;
    interrupt_global_enable(primask);
    if(!running && !screen_ring_snapshot.fault && screen_preview_ready)
        screen_ring_snapshot=screen_preview_state;
    const ring_control_state_t *state=&screen_ring_snapshot;
    const ring_corner_frame_t *corners=&screen_track_result.ring.corners;
    uint8_t same_frame=(uint8_t)(!RING_FEATURE_ENABLED || state->corner_frame_number==screen_track_result.frame_number);
    uint8_t approach_valid=(uint8_t)(screen_track_result.ring.approach_path.valid &&
        (screen_track_result.ring.approach_source==TRACK_PATH_LEFT || screen_track_result.ring.approach_source==TRACK_PATH_RIGHT));
    screen_guide_id=RING_GUIDE_NONE;screen_path_preview=(uint8_t)(!running && screen_preview_ready);screen_approach_source=TRACK_PATH_NORMAL;
    screen_path_source=state->fault ? '?' : 'N';
    if(!running && !state->fault && !state->action_locked) {
        if(approach_valid) {
            screen_approach_source=screen_track_result.ring.approach_source;
            screen_path_preview=1U;
        } else {
            screen_guide_id=screen_preview_guide(corners);
            screen_path_preview=(uint8_t)(screen_guide_id!=RING_GUIDE_NONE);
            if(screen_path_preview) screen_path_source='G';
        }
    } else if(!state->fault && state->corner_guide_held) {
        screen_path_source='H';
    } else if(!state->fault && !same_frame) {
        screen_path_source='?';
    } else if(!state->fault && !state->action_locked && approach_valid &&
              state->path_mode==screen_track_result.ring.approach_source) {
        screen_approach_source=screen_track_result.ring.approach_source;
    } else if(!state->fault && state->corner_guide_id!=RING_GUIDE_NONE &&
              state->corner_guide_frame_number==screen_track_result.frame_number &&
              state->corner_guide_id<RING_CORNER_GUIDE_COUNT &&
              corners->guide[state->corner_guide_id].valid) {
        screen_guide_id=state->corner_guide_id;
        screen_path_source='G';
    } else if(!state->fault && (state->line_active || state->path_mode!=TRACK_PATH_NORMAL)) {
        screen_path_source='?';
    }
    if(screen_approach_source!=TRACK_PATH_NORMAL)
        screen_path_source=screen_approach_source==TRACK_PATH_LEFT ? 'L' : 'R';
    for(unsigned row=MT9V03X_H*RING_SEARCH_START_PERCENT/100U;
        row<MT9V03X_H*RING_SEARCH_END_PERCENT/100U;++row) {
        if(screen_track_result.raw_flags[row]&1U) screen_point((float)screen_track_result.raw_left[row],(float)row,RGB565_RED);
        if(screen_track_result.raw_flags[row]&2U) screen_point((float)screen_track_result.raw_right[row],(float)row,RGB565_RED);
        /* The approach array belongs to this image; rejected rows stay gaps. */
        if(screen_approach_source!=TRACK_PATH_NORMAL && screen_track_result.approach_valid[row])
            screen_point((float)screen_track_result.approach_center[row],(float)row,RGB565_GREEN);
        else if(screen_guide_id==RING_GUIDE_NONE && screen_track_result.row_valid[row])
            screen_point((float)screen_track_result.center[row],(float)row,RGB565_GREEN);
    }
    for(unsigned side=0U;side<2U;++side) for(unsigned i=0U;i<corners->count[side] && i<RING_CORNER_MAX_PER_SIDE;++i) {
        const ring_corner_point_t *point=&corners->point[side][i];
        if(point->quality<RING_CORNER_MIN_QUALITY || !isfinite(point->x) || !isfinite(point->y)) continue;
        for(int d=-3;d<=3;++d) {
            screen_point(point->x+(float)d,point->y,RGB565_RED);
            screen_point(point->x,point->y+(float)d,RGB565_RED);
        }
        char label[2]={point->kind==RING_CORNER_A ? 'A' : (point->kind==RING_CORNER_B ? 'B' : '?'),0};
        unsigned x=(unsigned)(point->x*(float)TRACK_DISPLAY_WIDTH/(float)MT9V03X_W);
        unsigned y=(unsigned)(point->y*(float)TRACK_DISPLAY_HEIGHT/(float)MT9V03X_H);
        if(x+8U<TRACK_DISPLAY_WIDTH && y+16U<TRACK_DISPLAY_HEIGHT) tft180_show_string((uint16_t)x,(uint16_t)y,label);
    }
    /* Lower-case a means a locally reacquired, current-image A, not an extra
     * raw corner count or an old-frame overlay. */
    for(unsigned side=0U;side<2U;++side) if(corners->continued_mask&(1U<<side)) {
        const ring_corner_guide_t *tracked=&corners->guide[RING_GUIDE_A_LEFT+side];
        if(!tracked->valid) continue;
        unsigned x=(unsigned)(tracked->corner_x*(float)TRACK_DISPLAY_WIDTH/(float)MT9V03X_W);
        unsigned y=(unsigned)(tracked->corner_y*(float)TRACK_DISPLAY_HEIGHT/(float)MT9V03X_H);
        if(x+8U<TRACK_DISPLAY_WIDTH && y+16U<TRACK_DISPLAY_HEIGHT)
            tft180_show_string((uint16_t)x,(uint16_t)y,"a");
    }
    if(screen_guide_id==RING_GUIDE_NONE) return;
    const ring_corner_guide_t *guide=&corners->guide[screen_guide_id];
    for(unsigned side=0U;side<2U;++side) {
        const ring_corner_line_t *line=&guide->boundary[side];
        if(!line->valid) continue;
        float steps=fmaxf(fabsf(line->x1-line->x0),fabsf(line->y1-line->y0));
        if(steps>(float)(MT9V03X_H+MT9V03X_W)) continue;
        unsigned count=(unsigned)steps+1U;
        for(unsigned i=0U;i<=count;++i) {
            float t=(float)i/(float)count;
            screen_point(line->x0+t*(line->x1-line->x0),line->y0+t*(line->y1-line->y0),RGB565_BLUE);
        }
    }
    unsigned y0=MT9V03X_H*RING_GUIDE_FAR_PERCENT/100U,y1=MT9V03X_H*RING_GUIDE_NEAR_PERCENT/100U-1U;
    for(unsigned i=0U;i<RING_CORNER_CENTER_SAMPLES;++i) {
        if(!(guide->center_valid&(1U<<i))) continue;
        unsigned y=y0+i*(y1-y0)/(RING_CORNER_CENTER_SAMPLES-1U);
        screen_point((float)guide->center[i],(float)y,RGB565_GREEN);
        // Only connect consecutive valid samples; never bridge a rejected row gap.
        if(i>0U && (guide->center_valid&(1U<<(i-1U))) && (guide->center_link_valid&(1U<<i))) {
            unsigned previous=y0+(i-1U)*(y1-y0)/(RING_CORNER_CENTER_SAMPLES-1U);
            for(unsigned row=previous+1U;row<y;++row) {
                float t=(float)(row-previous)/(float)(y-previous);
                float x=(float)guide->center[i-1U]+t*((float)guide->center[i]-(float)guide->center[i-1U]);
                screen_point(x,(float)row,RGB565_GREEN);
            }
        }
    }
}

static track_ring_vision_state_t ring_vision_model;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    Init();
    // 此发送器接入 LPUART1_IRQHandler，改变调试串口时必须同时迁移中断。
    zf_assert(DEBUG_UART_INDEX == UART_1);
    timer_init(GPT_TIM_1, TIMER_MS);
    timer_start(GPT_TIM_1);
    gpio_init(SCREEN_TOGGLE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    interrupt_enable(LPUART1_IRQn);
    tft180_full(RGB565_WHITE);
    interrupt_global_enable(0);
    camera_exposure_status = mt9v03x_set_exposure_time(CAMERA_TUNING_EXPOSURE);
#if CAMERA_TUNING_AUTO_BLACK_LEVEL
    (void)mt9v03x_set_reg(0x47U, 0x0080U);
#endif
    uint8_t displayed_motor_state = motor_enabled;
    uint8_t displayed_protection_state = protection_enabled;
    uint8_t displayed_speed_mode = speed_full_enabled;
    uint32_t last_status_ms = 0;
    uint32_t last_processed_frame = 0;
    uint32_t last_display_ms = 0;

    while (1)
    {
        uint32_t now = timer_get(GPT_TIM_1);
#if SPEED_BENCH_MODE
#if SPEED_SCOPE_MODE
        assistant_send_scope(now);
#else
        assistant_send_speed(now);
#endif
        // 架空调速不处理摄像头和屏幕，保持串口采样接近50ms。
        continue;
#endif
        screen_button_poll(now);
        update_frame_rates(now);
        if ((uint32_t)(now - last_status_ms) >= 100U)
        {
            last_status_ms = now;
            screen_stats_dirty = 1;
        }
        if (displayed_motor_state != motor_enabled)
        {
            displayed_motor_state = motor_enabled;
            screen_stats_dirty = 1;
        }
        if (displayed_protection_state != protection_enabled)
        {
            displayed_protection_state = protection_enabled;
            screen_stats_dirty = 1;
        }
        if (displayed_speed_mode != speed_full_enabled)
        {
            displayed_speed_mode = speed_full_enabled;
            screen_stats_dirty = 1;
        }
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 原子取得帧指针和编号，重复帧不能被计入连续三帧就绪条件。
            uint32_t primask = interrupt_global_disable();
            uint32_t frame_number = mt9v03x_frame_count;
            const uint8_t *frame_source = (const uint8_t *)mt9v03x_image;
            mt9v03x_finish_flag = 0;
            interrupt_global_enable(primask);
            if (frame_number == last_processed_frame)
                continue;
            last_processed_frame = frame_number;
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, frame_source, sizeof(image));
            // CSI 可能在复制过程中回收旧缓冲。遇到切帧就丢弃副本，
            // 禁止将可能拼接的图像发布给控制器，也不长时间关闭中断。
            __DMB();
            if (mt9v03x_frame_count != frame_number)
                continue;
            // 摄像头反装：先交换上下行，再纠正左右镜像。
            // 在图传和图像处理之前翻转，使电脑、屏幕及寻线使用同一方向。
            for (uint16_t row = 0; row < MT9V03X_H / 2; ++row)
            {
                for (uint16_t col = 0; col < MT9V03X_W; ++col)
                {
                    uint8_t pixel = image[row][col];
                    image[row][col] = image[MT9V03X_H - 1 - row][col];
                    image[MT9V03X_H - 1 - row][col] = pixel;
                }
            }
            for (uint16_t row = 0; row < MT9V03X_H; ++row)
            {
                for (uint16_t col = 0; col < MT9V03X_W / 2; ++col)
                {
                    uint8_t pixel = image[row][col];
                    image[row][col] = image[row][MT9V03X_W - 1 - col];
                    image[row][MT9V03X_W - 1 - col] = pixel;
                }
            }
#if CAMERA_SEND_RAW
            if (screen_enabled && !SPEED_BENCH_MODE && !SPEED_SCOPE_MODE)
                assistant_prepare_raw_image();
#endif
            // 复用普通/环岛共同需要的地面范围，滤波也不跨越该范围边界。
            // 灰度图传快照保持原始值，屏幕与所有视觉识别共用处理后的二值图。
            if (!image_track_preprocess_stable((uint8_t *)image, MT9V03X_H, MT9V03X_W,
                    (uint16_t)((uint32_t)MT9V03X_H * RING_SEARCH_START_PERCENT / 100U),
                    (uint16_t)((uint32_t)MT9V03X_H * TRACK_VISION_ROI_END_PERCENT / 100U),
                    image_filter_scratch, sizeof(image_filter_scratch), hist_gram, &camera_threshold,
                    &image_preprocess_state, timer_get(GPT_TIM_1)))
            {
                camera_threshold = 255U;
            }
            // 在叠加调试线之前识别赛道，并将一帧完整结果交给控制中断。
            if (camera_threshold != 255U)
            {
                track_vision_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, &track_result);
                track_result.frame_number = frame_number;
                track_ring_vision_process(&ring_vision_model, (uint8_t *)image,
                                          MT9V03X_H, MT9V03X_W, &track_result);
            }
            else
            {
                /* Display current pixels, but never publish an unreliable road. */
                memset(&track_result, 0, sizeof(track_result));
                track_result.frame_number = frame_number;
            }
            if (!track_result.valid)
                ++invalid_vision_frames;
            track_frame_publish(&track_result);
            screen_preview_observe(timer_get(GPT_TIM_1));

            uint32_t display_now = timer_get(GPT_TIM_1);
            if (screen_enabled && (displayed_frames == 0U ||
                (uint32_t)(display_now - last_display_ms) >= SCREEN_FRAME_PERIOD_MS))
            {
                last_display_ms = display_now;
                /* Drawing is slower than camera capture. Control receives
                 * every processed frame; preview and text share one snapshot. */
                screen_track_result = track_result;
                tft180_displayimage03x((uint8_t *)image,
                                       TRACK_DISPLAY_WIDTH, TRACK_DISPLAY_HEIGHT);
                screen_draw_track_overlay();
                ++displayed_frames;
                screen_stats_dirty = 1U;
                screen_draw_stats();
            }
            ++processed_frames;

            // 发送已冻结的本帧灰度图，或按 CAMERA_SEND_RAW 配置发送处理图。
            if (screen_enabled && !SPEED_BENCH_MODE && !SPEED_SCOPE_MODE)
                assistant_send_image();

        }
        // 摄像头停帧时也要显示超时/停机状态，不能依赖下一帧到来。
        if (screen_enabled && screen_stats_dirty)
            screen_draw_stats();
    }
}
