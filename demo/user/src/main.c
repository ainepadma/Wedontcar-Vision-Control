#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "zf_device_imu660rb.h"

// 暗场先轻量去噪，再使用原始大津阈值区分白轨和蓝底。
// C13 控制启停；C14 开启时需赛道有效，关闭时只保留人工停机和 PWM 硬限幅。
#define CAMERA_SEND_RAW 1
#define CAMERA_NEAR_COLOR_DENOISE_ENABLED 1
#define CAMERA_NEAR_COLOR_RADIUS 4U
#define CAMERA_TUNING_EXPOSURE 1024U
#define CAMERA_THRESHOLD_BAND_ROWS 8U
#define CAMERA_THRESHOLD_OFFSET 12
#define CAMERA_STRETCH_LOW_PERCENT 1U
#define CAMERA_STRETCH_HIGH_PERCENT 96U
#define TRACK_DISPLAY_WIDTH 125U
#define TRACK_DISPLAY_HEIGHT 100U
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
    gpio_init(SPEED_MODE_TOGGLE_KEY, GPI, GPIO_HIGH, GPI_PULL_UP);
    track_runtime_init();
    pit_ms_init(PIT_CH0, SPEED_CONTROL_PERIOD_MS);

    // IMU 失败只记录状态，不阻塞摄像头、按键或循迹控制。
    imu_device_status = imu660rb_init();
    imu_navigation_init(&imu_navigation,
                        imu_device_status == IMU660RB_RESULT_OK);
}

AT_OCRAM_SECTION_ALIGN(uint8_t image[MT9V03X_H][MT9V03X_W], 64);
uint32_t hist_gram[256];
#if CAMERA_NEAR_COLOR_DENOISE_ENABLED
static uint8_t image_filter_scratch[2U * MT9V03X_W];
#endif
static uint8_t track_display_image[TRACK_DISPLAY_HEIGHT][TRACK_DISPLAY_WIDTH];
static track_vision_result_t track_result;

// 冻结一帧快照，由 UART 中断慢慢发送。传输中不覆盖，忙时跳过新预览帧。
// 数据头及诊断文本也纳入同一队列，禁止 printf 插入正在发送的图像。
#define CAMERA_DIAGNOSTIC_SIZE 768U
AT_OCRAM_SECTION_ALIGN(static uint8_t camera_tx_packet[
    CAMERA_DIAGNOSTIC_SIZE + 12U + sizeof(image)], 64);
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
    ring_control_state_t snapshot_ring = ring_control;
    uint8_t snapshot_imu_status = imu_device_status;
    interrupt_global_enable(primask);
    int prefix_length = snprintf((char *)camera_tx_packet, CAMERA_DIAGNOSTIC_SIZE,
        "#CAM FW=0.3.3 T=%lu CAP=%lu VF=%lu PROC=%lu DISP=%lu CFPS=%lu PFPS=%lu DFPS=%lu IO=%u MOTOR=%u MFAULT=%u PROTECT=%u FULL=%u EXP=%u STATUS=%u TRACK=%u Q=%u VR=%u/%u N=%u F=%u NW=%u FW=%u WG=%u RC=%u RD=%d REJ=%02X NC=%u AM=%u DC=%u ERR=%d HEAD=%d READY=%u STOP=%u SERVO10=%d TARGET10=%d LSPD=%d RSPD=%d LPWM=%u RPWM=%u THR=%u BAD=%lu IMU=%u ID=%02X IS=%lu IB=%u IBS=%u AF=%d AL=%d AU=%d GY=%d YR10=%d RS=%u RDIR=%d RY10=%d\r\n",
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
        (int)(snapshot_ring.yaw_progress_deg * 10.0f));
    if (prefix_length <= 0 || (uint32_t)prefix_length >= CAMERA_DIAGNOSTIC_SIZE)
        return;
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
    snprintf(line, sizeof(line), "E:%4d H:%4d", (int)track_result.error,
             (int)track_result.heading);
    tft180_show_string(0, 102, line);
    uint32_t primask = interrupt_global_disable();
    uint8_t ring_phase = ring_control.phase;
    float ring_yaw = ring_control.yaw_progress_deg;
    interrupt_global_enable(primask);
    snprintf(line, sizeof(line), "Q%3u R%u C%u Y%3d",
             (unsigned int)track_result.confidence,
             (unsigned int)ring_phase,
             (unsigned int)track_result.ring_candidate,
             (int)ring_yaw);
    tft180_show_string(0, 118, line);
    if (motor_feedback_fault)
    {
        snprintf(line, sizeof(line), "ENC WARN:%u    ",
                 (unsigned int)motor_feedback_fault);
        tft180_show_string(0, 134, line);
    }
    else if (motor_enabled && !track_ready)
        tft180_show_string(0, 134, "CAM WARN      ");
    else if (motor_enabled && (!imu_navigation.device_ready ||
                               !imu_navigation.bias_ready))
        tft180_show_string(0, 134, "IMU WARN      ");
    else if (motor_enabled)
        tft180_show_string(0, 134, speed_full_enabled ? "RUN FULL      " : "RUN LOW       ");
    else
        tft180_show_string(0, 134, "C13 START     ");
#endif
    screen_stats_dirty = 0;
}

// 将中央裁剪区域完整映射到 TFT；不改动二值图或控制器输入。
static void screen_prepare_track_crop(void)
{
    const uint16_t source_top = (uint16_t)((uint32_t)MT9V03X_H *
        TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t source_bottom = (uint16_t)((uint32_t)MT9V03X_H *
        TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t source_left = (uint16_t)((uint32_t)MT9V03X_W *
        TRACK_VISION_ROI_LEFT_PERCENT / 100U);
    const uint16_t source_right = (uint16_t)((uint32_t)MT9V03X_W *
        TRACK_VISION_ROI_RIGHT_PERCENT / 100U);
    const uint16_t source_height = source_bottom - source_top;
    const uint16_t source_width = source_right - source_left;

    for (uint16_t y = 0U; y < TRACK_DISPLAY_HEIGHT; ++y)
    {
        const uint16_t row = (uint16_t)(source_top +
            (uint32_t)y * (source_height - 1U) /
            (TRACK_DISPLAY_HEIGHT - 1U));
        for (uint16_t x = 0U; x < TRACK_DISPLAY_WIDTH; ++x)
        {
            const uint16_t col = (uint16_t)(source_left +
                (uint32_t)x * (source_width - 1U) /
                (TRACK_DISPLAY_WIDTH - 1U));
            track_display_image[y][x] = image[row][col];
        }
    }
}

// 只叠加到 TFT：红色为左右边沿，蓝色为判定中线。
static void screen_draw_track_overlay(void)
{
    const uint16_t source_top = (uint16_t)((uint32_t)MT9V03X_H *
        TRACK_VISION_ROI_START_PERCENT / 100U);
    const uint16_t source_bottom = (uint16_t)((uint32_t)MT9V03X_H *
        TRACK_VISION_ROI_END_PERCENT / 100U);
    const uint16_t source_left = (uint16_t)((uint32_t)MT9V03X_W *
        TRACK_VISION_ROI_LEFT_PERCENT / 100U);
    const uint16_t source_right = (uint16_t)((uint32_t)MT9V03X_W *
        TRACK_VISION_ROI_RIGHT_PERCENT / 100U);
    const uint16_t source_height = source_bottom - source_top;
    const uint16_t source_width = source_right - source_left;

    for (uint16_t row = source_top; row < source_bottom; ++row)
    {
        if (!track_result.row_valid[row])
            continue;

        const uint16_t y = (uint16_t)((uint32_t)(row - source_top) *
            (TRACK_DISPLAY_HEIGHT - 1U) / (source_height - 1U));
        if (track_result.left[row] < source_left ||
            track_result.right[row] >= source_right)
            continue;

        uint16_t left = (uint16_t)((uint32_t)(track_result.left[row] - source_left) *
                                   (TRACK_DISPLAY_WIDTH - 1U) / (source_width - 1U));
        uint16_t center = (uint16_t)((uint32_t)(track_result.center[row] - source_left) *
                                     (TRACK_DISPLAY_WIDTH - 1U) / (source_width - 1U));
        uint16_t right = (uint16_t)((uint32_t)(track_result.right[row] - source_left) *
                                    (TRACK_DISPLAY_WIDTH - 1U) / (source_width - 1U));
        if (left >= TRACK_DISPLAY_WIDTH)
            left = TRACK_DISPLAY_WIDTH - 1U;
        if (center >= TRACK_DISPLAY_WIDTH)
            center = TRACK_DISPLAY_WIDTH - 1U;
        if (right >= TRACK_DISPLAY_WIDTH)
            right = TRACK_DISPLAY_WIDTH - 1U;

        tft180_draw_point(left, y, RGB565_RED);
        tft180_draw_point(right, y, RGB565_RED);
        tft180_draw_point(center, y, RGB565_BLUE);
    }
}

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
    uint8_t displayed_motor_state = motor_enabled;
    uint8_t displayed_protection_state = protection_enabled;
    uint8_t displayed_speed_mode = speed_full_enabled;
    uint32_t last_status_ms = 0;
    uint32_t last_processed_frame = 0;

    while (1)
    {
        uint32_t now = timer_get(GPT_TIM_1);
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
            if (screen_enabled)
                assistant_prepare_raw_image();
#endif
            // 摄像头已在传感器端只输出原生 600x307 有效窗口，区域外像素
            // 不进入 CSI；此处直接处理完整窗口，不再做软件清零裁剪。
            // 只按每个像素周围 3x3 邻域中的相似灰度组降噪，不做逐行、
            // 逐列修正，也不跨越白轨/蓝底的强边沿混合。
#if CAMERA_NEAR_COLOR_DENOISE_ENABLED
            if (!image_near_color_denoise3x3(
                    (uint8_t *)image, MT9V03X_H, MT9V03X_W,
                    CAMERA_NEAR_COLOR_RADIUS, image_filter_scratch,
                    sizeof(image_filter_scratch)))
                memset(image, 0, sizeof(image));
#endif
            if (!image_banded_percentile_stretch(
                    (uint8_t *)image, MT9V03X_H, MT9V03X_W,
                    CAMERA_THRESHOLD_BAND_ROWS, CAMERA_STRETCH_LOW_PERCENT,
                    CAMERA_STRETCH_HIGH_PERCENT, hist_gram))
                memset(image, 0, sizeof(image));
            // 车载电源下原始灰度集中在低动态范围：逐带拉伸后局部阈值化，
            // 不使用会抹平赛道边沿的横向降噪。
            camera_threshold = image_banded_otsu_binary_offset(
                (uint8_t *)image, MT9V03X_H, MT9V03X_W,
                CAMERA_THRESHOLD_BAND_ROWS, CAMERA_THRESHOLD_OFFSET, hist_gram);
            // 在叠加调试线之前识别赛道，并将一帧完整结果交给控制中断。
            track_vision_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, &track_result);
            if (track_vision_fit_binary((uint8_t *)image, MT9V03X_H,
                                        MT9V03X_W, &track_result))
                track_vision_process((uint8_t *)image, MT9V03X_H,
                                     MT9V03X_W, &track_result);
            track_result.frame_number = frame_number;
            if (!track_result.valid)
                ++invalid_vision_frames;
            track_frame_publish(&track_result);

            if (screen_enabled)
            {
                screen_prepare_track_crop();
                tft180_show_gray_image(0, 0, (uint8_t *)track_display_image,
                                       TRACK_DISPLAY_WIDTH, TRACK_DISPLAY_HEIGHT,
                                       TRACK_DISPLAY_WIDTH, TRACK_DISPLAY_HEIGHT, 0);
                screen_draw_track_overlay();
                ++displayed_frames;
                // 图像、边线与诊断全部读取刚完成的 track_result，禁止跨帧混看。
                screen_stats_dirty = 1U;
                screen_draw_stats();
            }
            ++processed_frames;

            // 发送已冻结的本帧灰度图，或按 CAMERA_SEND_RAW 配置发送处理图。
            if (screen_enabled)
                assistant_send_image();

        }
        // 摄像头停帧时也要显示超时/停机状态，不能依赖下一帧到来。
        if (screen_enabled && screen_stats_dirty)
            screen_draw_stats();
    }
}
