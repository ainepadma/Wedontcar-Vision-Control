#include "main.h" // 所有引脚信息更改在main.h里改宏

// 降低二值化阈值以保留偏暗的白色赛道；100 为原始大津法阈值。
#define TRACK_THRESHOLD_PERCENT 70U
// 静态图像标定期间禁止电机/舵机动作，避免串口阻塞影响行驶控制。
#define CAMERA_TUNING_MODE 1
#define CAMERA_SEND_RAW 1
#define CAMERA_TUNING_EXPOSURE 1024U
static uint8_t camera_exposure_status;

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

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

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
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
#if !CAMERA_TUNING_MODE
    pit_ms_init(PIT_CH0, 10);
#endif
}

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];

// 逐飞助手 V2：12 字节图像配置头，后接逐像素灰度数据。
// 与官方 seekfree_assistant_camera_config / camera_send 的线格式一致。
static void assistant_send_image(void)
{
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
    uart_write_buffer(DEBUG_UART_INDEX, header, sizeof(header));
    uart_write_buffer(DEBUG_UART_INDEX, (uint8_t *)image, sizeof(image));
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    Init();
    interrupt_global_enable(0);
    camera_exposure_status = mt9v03x_set_exposure_time(CAMERA_TUNING_EXPOSURE);

    while (1)
    {
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
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
            printf("#CAM EXP=%u STATUS=%u\r\n", CAMERA_TUNING_EXPOSURE, camera_exposure_status);
            assistant_send_image();
#endif
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            unsigned int adjusted_threshold =
                (unsigned int)get_threshold_otsu(hist_gram) * TRACK_THRESHOLD_PERCENT / 100U;
            // 寻边使用严格大小比较，阈值保持在黑白像素值之间。
            unsigned char threshold = (unsigned char)MAX(1U, MIN(254U, adjusted_threshold));
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);

            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][mid_line[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 100);

            // 通过调试串口向逐飞助手发送处理后的图像
#if !CAMERA_SEND_RAW
            assistant_send_image();
#endif

            // 取图像下1/4处做误差判断
            float mid_err = (MT9V03X_W/2) - mid_line[MT9V03X_H-MT9V03X_H/4];
            // 舵机根据误差乘以系数打角
            float tmp_duty = servo_motor_duty_middle - mid_err * 0.5 * SERVO_DIR;
            // 限幅
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
#if !CAMERA_TUNING_MODE
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
#endif

            // 显示关键信息
            tft180_show_int(0, 0, encoder_data_1, 3);
            tft180_show_int(0, 30, encoder_data_2, 3);
            tft180_show_float(0, 50, speed_pwm, 4, 2);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
    }
}
