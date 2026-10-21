/* ZORE original fuzzy steering and signed rounded mul/div; hardware stub only. */
typedef int16_t int16;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef int32_t int32;
typedef struct {int16 Error0,Error1,Out,OutMin,OutMax;} PID_t;
#define SERVO_FUZZY_LEVELS 7
#define FUZZY_MEM_SCALE 16
#define SERVO_GAIN_DEN 5
#define SERVO_E_MAX 30
#define SERVO_EC_MAX 20
#define SERVO_KP_MIN_X5 50
#define SERVO_KP_MAX_X5 256
#define SERVO_KD_MIN_X5 0
#define SERVO_KD_MAX_X5 80
#define Cross_ture 4
#define Cross_mid 5
#define func_limit_ab(x,lo,hi) ((x)<(lo) ? (lo) : ((x)>(hi) ? (hi) : (x)))
static int cross_ring_state;
static int16 imu660rb_gyro_z;
#define imu660rb_get_gyro() ((void)0)
#define imu660rb_gyro_transition(x) (x)
int16 kp_base = 120;
int16 kd_base = 0;


int16 gyro_num = 14;
int16 gyro_den = 10;


int16 ring_gyro_num = 13; 
int16 ring_gyro_den = 10;


int16 cross_ring_gyro_num = 14; 
int16 cross_ring_gyro_den = 10;

static const uint8 servo_kp_table_x5[SERVO_FUZZY_LEVELS][SERVO_FUZZY_LEVELS]=
{
    
    {215, 215, 185, 185, 175, 130, 130},  
    {215, 215, 185, 175, 175, 130, 175},  
    {185, 185, 185, 175, 130, 175, 175},  
    {185, 185, 175, 130, 175, 185, 185},  
    {175, 175, 130, 175, 175, 185, 185},  
    {175, 130, 175, 185, 185, 215, 215},  
    {130, 130, 185, 185, 185, 215, 215}   
};













static const uint8 servo_kd_table_x5[SERVO_FUZZY_LEVELS][SERVO_FUZZY_LEVELS] =
{
    
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0},  
    {0, 0, 0, 0, 0, 0, 0}   
	};

int16 mul_div_i16(int16 value, int16 num, int16 den)
{
    int32 temp;

    if (den == 0)
    {
        return 0;
    }

    temp = (int32)value * (int32)num;

    if (den == 256)
    {
        if (temp >= 0)
        {
            temp = (temp + 128) >> 8;    
        }
        else
        {
            temp = (temp - 128) >> 8;    
        }
    }
    else
    {
        if (temp >= 0)
        {
            temp = (temp + den / 2) / den;
        }
        else
        {
            temp = (temp - den / 2) / den;
        }
    }

    return (int16)func_limit_ab(temp, -32768, 32767);
}
static void fuzzy_get_membership(int16 x,
                                 int16 x_max,
                                 uint8 *left_index,
                                 uint8 *right_index,
                                 uint16 *left_mu,
                                 uint16 *right_mu)
{
    int16 q;    
    uint8 index;
    uint16 frac;

    if (x_max <= 0)
    {
        *left_index = 3;
        *right_index = 3;
        *left_mu = FUZZY_MEM_SCALE;
        *right_mu = 0;
        return;
    }

    
    if (x <= -x_max)
    {
        *left_index = 0;
        *right_index = 0;
        *left_mu = FUZZY_MEM_SCALE;
        *right_mu = 0;
        return;
    }

    
    if (x >= x_max)
    {
        *left_index = 6;
        *right_index = 6;
        *left_mu = FUZZY_MEM_SCALE;
        *right_mu = 0;
        return;
    }

    q = ((int16)(x + x_max) * (SERVO_FUZZY_LEVELS - 1) * FUZZY_MEM_SCALE)
        / ((int16)2 * x_max);

    index = (uint8)(q / FUZZY_MEM_SCALE);
    frac  = (uint16)(q - (int32)index * FUZZY_MEM_SCALE);

    if (index >= 6)
    {
        *left_index = 6;
        *right_index = 6;
        *left_mu = FUZZY_MEM_SCALE;
        *right_mu = 0;
        return;
    }

    *left_index = index;
    *right_index = index + 1;

    *left_mu = FUZZY_MEM_SCALE - frac;
    *right_mu = frac;
}
static int16 fuzzy_calc_table_x5(const uint8 table[SERVO_FUZZY_LEVELS][SERVO_FUZZY_LEVELS],
                                 uint8 e_l,
                                 uint8 e_r,
                                 uint16 e_mu_l,
                                 uint16 e_mu_r,
                                 uint8 ec_l,
                                 uint8 ec_r,
                                 uint16 ec_mu_l,
                                 uint16 ec_mu_r)
{
    int32 sum;
    int32 den;

    den = (int32)FUZZY_MEM_SCALE * FUZZY_MEM_SCALE;

    sum = 0;

    sum += (int32)e_mu_l * ec_mu_l * table[e_l][ec_l];
    sum += (int32)e_mu_l * ec_mu_r * table[e_l][ec_r];
    sum += (int32)e_mu_r * ec_mu_l * table[e_r][ec_l];
    sum += (int32)e_mu_r * ec_mu_r * table[e_r][ec_r];
	
    
     return (int16)((sum + 128) >> 8);
}
static void Servo_Fuzzy_Get_Kp_Kd(int16 error,
                                  int16 ec,
                                  int16 *kp_x5,
                                  int16 *kd_x5)
{
    uint8 e_l;
    uint8 e_r;
    uint8 ec_l;
    uint8 ec_r;

    uint16 e_mu_l;
    uint16 e_mu_r;
    uint16 ec_mu_l;
    uint16 ec_mu_r;

    fuzzy_get_membership(error , SERVO_E_MAX , &e_l , &e_r , &e_mu_l , &e_mu_r);

    fuzzy_get_membership(ec ,  SERVO_EC_MAX, &ec_l,  &ec_r, &ec_mu_l, &ec_mu_r);

    *kp_x5 = fuzzy_calc_table_x5(servo_kp_table_x5 , e_l , e_r , e_mu_l , e_mu_r , ec_l , ec_r , ec_mu_l , ec_mu_r);

    *kd_x5 = fuzzy_calc_table_x5(servo_kd_table_x5 , e_l , e_r , e_mu_l , e_mu_r , ec_l , ec_r , ec_mu_l , ec_mu_r);

    *kp_x5 = (int16)func_limit_ab(*kp_x5, SERVO_KP_MIN_X5, SERVO_KP_MAX_X5);
    *kd_x5 = (int16)func_limit_ab(*kd_x5, SERVO_KD_MIN_X5, SERVO_KD_MAX_X5);
}
void PID_Update_Servo(PID_t *p, int16 error)
{
    int16 ec, kp_x5, kd_x5, term_p, term_d, term_gyro, out_temp;
    
    int16 current_gyro_num;
    int16 current_gyro_den;

    p->Error1 = p->Error0;
    p->Error0 = error;
    
    ec = -(p->Error0 - p->Error1); 
        
    Servo_Fuzzy_Get_Kp_Kd(p->Error0, ec, &kp_x5, &kd_x5);
    
    term_p = mul_div_i16(p->Error0, kp_x5 + kp_base, SERVO_GAIN_DEN);
    term_d = mul_div_i16(ec,        kd_x5 + kd_base, SERVO_GAIN_DEN);
    
    
    
    if (cross_ring_state == 7)
    {
        
        current_gyro_num = cross_ring_gyro_num;
        current_gyro_den = cross_ring_gyro_den;
    }
    else if ((ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque) && ImageFlag.image_element_rings_flag >= 6&& ImageFlag.image_element_rings_flag <= 7)
    {
        
        current_gyro_num = ring_gyro_num;
        current_gyro_den = ring_gyro_den;
    }
    else
    {
        
        current_gyro_num = gyro_num;
        current_gyro_den = gyro_den;
    }

		imu660rb_get_gyro();
    
    term_gyro = mul_div_i16((int16)imu660rb_gyro_transition(imu660rb_gyro_z), current_gyro_num, current_gyro_den);
    
    out_temp = term_p + term_d + term_gyro;

    
    if (ImageStatus.Road_type == Cross_ture || ImageStatus.Road_type == Cross_mid)
    {
        
        if (out_temp - p->Out > 30)       out_temp = p->Out + 30;
        else if (out_temp - p->Out < -30) out_temp = p->Out - 30;
    }

    p->Out = (int16)func_limit_ab(out_temp, p->OutMin, p->OutMax);
}
