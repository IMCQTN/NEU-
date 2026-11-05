/* 本文件从用户粘贴的原程序直接添加蓝牙接口。
 * 恢复原超声波自动显示/报警；温度独立计时，蓝牙接口和按键保留，无01/00模式切换。
 * 用Keil C51编译。串口9600 8N1，普通命令LF结尾，ONA/ONB/ONL/ONR/ONS可不换行。
 * 例：PING、START、STOP、MOVE F、GEAR UP、TURN L、HORN 1、GET、TIME 12:34:56。
 * MOVE L/R沿用原单轮函数；LOCK/BREATH沿用原呼吸灯继电器联动；PAGE只有TEMP/TIME。
 * 保留原阻塞执行顺序及持续运行习惯，KEEP仅应答，无新增自动停车超时。
 */
/*
 * ================================================================
 * 汽车座舱综合考核程序（单文件、现场改参数版）
 * 芯片：IAP15F2K61S2；Keil C51；矩阵键盘；一份 HEX 演示全部功能。
 * 基础：已经测试过的 Timer2 超声波测距版本。
 * 中文编码：GB2312（在 Keil 中按 ANSI/简体中文打开）。
 *
 * 现场改题：优先只修改下面的 #define 常量，然后重新编译烧录。
 * 时间的单位为毫秒 ms；次数为完整的亮-灭或亮度升-降周期。
 * ================================================================
 */

#include <REG52.H>
#include <INTRINS.H>

/* 仅新增蓝牙队列和文本命令解析。原硬件驱动、参数、按键和主循环结构保留。 */
typedef unsigned char u8;
typedef unsigned int u16;
#define RX_MASK 127U
#define TX_MASK 511U
#define LINE_MAX 63U
#define REPLY_MAX 240U
static volatile u8 xdata rxbuf[128];
static volatile u8 rx_head,rx_tail,rx_overflow,rx_drops,rx_value,rx_next;
static volatile u8 xdata txbuf[512];
static volatile u16 tx_head,tx_tail;
static volatile u8 tx_busy;
static u8 xdata tx_drops;
static char xdata linebuf[64];
static u8 xdata line_len,line_discard,line_error;
static u8 xdata arg_start[4],arg_count;
static u8 xdata reply_buf[REPLY_MAX];
static u8 xdata reply_len,reply_overflow;
static u16 xdata parsed_value;
static u8 xdata api_radar_enabled=1; /* 原程序雷达常开，不允许接口误关 */
static u16 xdata api_distance_cm=65535U,api_distance_at;
static u16 api_distance_cached(void);
void BluetoothTask(void);


/* ================== 现场改题参数区（最重要） ================== */

/* 第1题：SW1 启动动画；亮和响默认同步各三次。 */
#define START_LED_COUNT        6U      /* LED 闪烁次数；5U 即闪五次 */
#define START_BEEP_COUNT       5U      /* 蜂鸣器响的次数；可与 LED 不同 */
#define START_LED_ON_MS      500U      /* 每次 LED 亮多久 */
#define START_BEEP_ON_MS     500U      /* 每次蜂鸣器响多久 */
#define START_OFF_MS         500U      /* 每轮结束后，静默和灭灯多久 */

/* 第2题：SW5 左转向。短按至少闪 LEFT_COUNT 次；长按反复闪。 */
#define LEFT_COUNT             9U      /* 左转向最少闪烁次数 */
#define LEFT_ON_MS           200U      /* 左侧四灯点亮时长 */
#define LEFT_OFF_MS          200U      /* 左侧四灯熄灭时长 */

/* 第2题：SW9 右转向。短按三次；长按反复闪。 */
#define RIGHT_COUNT            9U      /* 右转向最少闪烁次数 */
#define RIGHT_ON_MS          200U      /* 右侧四灯点亮时长 */
#define RIGHT_OFF_MS         200U      /* 右侧四灯熄灭时长，对应题目0.2秒 */

/* 左右接线若相反，交换这两个掩码，不要改按键编号。 */
#define LEFT_LED_MASK       0x0FU     /* P80_00 ~ P80_03 */
#define RIGHT_LED_MASK      0xF0U     /* P80_04 ~ P80_07 */

/* 第5题：雷达自动检测及报警；保留已实测接近准确的标定系数。 */
#define RADAR_ALERT_CM        5U      /* 距离低于15厘米开始报警 */
#define RADAR_ALWAYS_CM        5U      /* 距离低于5厘米蜂鸣器常响 */
#define RADAR_SAMPLE_MS      120U      /* 超声波采样周期 */
#define RADAR_BEEP_MS         60U      /* 每次短报警蜂鸣器持续时间 */
#define ULTRA_TICKS_PER_CM    54U      /* Timer2每厘米计数；测距不准再校准 */

/* 第6题：SW10 时钟初始值，DS1302采用BCD编码。 */
#define CLOCK_HOUR_BCD       0x15      /* 0x12代表12时，不是十进制18 */
#define CLOCK_MIN_BCD        0x15      /* 0x30代表30分 */
#define CLOCK_SEC_BCD        0x00      /* 0x45代表45秒 */
#define CLOCK_REFRESH_MS     500U      /* 数码管时间刷新周期 */

/* 第7题：温度读取周期。接入DHT11或DS18B20自动识别。 */
#define DHT11_READ_MS       2500U      /* DHT11读取间隔，勿设太短 */
#define DS18B20_READ_MS      850U      /* DS18B20转换后读取间隔 */

/* 第8题：SW14 PWM呼吸灯、继电器。 */
#define PWM_LEVELS            10U      /* PWM亮度级数，默认0~10 */
#define PWM_PERIOD_TICKS      10U      /* PWM周期：10次1ms节拍，约10ms */
#define PWM_STEP_MS           20U      /* 每20ms变化一个亮度等级 */
#define PWM_BREATH_CYCLES      0U      /* 0=一直呼吸至再次按键；3=呼吸3次后自动结束 */

/* 系统时钟与按键。922对应11.0592MHz、12T时基约1ms。 */
#define TIMER0_1MS_COUNTS    922U      /* 如果实际IRC时钟不同，先测量再改 */
#define KEY_DEBOUNCE_MS       10U      /* 消抖时间 */
#define LOOP_IDLE_MS           1U      /* 主循环每轮等待 */

/* ================== 参数区结束，不熟悉不要改下面驱动 ================== */

sfr  P4   = 0xC0;
sfr  AUXR = 0x8E;
sfr  T2H  = 0xD6;
sfr  T2L  = 0xD7;
sfr OW_PCA_CCON = 0xD8;
sfr OW_PCA_CMOD = 0xD9;
sfr OW_PCA_L = 0xE9;
sfr OW_PCA_H = 0xF9;


sbit DHT    = P1^7;
sbit DS_SCK = P1^4;
sbit DS_SDA = P1^5;
sbit DS_RST = P1^6;
sbit TRIG   = P1^0;
sbit ECHO   = P1^1;

unsigned char code SEG[18] =
{0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,
 0x80,0x90,
 0xbf,0xff,0xa1,0xc6,0x8e,0xa3,0x7f,0x00};

unsigned char LED_dat = 0xff;
unsigned char E0_dat  = 0xff;

unsigned char disp_buf[8];
unsigned char disp_pos = 0;

unsigned char locked   = 0;
unsigned char led_busy = 0;
unsigned char pwm_cnt  = 0;
unsigned char pwm_duty = 0;
unsigned char pwm_dir  = 1;
unsigned int  pwm_step = 0;
volatile unsigned int pwm_cycles_done = 0; /* 已完成的完整呼吸次数 */

signed char   gear = 0;
unsigned char temp_h=0, temp_l=0, hum_h=0, hum_l=0;
unsigned char temp_dec2 = 0;
unsigned char temp_ok  = 0;
unsigned char temp_type = 0;

unsigned char page = 0;
unsigned char radar_on = 0;

volatile unsigned int sys_tick = 0;
unsigned int temp_t0 = 0;
unsigned int clk_t0  = 0;
unsigned int rad_t0  = 0;
unsigned int ds_t0   = 0;

//volatile unsigned char bt_cmd = 0;

/* 蓝牙接收状态机：0=等O，1=已收O，2=已收ON */
volatile unsigned char bt_state = 0;
volatile unsigned char bt_event = 0;   /* 1=收到ONA(正转)，2=收到ONB(反转) */


/* 三个不同任务同时申请蜂鸣器，避免相互误关。 */
#define BEEP_START  0x01
#define BEEP_RADAR  0x02
#define BEEP_HORN   0x04
unsigned char beep_flags = 0;

unsigned char xdata cntbuf[40];

/* 74HC138译码：Y4 LED；Y5 段选；Y6 位选；Y7 蜂鸣器/继电器。 */
void InitHC138(unsigned char n)
{
    switch(n)
    {
        case 4: P2 = (P2 & 0x1f) | 0x80; break;
        case 5: P2 = (P2 & 0x1f) | 0xa0; break;
        case 6: P2 = (P2 & 0x1f) | 0xc0; break;
        case 7: P2 = (P2 & 0x1f) | 0xe0; break;
    }
}

/*
 * 写74HC573锁存器。先把数据放P0，再选通锁存器，最后退出。
 * 禁止被数码管中断打断，否则会把LED或蜂鸣器数据写入错误的锁存器。
 * 本接法需要主板WR跳帽按原工程选择常低端，具体见README。
 */
void OutPutP0(unsigned char ch, unsigned char dat)
{
    unsigned char old_ea;
    old_ea = EA;
    EA = 0;
    P2 &= 0x1f;
    P0 = dat;
    InitHC138(ch);
    _nop_();
    P2 &= 0x1f;
    P0 = 0x00;
    EA = old_ea;
}

/*
 * 中断内专用写锁存器：中断不会与主循环同时执行，不能反复使用主循环
 * 的OutPutP0()，否则Keil C51可能出现函数重入/存储区覆盖风险。
 */
void ISRWriteP0(unsigned char ch, unsigned char dat)
{
    P2 &= 0x1f;
    P0 = dat;
    P2 = (P2 & 0x1f) | ((unsigned char)ch << 5);
    _nop_();
    P2 &= 0x1f;
    P0 = 0x00;
}

/* 读取16位毫秒计数时短暂关中断，避免读到撕裂的高低字节。 */
unsigned int TickNow(void)
{
    unsigned int t;
    unsigned char old_ea;
    old_ea = EA;
    EA = 0;
    t = sys_tick;
    EA = old_ea;
    return t;
}

/* 基于Timer0的毫秒延时，比原来的空循环更方便现场改参数。 */
void Delay(unsigned int ms)
{
    unsigned int t0;
    t0 = TickNow();
    while((unsigned int)(TickNow() - t0) < ms);
}

/* DS18B20与超声波触发用的短延时；这里不是毫秒。 */
void Dly(unsigned int n)
{
    while(n--)
    {
        _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
        _nop_(); _nop_(); _nop_();
    }
}

/* 调试串口发送一个字符，发送期间关串口中断，避免TI干扰。 */
void UartSend(unsigned char c)
{
    ES = 0;
    TI = 0;
    SBUF = c;
    while(TI == 0);
    TI = 0;
    ES = 1;
}

/* 输出传感器状态调试字符；正常演示不需要手机参与。 */
void Report(unsigned char c, unsigned char idx)
{
    static unsigned char last = 0;
    static unsigned int  t0 = 0;
    unsigned char due;

    return; /* 蓝牙接口使用完整行回复，禁用原串口裸调试字符。 */

    due = (unsigned char)(((unsigned int)(TickNow() - t0) >= 5000) ? 1 : 0);
    if(c == last)
    {
        if(c == 'Y' || due == 0) return;
    }
    last = c;
    t0 = TickNow();
    UartSend(c);
    if(c == 'D' || c == 'E')
    {
        UartSend((unsigned char)('0' + idx / 10));
        UartSend((unsigned char)('0' + idx % 10));
    }
}

/* LED低电平亮：on=1全亮，on=0全灭。 */
void LedAll(unsigned char on)
{
    LED_dat = on ? 0x00 : 0xff;
    OutPutP0(4, LED_dat);
}

/* 只改左四灯，不覆盖右四灯状态。 */
void LedLeft(unsigned char on)
{
    if(on) LED_dat &= (unsigned char)(~LEFT_LED_MASK);
    else   LED_dat |= LEFT_LED_MASK;
    OutPutP0(4, LED_dat);
}

/* 只改右四灯；左右实际位置以接线为准。 */
void LedRight(unsigned char on)
{
    if(on) LED_dat &= (unsigned char)(~RIGHT_LED_MASK);
    else   LED_dat |= RIGHT_LED_MASK;
    OutPutP0(4, LED_dat);
}

/* 蜂鸣器PE0_04低有效，三路请求取“或”，松开喇叭不会关掉雷达报警。 */
void BuzzerRequest(unsigned char source, unsigned char on)
{
    unsigned char old_ea, old_flags;
    old_ea = EA;
    EA = 0;
    old_flags = beep_flags;
    if(on) beep_flags |= source;
    else   beep_flags &= (unsigned char)(~source);
    if(old_flags != beep_flags)
    {
        if(beep_flags) E0_dat &= 0xef;
        else           E0_dat |= 0x10;
        OutPutP0(7, E0_dat);
    }
    EA = old_ea;
}

/* 上电必须主动关闭蜂鸣器，不能依赖默认端口电平。 */
void BuzzerInit(void)
{
    beep_flags = 0;
    E0_dat |= 0x10;
    OutPutP0(7, E0_dat);
}

/* 继电器PE0_05同样是低有效，写入时保留蜂鸣器状态。 */
void RelayOn(void)
{
    unsigned char old_ea;
    old_ea = EA;
    EA = 0;
    E0_dat &= 0xdf;
    OutPutP0(7, E0_dat);
    EA = old_ea;
}
void RelayOff(void)
{
    unsigned char old_ea;
    old_ea = EA;
    EA = 0;
    E0_dat |= 0x20;
    OutPutP0(7, E0_dat);
    EA = old_ea;
}

/* 4x4矩阵键盘扫描：返回SW1~SW16编号，0表示未按。 */
unsigned char KeyScan(void)
{
    unsigned char row, col, c;

    P2 = 0x0f;
    P4 = (P4 & 0xf0) | 0x0f;

    for(row=0; row<4; row++)
    {
        P2 = 0x0f & ~(1<<row);
        col = P4 & 0x0f;

        if     ((col & 0x08)==0) c = 0;
        else if((col & 0x04)==0) c = 1;
        else if((col & 0x02)==0) c = 2;
        else if((col & 0x01)==0) c = 3;
        else continue;

        return c*4 + row + 1;
    }
    return 0;
}

/*
 * 第1题：SW1汽车启动。LED亮与蜂鸣器响默认同时开始；各自时长可分开改。
 * 分别计数，老师要求“LED闪5次，蜂鸣器响3次”也只改顶部参数。
 */
void CarStart(void)
{
    unsigned int i, total;
    total = (START_LED_COUNT > START_BEEP_COUNT) ? START_LED_COUNT : START_BEEP_COUNT;
    led_busy = 1;
    for(i=0; i<total; i++)
    {
        LedAll(i < START_LED_COUNT);
        BuzzerRequest(BEEP_START, (unsigned char)(i < START_BEEP_COUNT));

        if(START_LED_ON_MS >= START_BEEP_ON_MS)
        {
            Delay(START_BEEP_ON_MS);
            BuzzerRequest(BEEP_START, 0);
            Delay(START_LED_ON_MS - START_BEEP_ON_MS);
            LedAll(0);
        }
        else
        {
            Delay(START_LED_ON_MS);
            LedAll(0);
            Delay(START_BEEP_ON_MS - START_LED_ON_MS);
            BuzzerRequest(BEEP_START, 0);
        }
        Delay(START_OFF_MS);
    }
    BuzzerRequest(BEEP_START, 0);
    LedAll(0);
    led_busy = 0;
}

/* 第2题：SW5短按至少LEFT_COUNT次；持续按住则不断重复。 */
void TurnLeft(void)
{
    unsigned int n = 0;
    led_busy = 1;
    while(1)
    {
        LedLeft(1);
        Delay(LEFT_ON_MS);
        LedLeft(0);
        Delay(LEFT_OFF_MS);
        n++;
        if(n >= LEFT_COUNT && KeyScan() != 5) break;
    }
    led_busy = 0;
}

/* 第2题：SW9亮RIGHT_ON_MS，灭RIGHT_OFF_MS；长按持续。 */
void TurnRight(void)
{
    unsigned int n = 0;
    led_busy = 1;
    while(1)
    {
        LedRight(1);
        Delay(RIGHT_ON_MS);
        LedRight(0);
        Delay(RIGHT_OFF_MS);
        n++;
        if(n >= RIGHT_COUNT && KeyScan() != 9) break;
    }
    led_busy = 0;
}

/* Timer0每约1ms中断一次，给显示刷新、PWM和软件定时提供时基。 */
void Timer0_Init(void)
{
    AUXR = 0x00;

    TMOD &= 0xF0;
    TMOD |= 0x01;
    TH0 = (65536UL - TIMER0_1MS_COUNTS) / 256;
    TL0 = (65536UL - TIMER0_1MS_COUNTS) % 256;
    ET0 = 1;
    EA  = 1;
    TR0 = 1;
}

/*
 * 中断1：数码管8位动态扫描；SW14开启后执行10级软件PWM。
 * 每次亮度从0升到最大再降回0，才算1次“完整呼吸”。
 * PWM_BREATH_CYCLES=0时无限循环；设为N时自动停止并释放继电器。
 */
void Timer0_ISR(void) interrupt 1
{
    unsigned char p2bak;

    TH0 = (65536UL - TIMER0_1MS_COUNTS) / 256;
    TL0 = (65536UL - TIMER0_1MS_COUNTS) % 256;
    sys_tick++;

    /* 先关所有位，再写段码，最后只打开一位，避免重影。 */
    p2bak = P2;
    ISRWriteP0(6, 0x00);
    ISRWriteP0(5, disp_buf[disp_pos]);
    ISRWriteP0(6, (unsigned char)(1U << disp_pos));
    P2 = p2bak;
    disp_pos++;
    if(disp_pos >= 8) disp_pos = 0;

    if(locked && !led_busy)
    {
        pwm_step++;
        if(pwm_step >= PWM_STEP_MS)
        {
            pwm_step = 0;
            if(pwm_dir)
            {
                if(pwm_duty < PWM_LEVELS) pwm_duty++;
                if(pwm_duty >= PWM_LEVELS) pwm_dir = 0;
            }
            else
            {
                if(pwm_duty > 0) pwm_duty--;
                if(pwm_duty == 0)
                {
                    pwm_dir = 1;
                    pwm_cycles_done++;
#if PWM_BREATH_CYCLES > 0
                    if(pwm_cycles_done >= PWM_BREATH_CYCLES)
                    {
                        /* 指定呼吸次数已完成：灯灭，继电器断开。 */
                        locked = 0;
                        E0_dat |= 0x20;
                        ISRWriteP0(7, E0_dat);
                        LED_dat = 0xff;
                        ISRWriteP0(4, LED_dat);
                    }
#endif
                }
            }
        }
        if(locked)
        {
            pwm_cnt++;
            if(pwm_cnt >= PWM_PERIOD_TICKS) pwm_cnt = 0;
            ISRWriteP0(4, (pwm_cnt < pwm_duty) ? 0x00 : 0xff);
        }
        P2 = p2bak;
    }
}

/* 第4题：双电机方向控制。1前进、-1后退、0停止。 */
void MotorRun(signed char dir)
{
    if(dir == 1)       P3 = (P3 & 0x0f) | 0x50;
    else if(dir == -1) P3 = (P3 & 0x0f) | 0xa0;
    else               P3 = (P3 & 0x0f) | 0x00;
}

/* ONL：左轮正转，右轮停止 */
void MotorLeftRun(void)
{
    P3 = (P3 & 0x0f) | 0x10;
}

/* ONR：右轮正转，左轮停止 */
void MotorRightRun(void)
{
    P3 = (P3 & 0x0f) | 0x40;
}
void CarTurnLeft(void)
{
    MotorRun(0);
    Delay(100);
    P3 = (P3 & 0x0F) | 0x80;
    TurnLeft();
    MotorRun(0);
    Delay(100);
    MotorRun(gear);
}

void CarTurnRight(void)
{
    MotorRun(0);
    Delay(100);
    P3 = (P3 & 0x0F) | 0x20;
    TurnRight();
    MotorRun(0);
    Delay(100);
    MotorRun(gear);
}

/* 第4题：挡位显示：01前进、00停车、-1后退。 */
void ShowGear(void)
{
    if(gear == 1)      { disp_buf[0] = SEG[0];  disp_buf[1] = SEG[1];  }
    else if(gear == -1){ disp_buf[0] = SEG[10]; disp_buf[1] = SEG[1];  }
    else               { disp_buf[0] = SEG[0];  disp_buf[1] = SEG[0];  }
}

/* 第4题：限幅到-1~1，防止连续按键出现超出题目范围的挡位。 */
void GearSet(signed char g)
{
    if(g >  1) g =  1;
    if(g < -1) g = -1;
    gear = g;
    MotorRun(gear);
    if(page == 0 && radar_on == 0)
        ShowGear();
}

/* 第7题：DHT11读取温湿度。此处沿用已测版本的40位脉宽分类算法。 */
unsigned char DHT11_Read(void)
{
    unsigned char i, j, idx, fail, cmin, cmax, mid;
    unsigned char buf[5];
    unsigned int  to;


    DHT = 1;
    to = 0;
    while(DHT == 0)
    {
        if(++to > 30000) { Report('L', 0); return 0; }
    }


    DHT = 0;
    Delay(2);
    {
        unsigned int t0 = TickNow();
        while((unsigned int)(TickNow() - t0) < 18);
    }
    DHT = 1;


    EA = 0;
    fail = 0;

    to = 0;
    while(DHT) { if(++to > 20000) { fail = 1; break; } }

    if(fail == 0) { to = 0; while(!DHT) { if(++to > 20000) { fail = 2; break; } } }
    if(fail == 0) { to = 0; while(DHT)  { if(++to > 20000) { fail = 3; break; } } }

    idx = 0;
    while(idx < 40 && fail == 0)
    {
        to = 0;
        while(!DHT) { if(++to > 20000) { fail = 4; break; } }
        if(fail) break;
        to = 0;
        while(DHT)  { if(++to > 20000) { fail = 5; break; } }
        if(fail) break;
        cntbuf[idx] = (to > 255) ? 255 : (unsigned char)to;
        idx++;
    }

    EA = 1;

    if(fail)
    {
        Report((unsigned char)('A' + fail - 1), idx);
        return 0;
    }


    cmin = 255; cmax = 0;
    for(i = 0; i < 40; i++)
    {
        if(cntbuf[i] < cmin) cmin = cntbuf[i];
        if(cntbuf[i] > cmax) cmax = cntbuf[i];
    }
    if((unsigned char)(cmax - cmin) < 8) { Report('W', 0); return 0; }
    mid = (unsigned char)(((unsigned int)cmin + cmax) >> 1);

    for(j = 0; j < 5; j++)
    {
        buf[j] = 0;
        for(i = 0; i < 8; i++)
        {
            buf[j] <<= 1;
            if(cntbuf[j * 8 + i] > mid) buf[j] |= 0x01;
        }
    }


    if((unsigned char)(buf[0] + buf[1] + buf[2] + buf[3]) != buf[4])
    { Report('F', 0); return 0; }
    if(buf[0] == 0 && buf[2] == 0) { Report('Z', 0); return 0; }

    hum_h  = buf[0];
    hum_l  = buf[1];
    temp_h = buf[2];
    temp_l = buf[3];
    temp_dec2 = (temp_l <= 9) ? (unsigned char)(temp_l * 10)
                              : (unsigned char)(temp_l % 100);
    temp_ok = 1;
    Report('Y', 0);
    return 1;
}

/* DS18B20：1-Wire复位与存在检测。 */
/* 仅修复P1.7上的DS18B20。温度只用独立PCA基准计数器，退出前停止；
 * Timer2完全保留给超声波，PCA不输出PWM、不启用中断。11.0592MHz/12。
 */
#define OW_US_TICKS(us) ((unsigned int)(((unsigned long)(us)*9216UL+5000UL)/10000UL))
static unsigned char xdata ow_frame[9];
static unsigned char xdata ow_pending=0;
static unsigned char xdata temp_error=0;
static unsigned char xdata temp_negative=0;
static void OW_TimerStart(void)
{
    OW_PCA_CCON=0;OW_PCA_CMOD=0;
    OW_PCA_L=0;OW_PCA_H=0;OW_PCA_CCON=0x40;
}
static unsigned int OW_Ticks(void)
{
    unsigned char h,l;
    do{h=OW_PCA_H;l=OW_PCA_L;}while(h!=OW_PCA_H);
    return ((unsigned int)h<<8)|l;
}
static void OW_Wait(unsigned int ticks)
{
    unsigned int at=OW_Ticks();
    while((unsigned int)(OW_Ticks()-at)<ticks);
}
unsigned char OW_Reset(void)
{
    unsigned char old_ea,at,ok;
    OW_TimerStart();DHT=1;
    if(!DHT){OW_PCA_CCON&=0xBF;return 0;}
    DHT=0;OW_Wait(OW_US_TICKS(500));
    old_ea=EA;EA=0;DHT=1;at=OW_PCA_L;
    while((unsigned char)(OW_PCA_L-at)<(unsigned char)OW_US_TICKS(70));
    ok=(unsigned char)!DHT;EA=old_ea;
    OW_Wait(OW_US_TICKS(410));
    return ok;
}
void OW_WriteBit(unsigned char b)
{
    unsigned char old_ea,at,low;
    low=b?(unsigned char)OW_US_TICKS(6):(unsigned char)OW_US_TICKS(60);
    old_ea=EA;EA=0;DHT=0;at=OW_PCA_L;
    while((unsigned char)(OW_PCA_L-at)<low);
    DHT=1;EA=old_ea;OW_Wait(OW_US_TICKS(65));
}
void OW_WriteByte(unsigned char dat)
{
    unsigned char i;
    for(i=0;i<8;i++){OW_WriteBit(dat&1);dat>>=1;}
}
unsigned char OW_ReadBit(void)
{
    unsigned char old_ea,at,b;
    old_ea=EA;EA=0;DHT=0;at=OW_PCA_L;
    while((unsigned char)(OW_PCA_L-at)<(unsigned char)OW_US_TICKS(3));
    DHT=1;
    while((unsigned char)(OW_PCA_L-at)<(unsigned char)OW_US_TICKS(12));
    b=DHT;EA=old_ea;OW_Wait(OW_US_TICKS(65));
    return b;
}
unsigned char OW_ReadByte(void)
{
    unsigned char i,dat=0;
    for(i=0;i<8;i++){dat>>=1;if(OW_ReadBit())dat|=0x80;}
    return dat;
}
unsigned char OW_Present(void)
{
    unsigned char ok=OW_Reset();OW_PCA_CCON&=0xBF;return ok;
}
void OW_Convert(void)
{
    ow_pending=0;
    if(OW_Reset()) {
        OW_WriteByte(0xCC);OW_WriteByte(0x44);ow_pending=1;
    } else {temp_ok=0;temp_error=1;}
    OW_PCA_CCON&=0xBF;
}
static unsigned char OW_CRC(void)
{
    unsigned char i,j,v,mix,crc=0;
    for(i=0;i<8;i++) {
        v=ow_frame[i];
        for(j=0;j<8;j++) {
            mix=(crc^v)&1;crc>>=1;if(mix)crc^=0x8C;v>>=1;
        }
    }
    return crc;
}
static unsigned char OW_Decode(void)
{
    unsigned char i,any=0,resolution,negative;
    unsigned int raw,mag;
    for(i=0;i<9;i++)any|=ow_frame[i];
    if(!any||OW_CRC()!=ow_frame[8]){temp_ok=0;temp_error=2;return 0;}
    raw=((unsigned int)ow_frame[1]<<8)|ow_frame[0];
    resolution=ow_frame[4]&0x60;
    if(resolution==0)raw&=0xFFF8;
    else if(resolution==0x20)raw&=0xFFFC;
    else if(resolution==0x40)raw&=0xFFFE;
    negative=(raw&0x8000)!=0;
    mag=negative?(unsigned int)(~raw+1U):raw;
    if((negative&&mag>880U)||(!negative&&mag>2000U)) {
        temp_ok=0;temp_error=3;return 0;
    }
    temp_negative=negative;temp_h=(unsigned char)(mag>>4);
    temp_dec2=(unsigned char)(((mag&15U)*625U)/100U);
    temp_ok=1;temp_error=0;return 1;
}
void OW_ReadTemp(void)
{
    unsigned char i;
    if(!ow_pending){OW_Convert();return;}
    if(!OW_Reset()) {
        ow_pending=0;temp_ok=0;temp_error=1;OW_PCA_CCON&=0xBF;return;
    }
    OW_WriteByte(0xCC);OW_WriteByte(0xBE);
    for(i=0;i<9;i++)ow_frame[i]=OW_ReadByte();
    OW_PCA_CCON&=0xBF;OW_Decode();OW_Convert();
}
/* 温度只显示合法数字：正常xx.xxC；>=100为xxx.xC，负温为-xx.xC。
 * E1无应答，E2帧校验失败，E3范围/数据异常；不再把异常值当作段码下标。
 */
void ShowTemp(void)
{
    unsigned char xdata b[5];
    unsigned char i,old_ea;
    if(temp_ok&&(temp_h>125||temp_dec2>99||(temp_negative&&temp_h>55))) {
        temp_ok=0;temp_error=3;
    }
    b[4]=SEG[13];
    if(!temp_ok) {
        b[0]=temp_error?0x86:SEG[10];
        b[1]=temp_error?SEG[temp_error<=3?temp_error:3]:SEG[10];
        b[2]=SEG[10];b[3]=SEG[10];
    } else if(temp_negative) {
        b[0]=SEG[10];b[1]=SEG[temp_h/10];b[2]=SEG[temp_h%10]&0x7F;b[3]=SEG[temp_dec2/10];
    } else if(temp_h>=100) {
        b[0]=SEG[temp_h/100];b[1]=SEG[(temp_h/10)%10];b[2]=SEG[temp_h%10]&0x7F;b[3]=SEG[temp_dec2/10];
    } else {
        b[0]=SEG[temp_h/10];b[1]=SEG[temp_h%10]&0x7F;b[2]=SEG[temp_dec2/10];b[3]=SEG[temp_dec2%10];
    }
    old_ea=EA;EA=0;
    for(i=0;i<5;i++)disp_buf[i+3]=b[i];
    EA=old_ea;
}


/* DS1302软件串行时钟短延时。 */
void DS_Delay(void)
{
    unsigned char i;
    for(i=0; i<3; i++);
}

/* DS1302：向时钟芯片写一个字节。 */
void DS_WriteByte(unsigned char dat)
{
    unsigned char i;
    for(i=0; i<8; i++)
    {
        DS_SCK = 0;
        DS_SDA = dat & 0x01;
        DS_Delay();
        DS_SCK = 1;
        DS_Delay();
        dat >>= 1;
    }
    DS_SCK = 0;
}

/* DS1302：从时钟芯片读一个字节。 */
unsigned char DS_ReadByte(void)
{
    unsigned char i, dat = 0;
    DS_SDA = 1;
    for(i=0; i<8; i++)
    {
        DS_SCK = 0;
        DS_Delay();
        if(DS_SDA) dat |= (1 << i);
        DS_SCK = 1;
        DS_Delay();
    }
    DS_SCK = 0;
    return dat;
}

/* DS1302：按寄存器地址写数据。 */
void DS_Write(unsigned char addr, unsigned char dat)
{
    DS_RST = 0;
    DS_SCK = 0;
    DS_RST = 1;
    DS_Delay();
    DS_WriteByte(addr);
    DS_WriteByte(dat);
    DS_SCK = 0;
    DS_RST = 0;
}

/* DS1302：按寄存器地址读数据。 */
unsigned char DS_Read(unsigned char addr)
{
    unsigned char dat;
    DS_RST = 0;
    DS_SCK = 0;
    DS_RST = 1;
    DS_Delay();
    DS_WriteByte(addr | 0x01);
    dat = DS_ReadByte();
    DS_SCK = 0;
    DS_RST = 0;
    return dat;
}

/* 第6题：每次冷启动设置初始时分秒；BCD值见顶部参数区。 */
void DS1302_Init(void)
{
    unsigned char i, ok = 0;

    DS_RST = 0; DS_SCK = 0; DS_SDA = 1;

    for(i = 0; i < 3 && ok == 0; i++)
    {
        DS_Write(0x8e, 0x00);
        DS_Write(0x80, CLOCK_SEC_BCD);
        DS_Write(0x82, CLOCK_MIN_BCD);
        DS_Write(0x84, CLOCK_HOUR_BCD);
        DS_Write(0x8e, 0x80);

        if(((DS_Read(0x85) & 0x3f) == CLOCK_HOUR_BCD) &&
           ((DS_Read(0x83) & 0x7f) == CLOCK_MIN_BCD))
            ok = 1;
    }
}

/* 第6题：时钟格式12-00-00；数码管1~8位。 */
void ShowClock(void)
{
    unsigned char h, m, s;

    EA = 0;
    h = DS_Read(0x85);
    m = DS_Read(0x83);
    s = DS_Read(0x81);
    EA = 1;

    h &= 0x3f;  m &= 0x7f;  s &= 0x7f;

    disp_buf[0] = SEG[(h >> 4) & 0x0f];
    disp_buf[1] = SEG[h & 0x0f];
    disp_buf[2] = SEG[10];
    disp_buf[3] = SEG[(m >> 4) & 0x0f];
    disp_buf[4] = SEG[m & 0x0f];
    disp_buf[5] = SEG[10];
    disp_buf[6] = SEG[(s >> 4) & 0x0f];
    disp_buf[7] = SEG[s & 0x0f];
}

/* 显示页：page=0挡位+温度，page=1时钟；雷达遮挡时临时覆盖。 */
void ShowPage(void)
{
    if(page == 1) ShowClock();
    else
    {
        disp_buf[2] = 0xff;
        ShowGear();
        ShowTemp();
    }
}

/* 第5题：Timer2实测ECHO高电平脉宽，并换算厘米。 */
unsigned int UltraRead(void)
{
    unsigned int ticks, dist;

    AUXR &= 0xE3;
    T2L = 0x00;
    T2H = 0x00;

    TRIG = 0;
    Dly(5);
    TRIG = 1;
    Dly(15);
    TRIG = 0;

    AUXR |= 0x10;
    while(!ECHO)
    {
        if(T2H >= 0x75)
        {
            AUXR &= 0xEF;
            return 0xffff;
        }
    }

    AUXR &= 0xEF;
    T2L = 0x00;
    T2H = 0x00;
    AUXR |= 0x10;
    while(ECHO)
    {
        if(T2H >= 0x75)
        {
            AUXR &= 0xEF;
            return 0xffff;
        }
    }

    AUXR &= 0xEF;
    ticks = ((unsigned int)T2H << 8) | T2L;
    dist = (unsigned int)((ticks + ULTRA_TICKS_PER_CM / 2) / ULTRA_TICKS_PER_CM);
    if(dist > 300) return 0xffff;
    return dist;
}

/* 第5题：距离显示格式d    xxx。 */
void ShowRadar(unsigned int d)
{
    disp_buf[0] = SEG[12];
    disp_buf[1] = 0xff;
    disp_buf[2] = 0xff;
    disp_buf[3] = 0xff;
    disp_buf[4] = 0xff;
    disp_buf[5] = (d >= 100) ? SEG[(d/100)%10] : 0xff;
    disp_buf[6] = SEG[(d/10)%10];
    disp_buf[7] = SEG[d%10];
}

/*
 * 第5题：小于RADAR_ALERT_CM时显示距离并报警；
 * 小于RADAR_ALWAYS_CM时持续响；中间范围越近鸣叫越密集。
 */
void RadarTask(void)
{
    static unsigned char beat = 0;
    unsigned int d;
    unsigned char div;

    d = UltraRead();
    api_distance_cm=d;api_distance_at=TickNow();
    if(d < RADAR_ALERT_CM)
    {
        radar_on = 1;
        ShowRadar(d);
        if(d < RADAR_ALWAYS_CM)
        {
            BuzzerRequest(BEEP_RADAR, 1);
        }
        else
        {
            if     (d < 6)  div = 1;
            else if(d < 8)  div = 2;
            else if(d < 10) div = 3;
            else if(d < 12) div = 4;
            else            div = 5;
            if(beat == 0)
            {
                BuzzerRequest(BEEP_RADAR, 1);
                Delay(RADAR_BEEP_MS);
                BuzzerRequest(BEEP_RADAR, 0);
            }
            else BuzzerRequest(BEEP_RADAR, 0);
            beat++;
            if(beat >= div) beat = 0;
        }
    }
    else
    {
        BuzzerRequest(BEEP_RADAR, 0);
        beat = 0;
        if(radar_on)
        {
            radar_on = 0;
            ShowPage();
        }
    }
}

/*
 * 第8题：SW14开关锁车呼吸灯和继电器。
 * PWM_BREATH_CYCLES=0时再次按SW14关闭；设正数则呼吸N次自动关闭。
 */
void LockToggle(void)
{
    unsigned char old_ea;
    old_ea = EA;
    EA = 0;
    if(locked)
    {
        locked = 0;
        pwm_duty = 0;
        RelayOff();
        LedAll(0);
    }
    else
    {
        pwm_duty = 0;
        pwm_dir  = 1;
        pwm_step = 0;
        pwm_cnt  = 0;
        pwm_cycles_done = 0;
        RelayOn();
        locked = 1;
    }
    EA = old_ea;
}

/* 自由发挥：蓝牙HC-05串口9600bps，下载程序时要拔掉蓝牙模块。 */
void Uart_Init(void)
{
    TMOD = (TMOD & 0x0F) | 0x20;
    TH1  = 0xFD;
    TL1  = 0xFD;
    PCON &= 0x7F;
    SCON  = 0x50;
    ES    = 1;
    TR1   = 1;
}

/* 串口中断：接收手机发送的单字母指令。 */
//void Uart_ISR(void) interrupt 4
//{
//    if(RI)
//    {
//        RI = 0;
//        bt_cmd = SBUF;
//    }
//    if(TI) TI = 0;
//}
/* 串口中断：状态机识别 "ONA" / "ONB" 指令 */
static void uart_capture(void)
{
    if(RI) {
        rx_value=SBUF; RI=0;
        rx_next=(rx_head+1)&RX_MASK;
        if(rx_next!=rx_tail) {
            rxbuf[rx_head]=rx_value;
            rx_head=rx_next;
        } else {
            rx_overflow=1;
            if(rx_drops<255) rx_drops++;
        }
    }
}

void Uart_ISR(void) interrupt 4
{
    uart_capture();
    if(TI) {
        TI=0;
        if(tx_tail!=tx_head) {
            SBUF=txbuf[tx_tail];
            tx_tail=(tx_tail+1)&TX_MASK;
        } else tx_busy=0;
    }
}

/* 自由发挥：F前进、B后退、S停止、L/R转向灯、X锁车切换。 */
void BtHandle(unsigned char c)
{
    switch(c)
    {
        case 'F': case 'f': GearSet(1);   break;
        case 'B': case 'b': GearSet(-1);  break;
        case 'S': case 's': GearSet(0);   break;
        case 'L': case 'l': TurnLeft();   break;
        case 'R': case 'r': TurnRight();  break;
        case 'X': case 'x': LockToggle(); break;
        default: break;
    }
}

static u16 uart_free(void)
{
    u16 h,t;
    EA=0; h=tx_head; t=tx_tail; EA=1;
    return (t-h-1)&TX_MASK;
}

static void reply_char(u8 c)
{
    if(reply_len<REPLY_MAX-2) reply_buf[reply_len++]=c;
    else reply_overflow=1;
}

static void reply_text(char code *p)
{
    while(*p) reply_char(*p++);
}

static void reply_num(u16 n)
{
    static const u16 code powers[5]={10000,1000,100,10,1};
    u8 i,d,seen=0;
    for(i=0;i<5;i++) {
        d=n/powers[i]; n%=powers[i];
        if(d || seen || i==4) { reply_char('0'+d); seen=1; }
    }
}

static void reply_two(u8 n)
{
    reply_char('0'+n/10); reply_char('0'+n%10);
}

static void reply_begin(char code *prefix)
{
    reply_len=0; reply_overflow=0; reply_text(prefix);
}

static void reply_send(void)
{
    u16 h,i;
    if(reply_overflow) {
        reply_begin("ERR RESPONSE_TOO_LONG");
    }
    reply_buf[reply_len++]='\r'; reply_buf[reply_len++]='\n';
    if(uart_free()<reply_len) { if(tx_drops<255) tx_drops++; return; }
    EA=0; h=tx_head; EA=1;
    /* Publish the head only after all bytes are copied. */
    for(i=0;i<reply_len;i++) {
        txbuf[h]=reply_buf[i]; h=(h+1)&TX_MASK;
    }
    EA=0;
    tx_head=h;
    if(!tx_busy && tx_tail!=tx_head) {
        tx_busy=1; TI=0;
        SBUF=txbuf[tx_tail]; tx_tail=(tx_tail+1)&TX_MASK;
    }
    EA=1;
}

static void reply_error(char code *why)
{
    reply_begin("ERR "); reply_text(why); reply_send();
}

static void reply_ok(char code *what)
{
    reply_begin("OK "); reply_text(what); reply_send();
}

static char xdata *arg(u8 index)
{
    if(index>=arg_count) return &linebuf[63];
    return &linebuf[arg_start[index]];
}

static u8 eq(char xdata *p,char code *q)
{
    while(*p && *q && *p==*q) { p++; q++; }
    return *p==0 && *q==0;
}

static u8 number(char xdata *p,u16 max)
{
    u16 v=0;
    u8 d;
    if(!*p) return 0;
    while(*p) {
        if(*p<'0' || *p>'9') return 0;
        d=*p++-'0';
        if(v>max/10 || (v==max/10 && d>max%10)) return 0;
        v=v*10+d;
    }
    parsed_value=v; return 1;
}

static u8 tokenize(void)
{
    u8 i=0;
    arg_count=0; linebuf[63]=0;
    while(linebuf[i]) {
        while(linebuf[i]==' ' || linebuf[i]=='\t') linebuf[i++]=0;
        if(!linebuf[i]) break;
        if(arg_count==4) return 0;
        arg_start[arg_count++]=i;
        while(linebuf[i] && linebuf[i]!=' ' && linebuf[i]!='\t') i++;
    }
    return 1;
}


/* 新增蓝牙接口；只调用原程序函数，不替换原驱动。
 * 9600 8N1。普通命令以LF结束；原ONA/ONB/ONL/ONR及新增ONS不用换行。
 * 无01/00切换，无自检、无新的运行模式。动作仍按原程序执行。
 */
static u8 xdata api_turn_side,api_turn_on;
static u16 xdata api_turn_at,api_report_ms,api_report_at;
static u8 xdata api_h,api_m,api_s,api_rtc_ok;

static u8 api_bcd_value(u8 value,u8 max)
{
    return (value&15)<=9 && (value>>4)<=9 && ((value>>4)*10+(value&15))<=max;
}
static void api_clock_read(void)
{
    api_h=DS_Read(0x85)&0x3F;api_m=DS_Read(0x83)&0x7F;api_s=DS_Read(0x81)&0x7F;
    api_rtc_ok=api_bcd_value(api_h,23)&&api_bcd_value(api_m,59)&&api_bcd_value(api_s,59);
}
static void api_time_value(void)
{
    if(!api_rtc_ok) {reply_text("NA");return;}
    reply_two((api_h>>4)*10+(api_h&15));reply_char(':');
    reply_two((api_m>>4)*10+(api_m&15));reply_char(':');
    reply_two((api_s>>4)*10+(api_s&15));
}
static void api_temp_value(void)
{
    if(!temp_ok){reply_text("NA");return;}
    if(temp_negative)reply_char('-');
    reply_num(temp_h);reply_char('.');reply_two(temp_dec2);
}
static void api_stop(void)
{
    api_turn_side=0;GearSet(0);BuzzerRequest(BEEP_HORN,0);LedAll(0);
}
/* 蓝牙只读雷达任务的缓存，不额外发射超声波，避免密集触发串扰。 */
static u16 api_distance_cached(void)
{
    if((u16)(TickNow()-api_distance_at)>1000U)return 65535U;
    return api_distance_cm;
}

static void api_status(u8 event)
{
    u16 d=api_distance_cached();
    api_clock_read();
    reply_begin(event?"EVT STATUS LOCK=":"OK STATUS LOCK=");reply_num(locked);
    reply_text(" GEAR=");if(gear<0)reply_text("-1");else reply_num(gear);
    reply_text(" DRIVE=");
    if((P3&0xF0)==0x50)reply_char('F');else if((P3&0xF0)==0xA0)reply_char('B');
    else if((P3&0xF0)==0x10)reply_char('L');else if((P3&0xF0)==0x40)reply_char('R');else reply_char('S');
    reply_text(" TURN=");reply_num(api_turn_side);
    reply_text(" HORN=");reply_num((beep_flags&BEEP_HORN)!=0);
    reply_text(" BUZZ=");reply_num(beep_flags!=0);
    reply_text(" RELAY=");reply_num((E0_dat&0x20)==0);
    reply_text(" BREATH=");reply_num(locked);
    reply_text(" RADAR=");reply_num(api_radar_enabled);
    reply_text(" PAGE=");reply_text(page?"TIME":"TEMP");
    reply_text(" VIEW=");reply_text(radar_on?"RADAR":(page?"TIME":"TEMP"));
    reply_text(" LED=");reply_num((u8)~LED_dat);
    reply_text(" TEMP=");api_temp_value();reply_text(" RH=");
    if(temp_ok&&temp_type==2)reply_num(hum_h);else reply_text("NA");
    reply_text(" DIST_MM=");if(d!=65535U)reply_num(d*10U);else reply_text("NA");
    reply_text(" RTC=");reply_num(api_rtc_ok);reply_text(" TIME=");api_time_value();
    reply_text(" RXDROP=");reply_num(rx_drops);reply_text(" TXDROP=");reply_num(tx_drops);reply_send();
}
static void api_move(char dir)
{
    if(dir=='F')GearSet(1);
    else if(dir=='B') {
        if(gear==1){GearSet(0);Delay(200);}
        GearSet(-1);
    } else if(dir=='S')GearSet(0);
    else if(dir=='L'||dir=='R') {
        if(dir=='L')MotorLeftRun();else MotorRightRun();
        gear=0;if(page==0&&radar_on==0)ShowGear();
    } else {reply_error("ARGS");return;}
    reply_begin("OK MOVE ");reply_char(dir);reply_send();
}
static void api_turn(u8 side,u16 count)
{
    u16 n,on_ms,off_ms;
    api_turn_side=0;led_busy=1;
    on_ms=(side==2)?RIGHT_ON_MS:LEFT_ON_MS;off_ms=(side==2)?RIGHT_OFF_MS:LEFT_OFF_MS;
    for(n=0;n<count;n++) {
        if(side&1)LedLeft(1);if(side&2)LedRight(1);Delay(on_ms);
        if(side&1)LedLeft(0);if(side&2)LedRight(0);Delay(off_ms);
    }
    led_busy=0;
}
static void api_process(void)
{
    u8 side,on,i;
    u16 count;
    char xdata *p;
    if(!tokenize()){reply_error("ARGS");return;}
    if(!arg_count)return;
    if(eq(arg(0),"MOVE")) {
        p=arg(1);if(arg_count!=2||!p[0]||p[1]){reply_error("ARGS");return;}
        api_move(p[0]);return;
    }
    if(arg_count==1&&(eq(arg(0),"F")||eq(arg(0),"B")||eq(arg(0),"L")||eq(arg(0),"R")||eq(arg(0),"S"))) {api_move(*arg(0));return;}
    if(eq(arg(0),"START")||eq(arg(0),"A")) {
        if(arg_count!=1){reply_error("ARGS");return;}CarStart();reply_ok("START");return;
    }
    if(eq(arg(0),"STOP")) {if(arg_count!=1){reply_error("ARGS");return;}api_stop();reply_ok("STOP");return;}
    if(eq(arg(0),"PING")||eq(arg(0),"KEEP")) {
        if(arg_count!=1){reply_error("ARGS");return;}
        reply_ok(eq(arg(0),"PING")?"PONG":"KEEP");return;
    }
    if(eq(arg(0),"INFO")) {if(arg_count!=1){reply_error("ARGS");return;}reply_ok("INFO API=1 MCU=IAP15F2K61S2 BAUD=9600 TIMEOUT_MS=0");return;}
    if(eq(arg(0),"HELP")) {reply_ok("HELP START STOP MOVE GEAR TURN HORN LOCK BREATH RELAY PAGE TIME RADAR LED GET REPORT KEEP PING INFO ONA ONB ONL ONR ONS");return;}
    if(eq(arg(0),"GEAR")) {
        if(arg_count!=2){reply_error("ARGS");return;}
        if(eq(arg(1),"UP"))GearSet(gear+1);else if(eq(arg(1),"DOWN"))GearSet(gear-1);
        else if(eq(arg(1),"-1"))GearSet(-1);else if(eq(arg(1),"0"))GearSet(0);
        else if(eq(arg(1),"1"))GearSet(1);else {reply_error("ARGS");return;}
        reply_begin("OK GEAR ");if(gear<0)reply_text("-1");else reply_num(gear);reply_send();return;
    }
    if(eq(arg(0),"HORN")||eq(arg(0),"LOCK")||eq(arg(0),"BREATH")||eq(arg(0),"RELAY")||eq(arg(0),"RADAR")) {
        if(arg_count!=2||!number(arg(1),1)){reply_error("ARGS");return;}
        on=(u8)parsed_value;
        if(eq(arg(0),"HORN")){BuzzerRequest(BEEP_HORN,on);reply_begin("OK HORN ");}
        else if(eq(arg(0),"LOCK")||eq(arg(0),"BREATH")) {
            if(locked!=on)LockToggle(); /* 原程序呼吸灯/继电器联动原样保留 */
            reply_begin(eq(arg(0),"LOCK")?"OK LOCK ":"OK BREATH ");
        } else if(eq(arg(0),"RELAY")){if(on)RelayOn();else RelayOff();reply_begin("OK RELAY ");}
        else {
            if(!on){reply_error("RADAR_ALWAYS_ON");return;}
            api_radar_enabled=1;reply_begin("OK RADAR ");
        }
        reply_num(on);reply_send();return;
    }
    if(eq(arg(0),"TURN")) {
        if(arg_count<2||arg_count>3){reply_error("ARGS");return;}
        if(eq(arg(1),"OFF")&&arg_count==2){api_turn_side=0;LedLeft(0);LedRight(0);reply_ok("TURN OFF");return;}
        side=eq(arg(1),"L")?1:(eq(arg(1),"R")?2:(eq(arg(1),"BOTH")?3:0));
        if(!side){reply_error("ARGS");return;}
        if(arg_count==3&&eq(arg(2),"ON")){
            api_turn_side=side;api_turn_on=0;api_turn_at=TickNow();reply_begin("OK TURN ");reply_num(side);reply_text(" ON");reply_send();return;
        }
        count=(side==2)?RIGHT_COUNT:LEFT_COUNT;
        if(arg_count==3){if(!number(arg(2),1000)||!parsed_value){reply_error("ARGS");return;}count=parsed_value;}
        api_turn(side,count);reply_begin("OK TURN ");reply_num(side);reply_char(' ');reply_num(count);reply_send();return;
    }
    if(eq(arg(0),"PAGE")) {
        if(arg_count!=2){reply_error("ARGS");return;}
        if(eq(arg(1),"TIME"))page=1;
        else if(eq(arg(1),"NORMAL")||eq(arg(1),"TEMP"))page=0;
        else {reply_error("UNSUPPORTED");return;} /* 原程序无独立湿度显示页 */
        if(!radar_on)ShowPage();reply_ok(page?"PAGE TIME":"PAGE TEMP");return;
    }
    if(eq(arg(0),"TIME")) {
        if(arg_count!=2){reply_error("ARGS");return;}p=arg(1);
        for(i=0;i<8;i++) {
            if(!p[i]){reply_error("ARGS");return;}
            if(i==2||i==5){if(p[i]!=':'){reply_error("ARGS");return;}}
            else if(p[i]<'0'||p[i]>'9'){reply_error("ARGS");return;}
        }
        if(p[8]||(p[0]-'0')*10+p[1]-'0'>23||(p[3]-'0')*10+p[4]-'0'>59||(p[6]-'0')*10+p[7]-'0'>59){reply_error("ARGS");return;}
        api_h=((p[0]-'0')<<4)|(p[1]-'0');api_m=((p[3]-'0')<<4)|(p[4]-'0');api_s=((p[6]-'0')<<4)|(p[7]-'0');
        DS_Write(0x8E,0);DS_Write(0x84,api_h);DS_Write(0x82,api_m);DS_Write(0x80,api_s);DS_Write(0x8E,0x80);
        api_clock_read();if(!api_rtc_ok){reply_error("RTC");return;}
        if(page&&!radar_on)ShowClock();reply_begin("OK TIME ");api_time_value();reply_send();return;
    }
    if(eq(arg(0),"LED")) {
        if(arg_count!=2){reply_error("ARGS");return;}
        if(eq(arg(1),"AUTO")){LedAll(0);reply_ok("LED AUTO");return;}
        if(!number(arg(1),255)){reply_error("ARGS");return;}
        api_turn_side=0;LED_dat=(u8)~parsed_value;OutPutP0(4,LED_dat);reply_begin("OK LED ");reply_num(parsed_value);reply_send();return;
    }
    if(eq(arg(0),"REPORT")) {
        if(arg_count!=2||!number(arg(1),5000)||(parsed_value&&parsed_value<500)){reply_error("ARGS");return;}
        api_report_ms=parsed_value;api_report_at=TickNow();reply_begin("OK REPORT ");reply_num(api_report_ms);reply_send();return;
    }
    if(eq(arg(0),"GET")) {
        if(arg_count==1||(arg_count==2&&eq(arg(1),"STATUS"))){api_status(0);return;}
        if(arg_count!=2){reply_error("ARGS");return;}
        if(eq(arg(1),"TEMP")) {
            reply_begin("OK TEMP VALID=");reply_num(temp_ok);reply_text(" TEMP=");api_temp_value();reply_text(" RH=");
            if(temp_ok&&temp_type==2)reply_num(hum_h);else reply_text("NA");reply_send();return;
        }
        if(eq(arg(1),"TIME")) {api_clock_read();reply_begin("OK TIME VALID=");reply_num(api_rtc_ok);reply_text(" TIME=");api_time_value();reply_send();return;}
        if(eq(arg(1),"DIST")) {
            count=api_distance_cached();reply_begin("OK DIST VALID=");reply_num(count!=65535U);reply_text(" MM=");
            if(count!=65535U)reply_num(count*10U);else reply_text("NA");reply_text(" ALARM=");reply_num(count<RADAR_ALERT_CM);reply_send();return;
        }
        reply_error("ARGS");return;
    }
    reply_error("COMMAND");
}
void BluetoothTask(void)
{
    u8 c,ev;
    u16 now=TickNow(),on_ms,off_ms;
    if(api_turn_side&&!led_busy) {
        on_ms=(api_turn_side==2)?RIGHT_ON_MS:LEFT_ON_MS;off_ms=(api_turn_side==2)?RIGHT_OFF_MS:LEFT_OFF_MS;
        if((u16)(now-api_turn_at)>=(api_turn_on?on_ms:off_ms)) {
            api_turn_at=now;api_turn_on=!api_turn_on;
            if(api_turn_side&1)LedLeft(api_turn_on);if(api_turn_side&2)LedRight(api_turn_on);
        }
    }
    if(uart_free()<REPLY_MAX)return;
    if(rx_overflow){EA=0;rx_tail=rx_head;rx_overflow=0;EA=1;line_len=0;line_discard=1;line_error=1;reply_error("RX_OVERFLOW");return;}
    while(rx_tail!=rx_head) {
        c=rxbuf[rx_tail];rx_tail=(rx_tail+1)&RX_MASK;
        if(c=='\r')continue;
        if(c=='\n') {
            if(line_discard){line_len=0;line_discard=0;if(line_error!=1)reply_error("LINE");return;}
            linebuf[line_len]=0;line_len=0;api_process();return;
        }
        if(line_discard)continue;
        if((c<32&&c!='\t')||c>126||line_len>=LINE_MAX){line_discard=1;line_error=2;continue;}
        if(c>='a'&&c<='z')c-=32;linebuf[line_len++]=c;
        /* 原始ONA/ONB/ONL/ONR直接交给原程序的bt_event分支执行。 */
        if(line_len==3&&linebuf[0]=='O'&&linebuf[1]=='N') {
            ev=(c=='A')?1:((c=='B')?2:((c=='L')?3:((c=='R')?4:((c=='S')?5:0))));
            if(ev){bt_event=ev;line_len=0;reply_ok("BT");return;}
        }
    }
    if(api_report_ms&&!tx_busy&&!line_len&&(u16)(now-api_report_at)>=api_report_ms){api_report_at=now;api_status(1);}
}

/* 主程序：先主动关闭所有输出，再依次调度温度、时钟、雷达、按键和蓝牙。 */
void main(void)
{
    unsigned char k, i, btc;

    /* 上电防御型初始化：LED灭、蜂鸣器停、继电器断、电机停。 */
    P0 = 0x00;
    P2 &= 0x1f;
    LedAll(0);
    BuzzerInit();
    RelayOff();
    MotorRun(0);

    for(i = 0; i < 8; i++) disp_buf[i] = 0xff;

    Timer0_Init();
    Uart_Init();
    GearSet(0);
    DS1302_Init();
    /* 本实物是DS18B20，直接走温度驱动，避免误识别为DHT11。 */
    temp_type=1;OW_Convert();ds_t0=TickNow();
    ShowPage();

    while(1)
    {

        /* 温度类型自动识别：0未知，1是DS18B20，2是DHT11。 */
        if(temp_type == 0)
        {
            if((unsigned int)(TickNow() - temp_t0) >= DHT11_READ_MS)
            {
                temp_t0 = TickNow();

                if(OW_Present())
                {
                    temp_type = 1;
                    ds_t0 = TickNow();
                    OW_Convert();
                    Report('S', 0);
                }
                else if(DHT11_Read())
                {
                    temp_type = 2;
                    if(radar_on == 0 && page == 0) ShowTemp();
                }
            }
        }
        else if(temp_type == 1)
        {
            if((unsigned int)(TickNow() - ds_t0) >= DS18B20_READ_MS)
            {
                ds_t0 = TickNow();
                OW_ReadTemp();
                if(radar_on == 0 && page == 0) ShowTemp();
            }
        }
        else
        {
            if((unsigned int)(TickNow() - temp_t0) >= DHT11_READ_MS)
            {
                temp_t0 = TickNow();
                if(DHT11_Read())
                {
                    if(radar_on == 0 && page == 0) ShowTemp();
                }
            }
        }


        if((unsigned int)(TickNow() - clk_t0) >= CLOCK_REFRESH_MS)
        {
            clk_t0 = TickNow();
            if(radar_on == 0 && page == 1)
                ShowClock();
        }


        if((unsigned int)(TickNow() - rad_t0) >= RADAR_SAMPLE_MS)
        {
            rad_t0 = TickNow();
            RadarTask();
        }


        /* 按键编号：SW1启动，SW2加挡，SW5左转，SW6减挡，SW9右转，
         * SW10时钟，SW13喇叭，SW14锁车。需要改按键映射可从这里找。
         */
        k = KeyScan();

        /* SW1：启动动画 */
        if(k == 1)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 1)
            {
                CarStart();
                while(KeyScan() == 1);
            }
        }

        /* SW2：加挡 */
        if(k == 2)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 2)
            {
                GearSet(gear + 1);
                while(KeyScan() == 2);
            }
        }

        /* SW5：左转向 */
        if(k == 5)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 5) CarTurnLeft();
        }

        /* SW6：减挡 */
        if(k == 6)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 6)
            {
                GearSet(gear - 1);
                while(KeyScan() == 6);
            }
        }

        /* SW9：右转向 */
        if(k == 9)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 9) CarTurnRight();
        }

        /* SW10：切换时钟和常规显示 */
        if(k == 10)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 10)
            {
                page = !page;
                ShowPage();
                while(KeyScan() == 10);
            }
        }

        /* SW13：按住喇叭 */
        if(k == 13)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 13)
            {
                /* 第3题：SW13按住喇叭常响，松开关闭该路申请。 */
                BuzzerRequest(BEEP_HORN, 1);
                while(KeyScan() == 13);
                BuzzerRequest(BEEP_HORN, 0);
            }
        }

        /* SW14：切换呼吸灯与继电器 */
        if(k == 14)
        {
            Delay(KEY_DEBOUNCE_MS);
            if(KeyScan() == 14)
            {
                LockToggle();
                while(KeyScan() == 14);
            }
        }


//        if(bt_cmd)
//        {
//            btc = bt_cmd;
//            bt_cmd = 0;
//            BtHandle(btc);
//        }
		
        BluetoothTask();

		if(bt_event)
{
    unsigned char ev;

    EA = 0;
    ev = bt_event;
    bt_event = 0;
    EA = 1;

    if(ev == 1)                 /* ONA：双轮正转 */
    {
        GearSet(1);
    }
    else if(ev == 2)            /* ONB：双轮反转 */
    {
        if(gear == 1)           /* 正转中，先停止死区 */
        {
            GearSet(0);
            Delay(200);
        }
        GearSet(-1);
    }
    else if(ev == 3)            /* ONL：左轮转，右轮停 */
    {
        MotorLeftRun();
        gear = 0;               /* 非对称状态，gear 记为 0 */
        if(page == 0 && radar_on == 0) ShowGear();
    }
    else if(ev == 4)            /* ONR：右轮转，左轮停 */
    {
        MotorRightRun();
        gear = 0;
        if(page == 0 && radar_on == 0) ShowGear();
    }
    else if(ev == 5) { GearSet(0); } /* 新增ONS停止 */
}

        Delay(LOOP_IDLE_MS);
    }
}