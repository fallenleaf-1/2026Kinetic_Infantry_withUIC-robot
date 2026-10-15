/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意：底盘采用左手系解算，z轴向下为正方向，车前进为y轴正方向，右移为x轴正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <string.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x))) // 限幅宏定义

#define ABS(x) (((x)>0)?(x):(-(x)))

#ifdef CHASSIS_RC_DIRECT
/* 单板直控: 遥控器右摇杆输出量纲±660(已在remote_control.c中减去1024偏置).
 * 25.0f = 满速(660*25 = 16500 = CHASSIS_WHEEL_SPEED_MAX);
 * 首次带电从8.0f(约1/3满速)起步,台架空载确认舵向/轮向后再逐级上调,场测不超过25.0f */
#define CHASSIS_RC_SPEED_SCALE 8.0f

/* 上电初始回正 / 静止回正: 把4个舵向电机驱动到标定的正前方(驱动轮始终零输出).
 * - 容差由5°收紧到2°: 原5°容差下角度环"刚进容差即结束",实测四轮停在3~4.5°偏差处,肉眼看就是没回正;
 *   容差不取更小值的原因: 末段收敛受静摩擦限制是"爬行"(实测1.3°→0.6°要好几秒),
 *   取1°会把交回遥控器的时刻推迟到5s以上;而交回后"静止即回正"(见CHASSIS_STEER_RETURN_ON_STOP /
 *   CHASSIS_STEER_HOLD_FORWARD)会继续把舵轮压到1°以内,所以视觉结果一致、响应更快.
 * - 需连续CHASSIS_ALIGN_SETTLE_CNT个周期都满足容差才算回正,避免过渡过程(如刚起步的过冲)被误判为到位;
 * - 超时上限兜底"某个舵轮卡住/舵机没反馈"的情况,避免角度环长期堵转带电.
 * 注意: 回正会驱动舵轮转动(最多180°),上电前请确认舵轮能自由转动或整车已架空. */
#define CHASSIS_ALIGN_TIMEOUT_MS 8000u   // 回正阶段最长持续时间(实测RB受阻力时从18.8°收到<5°需约4.5s,留足余量)
#define CHASSIS_ALIGN_TOLERANCE_DEG 2.0f // 单轮|目标-实际|小于该值才算该轮对准(交回遥控器的门槛)
#define CHASSIS_ALIGN_SETTLE_CNT 20u     // 连续满足容差的周期数(200Hz下=100ms)

/* 静止时回到正前方: 摇杆回中/停车后舵轮主动转回标定正前方,而不是保持上次行驶方向.
 * 关掉即恢复旧行为"停车保持上个方向"(注意: 旧行为下停车方向可能是"正前方+180°"的等价表示,
 * 车轮实际朝后,视觉上就不是正前方了) */
#define CHASSIS_STEER_RETURN_ON_STOP
/* 遥控器离线/急停(零力)时舵轮仍通电稳住正前方,只让4个驱动轮零输出.
 * 关掉即恢复旧行为"回正后舵机断电,轮子可自由推动(也会歪)" */
#define CHASSIS_STEER_HOLD_FORWARD
#endif

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
#ifndef CHASSIS_RC_DIRECT // 单板直控不初始化CANComm(无云台板,发0x311收不到ACK会占满CAN1邮箱)
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
#endif
attitude_t *Chassis_IMU_data;
#if defined(RC_RECEIVER_ON_CHASSIS) || defined(CHASSIS_RC_DIRECT) // 遥控器(DBUS)接在底盘板
#include "remote_control.h"
#include "bsp_log.h"
static RC_ctrl_t *rc_data; // RemoteControlInit返回的遥控器数据指针
#endif
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
#if defined(ONE_BOARD) || (defined(CHASSIS_BOARD) && !defined(CHASSIS_RC_DIRECT)) // 仅发布/发送该反馈的构建下声明,避免-Werror=unused-variable
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
#endif
static attitude_t *gimba_IMU_data; // 云台IMU数据

// 以下裁判系统/超级电容变量随阶段D.4(裁判系统接回)一起恢复,当前其全部使用点均为注释代码,先注释声明避免未使用告警
// static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
// static referee_info_t *referee_data;       // 用于获取裁判系统的数据
// static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

// static SuperCapInstance *cap; // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DJIMotorInstance *servo_lf, *servo_rf, *servo_lb, *servo_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘电机速度解算后的临时输出,待进行限幅
static float angle_lf, angle_rf, angle_lb, angle_rb;      // 底盘舵机角度解算后的临时输出,待进行限幅

// 底盘跟随PID
static PIDInstance Chassis_Follow_PID;

#ifdef CHASSIS_RC_DIRECT
/* 单板直控: 现场"示教 + 存Flash"标定
 * 背景: 舵轮的正前方标定值(GM6020_*_ANGLE)随装配变化,改一次就要重编译+烧录;
 *       这里提供现场标定: 遥控器左[下]+右[下]保持1.5s进示教模式(四舵轮断电可手摆),
 *       把轮子摆到视觉正前方后,右摇杆前推到底保持1s即把当前四角存进Flash;
 *       以后上电自动优先使用Flash里的值(robot_def.h里的值作为出厂默认).
 * 存储: 最后一个空扇区0x080E0000(固件只用到0x08019913,扇区11未被占用);
 *       擦除一个128KB扇区会让CPU停顿1~2s(STM32F4无RWW),本工程未开看门狗,可接受. */
#include "bsp_flash.h"

static float Normalize360(float a); // 前向声明: 定义在文件后半部分,标定保存时要用

#define STEER_CALIB_ADDR ADDR_FLASH_SECTOR_11 // 0x080E0000
#define STEER_CALIB_MAGIC 0x53544552u         // "RETS"的小端写法,用于识别有效记录
#define STEER_CALIB_VERSION 1u

typedef struct
{
    uint32_t magic;
    uint32_t version;
    float angle[4]; // 依次为 LF, RF, LB, RB (与steering_dirt/SERVO_BASE_DIR同序)
    uint32_t checksum;
} SteerCalib_s;

/* 当前生效的正前方标定: 上电先取robot_def.h默认值,若Flash有有效记录则被覆盖 */
static float steer_forward_angle[4] = {GM6020_LF_ANGLE, GM6020_RF_ANGLE, GM6020_LB_ANGLE, GM6020_RB_ANGLE};

/* 校验和: 把magic/version/angle四个浮点逐字异或加权累加,防止Flash空白/半写被当成有效记录 */
static uint32_t SteerCalibChecksum(const SteerCalib_s *c)
{
    uint32_t sum = 0x9E3779B9u;
    uint32_t w;
    sum = sum * 33u + c->magic;
    sum = sum * 33u + c->version;
    for (uint8_t i = 0; i < 4; ++i)
    {
        memcpy(&w, &c->angle[i], sizeof(w)); // memcpy避免严格别名/对齐告警
        sum = sum * 33u + w;
    }
    return sum;
}

static uint8_t SteerCalibAngleValid(const float *a)
{
    for (uint8_t i = 0; i < 4; ++i)
        if (!isfinite(a[i]) || a[i] < 0.0f || a[i] >= 360.0f)
            return 0;
    return 1;
}

/* 上电加载: 有有效记录就用Flash的,否则用robot_def.h的默认值 */
static void SteerCalibLoad(void)
{
    SteerCalib_s c;
    memcpy(&c, (const void *)STEER_CALIB_ADDR, sizeof(c));
    if (c.magic == STEER_CALIB_MAGIC && c.version == STEER_CALIB_VERSION &&
        c.checksum == SteerCalibChecksum(&c) && SteerCalibAngleValid(c.angle))
    {
        for (uint8_t i = 0; i < 4; ++i)
            steer_forward_angle[i] = c.angle[i];
        LOGINFO("[chassis] steer calib from flash: LF/RF/LB/RB x100 = %d %d %d %d",
                (int)(steer_forward_angle[0] * 100.0f), (int)(steer_forward_angle[1] * 100.0f),
                (int)(steer_forward_angle[2] * 100.0f), (int)(steer_forward_angle[3] * 100.0f));
    }
    else
    {
        LOGINFO("[chassis] steer calib: no valid flash record, use robot_def.h defaults");
    }
}

/* 把当前四个舵轮的实际角度存成新的"正前方" */
static void SteerCalibSave(void){
    SteerCalib_s c;
    DJIMotorInstance *servos[4] = {servo_lf, servo_rf, servo_lb, servo_rb};

    c.magic = STEER_CALIB_MAGIC;
    c.version = STEER_CALIB_VERSION;
    for (uint8_t i = 0; i < 4; ++i)
        c.angle[i] = Normalize360(servos[i]->measure.angle_single_round);
    c.checksum = SteerCalibChecksum(&c);

    __disable_irq(); // 擦写期间CPU会被Flash控制器挂起,关中断避免ISR取指异常
    flash_erase_address(STEER_CALIB_ADDR, 1);
    int8_t res = flash_write_single_address(STEER_CALIB_ADDR, (uint32_t *)&c, sizeof(c) / 4u);
    __enable_irq();

    if (res == 0)
    {
        for (uint8_t i = 0; i < 4; ++i)
            steer_forward_angle[i] = c.angle[i];
        LOGWARNING("[chassis] steer calib SAVED to flash: LF/RF/LB/RB x100 = %d %d %d %d",
                   (int)(c.angle[0] * 100.0f), (int)(c.angle[1] * 100.0f),
                   (int)(c.angle[2] * 100.0f), (int)(c.angle[3] * 100.0f));
    }
    else
    {
        LOGERROR("[chassis] steer calib SAVE failed (flash write err), keep old value");
    }
}

/* 擦除Flash里的现场标定,恢复robot_def.h的出厂默认值(现场标错时的后悔药) */
static void SteerCalibRestoreDefault(void)
{
    __disable_irq();
    flash_erase_address(STEER_CALIB_ADDR, 1);
    __enable_irq();

    steer_forward_angle[0] = GM6020_LF_ANGLE;
    steer_forward_angle[1] = GM6020_RF_ANGLE;
    steer_forward_angle[2] = GM6020_LB_ANGLE;
    steer_forward_angle[3] = GM6020_RB_ANGLE;
    LOGWARNING("[chassis] steer calib ERASED, back to robot_def.h defaults: LF/RF/LB/RB x100 = %d %d %d %d",
               (int)(steer_forward_angle[0] * 100.0f), (int)(steer_forward_angle[1] * 100.0f),
               (int)(steer_forward_angle[2] * 100.0f), (int)(steer_forward_angle[3] * 100.0f));
}

/* 示教模式参数(底盘任务200Hz,1tick=5ms) */
#define STEER_TEACH_ENTER_TICKS 300u  // 左下+右下 保持1.5s进入
#define STEER_TEACH_SAVE_TICKS 200u   // 右摇杆前推/后拉到底 保持1.0s保存/恢复默认
#define STEER_TEACH_LOG_TICKS 100u    // 示教中每0.5s打印一次当前四角
#define STEER_TEACH_STICK_FWD 600     // 右摇杆"推到底"的判定阈值(量纲±660)
#define STEER_TEACH_STICK_BACK (-600) // 右摇杆"拉到底"

static uint8_t steer_teach_on = 0;
static uint16_t steer_teach_enter_cnt = 0;
static uint16_t steer_teach_save_cnt = 0;
static uint16_t steer_teach_log_cnt = 0;

/**
 * @brief 示教模式状态机: 处理进入/保存/退出,并周期打印四轮角度
 * @note  进入: 左下+右下保持1.5s(遥控器必须在线); 保存: 右摇杆前推到底保持1s;
 *        退出不保存: 左开关[上](急停)或遥控器掉线
 * @return 1 当前处于示教模式(调用者应让8个电机全部零输出并跳过正常控制)
 */
static uint8_t RCDirectTeachUpdate(void)
{
    uint8_t swl = rc_data[TEMP].rc.switch_left;
    uint8_t swr = rc_data[TEMP].rc.switch_right;
    uint8_t online = RemoteControlIsOnline();

    if (!steer_teach_on)
    {
        if (online && swl == RC_SW_DOWN && swr == RC_SW_DOWN)
        {
            if (++steer_teach_enter_cnt >= STEER_TEACH_ENTER_TICKS)
            {
                steer_teach_on = 1;
                steer_teach_enter_cnt = 0;
                steer_teach_save_cnt = 0;
                steer_teach_log_cnt = 0;
                LOGWARNING("[chassis] steer teach ON: 4 servos unpowered, set wheels to forward, "
                           "right stick FULL FORWARD 1s to SAVE, FULL BACK 1s to RESTORE DEFAULTS, left switch UP to exit");
            }
        }
        else
            steer_teach_enter_cnt = 0;
        return 0;
    }

    // 遥控器掉线: 直接退出,避免无人看管时长期保持"舵轮无力矩"
    if (!online)
    {
        steer_teach_on = 0;
        LOGWARNING("[chassis] steer teach OFF (rc lost), nothing saved");
        return 0;
    }

    // 退出不保存: 左开关打到[上](与急停同位置)
    if (swl == RC_SW_UP)
    {
        steer_teach_on = 0;
        steer_teach_save_cnt = 0;
        LOGWARNING("[chassis] steer teach OFF (left switch UP), nothing saved");
        return 0;
    }

    // 保存: 右摇杆前推到底并保持; 恢复出厂: 右摇杆后拉到底并保持
    if (rc_data[TEMP].rc.rocker_r1 >= STEER_TEACH_STICK_FWD ||
        rc_data[TEMP].rc.rocker_r1 <= STEER_TEACH_STICK_BACK)
    {
        if (++steer_teach_save_cnt >= STEER_TEACH_SAVE_TICKS)
        {
            if (rc_data[TEMP].rc.rocker_r1 >= STEER_TEACH_STICK_FWD)
                SteerCalibSave();
            else
                SteerCalibRestoreDefault();
            steer_teach_on = 0;
            steer_teach_save_cnt = 0;
            return 0;
        }
    }
    else
        steer_teach_save_cnt = 0;

    // 周期打印当前四角(便于用RTT核对)
    if (++steer_teach_log_cnt >= STEER_TEACH_LOG_TICKS)
    {
        steer_teach_log_cnt = 0;
        LOGINFO("[chassis] teach now LF/RF/LB/RB x100 = %d %d %d %d",
                (int)(servo_lf->measure.angle_single_round * 100.0f), (int)(servo_rf->measure.angle_single_round * 100.0f),
                (int)(servo_lb->measure.angle_single_round * 100.0f), (int)(servo_rb->measure.angle_single_round * 100.0f));
    }

    return 1;
}
#endif // CHASSIS_RC_DIRECT

void ChassisInit()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取云台姿态数据指针

    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_M3508_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3, // 4.5
                .Ki = 0,   // 0
                .Kd = 0,   // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    
    // // CAN1: 前两轮 LF/RF (电调ID: LF=1, RF=3)
    chassis_M3508_motor_config.can_init_config.can_handle = &hcan1;
    chassis_M3508_motor_config.can_init_config.tx_id = 1;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_M3508_motor_config);

    chassis_M3508_motor_config.can_init_config.tx_id = 3;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_M3508_motor_config);

    // // CAN2: 后两轮 LB/RB (电调ID: LB=1, RB=3)
    chassis_M3508_motor_config.can_init_config.can_handle = &hcan2;
    chassis_M3508_motor_config.can_init_config.tx_id = 1;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_M3508_motor_config);

    chassis_M3508_motor_config.can_init_config.tx_id = 3;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_M3508_motor_config);


    // GM6020电机的初始化(舵机电机)
     Motor_Init_Config_s chassis_GM6020_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50,
                .Ki = 5,
                .Kd = 1,
                .MaxOut = 3000, // 角度环输出即速度环参考(deg/s),GM6020物理极限约1920°/s,取1.5倍留超调余量
                .IntegralLimit = 1000, // 必须为正,否则PID_Integral_Limit会把积分清零(速度环参考量纲为deg/s)
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            .speed_PID = {
                .Kp = 10,
                .Ki = 0.1,
                .Kd = 0,
                .MaxOut = 20000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            },
            // .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            // .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, 
        },
        .motor_type = GM6020};


    // // CAN1: 前两轮舵向 LF/RF (电调ID: LF=2, RF=4)
    chassis_GM6020_motor_config.can_init_config.can_handle = &hcan1;
    chassis_GM6020_motor_config.can_init_config.tx_id = 2;
    servo_lf = DJIMotorInit(&chassis_GM6020_motor_config);
    chassis_GM6020_motor_config.can_init_config.tx_id = 4;
    servo_rf = DJIMotorInit(&chassis_GM6020_motor_config);
    // // CAN2: 后两轮舵向 LB/RB (电调ID: LB=2, RB=4)
    chassis_GM6020_motor_config.can_init_config.can_handle = &hcan2;
    chassis_GM6020_motor_config.can_init_config.tx_id = 2;
    servo_lb = DJIMotorInit(&chassis_GM6020_motor_config);
    chassis_GM6020_motor_config.can_init_config.tx_id = 4;
    servo_rb = DJIMotorInit(&chassis_GM6020_motor_config);


    // 底盘跟随PID(输入offset_angle°,输出wz;因运动学杠杆臂已修正,wz量纲缩小约4.1倍,以下参数为理论预缩放值,场测微调)
    PID_Init_Config_s Chassis_Follow_PID_config = {
        .Kp = 1.25f,
        .Ki = 60.0f,
        .Kd = 0.0004f,
        .IntegralLimit = 25.0f,
        .MaxOut = 1250.0f,
        .DeadBand = 0.0f,
        .CoefA = 0.0f,
        .CoefB = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate,
    };
    PIDInit(&Chassis_Follow_PID, &Chassis_Follow_PID_config);

//     referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

// /* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
//     PID_Init_Config_s Buffer_pid_conf = {
//         .Kp = 0.1,
//         .Ki = 0,
//         .Kd = 0,
//         .IntegralLimit = 1000,
//         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//         .MaxOut = 1000,
//     };
//     PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化
//     SuperCap_Init_Config_s cap_conf = {
//         .can_config = {
//             .can_handle = &hcan2,
//             .tx_id = 0x302, // 超级电容默认接收id
//             .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
//         }};
//     cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

#ifdef CHASSIS_RC_DIRECT
    /* 单板直控: 遥控器(DBUS)接本板USART3(PC11,ioc已配100000bps/Even/9bit+DMA),
     * 不初始化CANComm(CANComm只用于与云台板互发0x311/0x312,单板下无对端) */
    SteerCalibLoad(); // 现场标定: 优先用Flash里保存的正前方(示教模式存的),没有则用robot_def.h默认值
    rc_data = RemoteControlInit(&huart3);
    if (rc_data == NULL) // 防御性判空(RemoteControlInit返回静态数组,正常不会失败)
    {
        LOGERROR("[chassis] remote control init failed, check USART3 config");
        BSP_FaultCode = 1;
        while (1)
            ;
    }
#else  // CHASSIS_RC_DIRECT
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化

#ifdef RC_RECEIVER_ON_CHASSIS
    // 遥控器(DBUS)改由底盘板接收: USART3参数(100000bps/Even/9bit)已在CubeMX中配置
    rc_data = RemoteControlInit(&huart3);
    if (rc_data == NULL) // 防御性判空(RemoteControlInit返回静态数组,正常不会失败)
    {
        LOGERROR("[chassis] remote control init failed, check USART3 config");
        BSP_FaultCode = 1;
        while (1)
            ;
    }
#endif // RC_RECEIVER_ON_CHASSIS
#endif // CHASSIS_RC_DIRECT
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

//将电机转子转向内侧时 修正方向
/* 舵向电机默认驱动方向(与steering_dirt初值一致): 运动学解算与上电回正共用,避免两处字面量不同步.
 * 顺序为 LF, RF, LB, RB; 含义 = "舵轮处于标定正前方时,该轮要往车头方向推所需的驱动符号".
 * 2026-10-14 现场实测修正: 原值 {1,-1,1,-1} 只考虑了左右镜像,但本车前/后模组驱动方向相反
 *   —— 推杆前进时前轮滚动方向正确、后轮相反(前后对顶,车无法前进后退),故后两轮取反.
 * 注意: 某个舵轮若被装成"正前方+180"的方位,与该轮驱动符号取反是等价的(整车推力一致),
 *       所以这里改符号即可,不必去动 GM6020_*_ANGLE 标定值. */
static const int8_t SERVO_BASE_DIR[4] = {1, -1, -1, 1};
int8_t steering_dirt[4] = {1, -1, -1, 1};
// 单摇杆满幅等效的轮速参考上限,超出按比例缩小保持方向比(场测微调)
#define CHASSIS_WHEEL_SPEED_MAX 16500.0f
// 改为接收指针，写回调用者的变量
// 舵轮运动学:x分量旋转杠杆为前后位置HALF_WHEEL_BASE,y分量为左右位置HALF_TRACK_WIDTH(左手系z向下)
void chassis_vector_to_M3508_wheel_speed(float vx_set, float vy_set, float wz_set,float *vt_lf, float *vt_rf, float *vt_lb, float *vt_rb)
{
    // float wheel_rpm_ratio;
    // wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * REDUCTION_RATIO_WHEEL * 1000;

    *vt_lf = (float)(steering_dirt[0] * sqrtf(powf(vy_set + wz_set * HALF_TRACK_WIDTH,2)
                       + powf(vx_set + wz_set * HALF_WHEEL_BASE,2)
                       ));
    *vt_rf = (float)(steering_dirt[1] * sqrtf(powf(vy_set - wz_set * HALF_TRACK_WIDTH,2)
                       + powf(vx_set + wz_set * HALF_WHEEL_BASE,2)
                       ));
    *vt_lb = (float)(steering_dirt[2] * sqrtf(powf(vy_set + wz_set * HALF_TRACK_WIDTH,2)
                       + powf(vx_set - wz_set * HALF_WHEEL_BASE,2)
                       ));
    *vt_rb = (float)(steering_dirt[3] * sqrtf(powf(vy_set - wz_set * HALF_TRACK_WIDTH,2)
                       + powf(vx_set - wz_set * HALF_WHEEL_BASE,2)
                       ));

    // 平移+自旋组合时参考值可能远超单摇杆满幅,按最大轮速等比缩小,保持各轮方向/速度比不变
    float vt_max = ABS(*vt_lf);
    if (ABS(*vt_rf) > vt_max) vt_max = ABS(*vt_rf);
    if (ABS(*vt_lb) > vt_max) vt_max = ABS(*vt_lb);
    if (ABS(*vt_rb) > vt_max) vt_max = ABS(*vt_rb);
    if (vt_max > CHASSIS_WHEEL_SPEED_MAX)
    {
        float scale = CHASSIS_WHEEL_SPEED_MAX / vt_max;
        *vt_lf *= scale;
        *vt_rf *= scale;
        *vt_lb *= scale;
        *vt_rb *= scale;
    }
}

/*
 * 将角度差归一化到 (-180, 180]
 */
static float Normalize180(float a)
{
    if (!isfinite(a)) // 非有限输入(Inf/NaN)直接返回,避免while归一化死循环(R31)
        return a;
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/*
 * 将角度规范化到 [0, 360)
 * 使用简单的循环避免依赖 fmodf / math.h，适合嵌入式使用
 * 支持任意大的正负角度，包括大偏置值（如-108°）
 */
static float Normalize360(float a)
{
    if (!isfinite(a)) // 非有限输入(Inf/NaN)直接返回,避免while归一化死循环(R31)
        return a;
    // 处理大的负数：快速调整到接近[0, 360)范围
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static float atan_angle[4];
/*
 * 计算目标多圈角度（不限制在0~360），并根据最小转角原则决定是否翻转驱动方向
 */
void chassis_vector_to_GM6020_wheel_angle_ecd(float vx_set, float vy_set, float wz_set, float *angle_lf, float *angle_rf, float *angle_lb, float *angle_rb)
{
    // GM6020目标角度计算
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
    {
        //由于atan2f算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值  左手系解算：z下 x右 y前
        // 注意使用单精度atan2f,Cortex-M4F无双精度硬件,atan2()会走软件库开销极大
        // 旋转杠杆与轮速解算一致:x分量用HALF_WHEEL_BASE(前后位置),y分量用HALF_TRACK_WIDTH(左右位置)

        atan_angle[0] = atan2f((vx_set + wz_set * HALF_WHEEL_BASE),(vy_set + wz_set * HALF_TRACK_WIDTH)) * 180.0f / PI;
        atan_angle[1] = atan2f((vx_set + wz_set * HALF_WHEEL_BASE),(vy_set - wz_set * HALF_TRACK_WIDTH)) * 180.0f / PI;
        atan_angle[2] = atan2f((vx_set - wz_set * HALF_WHEEL_BASE),(vy_set + wz_set * HALF_TRACK_WIDTH)) * 180.0f / PI;
        atan_angle[3] = atan2f((vx_set - wz_set * HALF_WHEEL_BASE),(vy_set - wz_set * HALF_TRACK_WIDTH)) * 180.0f / PI;
    }  
    // 目标单圈角度（0..360) 基于偏置+atan结果
    float desired_single[4];
    desired_single[0] = GM6020_LF_ANGLE + (float)atan_angle[0];//45
    desired_single[1] = GM6020_RF_ANGLE + (float)atan_angle[1];//135
    desired_single[2] = GM6020_LB_ANGLE + (float)atan_angle[2];//-45
    desired_single[3] = GM6020_RB_ANGLE + (float)atan_angle[3];//-135

    // 使用多圈全角（total_angle）作为基准生成无界目标角度，避免限制在0~360
    DJIMotorInstance *servos[4] = {servo_lf, servo_rf, servo_lb, servo_rb};
    float *targets[4] = {angle_lf, angle_rf, angle_lb, angle_rb};
    const int8_t *base_dir = SERVO_BASE_DIR; // 对应每个舵轮的默认驱动方向

    for (int i = 0; i < 4; i++)
    {
        // 规范化 desired_single 到 [0,360)
        float ds = Normalize360(desired_single[i]);

        // 当前电机多圈角度和单圈角度
        float cur_total = servos[i]->measure.total_angle;
        float cur_single = servos[i]->measure.angle_single_round;

        // 计算使目标最接近当前位置的单圈差值（范围在(-180,180]）
        float diff_single = ds - cur_single;
        diff_single = Normalize180(diff_single);

        // 目标多圈角度 = 当前多圈角度 + 差值
        *targets[i] = cur_total + diff_single;

        // 再基于多圈差值判断是否需要翻转驱动方向（优劣弧）
        float angle_diff = Normalize180(*targets[i] - cur_total);
        if (angle_diff > 90.0f)
        {
            steering_dirt[i] = -base_dir[i];
            *targets[i] = *targets[i] - 180.0f; // 仍为无界值
        }
        else if (angle_diff < -90.0f)
        {
            steering_dirt[i] = -base_dir[i];
            *targets[i] = *targets[i] + 180.0f;
        }
        else
        {
            steering_dirt[i] = base_dir[i];
        }
    }
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    // 麦克纳姆轮底盘运动学正解算
    // vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    // vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    // vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    // vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;

    // 舵轮底盘运动学正解算
    // 先解算舵向角(内部更新steering_dirt翻转标志),再解算轮速,保证驱动方向与舵向翻转同拍生效
    chassis_vector_to_GM6020_wheel_angle_ecd(chassis_vx, chassis_vy, chassis_cmd_recv.wz, &angle_lf, &angle_rf, &angle_lb, &angle_rb);
    chassis_vector_to_M3508_wheel_speed(chassis_vx, chassis_vy, chassis_cmd_recv.wz, &vt_lf, &vt_rf, &vt_lb, &vt_rb);
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
    DJIMotorSetRef(servo_lf, angle_lf);
    DJIMotorSetRef(servo_rf, angle_rf);
    DJIMotorSetRef(servo_lb, angle_lb);
    DJIMotorSetRef(servo_rb, angle_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 * @note  原空实现 EstimateSpeed() 已删除(A.9/F2):实现轮式里程计需标定轮径/减速比
 *        并与IMU融合,属独立工作量;TODO.md 保持「未实现」标注
 */

#ifdef CHASSIS_RC_DIRECT
/**
 * @brief 单板直控指令桥: 把本板遥控器(DBUS)数据映射为Chassis_Ctrl_Cmd_s
 * @note  安全优先级: ①遥控器离线→零力(修复RC-1) ②左开关[上]→软急停 ③右开关非法值(含0)→零力兜底
 *        与ChassisTask同频运行(200Hz,见robot_task.h),断连检测延迟=daemon 100ms+任务周期5ms
 *        仅单板(CHASSIS_BOARD)使用: 此时底盘指令的唯一来源是本桥,CANComm未初始化
 */
static void RCDirectSet()
{
    // 每周期先清空,杜绝上一周期的脏值残留(尤其是vx/vy/offset_angle)
    memset(&chassis_cmd_recv, 0, sizeof(Chassis_Ctrl_Cmd_s));

    // ① 离线保护(最高优先级): 遥控器断连立即零力,不依赖开关位置(RC-1)
    // ② 左开关[上] = 急停,语义沿用原robot_cmd的EmergencyHandler
    if (!RemoteControlIsOnline() || switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }

    // ③ 右开关决定底盘模式
    switch (rc_data[TEMP].rc.switch_right)
    {
    case RC_SW_UP: // [上] 小陀螺+全向平移(ChassisTask内强制wz=25)
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
        break;
    case RC_SW_MID:  // [中] 全向平移,不旋转
    case RC_SW_DOWN: // [下] 单板无云台编码器,FOLLOW_GIMBAL_YAW不可得,降级为NO_FOLLOW
        chassis_cmd_recv.chassis_mode = CHASSIS_NO_FOLLOW;
        break;
    default: // 非法开关值兜底(含断连清零后的0,以及daemon 1s宽限期内未收到任何帧的情况)
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }

    // ④ 右摇杆→底盘速度: 量纲±660,死区沿用CHASSIS_DEAD_BAND(10),缩放见CHASSIS_RC_SPEED_SCALE
    float vx_in = (float)rc_data[TEMP].rc.rocker_r_; // 右摇杆水平→x(右移为正)
    float vy_in = (float)rc_data[TEMP].rc.rocker_r1; // 右摇杆竖直→y(前推为正)
    chassis_cmd_recv.vx = (ABS(vx_in) > CHASSIS_DEAD_BAND) ? vx_in * CHASSIS_RC_SPEED_SCALE : 0.0f;
    chassis_cmd_recv.vy = (ABS(vy_in) > CHASSIS_DEAD_BAND) ? vy_in * CHASSIS_RC_SPEED_SCALE : 0.0f;
    chassis_cmd_recv.offset_angle = 0.0f; // 单板无云台反馈,坐标系不旋转(摇杆向前=车体向前)
    // wz不在此处设置,由下方ChassisTask的模式机统一设定(ROTATE=25 / NO_FOLLOW=0)
}
#endif // CHASSIS_RC_DIRECT

#ifdef CHASSIS_RC_DIRECT
/* 回正过程中每轮"标定正前方 - 当前角度"的残差(度),供回正结束/超时时打日志诊断 */
static float align_err[4];
static uint8_t align_done = 0;    // 上电回正阶段是否已完成(示教模式期间会清零,退出后自动重跑)
static uint16_t align_ok_cnt = 0; // 连续满足回正容差的周期数

/**
 * @brief 让4个舵向电机指向标定的正前方(不做±180°优劣弧翻转,保证视觉上就是正前方)
 * @note  每周期调用都重设参考值: 进了容差后继续调用仍会持续收敛,静止时可当作"角度保持"用.
 *        与运动学解算的区别: 这里强制走"正前方"目标,不做±180°翻转,否则轮子可能收敛到
 *        反向(驱动方向靠steering_dirt补偿,推力虽对但车轮视觉朝后);同时把steering_dirt复正.
 * @return 1 四个舵轮都在CHASSIS_ALIGN_TOLERANCE_DEG容差内
 */
static uint8_t ServosHoldForward()
{
    // 四个舵轮"正前方"对应的编码器角度(默认取robot_def.h标定值,上电可被Flash里的现场标定覆盖)
    const float *forward_angle = steer_forward_angle;
    DJIMotorInstance *servos[4] = {servo_lf, servo_rf, servo_lb, servo_rb};
    float angle_out[4];
    uint8_t aligned = 1;

    for (uint8_t i = 0; i < 4; ++i)
    {
        float diff = Normalize180(forward_angle[i] - servos[i]->measure.angle_single_round);
        align_err[i] = diff;
        if (ABS(diff) > CHASSIS_ALIGN_TOLERANCE_DEG)
            aligned = 0; // 任一轮未到位即认为尚未回正
        // 目标多圈角度: 不翻转,始终朝正前方收敛(轮子最多自转180°到位)
        angle_out[i] = servos[i]->measure.total_angle + diff;
        steering_dirt[i] = SERVO_BASE_DIR[i];
        DJIMotorEnable(servos[i]); // 舵向电机走角度环,需使能
    }

    DJIMotorSetRef(servo_lf, angle_out[0]);
    DJIMotorSetRef(servo_rf, angle_out[1]);
    DJIMotorSetRef(servo_lb, angle_out[2]);
    DJIMotorSetRef(servo_rb, angle_out[3]);
    return aligned;
}

/**
 * @brief 上电初始回正: 4个舵轮转到正前方(驱动轮始终保持零输出,回正过程不会产生位移)
 * @return 1 四个舵轮都已在容差内对准正前方
 */
static uint8_t RCDirectAlignServos()
{
    // 驱动轮保持停止: 与ZERO_FORCE行为一致,确保上电回正期间整车不动
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
    return ServosHoldForward();
}
#endif // CHASSIS_RC_DIRECT

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
#ifdef CHASSIS_RC_DIRECT
    /* 现场示教模式(最高优先级): 左下+右下保持1.5s进入,四舵轮断电便于手摆,
     * 右摇杆前推到底1s把当前四角存Flash,左[上]退出.示教期间8个电机全部零输出. */
    if (RCDirectTeachUpdate())
    {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(servo_lf); // 舵轮零电流 => 可用手自由摆动
        DJIMotorStop(servo_rf);
        DJIMotorStop(servo_lb);
        DJIMotorStop(servo_rb);
        align_done = 0; // 退出示教后重新执行一次回正(用(可能是新的)正前方标定)
        align_ok_cnt = 0;
        return;
    }

    /* 上电初始回正(仅一次,示教模式退出后会重跑): 把4个舵轮转到正前方后交回正常流程.
     * 该阶段会驱动舵轮转动,且此期间不响应摇杆. */
    if (!align_done)
    {
        uint8_t aligned = RCDirectAlignServos();
        align_ok_cnt = aligned ? (uint16_t)(align_ok_cnt + 1u) : 0u; // 必须连续满足容差,防止过渡过程误判
        if (align_ok_cnt >= CHASSIS_ALIGN_SETTLE_CNT || DWT_GetTimeline_ms() >= CHASSIS_ALIGN_TIMEOUT_MS)
        {
            align_done = 1;
            if (align_ok_cnt >= CHASSIS_ALIGN_SETTLE_CNT)
                LOGINFO("[chassis] servo align done, LF/RF/LB/RB x10 = %d %d %d %d",
                        (int)(servo_lf->measure.angle_single_round * 10.0f), (int)(servo_rf->measure.angle_single_round * 10.0f),
                        (int)(servo_lb->measure.angle_single_round * 10.0f), (int)(servo_rb->measure.angle_single_round * 10.0f));
            else // 超时: 可能某个舵轮卡住/舵机无反馈,不阻塞正常控制
                LOGWARNING("[chassis] servo align timeout(%dms), residual x10 LF/RF/LB/RB = %d %d %d %d",
                           (int)CHASSIS_ALIGN_TIMEOUT_MS,
                           (int)(align_err[0] * 10.0f), (int)(align_err[1] * 10.0f),
                           (int)(align_err[2] * 10.0f), (int)(align_err[3] * 10.0f));
        }
        return; // 回正期间不进入正常控制流程
    }
#endif // CHASSIS_RC_DIRECT

    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_RC_DIRECT
    // 单板直控: 指令由本板遥控器解算得到,不依赖CANComm/云台板(未初始化CANComm)
    RCDirectSet();
#elif defined(CHASSIS_BOARD)
    if (CANCommIsOnline(chasiss_can_comm)) // 超时约1s(daemon默认reload=100,100Hz)
    {
        chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    }
    else
    { // 双板通信离线,清空控制量并进入零力,防止按最后一帧指令持续运动导致失控
        memset(&chassis_cmd_recv, 0, sizeof(Chassis_Ctrl_Cmd_s));
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    }
#endif // CHASSIS_BOARD

#ifdef CHASSIS_BOARD
#ifdef RC_RECEIVER_ON_CHASSIS
    // 遥控器数据随底盘板上行帧(0x311 Chassis_Upload_Data_s)一并转发给云台板的robot_cmd
    // 复用既有链路的闲置载荷,不新增CAN ID/实例/滤波器,也不需要额外的守护进程
    chassis_feedback_data.rc.rocker_l_ = rc_data[TEMP].rc.rocker_l_;
    chassis_feedback_data.rc.rocker_l1 = rc_data[TEMP].rc.rocker_l1;
    chassis_feedback_data.rc.rocker_r_ = rc_data[TEMP].rc.rocker_r_;
    chassis_feedback_data.rc.rocker_r1 = rc_data[TEMP].rc.rocker_r1;
    chassis_feedback_data.rc.dial = rc_data[TEMP].rc.dial;
    chassis_feedback_data.rc.switch_left = rc_data[TEMP].rc.switch_left;
    chassis_feedback_data.rc.switch_right = rc_data[TEMP].rc.switch_right;
    chassis_feedback_data.rc.mouse_x = rc_data[TEMP].mouse.x;
    chassis_feedback_data.rc.mouse_y = rc_data[TEMP].mouse.y;
    chassis_feedback_data.rc.key_press = rc_data[TEMP].key[KEY_PRESS].keys;
    memcpy(chassis_feedback_data.rc.key_count, rc_data[TEMP].key_count[KEY_PRESS], sizeof(chassis_feedback_data.rc.key_count));
    chassis_feedback_data.rc.online = RemoteControlIsOnline(); // 一并转发遥控器离线状态,供云台板做离线保护
#endif // RC_RECEIVER_ON_CHASSIS
#endif // CHASSIS_BOARD

    // SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);//设置功率限制
#ifdef CHASSIS_RC_DIRECT
    // steer_forward: 本周期不需要行进(零力/急停,或摇杆回中且不自旋) => 舵轮回到标定正前方
    uint8_t steer_forward = 0;
#endif
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让4个驱动轮停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
#ifdef CHASSIS_STEER_HOLD_FORWARD
        // 舵轮仍通电稳住正前方(只断驱动轮): 离线/急停后车不会停在歪方向,重新起步也不用等回正
        ServosHoldForward();
        steer_forward = 1; // 置位以跳过下面的运动学解算,否则会把舵轮参考值覆盖成"上次行驶方向"
#else
        DJIMotorStop(servo_lf);
        DJIMotorStop(servo_rf);
        DJIMotorStop(servo_lb);
        DJIMotorStop(servo_rb);
#endif
        PIDReset(&Chassis_Follow_PID); // 跟随PID在零力期间不计算,复位防止残留积分在切回跟随模式时释放(F7)
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DJIMotorEnable(servo_lf);
        DJIMotorEnable(servo_rf);
        DJIMotorEnable(servo_lb);
        DJIMotorEnable(servo_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,采用PID  参数待调试
        chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, chassis_cmd_recv.offset_angle, 0.0f);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;杠杆臂修正后由100缩放为25,实际转速场测微调,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 25;
        break;
    default:
        break;
    }

#ifdef CHASSIS_RC_DIRECT
#ifdef CHASSIS_STEER_RETURN_ON_STOP
    // 摇杆回中且不自旋 => 车已停下: 舵轮主动转回标定正前方,而不是保持上次行驶方向(与上电回正一致)
    if (!steer_forward && chassis_cmd_recv.vx == 0.0f && chassis_cmd_recv.vy == 0.0f && chassis_cmd_recv.wz == 0.0f)
        steer_forward = 1;
#endif

    if (steer_forward)
    { // 静止: 驱动轮零输出 + 舵轮指向正前方
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorSetRef(motor_rf, 0.0f);
        DJIMotorSetRef(motor_lb, 0.0f);
        DJIMotorSetRef(motor_rb, 0.0f);
        ServosHoldForward();
    }
    else
#endif // CHASSIS_RC_DIRECT
    {
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    float rotate_angle = chassis_cmd_recv.offset_angle;

    // // 在CHASSIS_FOLLOW_GIMBAL_YAW模式下,vx/vy不要再用 offset_angle 旋转
    // if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    // {
    //     rotate_angle = 0.0f;
    // }
    cos_theta = arm_cos_f32(rotate_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(rotate_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    } // end of 有行进指令时的运动学解算分支(直控构建下为上面 if 的 else)

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#if defined(CHASSIS_BOARD) && !defined(CHASSIS_RC_DIRECT)
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#elif defined(CHASSIS_RC_DIRECT)
    // 单板直控: 无接收方,不发上行帧(发0x311收不到ACK会占满CAN1发送邮箱,拖垮前轮电机指令)
#endif
}