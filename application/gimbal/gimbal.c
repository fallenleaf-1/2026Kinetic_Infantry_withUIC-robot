#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "dmmotor.h"
#include "bsp_log.h"

#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 大YAW对齐时的角度,0-360

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
DJIMotorInstance *small_yaw_motor;  // 在INTtask中访问，计算大Yaw反馈
static DMMotorInstance *pitch_motor ;          // 声明DM4310电机实例

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static bool gimbal_SmallYaw_Init_Flag = false; // 小YAW电机初始化标志
/* 大yaw指令基准对齐(见GIMBAL_GYRO_MODE分支): 停机期间摇杆仍会累积指令,使能瞬间必须把基准
 * 挪到当前反馈,否则会出现几十~几百度的阶跃误差 ⇒ 电机满速冲过去(现场"一切到左下就疯狂旋转") */
static uint8_t yaw_ref_valid = 0;
#ifdef GIMBAL_YAW_POSITION_LOOP
static float yaw_ref_bias = 0.0f;
#endif
/* 大yaw摩擦(库仑/静摩擦)前馈: 由速度环分支每拍按指令方向更新, 通过 current_feedforward_ptr
 * 直接叠加在速度环输出上(与PID增益无关) ⇒ 既能压过静摩擦死区, 又不必把Kp提到自激的程度 */
static float yaw_current_ff = 0.0f;

#ifdef GIMBAL_YAW_IMU_HOLD
/* 大yaw世界角(IMU)保持 —— 详见 robot_def.h 中 GIMBAL_YAW_IMU_HOLD 的说明.
 * yaw_hold_target 是"要钉住的世界指向(°)", 摇杆接管时不断刷新为当前指向, 松杆后由P环拉回. */
static float yaw_hold_target = 0.0f;
static uint8_t yaw_hold_valid = 0; // 目标角是否已建立(停机/IMU异常时清零, 使能瞬间重新对齐)

/* 角度差归一化到 (-180,180] : 保持的是"朝向"而非"第几圈", 且避免为了一次误差反绕整圈 */
static float YawWrap180(float a)
{
    if (!isfinite(a))
        return 0.0f;
    while (a > 180.0f)
        a -= 360.0f;
    while (a <= -180.0f)
        a += 360.0f;
    return a;
}
#endif // GIMBAL_YAW_IMU_HOLD

#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
/* 小陀螺开环兜底(见robot_def.h GIMBAL_YAW_FF_ON_IMU_LOST):
 * 保持环正常工作时, "小陀螺 + 松杆"的稳态输出 = 需要补偿的底盘自转速率, 在线学下来;
 * IMU 一旦失效(姿态NaN/离线)就用这个实测值做反向补偿 —— 不需要任何传感器, 也不用人工标定. */
static float yaw_ff_learned = 0.0f;      // 学到的补偿速率(°/s, 已含正确符号, 直接叠加)
static uint8_t yaw_ff_learned_valid = 0; // 是否真的学到过(没学到则退回 robot_def.h 的初值)
static uint16_t yaw_ff_settle_ms = 0;    // 小陀螺已稳定运行的时长(ms): 先让保持环收敛再学
static uint16_t yaw_ff_log_ms = 0;       // 限频打印(1Hz), 便于现场用RTT确认学到了多少(验证兜底用)
#endif

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // 大YAW  角度用IMU-小YAW(编码器)作为反馈
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50, // 50
                .Ki = 0,
                .Kd = 1,  // 1
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 1000,
            },
            .speed_PID = {
                /* 大yaw速度环(现场整定 2026-10-15, 第三版: 量纲修正)
                 * 关键: 驱动层 measure.speed_aps = RPM × RPM_2_ANGLE_PER_SEC(6.0) ⇒ **单位是 deg/s, 不是RPM**
                 * (底盘板GM6020舵向电机注释也写明"角度环输出即速度环参考(deg/s)"). 因此
                 *   ① 参考要用 yaw_rate(°/s) 直接给, 不能除以6;
                 *   ② 增益必须按 °/s 量纲取 —— 底盘板同型号电机用 Kp=10, 之前我按RPM量纲取300(相当于30倍)必然自激.
                 * Kd=0: 转速反馈有量化与~1ms延迟, 微分易放大成振动。 */
                .Kp = 10,
                .Ki = 0.5,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .current_feedforward_ptr = &yaw_current_ff, // 摩擦前馈(直接叠加在速度环输出上)
            .other_angle_feedback_ptr = &gimba_IMU_data->BigYaw_Feed,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
#ifdef GIMBAL_YAW_FEEDBACK_IMU
            .angle_feedback_source = OTHER_FEED, // 设计原意: IMU yaw - 小yaw偏置(绝对指向)
            .speed_feedback_source = OTHER_FEED,
#else
            .angle_feedback_source = MOTOR_FEED, // 现场调试: 大yaw自身编码器,不依赖IMU
            .speed_feedback_source = MOTOR_FEED,
#endif
#ifdef GIMBAL_YAW_POSITION_LOOP
            .outer_loop_type = ANGLE_LOOP, // 位置环: angle→speed 级联
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
#else
            .outer_loop_type = SPEED_LOOP, // 速度环: 摇杆直接给角速度指令
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD, // 启用电流前馈(摩擦补偿),见 GIMBAL_YAW_FRICTION_FF
#endif
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    
    // 小YAW    采用编码器反馈
    Motor_Init_Config_s small_yaw_config = {
    .can_init_config = {
        .can_handle = &hcan2,
        .tx_id = 3,
    },
    .controller_param_init_config = {
        .angle_PID = {
            .Kp = 60, // 60
            .Ki = 0,
            .Kd = 0,
            .DeadBand = 0.1,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 100,

            .MaxOut = 500,
        },
        .speed_PID = {
            .Kp = 50,  // 50
            .Ki = 150, // 150
            .Kd = 0,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3000,
            .MaxOut = 20000,
        },
        .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
        // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
        .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
    },
    .controller_setting_init_config = {
        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    },
    .motor_type = GM6020};
    
    // PITCH
     Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2, // 1--->2 O.o
            .tx_id = 0x04,
            .rx_id = 0x14,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, // 使用IMU作为角度反馈
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            // .outer_loop_type = SPEED_LOOP,
            // .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .controller_param_init_config = {
            .DM_MIT_Kp = 0,
            .DM_MIT_Kd = 0.1,
            .angle_PID = {
                .Kp = 25,//0.31
                .Ki = 50,//1
                .Kd = 0.01,//0.001
                .MaxOut = 1000,
                .CoefA = 0.3,
                .CoefB = 0.2,
                .IntegralLimit = 100000,
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = 0.13,//0.12
                .Ki = 0,//0
                .Kd = 0.003,//0.01
                .MaxOut = 100,//53
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1],
        },
        .motor_type = DMMotor};

    pitch_motor = MITMotorInit(&pitch_config);
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    small_yaw_motor = DJIMotorInit(&small_yaw_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(small_yaw_motor);
        DMMotorStop(pitch_motor);
        gimbal_SmallYaw_Init_Flag = false;
        yaw_ref_valid = 0;   // 停机: 下次使能时重新把指令基准对齐到当前反馈
        yaw_current_ff = 0.0f; // 停机时清掉摩擦前馈
#ifdef GIMBAL_YAW_IMU_HOLD
        yaw_hold_valid = 0;    // 停机: 下次使能时把世界角保持目标重新对齐到当前指向
#endif
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
        yaw_ff_settle_ms = 0;  // 停机: 重新计数"小陀螺稳定时长"(学到的兜底常数保留)
#endif
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE:
        if(gimbal_SmallYaw_Init_Flag == false)  // 先初始化小yaw，再初始化大yaw
        {
            DJIMotorEnable(small_yaw_motor);
            if(small_yaw_motor->measure.ecd < SMALL_YAW_FORWORD_ALIGN_ECD + 5 || small_yaw_motor->measure.ecd > SMALL_YAW_FORWORD_ALIGN_ECD - 5)
            {
                gimbal_SmallYaw_Init_Flag = true; // 标志位置位,只初始化一次 
            }
        }
        if(gimbal_SmallYaw_Init_Flag == true)
        {
            DJIMotorEnable(yaw_motor);
#ifdef GIMBAL_YAW_POSITION_LOOP
#ifdef GIMBAL_YAW_FEEDBACK_IMU
            // 大yaw采IMU反馈(设计原意: 大yaw角度=IMU yaw-小yaw偏置,用于视觉/跟随的绝对指向)
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            float yaw_feedback_now = gimba_IMU_data->BigYaw_Feed;
#else
            // 大yaw采**电机自身编码器**反馈
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
            float yaw_feedback_now = yaw_motor->measure.total_angle;
#endif
            /* 使能瞬间把指令基准对齐到当前反馈(现场"一切到左下就疯狂旋转"的根因):
             * 停机(ZERO_FORCE/急停/遥控器离线)期间 yaw_motor 是停的, 但 robot_cmd 仍在按摇杆累积
             * gimbal_cmd_recv.yaw —— 直接用它当参考, 使能瞬间就有几十~几百度的阶跃误差, 角度环满输出冲过去. */
            if (!yaw_ref_valid)
            {
                yaw_ref_bias = yaw_feedback_now - gimbal_cmd_recv.yaw;
                yaw_ref_valid = 1;
                LOGINFO("[gimbal] yaw ref aligned: cmd=%d mdeg, feedback=%d mdeg, bias=%d mdeg",
                        (int)(gimbal_cmd_recv.yaw * 1000.0f), (int)(yaw_feedback_now * 1000.0f),
                        (int)(yaw_ref_bias * 1000.0f));
            }
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw + yaw_ref_bias); // 在cmd中处理多圈与单圈
#else
            /* 速度环(默认): robot_cmd 已把摇杆解成角速度指令(°/s, 见Gimbal_Ctrl_Cmd_s.yaw_rate).
             * 注意驱动层速度反馈 measure.speed_aps 的单位已经是 **deg/s**(=RPM×6), 所以这里直接给, 不做RPM换算.
             * 为什么不用位置环: 实测位置环在堵转时误差累积到几百度的同时角度环输出上限是1000RPM
             * (=6000°/s), 一旦脱开就满速冲出去且不可控; 速度环没有位置误差可累积, 输出也被限幅. */
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
            float yaw_spd_ref = gimbal_cmd_recv.yaw_rate; // °/s
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
            uint8_t yaw_hold_active = 0; // 本拍世界基准是否由保持环提供(否则走开环兜底)
#endif
#ifdef GIMBAL_YAW_IMU_HOLD
            /* 世界角保持(见robot_def.h): 摇杆在推时把目标刷新成当前指向(松杆即钉住), 回中后用P环
             * 补出保持速率. 小陀螺(底盘自旋)时摇杆回中 ⇒ 只有这里在起作用: 大yaw被钉在世界系,
             * offset_angle 才会随底盘自转连续变化, 底盘才会"朝着大yaw方向"直行而不是画圈. */
            float yaw_imu = gimba_IMU_data->BigYaw_Feed;
            uint8_t yaw_hold_ok = (uint8_t)(isfinite(yaw_imu) && INS_IMUIsAvailable());
#ifdef GIMBAL_YAW_FF_FORCE
            yaw_hold_ok = 0; // 强制开环兜底(现场标定/验证兜底方向用): 保持环不参与
#endif
            if (!yaw_hold_ok)
            {
                yaw_hold_valid = 0; // IMU姿态不可用/被强制: 保持环失效, 下次可用时重新对齐目标
            }
            else
            {
                if (!yaw_hold_valid) // 使能/恢复瞬间把目标对齐当前指向, 杜绝阶跃(与位置环同理)
                {
                    yaw_hold_target = yaw_imu;
                    yaw_hold_valid = 1;
                }
                if (yaw_spd_ref > GIMBAL_YAW_HOLD_STICK_TH || yaw_spd_ref < -GIMBAL_YAW_HOLD_STICK_TH)
                {
                    yaw_hold_target = yaw_imu; // 人在推杆: 目标跟随(摇杆速率即世界角速率)
                }
                else
                {
                    float yaw_hold_rate = GIMBAL_YAW_HOLD_KP * YawWrap180(yaw_hold_target - yaw_imu);
                    if (!(yaw_hold_rate <= GIMBAL_YAW_HOLD_LIMIT)) // NaN安全写法(与下方限幅一致)
                        yaw_hold_rate = GIMBAL_YAW_HOLD_LIMIT;
                    if (!(yaw_hold_rate >= -GIMBAL_YAW_HOLD_LIMIT))
                        yaw_hold_rate = -GIMBAL_YAW_HOLD_LIMIT;
                    yaw_spd_ref += yaw_hold_rate;
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
                    yaw_hold_active = 1;
                    /* 兜底常数在线自学: 只在"小陀螺(robot_cmd 填了非0速率) + 松杆 + 保持环未饱和"
                     * 且已稳定 1s 之后进行, 学到的就是"抵消底盘自转所需的那份反向速率". */
                    if (gimbal_cmd_recv.chassis_rotate_wz != 0.0f &&
                        yaw_hold_rate > -GIMBAL_YAW_HOLD_LIMIT && yaw_hold_rate < GIMBAL_YAW_HOLD_LIMIT)
                    {
                        if (yaw_ff_settle_ms < 1000)
                            yaw_ff_settle_ms++;
                        else
                        {
                            yaw_ff_learned += GIMBAL_YAW_FF_LEARN_ALPHA * (yaw_hold_rate - yaw_ff_learned);
                            if (yaw_ff_learned > GIMBAL_YAW_SPIN_FF_MAX || yaw_ff_learned < -GIMBAL_YAW_SPIN_FF_MAX)
                                yaw_ff_learned = 0.0f; // 学到明显不合理的值: 丢弃, 回到初值逻辑
                            else
                            {
                                yaw_ff_learned_valid = 1;
                                if (++yaw_ff_log_ms >= 1000) // 1Hz打印: 现场可据此确认兜底常数(不必读内存)
                                {
                                    yaw_ff_log_ms = 0;
                                    LOGINFO("[gimbal] spin FF learned = %d mdeg/s", (int)(yaw_ff_learned * 1000.0f));
                                }
                            }
                        }
                    }
                    else
                        yaw_ff_settle_ms = 0;
#endif
                }
            }
#endif // GIMBAL_YAW_IMU_HOLD
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
            if (!yaw_hold_active)
            {
                /* 开环兜底: IMU没有世界基准了, 用标定常数让大yaw反向转(非小陀螺档该通道为0, 不影响别的档).
                 * 优先用自学到的实测值; 没学过才用 robot_def.h 的初值(初值只保证量级, 符号按约定给出). */
                if (yaw_ff_learned_valid)
                    yaw_spd_ref += yaw_ff_learned;
                else
                    yaw_spd_ref -= gimbal_cmd_recv.chassis_rotate_wz;
            }
#endif
            float yaw_spd_lim = GIMBAL_YAW_SPEED_LIMIT;    // °/s
            if (!(yaw_spd_ref <= yaw_spd_lim))  // 同时兜住 NaN
                yaw_spd_ref = yaw_spd_lim;
            if (!(yaw_spd_ref >= -yaw_spd_lim))
                yaw_spd_ref = -yaw_spd_lim;
            /* 摩擦前馈: 按指令方向给一个恒定电流量(与PID增益无关), 让静摩擦死区里的"起步"有确定力矩。
             * 指令接近0时前馈也归0, 避免静止时一直往外顶(否则松杆后会被前馈慢慢推走)。阈值单位是 °/s。 */
            if (yaw_spd_ref > 3.0f)
                yaw_current_ff = GIMBAL_YAW_FRICTION_FF;
            else if (yaw_spd_ref < -3.0f)
                yaw_current_ff = -GIMBAL_YAW_FRICTION_FF;
            else
                yaw_current_ff = 0.0f;

            yaw_ref_valid = 0; // 速度环不用基准对齐; 清掉以免切回位置环时用到旧值
            DJIMotorSetRef(yaw_motor, yaw_spd_ref);
#endif // GIMBAL_YAW_POSITION_LOOP
        }
        // 小yaw采用编码器反馈
        DJIMotorChangeFeed(small_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(small_yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(small_yaw_motor, gimbal_cmd_recv.small_yaw);
        
        // pitch采用IMU反馈
        DMMotorEnable(pitch_motor);
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
       
        break;
    // 使用编码器的反馈初始化电机
    case GIMBAL_ENCODER_MODE: 
        break;
    default:
        break;
    }

    // DMMotorSetSpeedFeedForward(pitch_motor, gimbal_cmd_recv.pitch_omega_feedforword);
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.small_yaw_motor_single_round_angle = small_yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}