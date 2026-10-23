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

/* 小YAW 初始化标志已废弃: 原实现用它做"等小yaw对齐再使能大yaw", 但判据是恒真式且大yaw并不依赖
 * 小yaw(大yaw 用自身编码器做速度环). 现在小yaw 的进入条件改为"有反馈 + 使能对齐"(见GIMBAL_GYRO_MODE).*/
/* 大yaw指令基准对齐(见GIMBAL_GYRO_MODE分支): 停机期间摇杆仍会累积指令,使能瞬间必须把基准
 * 挪到当前反馈,否则会出现几十~几百度的阶跃误差 ⇒ 电机满速冲过去(现场"一切到左下就疯狂旋转") */
static uint8_t yaw_ref_valid = 0;
#ifdef GIMBAL_YAW_POSITION_LOOP
static float yaw_ref_bias = 0.0f;
#endif
/* 大yaw摩擦(库仑/静摩擦)前馈: 由速度环分支每拍按指令方向更新, 通过 current_feedforward_ptr
 * 直接叠加在速度环输出上(与PID增益无关) ⇒ 既能压过静摩擦死区, 又不必把Kp提到自激的程度 */
static float yaw_current_ff = 0.0f;
static float yaw_spd_ref_log = 0.0f; // 仅用于 1Hz 观测日志
static float gimbal_log_ms = 0.0f;   // 观测日志限频
/* 摩擦前馈的"起步脉冲"状态: 只在指令越过 FF_ON 的那一下点火, 脉冲结束即撤(不留稳态推力) */
static int8_t yaw_ff_dir = 0;
static uint16_t yaw_ff_cnt = 0;
static uint8_t yaw_ff_armed = 1;

/* 小yaw 指令整形状态: 使能瞬间把指令基准对齐到当前反馈(与大yaw 的 yaw_ref_valid 同一套路),
 * 否则"绝对编码器角语义的指令"会被拿去和"上电相对角"的反馈比较, 误差可达几百度量级 ⇒ 甩头 */
static uint8_t small_yaw_ref_valid = 0;
static float small_yaw_ref_bias = 0.0f;
static float small_yaw_cmd_limited = 0.0f;
static float small_yaw_zero_deg = 0.0f;   // 使能时的小yaw 位置(行程限位的基准)
static float small_yaw_warn_ms = 0.0f;    // 掉线告警限频

/* pitch 指令整形状态: 使能瞬间对齐 + 斜率限制(避免 -30° 阶跃直接砸到电机上) */
static uint8_t pitch_ref_valid = 0;
static float pitch_cmd_limited = 0.0f;
static float pitch_zero_deg = 0.0f; // 上电时 pitch 编码器角(约定此时枪管水平)

#ifdef GIMBAL_YAW_IMU_HOLD
/* 大yaw世界角(IMU)保持 —— 详见 robot_def.h 中 GIMBAL_YAW_IMU_HOLD 的说明.
 * yaw_hold_target 是"要钉住的世界指向(°)", 摇杆接管时不断刷新为当前指向, 松杆后由P环拉回. */
static float yaw_hold_target = 0.0f;
static uint8_t yaw_hold_valid = 0; // 目标角是否已建立(停机/IMU异常时清零, 使能瞬间重新对齐)
/* 松杆后的一次性冻结状态: 原实现用 |speed|>COAST 的无状态判据, 反向修正一旦超过阈值
 * 就会重新进入"跟随", 把已经越过目标的位置又当成新目标 ⇒ 停止点附近来回游走.
 * 现在只在"松杆 + 转速连续低于阈值"时冻结一次, 之后只有摇杆重新接管才解冻. */
static uint8_t yaw_hold_frozen = 0;
static uint8_t yaw_hold_stop_cnt = 0; // 连续低于冻结阈值的拍数

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
                .Kp = 16, // 10→16: 现场反馈"希望硬一点"(松杆后跟得更紧/抗扰更硬); 若出现嗡嗡抖, 退回 12
                .Ki = 0.15, // 0→0.15: 消掉稳态速度误差(纯P在低速段需要靠误差维持电流 ⇒ 手感软、易爬行)
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
            .DeadBand = 0, // 0.1→0: 死区内 Output 会被清 0, 有负载时表现为间歇抖动
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 100,

            .MaxOut = GIMBAL_SMALL_YAW_RATE_LIMIT, // 500→150: 角度环输出是"速率指令"(°/s), 500°/s 会甩头
        },
        .speed_PID = {
            .Kp = 50,  // 50
            .Ki = 20,  // 150→20: 撞限位时积分饱和会把堵转电流攒满, 脱开瞬间猛冲
            .Kd = 0,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3000,
            .MaxOut = 8000, // 20000→8000: 先保守, 确认不撞限位后再放开
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
#ifdef GIMBAL_PITCH_FEEDBACK_MOTOR
            /* 云台板装在小yaw 转台上 ⇒ IMU 不随 pitch 关节转, 必须用电机自身编码器.
             * 驱动层会把 MOTOR_FEED 的角度取 measure.total_angle(deg)、速度取 velocity(rad/s)→deg/s,
             * 与 gimbal_cmd_recv.pitch(deg)/角度环输出(deg/s) 同量纲. */
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
#else
            .angle_feedback_source = OTHER_FEED, // 仅当板子装在随 pitch 转的部件上才可用
            .speed_feedback_source = OTHER_FEED,
#endif
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            /* 换反馈源后这个符号必须重新确认(只翻最终力矩, 两级环共用):
             * 手转枪管上行看 total_angle 是否增大 → 开环给 +1.5N·m 看实际转向 → 两步同向才可闭环. */
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .controller_param_init_config = {
            .DM_MIT_Kp = 0,
            .DM_MIT_Kd = 0.1, // 电机自带的速度阻尼(N·m/(rad/s)), 先靠它压高频
            .angle_PID = {
                /* 角度环: 输入/输出都是 deg → deg/s, MaxOut 当**速率上限**用(不再是 1000 这种无意义的数).
                 * Ki=0: 稳态重力矩交给 GIMBAL_PITCH_GRAVITY_FF, 不再用积分去"攒"(原 Ki=50/限幅1e5 会把
                 * 堵转/重力攒成 windup, 配合死区清输出就是现场的"上下持续抖动"). */
                .Kp = 25,
                .Ki = 0,
                .Kd = 0.01,
                .MaxOut = GIMBAL_PITCH_RATE_LIMIT,
                .CoefA = 0.3,
                .CoefB = 0.2,
                .IntegralLimit = 100,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
            },
            .speed_PID = {
                /* 速度环: deg/s → N·m. 原 Kp=0.13 是按 rad/s 反馈凑出来的等效增益,
                 * 换成 deg/s 反馈后必须同步放大(57.3×) ⇒ 1.5 左右; MaxOut 收紧到真实可用力矩. */
                .Kp = 1.5,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 8, // N·m: 不要用 DM_T_MAX(±54)当力矩上限, 那是线协议量程
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
        DMMotorSetCurrentFeedForward(pitch_motor, 0.0f); // 清掉重力前馈, 避免停机还挂着力矩
        /* 停机时复位三个环的计算区(只清 Iout/Dout/Err 等, 不动 Kp/Ki 配置):
         * 否则从急停/掉线恢复时会带着旧积分与旧微分冲一下, 现场表现为"一切档云台先猛甩一下". */
        PIDReset(&yaw_motor->motor_controller.speed_PID);
        PIDReset(&small_yaw_motor->motor_controller.angle_PID);
        PIDReset(&small_yaw_motor->motor_controller.speed_PID);
        PIDReset(&pitch_motor->angle_PID);
        PIDReset(&pitch_motor->speed_PID);
        yaw_ref_valid = 0;   // 停机: 下次使能时重新把指令基准对齐到当前反馈
        yaw_current_ff = 0.0f; // 停机时清掉摩擦前馈
        yaw_ff_cnt = 0;
        yaw_ff_dir = 0;
        yaw_ff_armed = 1;
        small_yaw_ref_valid = 0; // 下次使能重新对齐小yaw 指令基准
        pitch_ref_valid = 0;     // 下次使能重新对齐 pitch 指令基准
        /* 停机期间让"观测日志"里的 ref 跟着 fb 走, 避免看到 ref=0 误以为环在出力 */
        small_yaw_cmd_limited = small_yaw_motor->measure.total_angle;
        pitch_cmd_limited = pitch_motor->measure.total_angle;
#ifdef GIMBAL_YAW_IMU_HOLD
        yaw_hold_valid = 0;    // 停机: 下次使能时把世界角保持目标重新对齐到当前指向
        yaw_hold_frozen = 0;
        yaw_hold_stop_cnt = 0;
#endif
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
        yaw_ff_settle_ms = 0;  // 停机: 重新计数"小陀螺稳定时长"(学到的兜底常数保留)
#endif
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE:
        /* ===================== 小yaw =====================
         * 原实现的三处问题(现场已复现):
         *  ① "等小yaw对齐再使能大yaw"的判据 `ecd < A+5 || ecd > A-5` 是**恒真式**(任何值都成立),
         *     保护形同虚设;
         *  ② 没有使能对齐/软件限位/速率限制, 而指令是"绝对编码器角-360"语义、反馈却是
         *     measure.total_angle(上电相对角) ⇒ 误差可达几十~350°, 角度环 MaxOut=500°/s 直接满速
         *     甩到机械限位, 堵转大电流 ⇒ 掉电/电调保护 ⇒ 现场"甩头后卡死";
         *  ③ 掉线时仍拿陈旧反馈闭环。
         * 现在: 有反馈才进闭环 + 使能对齐 + ±SMALL_YAW_MAN_ANGLE 限位 + 斜率限制。
         * 注: 大yaw 不再等小yaw"对齐"——大yaw 用自身编码器做速度环, 不依赖小yaw。 */
        if (DaemonIsOnline(small_yaw_motor->daemon))
        {
            DJIMotorEnable(small_yaw_motor);
            DJIMotorChangeFeed(small_yaw_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(small_yaw_motor, SPEED_LOOP, MOTOR_FEED);
            float sy_fb = small_yaw_motor->measure.total_angle; // 上电相对角(deg)
            if (!small_yaw_ref_valid) // 使能瞬间: 指令=当前反馈 ⇒ 不出力, 无阶跃
            {
                small_yaw_zero_deg = sy_fb;
                small_yaw_ref_bias = sy_fb - gimbal_cmd_recv.small_yaw;
                small_yaw_cmd_limited = sy_fb;
                small_yaw_ref_valid = 1;
                LOGINFO("[gimbal] small yaw ref aligned: cmd=%d mdeg, feedback=%d mdeg, bias=%d mdeg",
                        (int)(gimbal_cmd_recv.small_yaw * 1000.0f), (int)(sy_fb * 1000.0f),
                        (int)(small_yaw_ref_bias * 1000.0f));
            }
            float sy_ref = gimbal_cmd_recv.small_yaw + small_yaw_ref_bias;
            /* 行程限位: 相对使能位 ±SMALL_YAW_MAN_ANGLE(以"上电时机械处于对齐位"为前提) */
            if (sy_ref > small_yaw_zero_deg + SMALL_YAW_MAN_ANGLE)
                sy_ref = small_yaw_zero_deg + SMALL_YAW_MAN_ANGLE;
            else if (sy_ref < small_yaw_zero_deg - SMALL_YAW_MAN_ANGLE)
                sy_ref = small_yaw_zero_deg - SMALL_YAW_MAN_ANGLE;
            /* 斜率限制: 200Hz 任务 ⇒ 每拍最多 GIMBAL_SMALL_YAW_CMD_SLEW/200 度 */
            float sy_step = GIMBAL_SMALL_YAW_CMD_SLEW / ROBOT_TASK_HZ;
            if (sy_ref > small_yaw_cmd_limited + sy_step)
                small_yaw_cmd_limited += sy_step;
            else if (sy_ref < small_yaw_cmd_limited - sy_step)
                small_yaw_cmd_limited -= sy_step;
            else
                small_yaw_cmd_limited = sy_ref;
            DJIMotorSetRef(small_yaw_motor, small_yaw_cmd_limited);
        }
        else
        {
            DJIMotorStop(small_yaw_motor); // 无反馈不闭环(DJI驱动F3旁路之外再显式停机)
            if (small_yaw_ref_valid)
            {
                small_yaw_ref_valid = 0; // 恢复后重新对齐
                if (DWT_GetTimeline_ms() - small_yaw_warn_ms > 1000.0f) // 1Hz限频
                {
                    small_yaw_warn_ms = DWT_GetTimeline_ms();
                    LOGWARNING("[gimbal] small yaw feedback lost -> small yaw stopped");
                }
            }
        }

        /* ===================== 大yaw ===================== */
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
                yaw_hold_frozen = 0;
                yaw_hold_stop_cnt = 0;
            }
            else
            {
                if (!yaw_hold_valid) // 使能/恢复瞬间把目标对齐当前指向, 杜绝阶跃(与位置环同理)
                {
                    yaw_hold_target = yaw_imu;
                    yaw_hold_valid = 1;
                    yaw_hold_frozen = 0;
                    yaw_hold_stop_cnt = 0;
                }
                /* 让位判据必须用**原始摇杆指令**(gimbal_cmd_recv.yaw_rate), 不能用已叠加保持环输出的
                 * yaw_spd_ref: 否则保持环会把自己顶过阈值 ⇒ 反复让位/接管 ⇒ 抖动(原实现踩的就是这个坑). */
                uint8_t stick_active = (uint8_t)(fabsf(gimbal_cmd_recv.yaw_rate) > GIMBAL_YAW_HOLD_STICK_TH);
                /* 小陀螺时电机相对底盘长期有转速, 不能用"转速降到 0"判断停住; 该标志同时供下面的
                 * 保持环/前馈学习使用. */
                uint8_t yaw_in_spin = (uint8_t)(gimbal_cmd_recv.chassis_rotate_wz != 0.0f);

                if (stick_active)
                {
                    /* 人在推杆: 目标跟随当前指向, 并退出冻结(下次松杆重新走一遍"减速->冻结") */
                    yaw_hold_target = yaw_imu;
                    yaw_hold_frozen = 0;
                    yaw_hold_stop_cnt = 0;
                }
                else if (!yaw_hold_frozen)
                {
                    if (yaw_in_spin)
                    {
                        /* 小陀螺: 松杆即冻结世界角目标, 由保持环持续抵消底盘自转.
                         * 此时电机相对底盘的转速长期非零, 不能等它降到 0 再冻结. */
                        yaw_hold_target = yaw_imu;
                        yaw_hold_frozen = 1;
                        yaw_hold_stop_cnt = 0;
                    }
                    else
                    {
                        /* 松杆后先让目标跟随, 等轴真正慢下来再一次性冻结.
                         * 不再用"|speed|>阈值就永远跟随"的无状态判据: 反向修正一旦超过阈值
                         * 会重新进入跟随, 目标被拉回、P 修正被撤销 ⇒ 停止点附近来回游走. */
                        yaw_hold_target = yaw_imu;
                        if (fabsf(yaw_motor->measure.speed_aps) < GIMBAL_YAW_HOLD_COAST_TH)
                        {
                            if (++yaw_hold_stop_cnt >= GIMBAL_YAW_HOLD_FREEZE_TICKS)
                            {
                                yaw_hold_frozen = 1;
                                yaw_hold_stop_cnt = 0;
                                yaw_hold_target = yaw_imu; // 在真正停住处冻结
                            }
                        }
                        else
                        {
                            yaw_hold_stop_cnt = 0;
                        }
                    }
                }

                if (yaw_hold_frozen)
                {
                    float yaw_hold_p = GIMBAL_YAW_HOLD_KP * YawWrap180(yaw_hold_target - yaw_imu);
                    yaw_hold_p *= GIMBAL_YAW_HOLD_SIGN; // 符号: 修正 IMU 安装方向后若变成正反馈, 改宏为 -1
                    if (!(yaw_hold_p <= GIMBAL_YAW_HOLD_LIMIT)) // NaN安全写法(与下方限幅一致)
                        yaw_hold_p = GIMBAL_YAW_HOLD_LIMIT;
                    if (!(yaw_hold_p >= -GIMBAL_YAW_HOLD_LIMIT))
                        yaw_hold_p = -GIMBAL_YAW_HOLD_LIMIT;
                    float yaw_hold_rate = yaw_hold_p; // 本拍加给速度环的补偿速率
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
                    float spin_sign = (gimbal_cmd_recv.chassis_rotate_wz >= 0.0f) ? 1.0f : -1.0f;
                    /* 小陀螺: 把学到的"抵消自转所需速率"作为**前馈**, P 环只修残差.
                     * 纯 P 在 ω=58°/s、Kp=5 时稳态偏差 ≈11.6° ⇒ 枪口一直歪着, 推前进也沿着歪的方向走. */
                    if (yaw_in_spin && yaw_ff_learned_valid)
                        yaw_hold_rate += yaw_ff_learned * spin_sign;
#endif
                    yaw_spd_ref += yaw_hold_rate;
#if defined(GIMBAL_YAW_FF_ON_IMU_LOST) || defined(GIMBAL_YAW_FF_FORCE)
                    yaw_hold_active = 1;
                    /* 兜底常数在线自学: 只在"正方向小陀螺 + 松杆 + 保持环P未饱和"且稳定 1s 之后进行.
                     * 学的对象是**总补偿量**(前馈+P): 收敛后 P→0 ⇒ 学到的正好是自转速率, 与兜底同量纲. */
                    if (yaw_in_spin && spin_sign > 0.0f &&
                        yaw_hold_p > -GIMBAL_YAW_HOLD_LIMIT && yaw_hold_p < GIMBAL_YAW_HOLD_LIMIT)
                    {
                        if (yaw_ff_settle_ms < 1000) // 按 ms 计(200Hz ⇒ 每拍 5ms), 原来按拍数累加 ⇒ 实际等 5s
                            yaw_ff_settle_ms = (uint16_t)(yaw_ff_settle_ms + (uint16_t)(1000.0f / ROBOT_TASK_HZ));
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
            /* 兜底只在**小陀螺档**生效: yaw_ff_learned 是"抵消底盘自转"的速率, 学到的值不会自动清零;
             * 若普通档也叠加, 会把正常 yaw 持续往一个方向推(松杆减速段尤其明显). */
            if (!yaw_hold_active && gimbal_cmd_recv.chassis_rotate_wz != 0.0f)
            {
                /* 开环兜底: IMU没有世界基准了, 用标定常数让大yaw反向转.
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
            /* 摩擦前馈(库仑补偿)改成**起步脉冲**: 只在"开始动/换向"的那一下给一段固定时长(默认150ms)的电流,
             * 之后交给速度环(带 Ki, 会自己把维持转动的力矩攒出来).
             * 为什么不再用"恒定给到退出门限": 状态机是粘滞的 —— 松杆后指令落进 (OFF, ON) 区间时前馈仍保持
             * 上一次方向继续顶, 轴被推着走 ⇒ 现场"大yaw 停转之后还小范围漂移/游走", 直到反方向误差把
             * 状态翻过去, 于是变成一来一回的慢速游走. 脉冲式没有稳态推力, 不漂. */
            {
                /* 摩擦前馈只跟随**摇杆指令**, 不跟随保持环的停止修正:
                 * 否则轴刚越过目标、保持环反向修正超过 FF_ON 时也会点火 ±1200/150ms,
                 * 把"拉回"变成一次反向冲过头, 停止点来回游走. */
                float yaw_pilot_ref = gimbal_cmd_recv.yaw_rate;
                float yaw_ref_abs = fabsf(yaw_pilot_ref);
                if (yaw_ref_abs < GIMBAL_YAW_FF_OFF) // 指令已接近 0: 撤火并重新武装
                {
                    yaw_ff_cnt = 0;
                    yaw_ff_dir = 0;
                    yaw_ff_armed = 1;
                }
                else if (yaw_ref_abs > GIMBAL_YAW_FF_ON)
                {
                    int8_t ff_dir = (yaw_pilot_ref > 0.0f) ? 1 : -1;
                    if (yaw_ff_armed || ff_dir != yaw_ff_dir) // 新的一次起步(或换向)才点火
                    {
                        yaw_ff_dir = ff_dir;
                        yaw_ff_cnt = GIMBAL_YAW_FF_PULSE_TICKS;
                        yaw_ff_armed = 0;
                    }
                    else if (yaw_ff_cnt)
                        yaw_ff_cnt--; // 脉冲到时自动撤, 不留稳态推力
                }
                yaw_current_ff = yaw_ff_cnt ? (float)yaw_ff_dir * GIMBAL_YAW_FRICTION_FF : 0.0f;
            }

            yaw_ref_valid = 0; // 速度环不用基准对齐; 清掉以免切回位置环时用到旧值
            DJIMotorSetRef(yaw_motor, yaw_spd_ref);
            yaw_spd_ref_log = yaw_spd_ref;
#endif // GIMBAL_YAW_POSITION_LOOP

        /* ===================== pitch(DM4310) =====================
         * 云台板装在小yaw 转台上 ⇒ IMU 不随 pitch 关节转, IMU.Pitch 不是枪管俯仰角,
         * 所以反馈改用电机自身编码器(驱动层已按 MOTOR_FEED 取 measure.total_angle / velocity·57.3).
         * 使能对齐 + 斜率限制消除 -30° 阶跃; 稳态力矩由重力前馈承担(角度环 Ki=0). */
        /* T_Mos>1 作为"确实收到过有效反馈"的判据: 电机没插/没上电时 id/state/position 全是 0,
         * 此时若把 0 当作"上电零位"锁存, 等电机插回来就会瞬间产生上百度的误差 ⇒ 猛冲. */
        if (DaemonIsOnline(pitch_motor->motor_daemon) && pitch_motor->measure.T_Mos > 1.0f)
        {
            DMMotorEnable(pitch_motor);
            float pitch_fb = pitch_motor->measure.total_angle; // deg(电机侧)
            if (!pitch_ref_valid)
            {
                pitch_zero_deg = pitch_fb;
                pitch_cmd_limited = pitch_fb;
                pitch_ref_valid = 1;
                LOGINFO("[gimbal] pitch ref aligned: cmd=%d mdeg, feedback=%d mdeg",
                        (int)(gimbal_cmd_recv.pitch * 1000.0f), (int)(pitch_fb * 1000.0f));
            }
            float pitch_ref = pitch_zero_deg + gimbal_cmd_recv.pitch; // 以上电姿态为 0(约定此时枪管水平)
            float p_step = GIMBAL_PITCH_CMD_SLEW / ROBOT_TASK_HZ;
            if (pitch_ref > pitch_cmd_limited + p_step)
                pitch_cmd_limited += p_step;
            else if (pitch_ref < pitch_cmd_limited - p_step)
                pitch_cmd_limited -= p_step;
            else
                pitch_cmd_limited = pitch_ref;
            DMMotorSetRef(pitch_motor, pitch_cmd_limited);
#ifdef GIMBAL_PITCH_GRAVITY_FF
            /* 重力前馈 τ = DIR·A·cos(相对水平角): 用**反馈角**而不是指令角, 保证前馈跟着真实姿态走.
             * DIR 一次定符号(装反表现为"越补越往下掉", 改 GIMBAL_PITCH_GRAVITY_DIR). */
            float pitch_rel = pitch_fb - pitch_zero_deg;
            float tau_g = GIMBAL_PITCH_GRAVITY_A * cosf((pitch_rel - GIMBAL_PITCH_ZERO_DEG) * DEGREE_2_RAD);
            DMMotorSetCurrentFeedForward(pitch_motor, GIMBAL_PITCH_GRAVITY_DIR * tau_g);
#endif
        }
        else
        {
            DMMotorStop(pitch_motor); // 反馈掉线: 停机, 不要拿陈旧反馈闭环
            pitch_ref_valid = 0;
            DMMotorSetCurrentFeedForward(pitch_motor, 0.0f);
        }

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

    /* 1Hz 现场观测日志(限频, 否则 RTT 带宽不够): 三个轴的"指令 / 反馈 / 输出"一眼可见,
     * 直接对应验收标准(静止误差、是否饱和、是否撞限位), 不必再读内存. */
    if (DWT_GetTimeline_ms() - gimbal_log_ms >= 1000.0f)
    {
        gimbal_log_ms = DWT_GetTimeline_ms();
        LOGINFO("[gimbal] pitch ref=%d fb=%d ff=%d mNm | yaw spd_ref=%d fb=%d ff=%d | sy ref=%d fb=%d",
                (int)(pitch_cmd_limited * 1000.0f), (int)(pitch_motor->measure.total_angle * 1000.0f),
                (int)(pitch_motor->Current_feedforward * 1000.0f),
                (int)(yaw_spd_ref_log * 1000.0f), (int)(yaw_motor->measure.speed_aps * 1000.0f),
                (int)(yaw_current_ff),
                (int)(small_yaw_cmd_limited * 1000.0f), (int)(small_yaw_motor->measure.total_angle * 1000.0f));
    }

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.small_yaw_motor_single_round_angle = small_yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}