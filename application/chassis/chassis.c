/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
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

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define ABS(x) (((x)>0)?(x):(-(x)))

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DJIMotorInstance *servo_lf, *servo_rf, *servo_lb, *servo_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘电机速度解算后的临时输出,待进行限幅
static float angle_lf, angle_rf, angle_lb, angle_rb;      // 底盘舵机角度解算后的临时输出,待进行限幅

void ChassisInit()
{
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
    chassis_M3508_motor_config.can_init_config.tx_id = 1;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_M3508_motor_config);

    chassis_M3508_motor_config.can_init_config.tx_id = 2;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_M3508_motor_config);

    chassis_M3508_motor_config.can_init_config.tx_id = 4;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_M3508_motor_config);

    chassis_M3508_motor_config.can_init_config.tx_id = 3;
    chassis_M3508_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_M3508_motor_config);


    // GM6020电机的初始化(舵机电机)
     Motor_Init_Config_s chassis_GM6020_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 20000,
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

    chassis_GM6020_motor_config.can_init_config.tx_id = 1;
    servo_lf = DJIMotorInit(&chassis_GM6020_motor_config);
    chassis_GM6020_motor_config.can_init_config.tx_id = 6;
    servo_rf = DJIMotorInit(&chassis_GM6020_motor_config);
    chassis_GM6020_motor_config.can_init_config.tx_id = 4;
    servo_rb = DJIMotorInit(&chassis_GM6020_motor_config);
    chassis_GM6020_motor_config.can_init_config.tx_id = 5;
    servo_lb = DJIMotorInit(&chassis_GM6020_motor_config);

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

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
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
int8_t steering_dirt[4] = {1, -1, 1, -1};
// 改为接收指针，写回调用者的变量
void chassis_vector_to_M3508_wheel_speed(float vx_set, float vy_set, float wz_set,
                                         float *vt_lf, float *vt_rf, float *vt_lb, float *vt_rb)
{
    // float wheel_rpm_ratio;
    // wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * REDUCTION_RATIO_WHEEL * 1000;

    *vt_lf = (float)(steering_dirt[0] * sqrtf(powf(vy_set + wz_set * RADIUS_WHEEL * 0.707107f,2)
                       + powf(vx_set - wz_set * RADIUS_WHEEL * 0.707107f,2)
                       ));
    *vt_rf = (float)(steering_dirt[1] * sqrtf(powf(vy_set - wz_set * RADIUS_WHEEL * 0.707107f,2)
                       + powf(vx_set - wz_set * RADIUS_WHEEL * 0.707107f,2)
                       )); 
    *vt_rb = (float)(steering_dirt[2] * sqrtf(powf(vy_set - wz_set * RADIUS_WHEEL * 0.707107f,2)
                       + powf(vx_set + wz_set * RADIUS_WHEEL * 0.707107f,2)
                       ));
    *vt_lb = (float)(steering_dirt[3] * sqrtf(powf(vy_set + wz_set * RADIUS_WHEEL * 0.707107f,2)
                       + powf(vx_set + wz_set * RADIUS_WHEEL * 0.707107f,2)
                       ));
}

double atan_angle[4];
/*
 * 将角度差归一化到 (-180, 180]
 */
static float Normalize180(float a)
{
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
    // 处理大的负数：快速调整到接近[0, 360)范围
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

/*
 * 计算目标多圈角度（不限制在0~360），并根据最小转角原则决定是否翻转驱动方向
 */
void chassis_vector_to_GM6020_wheel_angle_ecd(float vx_set, float vy_set, float wz_set, float *angle_lf, float *angle_rf, float *angle_lb, float *angle_rb)
{
    // GM6020目标角度计算
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
    {
        //由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2)
        atan_angle[0] = atan2((vx_set - wz_set * RADIUS_WHEEL * 0.707107f),(vy_set - wz_set * RADIUS_WHEEL * 0.707107f)) * 180.0f / PI;		
        atan_angle[1] = atan2((vx_set - wz_set * RADIUS_WHEEL * 0.707107f),(vy_set + wz_set * RADIUS_WHEEL * 0.707107f)) * 180.0f / PI;
        atan_angle[2] = atan2((vx_set + wz_set * RADIUS_WHEEL * 0.707107f),(vy_set - wz_set * RADIUS_WHEEL * 0.707107f)) * 180.0f / PI;
        atan_angle[3] = atan2((vx_set + wz_set * RADIUS_WHEEL * 0.707107f),(vy_set + wz_set * RADIUS_WHEEL * 0.707107f)) * 180.0f / PI;
    }  
    // 目标单圈角度（0..360) 基于偏置+atan结果
    float desired_single[4];
    desired_single[0] = GM6020_LF_ANGLE + (float)atan_angle[0];
    desired_single[1] = GM6020_RF_ANGLE + (float)atan_angle[1];
    desired_single[2] = GM6020_LB_ANGLE + (float)atan_angle[2];
    desired_single[3] = GM6020_RB_ANGLE + (float)atan_angle[3];

    // 使用多圈全角（total_angle）作为基准生成无界目标角度，避免限制在0~360
    DJIMotorInstance *servos[4] = {servo_lf, servo_rf, servo_lb, servo_rb};
    float *targets[4] = {angle_lf, angle_rf, angle_lb, angle_rb};
    int8_t base_dir[4] = {1, -1, 1, -1}; // 对应每个舵轮的默认驱动方向

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
    chassis_vector_to_M3508_wheel_speed(chassis_vx, chassis_vy, chassis_cmd_recv.wz, &vt_lf, &vt_rf, &vt_rb, &vt_lb);
    chassis_vector_to_GM6020_wheel_angle_ecd(chassis_vx, chassis_vy, chassis_cmd_recv.wz, &angle_lf, &angle_rf, &angle_lb, &angle_rb);
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
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    // SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);//设置功率限制
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(servo_lf);
        DJIMotorStop(servo_rf);
        DJIMotorStop(servo_lb);
        DJIMotorStop(servo_rb);
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
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 100;
        break;
    default:
        break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

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
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}