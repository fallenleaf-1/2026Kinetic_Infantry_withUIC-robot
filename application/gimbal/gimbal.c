#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "dmmotor.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
DJIMotorInstance *small_yaw_motor;  // 在INTtask中访问，计算大Yaw反馈
static DMMotorInstance *pitch_motor ;          // 声明DM4310电机实例

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU
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
                .Kp = 8, // 8
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
            .other_angle_feedback_ptr = &gimba_IMU_data->Big_Yaw_Feed,
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    
    // 小YAW    C板在小YAW上，角度直接用IMU反馈
    Motor_Init_Config_s small_yaw_config = {
    .can_init_config = {
        .can_handle = &hcan2,
        .tx_id = 3,
    },
    .controller_param_init_config = {
        .angle_PID = {
            .Kp = 8, // 8
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
        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    },
    .motor_type = GM6020};
    
    // PITCH
     Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1, // 1--->2 O.o
            .tx_id = 0x04,
            .rx_id = 0x14,
        },
        .controller_param_init_config = {
            .DM_MIT_Kp = 0,
            .DM_MIT_Kd = 0.1,
            .angle_PID = {
                .Kp = 50,//0.31
                .Ki = 50,//1
                .Kd = 0,//0.001
                .MaxOut = 1000,
                .CoefA = 0.3,
                .CoefB = 0.2,
                .IntegralLimit = 100000,
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit,
            },
            .speed_PID = {
                .Kp = 0.13,//0.12
                .Ki = 0,//0
                .Kd = 0.001,//0.01
                .MaxOut = 53,//53
            },
            //.other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            //.other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            //.other_angle_feedback_ptr = &gimbal_IMU_data_ch->YawTotalAngle,
            //.other_speed_feedback_ptr = &gimbal_IMU_data_ch->gyr[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            // .outer_loop_type = SPEED_LOOP,
            // .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = DMMotor};

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // gimbal_6020_conf.can_init_config.tx_id = 1;
    // motor_yaw = DJIMotorInit(&gimbal_6020_conf);

    // gimbal_3508_conf.can_init_config.tx_id = 2;
    // motor_pitch = DJIMotorInit(&gimbal_3508_conf);

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

    DMMotorSetSpeedFeedForward(pitch_motor, gimbal_cmd_recv.pitch_omega_feedforword);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(small_yaw_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(small_yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(small_yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(small_yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈

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

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}