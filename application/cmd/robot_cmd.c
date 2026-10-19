// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "buzzer.h"
#include "solvetrajectory.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <string.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 大YAW对齐时的角度,0-360
#define SMALL_YAW_ALIGN_ANGLE (SMALL_YAW_FORWORD_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 小YAW对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#ifdef RC_RECEIVER_ON_CHASSIS
/* 遥控器接收已挪到底盘板: 遥控器数据随底盘板上行帧(chassis_fetch_data.rc)送达本板,无需新增CAN实例 */
static RC_ctrl_t rc_remote_ctrl[2]; // 本地镜像,保持既有代码继续以rc_data[TEMP]访问(与remote_control.c一致使用2元素数组)
#endif                              // RC_RECEIVER_ON_CHASSIS
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD) // 底盘板构建不接收底盘回传,条件编译避免未使用告警
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
#endif

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *solvetrajectory_need_pub;   // 弹道解算所需消息发布者
static Subscriber_t *solvetrajectory_feed_sub;  // 弹道解算反馈信息订阅者
static SolveTrajectory_Recv_s solvetrajectory_need_data; // 弹道解算所需信息
static SolveTrajectory_Send_s solvetrajectory_fetch_data; // 弹道解算获取的反馈信息


static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

BuzzzerInstance* robocmd_alarm;
/** 
 * @brief Alarm实例化
 *
 */
void AlarmInit()
{
    Buzzer_config_s buzzer_config ={
            .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
            .loudness = 0.4, //设置响度
            .octave = OCTAVE_1, // 设置音阶
    };
    robocmd_alarm = BuzzerRegister(&buzzer_config);

    AlarmSetStatus(robocmd_alarm, ALARM_OFF);
}

void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
#if defined(RC_RECEIVER_ON_CHASSIS) && defined(GIMBAL_BOARD)
    // 遥控器(DBUS)已改接到底盘板,本板不再注册遥控器串口,改用底盘板上行帧转发来的数据(见下方rc_data赋值)
#else
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    solvetrajectory_need_pub = PubRegister("SolveTrajectory_need",sizeof(SolveTrajectory_Recv_s));
    solvetrajectory_feed_sub = SubRegister("SolveTrajectory_feed",sizeof(SolveTrajectory_Send_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#ifdef RC_RECEIVER_ON_CHASSIS
    // 遥控器数据来源改为底盘板随其上行帧转发来的镜像(复用0x311链路,无需新增CAN实例)
    rc_data = rc_remote_ctrl;
#endif // RC_RECEIVER_ON_CHASSIS
#endif // GIMBAL_BOARD

    gimbal_cmd_send.pitch = 0.0f;
    gimbal_cmd_send.small_yaw = YAW_ALIGN_ANGLE -360.0f;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    AlarmInit();
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static float CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float offset_angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                           // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;

    offset_angle = chassis_cmd_send.offset_angle;
    // 保证在不跟随模式下offset_angle为0,确保底盘舵向电机角度解算不受yaw影响
    if(chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW)
    {
        chassis_cmd_send.offset_angle = 0.0f;
    }
    
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;

    offset_angle = chassis_cmd_send.offset_angle;
    // 保证在不跟随模式下offset_angle为0,确保底盘舵向电机角度解算不受yaw影响
    if(chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW)
    {
        chassis_cmd_send.offset_angle = 0.0f;
    }
#endif
    return offset_angle;
}

void GetSolvetrajectory_need_data()
{
    // 视觉数据
    solvetrajectory_need_data.tracking = vision_recv_data->tracking;
    solvetrajectory_need_data.xw = vision_recv_data->x;
    solvetrajectory_need_data.yw = vision_recv_data->y;
    solvetrajectory_need_data.zw = vision_recv_data->z;
    solvetrajectory_need_data.vxw = vision_recv_data->vx;
    solvetrajectory_need_data.vyw = vision_recv_data->vy;
    solvetrajectory_need_data.vzw = vision_recv_data->vz;
    solvetrajectory_need_data.tar_yaw = vision_recv_data->yaw;
    solvetrajectory_need_data.v_yaw = vision_recv_data->v_yaw;
    solvetrajectory_need_data.r1 = vision_recv_data->r1;
    solvetrajectory_need_data.r2 = vision_recv_data->r2;
    solvetrajectory_need_data.dz = vision_recv_data->dz;
    solvetrajectory_need_data.armor_id = vision_recv_data->armor_id;
    solvetrajectory_need_data.armor_num = vision_recv_data->armor_num;
    // 陀螺仪数据
    solvetrajectory_need_data.current_yaw = -gimbal_fetch_data.gimbal_imu_data.Yaw;
    solvetrajectory_need_data.current_pitch = -gimbal_fetch_data.gimbal_imu_data.Pitch;
    // 发射数据
    solvetrajectory_need_data.current_v = 15;   // shoot_fetch_data.bullet_speed;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE; // 底盘小陀螺模式
        /* 小陀螺自转速率通道(见robot_def.h GIMBAL_YAW_FF_ON_IMU_LOST): 云台板用它判断"在不在小陀螺",
         * 并在IMU姿态失效时作为开环兜底的反向补偿量; 其它档位在 RobotCMDTask 里被清零. */
        gimbal_cmd_send.chassis_rotate_wz = GIMBAL_YAW_SPIN_FF_RATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        // 死区判断
        float gimbal_yaw_input = ((float)rc_data[TEMP].rc.rocker_l_ > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_l_ < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_l_ : 0.0f; // _水平方向
        // float gimbal_pitch_input = ((float)rc_data[TEMP].rc.rocker_l1 > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_l1 < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_l1 : 0.0f; // 1竖直方向
        // 系数需要调整
        gimbal_cmd_send.yaw -= GIMBAL_YAW_RC_SCALE * gimbal_yaw_input;
        gimbal_cmd_send.yaw_rate = -GIMBAL_YAW_RC_SCALE * gimbal_yaw_input * ROBOT_TASK_HZ; // 速度环用: °/s
        // gimbal_cmd_send.pitch  += 0.0005f * gimbal_pitch_input;

        // 视觉控制
        static float pitch_vision_input;
        static float small_yaw_vision_input;
        if(vision_recv_data->tracking == true)
        {
            // 符号等待确定
            pitch_vision_input += solvetrajectory_fetch_data.pitch;
            small_yaw_vision_input += solvetrajectory_fetch_data.yaw;
            // 对弹道解算控制量进行限位
            if(small_yaw_vision_input > SMALL_YAW_MAN_ANGLE)
            {
                small_yaw_vision_input = SMALL_YAW_MAN_ANGLE;
            }
            else if (small_yaw_vision_input < SMALL_YAW_MIN_ANGLE)
            {
                small_yaw_vision_input = SMALL_YAW_MIN_ANGLE;
            }

            if(pitch_vision_input > PITCH_MAX_ANGLE)
            {
                pitch_vision_input = PITCH_MAX_ANGLE;
            }
            else if (pitch_vision_input < PITCH_MIN_ANGLE)
            {
                pitch_vision_input = PITCH_MIN_ANGLE;
            }
        }else if (vision_recv_data->tracking == false)
        {
            pitch_vision_input = 0.0f;
            small_yaw_vision_input = 0.0f;
        }
        gimbal_cmd_send.pitch = pitch_vision_input;
        gimbal_cmd_send.small_yaw = SMALL_YAW_ALIGN_ANGLE - 360.0f - small_yaw_vision_input;// 确保输入为0时小yaw大yaw对齐朝前
        
        chassis_cmd_send.vx = ((float)rc_data[TEMP].rc.rocker_r_ > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r_ < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r_ * 5.0f : 0.0f; // _水平方向
        chassis_cmd_send.vy = ((float)rc_data[TEMP].rc.rocker_r1 > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r1 < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r1 * 5.0f : 0.0f; // 1竖直方向

        // if((float)rc_data[TEMP].rc.rocker_r_ > 500)
        // {
        //     chassis_cmd_send.wz = 100;
        // }
        // else if ((float)rc_data[TEMP].rc.rocker_r_ < -500)
        // {
        //     chassis_cmd_send.wz = -100;
        // }
        // else
        //     chassis_cmd_send.wz = 0;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; // 底盘不跟随云台
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        // 死区判断
        float gimbal_yaw_input = ((float)rc_data[TEMP].rc.rocker_l_ > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_l_ < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_l_ : 0.0f; // _水平方向
        float gimbal_pitch_input = ((float)rc_data[TEMP].rc.rocker_l1 > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_l1 < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_l1 : 0.0f; // 1竖直方向
        // 系数需要调整
        
        gimbal_cmd_send.small_yaw = SMALL_YAW_ALIGN_ANGLE - 360.0f;
        gimbal_cmd_send.yaw -= GIMBAL_YAW_RC_SCALE * gimbal_yaw_input;
        gimbal_cmd_send.yaw_rate = -GIMBAL_YAW_RC_SCALE * gimbal_yaw_input * ROBOT_TASK_HZ; // 速度环用: °/s

        static float pitch_rc;
        pitch_rc += 0.0005f * gimbal_pitch_input;
        if(pitch_rc > (PITCH_MAX_ANGLE - PITCH_MIN_ANGLE))
        {
            pitch_rc = PITCH_MAX_ANGLE - PITCH_MIN_ANGLE;
        }
        else if (pitch_rc < 0.0f)
        {
            pitch_rc = 0.0f;
        }
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE + pitch_rc;
        

        // 测试 小yaw
        // static float small_yaw_rc;
        // float gimbal_small_yaw_input = ((float)rc_data[TEMP].rc.rocker_r_ > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r_ < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r_ : 0.0f; // _水平方向
        // small_yaw_rc -= 0.0005f * gimbal_small_yaw_input;
        // if(small_yaw_rc > SMALL_YAW_MAN_ANGLE)
        // {
        //     small_yaw_rc = SMALL_YAW_MAN_ANGLE;
        // }
        // else if (small_yaw_rc < SMALL_YAW_MIN_ANGLE)
        // {
        //     small_yaw_rc = SMALL_YAW_MIN_ANGLE;
        // }
        // gimbal_cmd_send.small_yaw = small_yaw_rc + SMALL_YAW_ALIGN_ANGLE - 360.0f; // 确保输入为0时小yaw大yaw对齐朝前

        // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
        chassis_cmd_send.vx = ((float)rc_data[TEMP].rc.rocker_r_ > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r_ < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r_ * 25.0f : 0.0f; // _水平方向
        chassis_cmd_send.vy = ((float)rc_data[TEMP].rc.rocker_r1 > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r1 < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r1 * 25.0f : 0.0f; // 1竖直方向
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // 底盘跟随云台
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

        // 死区判断
        float gimbal_yaw_input = ((float)rc_data[TEMP].rc.rocker_l_ > GIMBAL_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_l_ < -GIMBAL_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_l_ : 0.0f; // _水平方向
        // 系数需要调整
        gimbal_cmd_send.yaw -= GIMBAL_YAW_RC_SCALE * gimbal_yaw_input;
        gimbal_cmd_send.yaw_rate = -GIMBAL_YAW_RC_SCALE * gimbal_yaw_input * ROBOT_TASK_HZ; // 速度环用: °/s

        // 视觉控制
        static float pitch_vision_input;
        static float small_yaw_vision_input;
        if(vision_recv_data->tracking == true)
        {
            // 符号等待确定
            pitch_vision_input += solvetrajectory_fetch_data.pitch;
            small_yaw_vision_input += solvetrajectory_fetch_data.yaw;
            // 对弹道解算控制量进行限位
            if(small_yaw_vision_input > SMALL_YAW_MAN_ANGLE)
            {
                small_yaw_vision_input = SMALL_YAW_MAN_ANGLE;
            }
            else if (small_yaw_vision_input < SMALL_YAW_MIN_ANGLE)
            {
                small_yaw_vision_input = SMALL_YAW_MIN_ANGLE;
            }

            if(pitch_vision_input > PITCH_MAX_ANGLE)
            {
                pitch_vision_input = PITCH_MAX_ANGLE;
            }
            else if (pitch_vision_input < PITCH_MIN_ANGLE)
            {
                pitch_vision_input = PITCH_MIN_ANGLE;
            }
        }else if (vision_recv_data->tracking == false)
        {
            pitch_vision_input = 0.0f;
            small_yaw_vision_input = 0.0f;
        }
        gimbal_cmd_send.pitch = pitch_vision_input;
        gimbal_cmd_send.small_yaw = SMALL_YAW_ALIGN_ANGLE - 360.0f - small_yaw_vision_input;// 确保输入为0时小yaw大yaw对齐朝前

        // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
        chassis_cmd_send.vx = ((float)rc_data[TEMP].rc.rocker_r_ > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r_ < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r_ * 25.0f : 0.0f; // _水平方向
        chassis_cmd_send.vy = ((float)rc_data[TEMP].rc.rocker_r1 > CHASSIS_DEAD_BAND || (float)rc_data[TEMP].rc.rocker_r1 < -CHASSIS_DEAD_BAND) ? (float)rc_data[TEMP].rc.rocker_r1 * 25.0f : 0.0f; // 1竖直方向
    }
    
    // 发射测试
    if(switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],发射控制
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.bullet_speed = SMALL_AMU_15;
        // 单发
        if (rc_data[TEMP].rc.dial < -500)
            shoot_cmd_send.load_mode = LOAD_1_BULLET;
        else
            shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }
}


/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    shoot_cmd_send.shoot_mode = SHOOT_OFF;

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.yaw_rate = (float)rc_data[TEMP].mouse.x / 660 * 10 * ROBOT_TASK_HZ; // 速度环用: °/s
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
    
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 只在"进入急停/恢复"的边沿打印: 遥控器离线是常态(开机前/关机后),每周期打印会刷屏
    static uint8_t estop_reported = 0;
    uint8_t estop_now;

    // 遥控器左侧开关为[上]进入急停模式.注意向打时下拨轮是正
    // 遥控器接收挪到底盘板后,"遥控器失效"包含两种情况:①双板转发链路离线 ②遥控器本身离线(底盘板判定),
    // 两者都必须触发急停;否则断连时rc数据被清零/冻结而开关判断全部不命中,会既不急停也不响应(RC-1)
#if defined(RC_RECEIVER_ON_CHASSIS) && defined(GIMBAL_BOARD)
    estop_now = (!CANCommIsOnline(cmd_can_comm) || !chassis_fetch_data.rc.online ||
                 switch_is_up(rc_data[TEMP].rc.switch_left) || robot_state == ROBOT_STOP);
#else
    // 标准双板/单板路径: 本板遥控器离线也必须急停.遥控器掉线时rc_data被清零,
    // 左右开关判断全部不命中 → RemoteControlSet/MouseKeySet都不会执行 →
    // chassis_cmd_send会沿用上一帧的vx/vy继续驱动底盘(RC-1同类风险,此处补上)
    estop_now = (!RemoteControlIsOnline() ||
                 switch_is_up(rc_data[TEMP].rc.switch_left) || robot_state == ROBOT_STOP);
#endif

    if (estop_now) // 还需添加重要应用和模块离线的判断
    {
        if (!estop_reported)
        {
            LOGERROR("[CMD] emergency stop!");
            estop_reported = 1;
        }
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        AlarmSetStatus(robocmd_alarm, ALARM_ON);
    }
    // 遥控器右侧开关为[中],恢复正常运行
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        if (estop_reported)
        {
            LOGINFO("[CMD] reinstate, robot ready");
            estop_reported = 0;
        }
        AlarmSetStatus(robocmd_alarm, ALARM_OFF);
    }
}

#ifdef GIMBAL_BOARD
/* 舵轮示教标定手势(双板: 遥控器接在云台板)
 * 进入: 左[中]+右[下]保持1.5s; 保存: 右摇杆前推到底保持1s; 恢复默认: 右摇杆后拉到底保持1s;
 * 退出不保存: 左开关离开[中](含[上]急停)或遥控器离线. 时间基准 RobotTask 200Hz(1tick=5ms).
 *
 * 为何是左[中]而不是左[下]: 左[下]是"遥控器控制"驾驶档,而右[下]是"底盘跟随云台"驾驶档,
 * 原手势"左下+右下"与一个**正常驾驶状态**完全重叠 —— 用跟随模式开车时只要保持1.5s就会误入
 * 示教(四舵轮断电可手摆 + 四驱动轮停机), 属于会突然失力的安全隐患.
 * 左[中]是"键鼠控制"档: 该档下 MouseKeySet() 只读鼠标/键盘, 右开关与右摇杆都不参与驾驶,
 * 因此该手势与任何驾驶状态都不冲突(手势期间车不可能被驱动).
 * 同时要求手势"松开过一次"才能再次进入: 保存/恢复后按住不放不会自动重入示教.
 * 命令写入 chassis_cmd_send.chassis_speed_buff 的高8位,随0x312转发给底盘板执行
 * (底盘板负责真正的"舵轮断电/写Flash"; 低8位保留拨轮速度档位语义). */
#define STEER_TEACH_ENTER_TICKS 300u
#define STEER_TEACH_ACT_TICKS 200u
#define STEER_TEACH_STICK_KEEP 450  // 右摇杆超过该值(量纲±660)开始/继续累计,不必压到机械极限
#define STEER_TEACH_STICK_RESET 300 // 回到该值以内才算"松手"并清零累计(滞回:摇杆抖动/未到底不会打断保持)
#define STEER_TEACH_CMD_HOLD_TICKS 40u // SAVE/RESTORE 的保持拍数(40拍=200ms),见函数尾部注释

static void SteerTeachCmdUpdate(void)
{
    static uint8_t teach_on = 0;
    static uint8_t rearm = 1; // 手势松开过才允许再次进入(防止保存后按住不放自动重入)
    static int8_t act_sign = 0; // 当前累计方向: +1=前推(保存) -1=后拉(恢复) 0=未动作
    static uint8_t latch_cmd = CHASSIS_TEACH_CMD_NONE; // 正在保持下发的一次性命令
    static uint16_t latch_cnt = 0;
    static uint16_t enter_cnt = 0;
    static uint16_t act_cnt = 0;
    uint8_t cmd = CHASSIS_TEACH_CMD_NONE;
    uint8_t online = RemoteControlIsOnline();
    uint8_t swl = rc_data[TEMP].rc.switch_left;
    uint8_t swr = rc_data[TEMP].rc.switch_right;
    int16_t stick = rc_data[TEMP].rc.rocker_r1;
    uint8_t gesture = (online && swl == RC_SW_MID && swr == RC_SW_DOWN) ? 1 : 0;

    if (!gesture)
        rearm = 1; // 手势已松开(或遥控器离线),重新武装

    if (!teach_on)
    {
        if (gesture && rearm)
        {
            if (++enter_cnt >= STEER_TEACH_ENTER_TICKS)
            {
                teach_on = 1;
                rearm = 0;
                enter_cnt = 0;
                act_cnt = 0;
                LOGWARNING("[cmd] steer teach ON (relayed to chassis): set wheels to forward, "
                           "right stick FULL FORWARD 1s = SAVE, FULL BACK 1s = RESTORE, "
                           "left switch out of MID (UP e-stop) = exit");
            }
        }
        else
            enter_cnt = 0;
    }
    else
    {
        cmd = CHASSIS_TEACH_CMD_ENTER; // 示教期间持续下发ENTER,底盘保持舵轮无力矩
        if (!online || swl != RC_SW_MID)
        {
            teach_on = 0;
            act_cnt = 0;
            act_sign = 0;
            cmd = CHASSIS_TEACH_CMD_NONE;
            LOGWARNING("[cmd] steer teach OFF (left switch left MID / rc lost), nothing saved");
        }
        else
        {
            /* 保存/恢复: 右摇杆前推/后拉到接近极限并保持1.0s.
             * 用滞回判定(KEEP=450 累计 / RESET=300 才清零): 实测"压到底保持1s"很容易因为摇杆抖动
             * 或没压到机械极限而中途被清零,原实现(stick>=600 否则立即清零)两次尝试都没能保存成功. */
            int8_t sign;
            if (stick >= STEER_TEACH_STICK_KEEP)
                sign = 1;
            else if (stick <= -STEER_TEACH_STICK_KEEP)
                sign = -1;
            else if (stick <= STEER_TEACH_STICK_RESET && stick >= -STEER_TEACH_STICK_RESET)
                sign = 0;          // 已松手: 清零累计
            else
                sign = act_sign;   // 中间带: 维持上一次方向继续累计(滞回)

            if (sign == 0)
                act_cnt = 0;
            else if (sign != act_sign)
                act_cnt = 1; // 换了方向: 重新计数
            else if (++act_cnt >= STEER_TEACH_ACT_TICKS)
            {
                cmd = (sign > 0) ? CHASSIS_TEACH_CMD_SAVE : CHASSIS_TEACH_CMD_RESTORE;
                latch_cmd = cmd;                       // 保持200ms下发,防止底盘侧漏掉这一拍
                latch_cnt = STEER_TEACH_CMD_HOLD_TICKS;
                teach_on = 0;
                act_cnt = 0;
            }
            act_sign = sign;
        }
    }

    /* SAVE/RESTORE 不是持续状态而是一次性动作, 只在命中那一拍有效; 而0x312是200Hz的多帧包,
     * 底盘任务对包的采样相位与发送相位不对齐时会整包不被读取, 也可能丢帧 —— 实测首次"推杆保存"
     * 就没到达底盘. 因此把命令保持 STEER_TEACH_CMD_HOLD_TICKS 拍; 底盘侧对这两个命令做边沿检测
     * (见 chassis.c SteerTeachCmd), 重复送达只执行一次, 不会重复擦写Flash. */
    if (latch_cnt)
    {
        cmd = latch_cmd;
        if (--latch_cnt == 0)
            latch_cmd = CHASSIS_TEACH_CMD_NONE;
    }

    chassis_cmd_send.chassis_speed_buff =
        (chassis_cmd_send.chassis_speed_buff & 0xFF) | ((int)cmd << CHASSIS_TEACH_CMD_SHIFT);
}
#endif // GIMBAL_BOARD

#if defined(RC_RECEIVER_ON_CHASSIS) && defined(GIMBAL_BOARD)
/**
 * @brief 将底盘板转发来的遥控器数据映射为本地RC_ctrl_t镜像
 * @note  字段与chassis.c中填充RC_Data_s的字段一一对应;
 *        若扩展了RC_Data_s,需在此同步补齐映射.
 */
static void RCForwardToCtrl()
{
    rc_remote_ctrl[TEMP].rc.rocker_l_ = chassis_fetch_data.rc.rocker_l_;
    rc_remote_ctrl[TEMP].rc.rocker_l1 = chassis_fetch_data.rc.rocker_l1;
    rc_remote_ctrl[TEMP].rc.rocker_r_ = chassis_fetch_data.rc.rocker_r_;
    rc_remote_ctrl[TEMP].rc.rocker_r1 = chassis_fetch_data.rc.rocker_r1;
    rc_remote_ctrl[TEMP].rc.dial = chassis_fetch_data.rc.dial;
    rc_remote_ctrl[TEMP].rc.switch_left = chassis_fetch_data.rc.switch_left;
    rc_remote_ctrl[TEMP].rc.switch_right = chassis_fetch_data.rc.switch_right;
    rc_remote_ctrl[TEMP].mouse.x = chassis_fetch_data.rc.mouse_x;
    rc_remote_ctrl[TEMP].mouse.y = chassis_fetch_data.rc.mouse_y;
    rc_remote_ctrl[TEMP].key[KEY_PRESS].keys = chassis_fetch_data.rc.key_press;
    memcpy(rc_remote_ctrl[TEMP].key_count[KEY_PRESS], chassis_fetch_data.rc.key_count, sizeof(chassis_fetch_data.rc.key_count));
}
#endif // RC_RECEIVER_ON_CHASSIS

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#if defined(RC_RECEIVER_ON_CHASSIS)
    // 更新遥控器镜像: 遥控器数据随底盘板上行帧送达,取用前先判定链路与遥控器是否在线;
    // 任一离线即清零镜像,避免沿用断连前的开关/摇杆值继续运动(RC-1离线保护,详见EmergencyHandler)
    if (CANCommIsOnline(cmd_can_comm) && chassis_fetch_data.rc.online)
        RCForwardToCtrl();
    else
        memset(rc_remote_ctrl, 0, sizeof(rc_remote_ctrl));
#endif // RC_RECEIVER_ON_CHASSIS
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(solvetrajectory_feed_sub,(void *)&solvetrajectory_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    /* 小陀螺自转速率通道每拍先清零(只有右侧[上]的RemoteControlSet会填值), 否则切档后会残留旧值 */
    gimbal_cmd_send.chassis_rotate_wz = 0.0f;
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中],键盘控制
        MouseKeySet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
#ifdef GIMBAL_BOARD
    SteerTeachCmdUpdate(); // 舵轮示教手势: 解析后写进chassis_speed_buff高8位,必须在下发CANCommSend之前
#endif

    setVisionSendData_rpy(-gimbal_fetch_data.gimbal_imu_data.Roll / (180.0f / PI),
                          gimbal_fetch_data.gimbal_imu_data.Pitch / (180.0f / PI),
                          -gimbal_fetch_data.gimbal_imu_data.Yaw / (180.0f / PI));
    setVisionSendData_aim(solvetrajectory_fetch_data.aim_x,
                          solvetrajectory_fetch_data.aim_y,
                          solvetrajectory_fetch_data.aim_z);
    // 设置弹道解算数据
    GetSolvetrajectory_need_data();

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(solvetrajectory_need_pub, (void *)&solvetrajectory_need_data);
}
