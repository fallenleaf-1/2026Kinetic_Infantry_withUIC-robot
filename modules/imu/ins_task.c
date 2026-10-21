/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "general_def.h"
#include "master_process.h"

#include "robot_def.h"
#include "gimbal.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#define SMALL_YAW_ALIGN_ANGLE (SMALL_YAW_FORWORD_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360

/* IMU(BMI088)初始化重试上限(ms): 原实现是无日志的无限重试,IMU不响应时整个RobotInit静默卡死
 * (实测现象: MX外设全部初始化完成、RTT无日志、uwTick停在2ms、电机/舵轮永远收不到指令).
 * 超时后: LOGERROR + BSP_FaultCode置位; 单板直控(CHASSIS_RC_DIRECT)不依赖IMU(仅跟随模式用),
 * 返回NULL并继续启动,保证舵轮能初始化与上电回正; 其它构建保留原有"不静默降级"语义
 * (gimbal.c 会解引用该指针,不能返回NULL). */
#define INS_INIT_TIMEOUT_MS 3000u
static uint8_t IMU_Available = 0; // BMI088是否初始化成功,供INS_Task判断能否读传感器
#ifndef CHASSIS_BOARD
static float last_small_yaw_warn_ms = 0; // 小yaw反馈失效告警限频
#endif

/* 供其它应用查询"姿态数据是否可信"(static变量外部拿不到).
 * 用途: gimbal.c 的大yaw世界角保持环 —— IMU不可用时必须让它失效并切到开环兜底,
 *       否则会拿着恒为0的姿态去闭环(目标虽会对齐到0,但等于没有世界基准). */
uint8_t INS_IMUIsAvailable(void)
{
    return IMU_Available;
}

static INS_t INS;
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度
static float last_nan_recover_ms = 0.0f; // 上一次姿态NaN自愈的时刻(限频用)

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;

    /* (N1, 2026-10-15) 初始姿态解算的NaN防护:
     * ① 加速度为0时 Norm3d 除法会得到NaN;
     * ② 两向量点积可能因浮点误差略大于1, acosf(1.0000001)=NaN;
     * ③ 加速度与重力平行时叉乘≈0(轴向量无法归一化);
     * 三者任一发生都会让初始四元数成NaN, 而EKF一旦以NaN起步就永久NaN(现场就是这种故障).
     * 因此这里逐项检查, 任何异常都退回单位四元数(即"按水平安装"起步), 并由日志暴露. */
    float acc_norm = Sqrt(acc_init[0] * acc_init[0] + acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2]);
    if (!(acc_norm > 1e-3f) || !(acc_norm < 1e3f))
    {
        LOGERROR("[ins] accel invalid during init (norm=%d), use identity quaternion", (int)(acc_norm * 1000.0f));
        init_q4[0] = 1.0f;
        init_q4[1] = init_q4[2] = init_q4[3] = 0.0f;
        return;
    }
    Norm3d(acc_init);

    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float cos_angle = Dot3d(acc_init, gravity_norm);
    if (!(cos_angle <= 1.0f))
        cos_angle = 1.0f; // 同时覆盖NaN
    if (!(cos_angle >= -1.0f))
        cos_angle = -1.0f;
    float angle = acosf(cos_angle);
    Cross3d(acc_init, gravity_norm, axis_rot);
    float axis_norm = Sqrt(axis_rot[0] * axis_rot[0] + axis_rot[1] * axis_rot[1] + axis_rot[2] * axis_rot[2]);
    if (!(axis_norm > 1e-6f)) // 加速度与重力几乎平行(水平安装): 旋转轴无意义,直接用单位四元数
    {
        init_q4[0] = 1.0f;
        init_q4[1] = init_q4[2] = init_q4[3] = 0.0f;
        return;
    }
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
    init_q4[3] = 0.0f;

    for (uint8_t i = 0; i < 4; ++i) // 最后兜底: 任何一项非有限都退回单位四元数
    {
        if (!isfinite(init_q4[i]))
        {
            LOGERROR("[ins] init quaternion not finite, use identity quaternion");
            init_q4[0] = 1.0f;
            init_q4[1] = init_q4[2] = init_q4[3] = 0.0f;
            return;
        }
    }
}


attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return &(INS.attitude);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    /* 单板直控不使用姿态(仅跟随模式需要IMU): 跳过零飘标定(6s级,且数据不满足条件时会反复重试),
     * 避免IMU标定把整个启动挡住——电机/舵轮初始化与上电回正都在其后. 代价只是IMU零飘未标定. */
#ifdef CHASSIS_RC_DIRECT
    const uint8_t bmi_calibrate = 0;
#else
    const uint8_t bmi_calibrate = 1;
#endif

    // 有上限的重试: 超时即报告并(按板型)决定继续或响亮停机,不再静默死循环
    float ins_init_start = DWT_GetTimeline_ms();
    while (BMI088Init(&hspi1, bmi_calibrate) != BMI088_NO_ERROR)
    {
        if (DWT_GetTimeline_ms() - ins_init_start > INS_INIT_TIMEOUT_MS)
        {
            LOGERROR("[ins] BMI088 init timeout [%dms], check SPI1 wiring / IMU power", (int)INS_INIT_TIMEOUT_MS);
            BSP_FaultCode = 1;
#ifdef CHASSIS_RC_DIRECT
            IMU_Available = 0;
            return NULL; // 单板直控不依赖IMU: 继续启动,保证舵轮能初始化与回正(IMU故障由日志/FaultCode可见)
#else
            while (1)
                ; // 其它构建: 云台侧会解引用IMU指针,不能返回NULL,保持不静默降级
#endif
        }
    }
    IMU_Available = 1;
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // imu heat init
    PID_Init_Config_s config = {.MaxOut = 2000,
                                .IntegralLimit = 300,
                                .DeadBand = 0,
                                .Kp = 1000,
                                .Ki = 20,
                                .Kd = 0,
                                .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);
    
    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
    return &(INS.attitude); // 返回最终姿态数据的指针
}

#ifndef CHASSIS_BOARD // 底盘板没有小yaw电机,small_yaw_motor为NULL,不能解引用
static float CalcSmallYawOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    static float offset_angle;
    angle = small_yaw_motor->measure.angle_single_round; // 获取的当前小yaw电机单圈角度
#if SMALL_YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > SMALL_YAW_ALIGN_ANGLE && angle <= 180.0f + SMALL_YAW_ALIGN_ANGLE)
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + SMALL_YAW_ALIGN_ANGLE)
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE - 360.0f;
    else
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE;
    
#else // 小于180度
    if (angle > SMALL_YAW_ALIGN_ANGLE)
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE;
    else if (angle <= SMALL_YAW_ALIGN_ANGLE && angle >= SMALL_YAW_ALIGN_ANGLE - 180.0f)
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE;
    else
        offset_angle = angle - SMALL_YAW_ALIGN_ANGLE + 360.0f;

#endif
    return offset_angle;
}
#endif // !CHASSIS_BOARD

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    if (!IMU_Available) // IMU未初始化成功: 不读传感器(死数据会污染解算/卡住静止检测),直接返回
        return;

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);

        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];

#ifdef INS_MOUNT_Y_DOWN
        /* 安装方向修正: 实测(2026-10-20)加速度计读数为 (0.78, 9.71, 0.39) —— 重力几乎全在 +Y,
         * 即 C 板是**立着**装的: 机体"上"方向 = 传感器 -Y, 于是 EKF 的 Pitch 恒≈90°(万向节死锁),
         * YawTotalAngle 也退化 ⇒ 大yaw 的世界角保持环拿着一个假的世界角在拉, 表现就是"转完还漂".
         * 这里把传感器系换成机体系(Z 上、右手系):
         *   x_b = z_s,  y_b = -x_s,  z_b = -y_s   (绕上方向逆时针转 ⇒ 角速度在 -Y_s ⇒ z_b 为正)
         * 如果以后把板子改成平放(Z 上), 注释掉 INS_MOUNT_Y_DOWN 即可。 */
        {
            float ax = INS.Accel[X], ay = INS.Accel[Y], az = INS.Accel[Z];
            float gx = INS.Gyro[X], gy = INS.Gyro[Y], gz = INS.Gyro[Z];
            INS.Accel[X] = az;
            INS.Accel[Y] = -ax;
            INS.Accel[Z] = -ay;
            INS.Gyro[X] = gz;
            INS.Gyro[Y] = -gx;
            INS.Gyro[Z] = -gy;
        }
#endif

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        /* ---- 陀螺零偏在线自愈(静止时学习) ----
         * 为什么需要: 6轴EKF没有磁力计做航向修正, 陀螺只要有一点残余零偏, YawTotalAngle 就会一直漂.
         * BMI088 上电标定在台架上很容易被判"被扰动"而失败(失败后回落离线常数), 实测航向漂移可达
         * -0.6°/s ⇒ 大yaw 的"世界角保持"环会把枪口慢慢拽走(现场表现: 转完松手后还小范围漂移).
         * 做法: 三轴角速度都很小(判定为静止)时, 用一阶低通把残余零偏学下来并从后续读数里扣掉.
         * 阈值 0.03rad/s≈1.7°/s: 只要不是持续慢速旋转就不会学错; 时间常数≈1s(1kHz × 0.001). */
        {
            static float gyro_bias[3] = {0};
            static uint8_t gyro_bias_valid = 0;
            static uint16_t still_cnt = 0;         // 静止置信度(非硬计数: 噪声只衰减, 不清零)
            const float GYRO_STATIC_TH = 0.03f;    // rad/s ≈1.7°/s: 只在这个量级的"静止噪声"范围内才学
            const float GYRO_BIAS_ALPHA = 0.001f;  // 时间常数≈1s(1kHz): 静止约 3~5s 收敛
            uint8_t quiet = (uint8_t)(fabsf(INS.Gyro[X]) < GYRO_STATIC_TH &&
                                      fabsf(INS.Gyro[Y]) < GYRO_STATIC_TH &&
                                      fabsf(INS.Gyro[Z]) < GYRO_STATIC_TH);
            /* 静止置信度用"快升慢降"而不是"一有噪声就清零": 实测偶发噪声尖峰会把清零式的门永远关死
             * (本次现场复现: 加了硬门限后零偏完全学不到, 漂移回到 -0.64°/s). */
            if (quiet)
            {
                if (still_cnt < 2000)
                    still_cnt += 2;
            }
            else if (still_cnt > 8)
                still_cnt -= 8;
            else
                still_cnt = 0;
            /* 置信度攒够(≈0.5s 净静止)才更新零偏: 手动快速转动时置信度迅速跌落, 不会把转动学成零偏 */
            if (still_cnt > 1000)
            {
                for (uint8_t i = 0; i < 3; ++i)
                    gyro_bias[i] += GYRO_BIAS_ALPHA * (INS.Gyro[i] - gyro_bias[i]);
                gyro_bias_valid = 1;
            }
            if (gyro_bias_valid) // 学到之后每拍都扣, 旋转时也扣同一个零偏
                for (uint8_t i = 0; i < 3; ++i)
                    INS.Gyro[i] -= gyro_bias[i];
        }

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        /* (N1, 2026-10-15 现场定位) 姿态NaN自愈
         * 现场故障: 云台 Roll/Pitch/Yaw/BigYaw_Feed 全为NaN, 大yaw/pitch的角度环反馈是NaN ⇒
         * PID输出恒0 ⇒ 电机完全没反应, 且永不恢复(需复位). 根因是预测重力方向反解acos越界.
         * 这里做两层保护: ①检测到非有限即刻用当前加速度重建滤波器(限频1s, 因为InitQuaternion要读100次
         * 加速度≈100ms); ②本拍直接返回, 让姿态输出保留上一拍的有限值, 不把NaN扩散到云台/视觉. */
        if (!isfinite(QEKF_INS.q[0]) || !isfinite(QEKF_INS.q[1]) ||
            !isfinite(QEKF_INS.q[2]) || !isfinite(QEKF_INS.q[3]) ||
            !isfinite(QEKF_INS.Yaw) || !isfinite(QEKF_INS.Pitch) || !isfinite(QEKF_INS.Roll))
        {
            if (DWT_GetTimeline_ms() - last_nan_recover_ms > 1000.0f)
            {
                float q_reinit[4];
                InitQuaternion(q_reinit);
                IMU_QuaternionEKF_Reset(q_reinit);
                last_nan_recover_ms = DWT_GetTimeline_ms();
                LOGERROR("[ins] attitude NaN detected, EKF re-initialized from accel");
            }
            return; // 本拍不更新姿态输出(保留上一拍的有限值)
        }

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

        // 为最终姿态数据赋值
        /* 必须用 memcpy: memccpy(dst,src,0,n) 遇到源字节 0x00 就停止拷贝,
         * 而浮点数的字节里出现 0x00 很常见(小数值高位为0) ⇒ attitude.Gyro/Accel 只被拷了前1~几个字节,
         * 其余保持旧值/0. pitch 的速度环正是读 &attitude.Gyro[1] (OTHER_FEED), 会间歇性拿到 0 或陈旧值. */
        memcpy(INS.attitude.Gyro, INS.Gyro, sizeof(INS.Gyro));
        memcpy(INS.attitude.Accel, INS.Accel, sizeof(INS.Accel));
        INS.attitude.Roll = INS.Roll;
        INS.attitude.Pitch = INS.Pitch;
        INS.attitude.Yaw = INS.Yaw;
        INS.attitude.YawTotalAngle = INS.YawTotalAngle;
#ifndef CHASSIS_BOARD // 底盘板没有小yaw电机,不能计算大yaw反馈
        /* 小yaw 反馈失效(未接/掉线)时 CalcSmallYawOffsetAngle() 会一直返回基于陈旧单圈角的常数,
         * BigYaw_Feed 就整体偏一个固定量 —— 世界角保持环会把枪口往错误方向拽(小陀螺尤其明显).
         * 失效时退回"IMU yaw 即大yaw"的原语义(相当于小yaw 关节角=0), 并限频告警. */
        if (small_yaw_motor && small_yaw_motor->daemon && DaemonIsOnline(small_yaw_motor->daemon))
        {
            INS.attitude.BigYaw_Feed = INS.YawTotalAngle - CalcSmallYawOffsetAngle(); // 计算大yaw反馈值
        }
        else
        {
            INS.attitude.BigYaw_Feed = INS.YawTotalAngle;
            if (DWT_GetTimeline_ms() - last_small_yaw_warn_ms > 1000.0f) // 1Hz限频,避免刷屏
            {
                last_small_yaw_warn_ms = DWT_GetTimeline_ms();
                LOGWARNING("[ins] small yaw feedback lost -> BigYaw_Feed = IMU yaw (joint offset ignored)");
            }
        }
#endif
        

    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
