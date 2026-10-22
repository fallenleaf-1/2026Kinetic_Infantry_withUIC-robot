/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
//#define CHASSIS_BOARD //底盘板
 #define GIMBAL_BOARD  //云台板

/* 单板(仅底盘板)遥控器直控底盘
 * 定义后: 遥控器(DBUS)接在【底盘板】USART3,由底盘板解析并直接解算底盘指令
 *         (见chassis.c RCDirectSet()),不初始化CANComm、不发送0x311上行帧.
 *         原因: 单板没有云台板,发0x311收不到ACK(总线无对端),邮箱会被占满并拖垮
 *         CAN1上的前轮指令;且底盘指令原本只来自CANComm,单板下必然恒为零力
 *         —— 这正是"遥控器接到底盘板却没反应"的真正原因(数据无人消费).
 * 注释后: 恢复双板路径(遥控器由云台板接收;或按下方RC_RECEIVER_ON_CHASSIS由底盘板转发).
 * 回退方式: 注释下面一行并重新编译.
 */
//#define CHASSIS_RC_DIRECT

/* 遥控器接收位置选择(仅双板有效)
 * 定义后: 遥控器(DBUS)接在【底盘板】USART3,由底盘板解析,并随底盘板上行帧
 *         (0x311 Chassis_Upload_Data_s,见RC_Data_s)转发给云台板的robot_cmd;
 *         robot_cmd.c 不再直接读取本板串口,并依据转发帧做遥控器/链路离线保护.
 * 注意: 该路线依赖0x311的多帧分包传输(38B数据+4B协议头尾=42B,需连发6帧),
 *       启用前必须先修复CANComm多帧丢帧问题,否则链路恒判离线
 *       (见.Doc/遥控器改到底盘板实施记录_2026-10-14.md 附录A).
 *       与CHASSIS_RC_DIRECT互斥: 直控是单板自用,转发是给云台板用.
 * 注释后: 恢复原状 —— 遥控器接在云台板,由robot_cmd直接解析(原链路零改动).
 * 回退方式: 注释下面一行并重新编译两块板.
 */
// #define RC_RECEIVER_ON_CHASSIS

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define SMALL_YAW_FORWORD_ALIGN_ECD 6763   //小YAW电机超前时的编码器数值
#define SMALL_YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算小yaw偏转角度
#define YAW_CHASSIS_ALIGN_ECD 5040     // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE    30.0f      // 云台最大仰角
#define PITCH_MIN_ANGLE    -30.0f
#define SMALL_YAW_MAN_ANGLE 30.0f       // 小YAW最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define SMALL_YAW_MIN_ANGLE -30.0f      // 小YAW最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define GIMBAL_DEAD_BAND 10        // 云台控制死区,低于该值的遥控器输入将被置为0
/* 遥控器左摇杆水平 → 大yaw参考角的增量(度/拍; RobotTask 200Hz)
 * 原值 0.0025 ≈ 满杆 330°/s,现场实测"太快/一推就猛转",降到 0.0012 ≈ 满杆 158°/s。
 * 需要更快/更慢只改这一个数(两个档位分支都用它)。 */
#define GIMBAL_YAW_RC_SCALE 0.0012f

/* RobotTask(robot_cmd)的调用频率: osDelay(5) ⇒ 200Hz。
 * 用于把"每拍的角度增量"换算成"角速度指令(°/s)"发给云台的速度环。 */
#define ROBOT_TASK_HZ 200.0f

/* 大yaw控制环(云台板; 仅"大yaw用自身编码器反馈"时相关)
 * 注释(=0, 当前): **速度环** —— 摇杆解出的角速度指令(°/s)直接给速度环。
 *                 没有位置误差(堵转2s也不会累积几百度的误差)、没有级联的积分饱和,
 *                 实测的位置环在这套参数下会"堵转→然后满速冲出去",故默认用速度环。
 * 定义(=1):       位置环(angle→speed 级联, 原设计)。参数 Kp=100/速度Kp=200/角度环 MaxOut=1000RPM
 *                 (=6000°/s)从未在硬件上验证过, 仅在需要"松手后钉住角度"时再启用并重新整定。 */
// #define GIMBAL_YAW_POSITION_LOOP
#define GIMBAL_YAW_SPEED_LIMIT 200.0f // 速度环角速度指令兜底限幅(°/s), 防止任何情况下指令过大
/* 大yaw 摩擦(库仑/静摩擦)前馈: 直接叠加在速度环输出上的恒定电流量(满量程20000)。
 * 为什么需要: 该轴静摩擦明显。线性PID只有两条路——输出太小则"压不过死区"(只慢慢爬),
 * 为压过死区把Kp提得很高则在死区边缘产生极限环(实测 Kp=1000 时 sOut 在 ±20000 每20ms翻转、电机剧烈振荡)。
 * 加入"按指令方向"的前馈后 P 增益可以取小(稳定), 起步力矩由前馈保证。现场按"能平稳起步且不抖"微调。 */
#define GIMBAL_YAW_FRICTION_FF 1200.0f
/* 摩擦前馈的滞回阈值(°/s): |指令| 超过 ON 才施加, 低于 OFF 才撤销.
 * 原实现是单点比较(>3°/s 给 ±2500、否则 0): 指令在阈值附近抖动时前馈在 ±2500 之间来回跳,
 * 现场表现就是"果冻状往复摆动、很久才静止". 滞回把这一项从"bang-bang"变成"有记忆的开关". */
#define GIMBAL_YAW_FF_ON  5.0f
#define GIMBAL_YAW_FF_OFF 2.0f
/* 前馈"起步脉冲"时长(ms): 越过 FF_ON 的那一下给这么久的恒定电流去顶开静摩擦, 到时自动撤.
 * 之前是"恒定给到退出门限", 状态粘滞 ⇒ 松杆后前馈继续顶 ⇒ 现场"转完还小范围漂移".
 * 调大到 200~300 仍旧发抖/发冲, 调小到 80~100 起不来就再加回 GIMBAL_YAW_FRICTION_FF. */
#define GIMBAL_YAW_FF_PULSE_MS 150.0f
#define GIMBAL_YAW_FF_PULSE_TICKS ((uint16_t)(GIMBAL_YAW_FF_PULSE_MS * ROBOT_TASK_HZ / 1000.0f))

/* 大yaw **世界角(IMU)保持** —— 小陀螺("开着转还能直行/能瞄")的前提条件
 * 现象(现场): 右开关[上]小陀螺时, 右摇杆推前进 → 车不直行而是画圈.
 * 根因: 速度环只保证"大yaw相对底盘不转"; 底盘自旋时大yaw被底盘带着一起转 ⇒
 *       robot_cmd的offset_angle(大yaw相对底盘角, robot_cmd.c:179) 冻结 ⇒
 *       底盘(chassis.c:1016)拿到的速度指令永远钉在车体系里 ⇒
 *       "车体系内恒定速度 + 车体自转" = 圆周(半径 v/ω), 数学上必然画圈.
 *       也就是说: 小陀螺/跟随都要求大yaw是**世界系的稳定基准**(原GIMBAL_GYRO_MODE的本意),
 *       而"相对底盘锁零"把基准丢了, 跟随模式看不出来(它只需要相对角), 小陀螺必然暴露.
 * 做法: 摇杆给速率时保持环让位(手感与现在完全一致); 摇杆回中后用P环把IMU世界角拉回
 *       "松杆那一刻的指向" ⇒ 大yaw在世界系稳住 ⇒ offset_angle随底盘自转连续变化 ⇒
 *       底盘严格按大yaw朝向直行(左摇杆还能改朝向), 并顺带获得"松杆钉住角度".
 * 符号: 已用现场日志定死 = **+1** —— yawwatch日志里同一时段 电机spd=+36~+60°/s 而
 *       IMU YawTotal -6.2→+99.2 同步上升(电机自身编码器327→350→5.7→20.9也同步上升),
 *       即"速度环正速率 ⇒ BigYaw_Feed 增大" ⇒ 用 +Kp*(target-fb) 才是负反馈.
 * Kp(量纲1/s): 底盘自旋ω时稳态会有 ω/Kp 的固定偏差(例 ω=150°/s, Kp=8 ⇒ 约19°).
 *       嫌"瞄的方向偏"就加大Kp(或加外环积分); 出现嗡嗡抖就把Kp减小. 先给小值起步.
 * 关闭方式: 注释 GIMBAL_YAW_IMU_HOLD 一行重新编译(只影响云台板). */
#define GIMBAL_YAW_IMU_HOLD
#define GIMBAL_YAW_HOLD_KP 10.0f       // 世界角误差(°) → 速率指令(°/s) 的比例增益
/* 保持环的**符号**: 修正了 IMU 安装方向(INS_MOUNT_Y_DOWN)之后, 世界角的符号可能与旧版相反.
 * 若进入 GYRO 档松杆时云台自己越转越快/往一边跑(而不是钉住), 把这里改成 -1.0f 重新编译即可。 */
#define GIMBAL_YAW_HOLD_SIGN 1.0f
#define GIMBAL_YAW_HOLD_LIMIT 150.0f   // 保持环单独贡献的速率上限(°/s), 兜底防跑飞
#define GIMBAL_YAW_HOLD_STICK_TH 8.0f  // 摇杆速率超过该值(°/s)视为"人在操作" ⇒ 保持环让位
                                       /* 3.0→8.0: 原值比"死区后的最小有效指令"(1.98°/s)只大一点,
                                        * 小幅打杆会被保持环整个吃掉(手感"滞后/推了不动"), 一越过阈值又叠加 2500 前馈(猛冲) */
/* 松杆后轴仍在滑行的判定(°/s): 实测转速高于它就继续刷新保持目标, 避免"追着已经越过头的角度拉回来".
 * 想让松杆更"钉住"可以调小(但太小会重新出现过冲); 20°/s 对应满杆速率(119°/s)的 1/6. */
#define GIMBAL_YAW_HOLD_COAST_TH 20.0f

/* 大yaw **小陀螺开环兜底**(IMU不可用时用) —— 保持环的"备胎"
 * 为什么需要: 保持环靠IMU的姿态(绝对指向). IMU一旦离线/姿态NaN(本项目出现过姿态恒NaN的故障),
 *            世界基准就没了, 小陀螺会立刻退回"画圈". 这条兜底不需要任何传感器.
 * 原理: 小陀螺的自转速率是**固定指令**(底盘 wz=25)决定的常数. 让大yaw按这个常数反向转,
 *       大yaw就近似世界稳定 ⇒ offset_angle照常随自转变化 ⇒ 车仍然能直行(只是方向随转速变化慢慢弧).
 * 常数怎么来(不用手工标定 —— **在线自学**):
 *       保持环工作时,"小陀螺 + 松杆"稳态下的保持环输出 = 正好等于需要补偿的自转速率,
 *       云台每拍把它低通学进 yaw_ff_learned(见 gimbal.c), IMU 一旦失效就用这个学到的值.
 *       下面这个宏只是**还没学到之前的初值**(按几何估算: 轮速指令 wz×150°... 折合底盘自转≈58°/s)。
 *       符号/大小若现场发现兜底方向反了(表现为"画得更紧"而不是变直), 用下面的 FORCE 宏就地验证并改符号.
 * 关闭方式: 注释 GIMBAL_YAW_FF_ON_IMU_LOST 一行(只影响云台板). */
#define GIMBAL_YAW_FF_ON_IMU_LOST
#define GIMBAL_YAW_SPIN_FF_RATE 58.0f  // 小陀螺底盘自转速率(°/s); 仅作兜底初值, 运行中会被自学值覆盖
#define GIMBAL_YAW_FF_LEARN_ALPHA 0.002f // 兜底常数在线自学的低通系数(1kHz ⇒ 时间常数≈0.5s)
#define GIMBAL_YAW_SPIN_FF_MAX 300.0f    // 自学值的可信上限(°/s), 越界即丢弃(防止学到垃圾)
// #define GIMBAL_YAW_FF_FORCE         // 强制走开环兜底(关掉保持环) —— 现场标定/验证兜底方向用

/* 大yaw角度环反馈来源(云台板)
 * 注释(=0, 当前): 用**大yaw电机自身编码器**反馈 —— 指令为相对上电位置的增量角, 不依赖IMU姿态,
 *                 小yaw未接/IMU异常时也能转, 适合当前"只挂大yaw电机"的调试阶段.
 * 定义(=1):       用IMU的 BigYaw_Feed(IMU yaw - 小yaw偏置)反馈, 这是设计原意(绝对指向, 供视觉/跟随),
 *                 需要小yaw电机接好并对齐, 否则反馈带固定偏置(会造成使能瞬间的阶跃).
 * 两种方式都已具备"使能时把指令基准对齐到当前反馈"的保护, 不会再出现使能瞬间满速冲过去. */
// #define GIMBAL_YAW_FEEDBACK_IMU

/* ---------------- pitch(DM4310) 控制参数 ----------------
 * 云台板现装在小yaw 转台上: 它随大yaw+小yaw 转, 但**不随 pitch 关节转** ⇒
 * IMU 的 Pitch 不是枪管俯仰角, 拿它闭环等于在追"转台点头+振动" ⇒ 必须改用电机编码器反馈.
 * 用编码器时量纲: 角度 = measure.total_angle(deg), 速度 = measure.velocity×57.3(deg/s),
 * 与 gimbal_cmd_recv.pitch(deg) 一致, 串级环才是同一个量纲. */
#define GIMBAL_PITCH_FEEDBACK_MOTOR
/* 重力前馈: 稳态力矩交给它, 角度环不再用积分(Ki)去"攒"重力 ⇒ 消除 windup 与死区极限环 */
#define GIMBAL_PITCH_GRAVITY_FF
#define GIMBAL_PITCH_GRAVITY_A 1.5f   // 枪管水平时的重力矩(N·m); 现场从 1.0 起、±0.5 调
#define GIMBAL_PITCH_GRAVITY_DIR 1    // 重力矩方向: +1 或 -1(装反了改这里)
#define GIMBAL_PITCH_ZERO_DEG 0.0f    // 编码器角为 0 时枪管的姿态(约定上电时枪管水平 ⇒ 0)
#define GIMBAL_PITCH_CMD_SLEW 90.0f   // 指令斜率(°/s): 防止 -30° 阶跃直接砸下去
#define GIMBAL_PITCH_RATE_LIMIT 120.0f // 角度环输出(速率指令)上限(°/s)

/* ---------------- 小yaw 指令整形 ----------------
 * 目标下发的是"绝对编码器角-360"(如 -62.8°), 而反馈是 measure.total_angle(上电相对角):
 * 两者语义不一致时误差可达几十~350°, 配合角度环 MaxOut=500°/s 就是"甩头撞限位".
 * 这里: ①使能瞬间把指令对齐当前反馈 ②限位 ③限速. */
#define GIMBAL_SMALL_YAW_CMD_SLEW 120.0f   // 指令斜率(°/s)
#define GIMBAL_SMALL_YAW_RATE_LIMIT 150.0f // 角度环输出(速率指令)上限(°/s)
// 发射参数
#define ENEMY_COLOR 0  // 敌方颜色：0蓝 1红
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
#define RAD_FRICTION_WHEEL 0.025f        // 摩擦轮半径,单位m
#define S_BIAS 0.1f // 枪口前推的距离 m
#define Z_BIAS 0.1f // yaw轴电机到枪口水平面的垂直距离 m
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 300              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define WHEEL_PERIMETER (2 * PI * RADIUS_WHEEL) // 轮子周长
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define CHASSIS_DEAD_BAND 10   // 底盘速度死区,低于该值的速度指令将被置为0
/* 遥控摇杆 → 地盘速度的系数(三档统一).
 * 原来小陀螺档是 5.0f(另两档 25.0f), 前进速度只有 1/5, 现场看起来像"推不动/不直行". */
#define CHASSIS_VXY_SCALE 25.0f
/* 小陀螺(ROTATE)档的底盘自旋速率指令. 改这里必须同步改 GIMBAL_YAW_SPIN_FF_RATE,
 * 后者是"IMU失效时反向补偿的初值", 单位是 °/s, 二者物理上对应同一个自转速率. */
#define CHASSIS_ROTATE_WZ 25
//四个驱动电机朝向前方时 转向电机初始编码值
#define GM6020_LF_ANGLE 133.40f		//装上去的时候Y轴正向对应的角度(2026-10-14现场手摆正前方后重标,旧值192.69)
#define GM6020_RF_ANGLE 223.93f     //旧值226.36
#define GM6020_RB_ANGLE 134.19f     //旧值133.98
#define GM6020_LB_ANGLE 102.15f     //旧值46.53


#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

/* IMU(C板)安装方向修正(见 ins_task.c INS_Task 里的轴映射):
 * 实测 2026-10-20 加速度计为 (0.78, 9.71, 0.39) ⇒ 重力在 +Y, 说明板子是**立着**装的,
 * 而 EKF 假设 Z 轴朝上 ⇒ Pitch 恒≈90°(万向节死锁), YawTotalAngle 退化 ⇒
 * 大yaw 的"世界角保持"拿着假世界角在拉(现场: 转完松手后还漂).
 * 定义时把传感器系换成机体系(x_b=z_s, y_b=-x_s, z_b=-y_s).
 * 板子改为平放(Z 上)后: 注释掉这一行并重新编译。 */
#define INS_MOUNT_Y_DOWN

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// 遥控器直控仅适用于单块底盘板: 无云台板可转发,单板下才由底盘自解算
#if defined(CHASSIS_RC_DIRECT) && (defined(ONE_BOARD) || defined(GIMBAL_BOARD))
#error CHASSIS_RC_DIRECT is single-chassis-board only, define CHASSIS_BOARD (not ONE_BOARD/GIMBAL_BOARD).
#endif

// 直控(单板自用)与转发(给云台板用)两条路线不能同时启用
#if defined(CHASSIS_RC_DIRECT) && defined(RC_RECEIVER_ON_CHASSIS)
#error CHASSIS_RC_DIRECT (local control) and RC_RECEIVER_ON_CHASSIS (forward to gimbal) are mutually exclusive.
#endif

// 遥控器接收挪到底盘板依赖双板配置(底盘板接收转发,云台板robot_cmd使用),单板下无转发对象
#if defined(RC_RECEIVER_ON_CHASSIS) && defined(ONE_BOARD)
#error RC_RECEIVER_ON_CHASSIS requires dual-board (CHASSIS_BOARD + GIMBAL_BOARD), not ONE_BOARD.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_ENCODER_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum
{
    BURSTFIRE_RATE_15 = 0,   //15
    BURSTFIRE_RATE_20,
    BURSTFIRE_RATE_25,
    BURSTFIRE_RATE_30,
    BURSTFIRE_RATE_35,
    BURSTFIRE_RATE_40,
} burstfire_rate_e; // 连发速率档位枚举  拨弹盘电机速度档位

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------遥控器数据(定义RC_RECEIVER_ON_CHASSIS时由底盘板随上行帧转发给云台板)---------------- */
/**
 * @brief 遥控器(DBUS)数据镜像,字段与robot_cmd实际从RC_ctrl_t读取的部分一一对应
 * @note  遥控器改由底盘板接收后,底盘板解析出的RC_ctrl_t无法跨板共享(RC_ctrl_t定义在
 *        remote_control.h中且含位域union),故在此定义用于CAN传输的紧凑结构体,
 *        并作为Chassis_Upload_Data_s的成员随底盘板上行帧(0x311)一并发送.
 *        robot_cmd侧通过RCForwardToCtrl()手工映射回本地RC_ctrl_t镜像.
 *        若后续robot_cmd用到了其他字段(如key[KEY_PRESS_WITH_SHIFT]/mouse.press_l等),
 *        需同步扩展本结构体并在映射函数中补齐,否则会读到默认值0.
 */
typedef struct
{
    int16_t rocker_l_;  // 左摇杆水平方向
    int16_t rocker_l1;  // 左摇杆竖直方向
    int16_t rocker_r_;  // 右摇杆水平方向
    int16_t rocker_r1;  // 右摇杆竖直方向
    int16_t dial;       // 侧边拨轮

    uint8_t switch_left;  // 左侧开关
    uint8_t switch_right; // 右侧开关

    int16_t mouse_x; // 鼠标x轴移动速度
    int16_t mouse_y; // 鼠标y轴移动速度

    uint16_t key_press;    // 当前按键位域,对应key[KEY_PRESS].keys
    uint8_t key_count[16]; // 普通按键按下计数,对应key_count[KEY_PRESS][0..15]

    uint8_t online; // 遥控器在线标志(底盘板RemoteControlIsOnline()结果),供云台板做离线保护
} RC_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *        例外: 定义RC_RECEIVER_ON_CHASSIS后,遥控器(DBUS)改接底盘板,遥控器原始数据
 *              随Chassis_Upload_Data_s上行帧转发给云台板的robot_cmd
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

/* 舵轮"示教标定"命令(双板用): 云台板robot_cmd把遥控器手势解析后写入
 * Chassis_Ctrl_Cmd_s.chassis_speed_buff 的高8位,随0x312转发给底盘板执行
 * (低8位保留原语义=拨轮速度档位; 单板直控时底盘板直接读本板遥控器手势,不使用该字段) */
#define CHASSIS_TEACH_CMD_NONE 0    // 正常
#define CHASSIS_TEACH_CMD_ENTER 1   // 进入示教: 底盘四舵轮断电,便于用手摆到正前方
#define CHASSIS_TEACH_CMD_SAVE 2    // 把当前四角存为新正前方并退出示教
#define CHASSIS_TEACH_CMD_RESTORE 3 // 擦除现场标定,恢复robot_def.h默认值并退出示教
#define CHASSIS_TEACH_CMD_SHIFT 8

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float small_yaw;
    float yaw;
    float pitch;
    /* 小陀螺自转速率通道(°/s): robot_cmd 只在 CHASSIS_ROTATE 档填值(其余档为0),
     * 云台板用它做两件事: ① 判断"现在在不在小陀螺"(非0即小陀螺) ② IMU失效时作为开环兜底
     * 的反向补偿量(见 robot_def.h 的 GIMBAL_YAW_FF_ON_IMU_LOST). 该结构体不过CAN, 可自由使用。 */
    float chassis_rotate_wz;
    float DM_ENABLE_FLAG;
    float pitch_omega_feedforword;

    /* 大yaw速度环用的角速度指令(°/s, 由robot_cmd按摇杆解出, 200Hz刷新)。
     * 与 yaw(角度指令)并存: 用速度环时云台取本字段, 用位置环时云台取 yaw。
     * 该结构体只在云台板内部经消息中心传递(不过CAN), 因此可以自由扩展字段。 */
    float yaw_rate;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    burstfire_rate_e burstfire_rate; // 连续发射的射频枚举
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

#ifdef RC_RECEIVER_ON_CHASSIS
    RC_Data_s rc; // 遥控器数据(接收在底盘板,随本上行帧一并转发给云台板的robot_cmd)
#endif

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t small_yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    float bullet_speed; // 弹速
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H