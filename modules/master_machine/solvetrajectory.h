#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78
typedef unsigned char uint8_t;

#include "master_process.h"

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输


enum BULLET_TYPE
{
    BULLET_17 = 0,  // 17mm子弹
    BULLET_42 = 1   // 42mm子弹
};  // 子弹类型


//设置参数
typedef struct
{
    bool tracking;       // 是否有目标
    float k;             //弹道系数
    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //右手坐标系下的x
    float yw;             //右手坐标系下的y
    float zw;             //右手坐标系下的z
    float vxw;            //右手坐标系下的vx
    float vyw;            //右手坐标系下的vy
    float vzw;            //右手坐标系下的vz
    float tar_yaw;        //目标yaw
    float v_yaw;          //目标yaw速度
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
}SolveTrajectoryParams;

//用于存储目标装甲板的信息
struct tar_pos
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};


typedef struct
{
    float aim_x;    // 目标点x坐标
    float aim_y;    // 目标点y坐标
    float aim_z;    // 目标点z坐标
    float pitch;    // 计算得到的pitch角度
    float yaw;      // 计算得到的yaw角度

} SolveTrajectory_Send_s;	// 发送数据结构体

typedef struct
{
    bool tracking;    // 是否有目标
    float xw;       // 目标点x坐标
    float yw;       // 目标点y坐标
    float zw;       // 目标点z坐标
    float vxw;      // 目标点x方向速度
    float vyw;      // 目标点y方向速度
    float vzw;      // 目标点z方向速度
    float current_v;// 当前子弹速度
    float current_pitch; // 当前pitch角度
    float current_yaw;   // 当前yaw角度
    ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵
    ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
    float bias_time;    // 偏置时间
    float s_bias;      // 枪口前推的距离
    float z_bias;      // yaw轴电机到枪口水平面的垂直距离
    float tar_yaw;     // 目标yaw
    float v_yaw;       // 目标yaw速度
    float r1;          // 目标中心到前后装甲板的距离
    float r2;          // 目标中心到左右装甲板的距离
    float dz;         // 另一对装甲板的相对于被跟踪装甲板的高度差

} SolveTrajectory_Recv_s;	// 接收数据结构体

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

// 弹道解算初始函数
void SolveTrajectoryInit();
// 弹道解算任务函数
void SolveTrajectoryTask();
// 在INS任务中调用,用于填充视觉发送数据


#endif /*__SOLVETRAJECTORY_H__*/