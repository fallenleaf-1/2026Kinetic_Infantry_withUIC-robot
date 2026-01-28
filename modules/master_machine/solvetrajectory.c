/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算
#include "robot_def.h"

#include <math.h>
#include <stdio.h>


#include "solvetrajectory.h"
#include "message_center.h"

// 弹道补偿相关参数 待测试
#define INFANTRY_K 0.092f          // 步兵弹道系数 0.085-0.095
#define BIAS_TIME 0.05f            //偏置时间 s

Subscriber_t *solvetrajectory_need_sub;    // 弹道解算所需信息订阅者
Publisher_t *solvetrajectory_feed_pub;      // 弹道解算反馈消息发布者
static SolveTrajectory_Recv_s solvetrajectory_need_data; // 弹道解算接收数据
static SolveTrajectory_Send_s solvetrajectory_feed_data; // 弹道解算发送数据

struct tar_pos tar_position[4]; //最多只有四块装甲板

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(SolveTrajectoryParams* st, float s, float v, float angle,float *flight_time)
{
    float z;
    //t为给定v与angle时的飞行时间
    *flight_time = (float)((exp(st->k * s) - 1) / (st->k * v * cos(angle)));
    if(*flight_time < 0)
    {
        *flight_time = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * (*flight_time) - GRAVITY * (*flight_time) * (*flight_time) / 2);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(SolveTrajectoryParams* st, float s, float v, float angle)
{

    return 0;

}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(SolveTrajectoryParams* st, float s, float z, float v, float* flight_time)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(st, s, v, angle_pitch, flight_time);
        if(!isfinite(z_actual))
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
            i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(SolveTrajectoryParams* st)
{
    float flight_time = 0;

    // 1.先用当前目标位置计算pitch角度，得到 flight_time
    float horizontal_distance = sqrt(st->xw * st->xw + st->yw * st->yw) - st->s_bias;
    float temp_pitch = -pitchTrajectoryCompensation(st, horizontal_distance, st->zw + st->z_bias, st->current_v, &flight_time);

    // 2.线性预测
    float timeDelay = st->bias_time/1000.0 + flight_time;
    st->tar_yaw += st->v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st->armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st->tar_yaw + i * PI;
            float r = st->r1;
            tar_position[i].x = st->xw - r*cos(tmp_yaw);
            tar_position[i].y = st->yw - r*sin(tmp_yaw);
            tar_position[i].z = st->zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(st->current_yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(st->current_yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st->armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st->tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st->r1 + st->r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st->xw - r*cos(tmp_yaw);
            tar_position[i].y = st->yw - r*sin(tmp_yaw);
            tar_position[i].z = st->zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st->tar_yaw + i * PI/2.0;
            float r = use_1 ? st->r1 : st->r2;
            tar_position[i].x = st->xw - r*cos(tmp_yaw);
            tar_position[i].y = st->yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st->zw : st->zw + st->dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(st->current_yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(st->current_yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
    }

	

    solvetrajectory_feed_data.aim_z = tar_position[idx].z + st->vzw * timeDelay;
    solvetrajectory_feed_data.aim_x = tar_position[idx].x + st->vxw * timeDelay;
    solvetrajectory_feed_data.aim_y = tar_position[idx].y + st->vyw * timeDelay;
    //这里符号给错了
    temp_pitch = -pitchTrajectoryCompensation(st, sqrt((solvetrajectory_feed_data.aim_x) * (solvetrajectory_feed_data.aim_x) + (solvetrajectory_feed_data.aim_y) * (solvetrajectory_feed_data.aim_y)) - st->s_bias,
            solvetrajectory_feed_data.aim_z + st->z_bias, st->current_v,&flight_time);
    if(temp_pitch)
        solvetrajectory_feed_data.pitch = temp_pitch;
    if(solvetrajectory_feed_data.aim_x || solvetrajectory_feed_data.aim_y)
        solvetrajectory_feed_data.yaw = (float)(atan2(solvetrajectory_feed_data.aim_y, solvetrajectory_feed_data.aim_x));
}

// 从坐标轴正向看向原点，逆时针方向为正

void SolveTrajectoryInit()
{
    // 成为发布者，发送弹道解算结果
    solvetrajectory_feed_pub = PubRegister("SolveTrajectory_feed", sizeof(SolveTrajectory_Send_s)); 
    // 成为订阅者，接收弹道解算所需数据
    solvetrajectory_need_sub = SubRegister("SolveTrajectory_need", sizeof(SolveTrajectory_Recv_s));
}

void SolveTrajectoryTask()
{
    // 获取订阅的数据
    SubGetMessage(solvetrajectory_need_sub, &solvetrajectory_need_data);

    // 填充解算参数
    SolveTrajectoryParams st = {
        .tracking = solvetrajectory_need_data.tracking, // 是否有目标
        // 弹道系数
        .k = INFANTRY_K,
        // 自身参数
        .bullet_type = BULLET_17,                                 // 17mm子弹
        .current_v = solvetrajectory_need_data.current_v,         // 当前弹速()
        .current_pitch = solvetrajectory_need_data.current_pitch, // 当前pitch
        .current_yaw = solvetrajectory_need_data.current_yaw,     // 当前yaw
        // 目标参数
        .xw = solvetrajectory_need_data.xw,              // 右手坐标系下的x
        .yw = solvetrajectory_need_data.yw,              // 右手坐标系下的y
        .zw = solvetrajectory_need_data.zw,              // 右手坐标系下的z
        .vxw = solvetrajectory_need_data.vxw,            // 右手坐标系下的vx
        .vyw = solvetrajectory_need_data.vyw,            // 右手坐标系下的vy
        .vzw = solvetrajectory_need_data.vzw,            // 右手坐标系下的vz
        .tar_yaw = solvetrajectory_need_data.tar_yaw,    // 目标yaw
        .v_yaw = solvetrajectory_need_data.v_yaw,        // 目标yaw速度
        .r1 = solvetrajectory_need_data.r1,              // 目标中心到前后装甲板的距离
        .r2 = solvetrajectory_need_data.r2,              // 目标中心到左右装甲板的距离
        .dz = solvetrajectory_need_data.dz,              // 另一对装甲板的相对于被跟踪装甲板的高度差
        .bias_time = BIAS_TIME,                          // 偏置时间
        .s_bias = S_BIAS,                                // 枪口前推的距离
        .z_bias = Z_BIAS,                                // yaw轴电机到枪口水平面的垂直距离
        .armor_id = solvetrajectory_need_data.armor_id,  // 装甲板类型  0-outpost 6-guard 7-base  1-英雄 2-工程 3-4-5-步兵
        .armor_num = solvetrajectory_need_data.armor_num // 装甲板数字  2-balance 3-outpost 4-normal

    };

    if(!st.tracking)
    {
        // 无目标时不解算，直接返回
        return;
    }
    
    // 弹道解算
    autoSolveTrajectory(&st);

    // 发布解算结果   结果为右手系
    PubPushMessage(solvetrajectory_feed_pub, &solvetrajectory_feed_data);

}

