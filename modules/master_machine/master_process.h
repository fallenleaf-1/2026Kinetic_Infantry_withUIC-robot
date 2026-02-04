#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include <stdbool.h>

#include "bsp_usart.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,    // 无射击
	AUTO_FIRE = 1,  // 自动射击
	AUTO_AIM = 2    // 自动瞄准
} Fire_Mode_e;

typedef enum
{
	COLOR_BLUE = 0,
	COLOR_RED = 1,
} Enemy_Color_e;	// 敌方颜色枚举

typedef enum
{
	VISION_MODE_AIM = 0,		// 自动瞄准模式
	VISION_MODE_SMALL_BUFF = 1,	// 小符模式：小能量机关
	VISION_MODE_BIG_BUFF = 2	// 大符模式：大能量机关
} Work_Mode_e;	// 视觉工作模式枚举

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10,
	SMALL_AMU_15,
	BIG_AMU_16,
	SMALL_AMU_18,
	SMALL_AMU_30,
} Bullet_Speed_e; // 子弹速度枚举

typedef enum
{
	ARMOR_OUTPOST = 0,  // 前哨站
    ARMOR_HERO = 1,     // 英雄
    ARMOR_ENGINEER = 2, // 工程
    ARMOR_INFANTRY3 = 3,// 步兵3
    ARMOR_INFANTRY4 = 4,// 步兵4
    ARMOR_INFANTRY5 = 5,// 步兵5
    ARMOR_GUARD = 6,    // 哨兵
    ARMOR_BASE = 7  // 基地
} ARMOR_ID; // ARMOR_ID枚举

typedef enum
{
    ARMOR_NUM_BALANCE = 2,  // 平衡步兵
    ARMOR_NUM_OUTPOST = 3,  // 前哨站
    ARMOR_NUM_NORMAL = 4    // 普通步兵
}ARMOR_NUM;  // 装甲板数量

typedef struct
{
	float aim_x;
	float aim_y;
	float aim_z;
	float roll;
	float pitch;
	float yaw;
	Enemy_Color_e enemy_color;
	bool reset_tracking;
} Vision_Send_s;	// 发给视觉的 数据 结构体

typedef struct
{
    uint8_t header;   // 0x5A
	bool reset_tracking : 1;
	uint8_t detect_color : 1; // 0:red 1:blue
	uint8_t reserved : 6;
    float aim_x;
    float aim_y;
    float aim_z;
	float roll;
    float pitch;
    float yaw;
    uint16_t checksum;
} Vision_Send_Packet_t; // 发给视觉 数据包 结构体

typedef struct
{
	bool tracking; // 是否有目标 
	ARMOR_ID armor_id; // 目标ID 
	ARMOR_NUM armor_num; // 装甲板数量
	float x;				// 目标x坐标
	float y;				// 目标y坐标
	float z;				// 目标z坐标
	float yaw;				// 目标yaw角度
	float vx;				// 目标x方向速度
	float vy;				// 目标y方向速度
	float vz;				// 目标z方向速度
	float v_yaw;			// 目标yaw角速度
	float r1;				// 目标中心到前后装甲板的距离
	float r2;				// 目标中心到左右装甲板的距离
	float dz;				// 另一对装甲板的相对于被跟踪装甲板的高度差
} Vision_Recv_s;	// 从视觉接收的 数据 结构体

typedef struct
{
    uint8_t header;        // 0xA5
    uint8_t tracking : 1;    // 是否有目标
	uint8_t id : 3;  // 目标ID
	uint8_t armors_num : 3; // 装甲板数量
    uint8_t reserved : 1;
	float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum;		//CRC16校验值
} Vision_Recv_Packet_t; // 从视觉接收 数据包 结构体
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

// 视觉数据填充接口
void setVisionSendData_rpy(float roll,float pitch,float yaw);
void setVisionSendData_aim(float aim_x,float aim_y,float aim_z);
// 发送视觉数据
void VisionSend();

#endif // !MASTER_PROCESS_H