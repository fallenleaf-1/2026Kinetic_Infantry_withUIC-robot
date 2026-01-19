/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "daemon.h"
#include "bsp_log.h"
#include "ROScrc16.h"
#include "robot_def.h"


static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance = NULL;
static USARTInstance  *vision_usart_instance = NULL;

Vision_Recv_Packet_t *pkt_recv;


static uint8_t vision_rx_buf[sizeof(Vision_Recv_Packet_t)]; // 接收缓冲区
static uint16_t vision_0xa5_cnt = 0;                          
static uint16_t vision_crc_cnt = 0;
static uint16_t vision_rx_cnt = 0;                            // 接收计数器
/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}


#ifdef VISION_USE_UART

#include "bsp_usart.h"

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision(const Vision_Recv_Packet_t *pkt, Vision_Recv_s *out)
{
    out->tracking    = pkt->tracking;
    out->armor_id    = pkt->id;
    out->armor_num   = pkt->armors_num;

    out->x = pkt->x;
    out->y = pkt->y;
    out->z = pkt->z;
    out->yaw = pkt->yaw;

    out->vx = pkt->vx;
    out->vy = pkt->vy;
    out->vz = pkt->vz;
    out->v_yaw = pkt->v_yaw;

    out->r1 = pkt->r1;
    out->r2 = pkt->r2;
    out->dz = pkt->dz;
}

static void DecodeVision_Adapter()
{
    uint8_t *buf = vision_usart_instance->recv_buff;   // usart 返回的接收缓冲区
    uint16_t recv_len = vision_usart_instance->recv_buff_size;

    for (uint16_t i = 0; i < recv_len; i++)
    {
        uint8_t byte = buf[i];

        if (vision_rx_cnt == 0)
        {
            if (byte != 0xA5)
                continue;
        }

        vision_rx_buf[vision_rx_cnt++] = byte;

        if (vision_rx_cnt > sizeof(Vision_Recv_Packet_t))
        {
            vision_rx_cnt = 0;
            continue;
        }

        if (vision_rx_cnt == sizeof(Vision_Recv_Packet_t))
        {
            Vision_Recv_Packet_t *pkt =
                (Vision_Recv_Packet_t *)vision_rx_buf;

            if (CRC16_Verify((uint8_t *)pkt,sizeof(Vision_Recv_Packet_t)))
            {
                DecodeVision(pkt, &recv_data);
                DaemonReload(vision_daemon_instance);
            }

            vision_rx_cnt = 0;
        }
    }
}// 为了适配回调函数定义

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision_Adapter;
    conf.recv_buff_size = sizeof(Vision_Recv_Packet_t);
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}


#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(const Vision_Recv_Packet_t *pkt, Vision_Recv_s *out)
{
    out->tracking    = pkt->tracking;
    out->armor_id    = pkt->id;
    out->armor_num   = pkt->armors_num;

    out->x = pkt->x;
    out->y = pkt->y;
    out->z = pkt->z;
    out->yaw = pkt->yaw;

    out->vx = pkt->vx;
    out->vy = pkt->vy;
    out->vz = pkt->vz;
    out->v_yaw = pkt->v_yaw;

    out->r1 = pkt->r1;
    out->r2 = pkt->r2;
    out->dz = pkt->dz;
}

static void DecodeVision_Adapter(uint16_t recv_len)
{
    uint8_t *buf = vis_recv_buff;   // USBInit 返回的接收缓冲区

    for (uint16_t i = 0; i < recv_len; i++)
    {
        uint8_t byte = buf[i];

        if (vision_rx_cnt == 0)
        {
            if (byte != 0xA5)
                continue;
        }

        vision_rx_buf[vision_rx_cnt++] = byte;

        if (vision_rx_cnt > sizeof(Vision_Recv_Packet_t))
        {
            vision_rx_cnt = 0;
            continue;
        }

        if (vision_rx_cnt == sizeof(Vision_Recv_Packet_t))
        {
            pkt_recv = (Vision_Recv_Packet_t *)vision_rx_buf;

            
            if (CRC16_Verify((uint8_t *)pkt_recv,sizeof(Vision_Recv_Packet_t)))
            {
                vision_crc_cnt++;
                DecodeVision(pkt_recv, &recv_data);
                DaemonReload(vision_daemon_instance);
            }

            vision_rx_cnt = 0;
        }
    }
}// 为了适配USB回调函数定义

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{

    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision_Adapter,};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

#endif // VISION_USE_VCP

// 发送视觉数据
void VisionSend()
{
    static Vision_Send_Packet_t pkt;

    pkt.header = 0x5A;
    pkt.aim_x = 100.0f; // 默认发一个远点
    pkt.aim_y = 100.0f;
    pkt.aim_z = 100.0f;

    pkt.roll  = 123.0f;
    pkt.pitch = 123.0f;
    pkt.yaw   = 123.0f;

    pkt.detect_color = 0;  // 0:blue
    pkt.reset_tracking = 0;

    CRC16_Append((uint8_t *)&pkt, sizeof(Vision_Send_Packet_t));

#ifdef VISION_USE_UART
    USARTSend(vision_usart_instance,
              (uint8_t *)&pkt,
              sizeof(Vision_Send_Packet_t),
              USART_TRANSFER_IT);
#endif

#ifdef VISION_USE_VCP
    USBTransmit((uint8_t *)&pkt,
                sizeof(Vision_Send_Packet_t));
#endif
}

// 将当前roll,pitch,yaw写入发送数据结构体
void setVisionSendData_rpy(float roll,float pitch,float yaw)
{
    send_data.roll = roll;
    send_data.pitch = pitch;
    send_data.yaw = yaw;
}
// 将目标预测aim_x,aim_y,aim_z写入发送数据结构体
void setVisionSendData_aim(float aim_x,float aim_y,float aim_z)
{
    send_data.aim_x = aim_x;
    send_data.aim_y = aim_y;
    send_data.aim_z = aim_z;
}
