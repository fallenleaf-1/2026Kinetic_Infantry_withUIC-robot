#include "can_comm.h"
#include "memory.h"
#include "stdlib.h"
#include "crc8.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance *ins)
{
    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state = 0;   // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 * @note  帧头重同步(附录A缺陷修复): 只要本帧是"帧头+长度匹配"的包头,无论当前是否正在收包,
 *        都从这一帧重新开始接收。原实现只在recv_state==0时接受帧头,因此发送端一旦因邮箱满
 *        丢掉中间某一帧,接收端就会永久停留在半包状态,后续所有包都被当作半包的续帧 →
 *        链路彻底失效(表现为恒判离线)。重同步后,丢帧最多损失一个周期。
 */
static void CANCommRxCallback(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id; // 注意写法,将can instance的id强制转换为CANCommInstance*类型

    /* 判断本帧是否为包头(帧头+datalen都匹配) */
    uint8_t is_header = (_instance->rx_len >= 2) &&
                        (_instance->rx_buff[0] == CAN_COMM_HEADER) &&
                        (_instance->rx_buff[1] == comm->recv_data_len);

    if (is_header)
    {
        if (comm->recv_state) // 之前有半包未收完: 丢弃并从这里重新开始
            comm->resync_cnt++;
        comm->recv_state = 1;
        comm->cur_recv_len = 0;
        memset(comm->raw_recvbuf, 0, comm->recv_buf_len);
    }
    else if (comm->recv_state == 0)
    {
        return; // 尚未开始接收,且本帧不是包头 → 直接丢弃
    }

    if (comm->recv_state) // 已经收到过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
            { // 通过校验,复制数据到unpack_data中
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
                { // 数据量大的话考虑使用DMA
                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
                    comm->update_flag = 1;           // 数据更新flag置为1
                    DaemonReload(comm->comm_daemon); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
                }
                else
                    comm->crc_err_cnt++;
            }
            else
                comm->tail_err_cnt++;
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }
    }
}

static void CANCommLostCallback(void *cancomm)
{
    CANCommInstance *comm = (CANCommInstance *)cancomm;
    CANCommResetRx(comm);
    LOGWARNING("[can_comm] can comm rx[%u] lost, reset rx state.", comm->can_ins->rx_id);
}

CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance));
    if (ins == NULL)
    { // 初始化期堆分配失败属致命配置错误,响亮停机(日志一次+故障标志)
        LOGERROR("[can_comm] CANCommInit malloc failed, check configTOTAL_HEAP_SIZE");
        BSP_FaultCode = 1;
        while (1)
            ;
    }
    memset(ins, 0, sizeof(CANCommInstance));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[0] = CAN_COMM_HEADER;            // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ins->raw_sendbuf[1] = comm_config->send_data_len; // datalen
    ins->raw_sendbuf[comm_config->send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;
    // can instance的设置
    comm_config->can_config.id = ins; // CANComm的实例指针作为CANInstance的id,回调函数中会用到
    comm_config->can_config.can_module_callback = CANCommRxCallback;
    ins->can_ins = CANRegister(&comm_config->can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = CANCommLostCallback,
        .owner_id = (void *)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    return ins;
}

/* 多帧发送参数(附录A缺陷修复)
 * 背景: 0x311 = 38+4 = 42B → 6帧, 0x312 = 21+4 = 25B → 4帧, 而bxCAN只有3个发送邮箱;
 *       原实现一次性连发、邮箱满即丢且调用方忽略返回值 → 接收端永远凑不齐一包(链路恒离线).
 * 现在: 每帧调用CANTransmitWait等空闲邮箱; 整包设CAN_COMM_TX_BUDGET_MS预算,超预算就作废本包
 *       (接收端有帧头重同步,半包会被丢弃),下个周期整包重发. 预算限制保证不会长时间阻塞调用任务
 *       (CANCommSend在200Hz的robot_cmd/ChassisTask里调用,周期5ms).
 * 实测: 总线空载时6帧≈0.7ms,预算2ms足够. */
#define CAN_COMM_TX_BUDGET_MS 2.0f

void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    static uint8_t crc8;
    static uint8_t send_len;
    static uint32_t tx_fail_cnt;
    float budget_start;

    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
    crc8 = crc_8(data, instance->send_data_len);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

    budget_start = DWT_GetTimeline_ms();
    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        float budget_left = CAN_COMM_TX_BUDGET_MS - (DWT_GetTimeline_ms() - budget_start);
        if (budget_left <= 0.0f)
        { // 预算用尽: 作废整包(已发出的前半包由接收端帧头重同步丢弃),下个周期重发
            tx_fail_cnt++;
            if (tx_fail_cnt == 1 || tx_fail_cnt % 200 == 0)
                LOGWARNING("[can_comm] tx aborted (no tx mailbox in %.1fms). Cnt [%u]",
                           (double)CAN_COMM_TX_BUDGET_MS, tx_fail_cnt);
            return;
        }
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        if (!CANTransmitWait(instance->can_ins, budget_left))
        { // 这一帧没能进邮箱: 整包作废,下个周期重发(不能只丢中间一帧)
            tx_fail_cnt++;
            if (tx_fail_cnt == 1 || tx_fail_cnt % 200 == 0)
                LOGWARNING("[can_comm] tx aborted at frame %u/%u. Cnt [%u]",
                           (unsigned)(i / 8 + 1), (unsigned)((instance->send_buf_len + 7) / 8), tx_fail_cnt);
            return;
        }
    }
}

void *CANCommGet(CANCommInstance *instance)
{
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

uint8_t CANCommIsOnline(CANCommInstance *instance)
{
    return DaemonIsOnline(instance->comm_daemon);
}
