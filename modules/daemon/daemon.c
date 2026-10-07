#include "daemon.h"
#include "bsp_dwt.h" // 后续通过定时器来计时?
#include "stdlib.h"
#include "memory.h"
#include "buzzer.h"
#include "main.h"     // __disable_irq/__enable_irq,用于保护temp_count的读-改-写(F8)
#include "bsp_log.h"  // LOGERROR/BSP_FaultCode

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    if (instance == NULL) // 初始化期堆分配失败属致命配置错误(R4)
    {
        LOGERROR("[daemon] DaemonRegister malloc failed, check configTOTAL_HEAP_SIZE");
        BSP_FaultCode = 1;
        while (1)
            ;
    }
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数

    if (idx >= DAEMON_MX_CNT) // 注册表越界检查,不越界写(R6),响亮停机(R5)
    {
        LOGERROR("[daemon] daemon instance exceeded MAX num [%d]", DAEMON_MX_CNT);
        BSP_FaultCode = 1;
        while (1)
            ;
    }
    daemon_instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0;
}

void DaemonTask()
{
    DaemonInstance *dins; // 提高可读性同时降低访存开销
    uint8_t offline;
    for (size_t i = 0; i < idx; ++i)
    {
        dins = daemon_instances[i];
        offline = 0;
        __disable_irq(); // 与CAN ISR的DaemonReload()互斥,保护任务侧temp_count的读-改-写(F8)
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
        {
            dins->temp_count--;
            dins->reported = 0; // 在线,重新武装离线上报标志
        }
        else
            offline = 1;       // 等于零说明超时了
        __enable_irq();

        if (offline && !dins->reported && dins->callback)
        { // 超时只回调一次(F3:原实现每拍重复,8电机x100Hz约800条/秒刷屏);喂狗后重新武装
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
            dins->reported = 1;
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}
// (需要id的原因是什么?) 下面是copilot的回答!
// 需要id的原因是因为有些module可能有多个实例,而我们需要知道具体是哪个实例offline
// 如果只有一个实例,则可以不用id,直接调用callback即可
// 比如: 有一个module叫做"电机",它有两个实例,分别是"电机1"和"电机2",那么我们调用电机的离线处理函数时就需要知道是哪个电机offline
