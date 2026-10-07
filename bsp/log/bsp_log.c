#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>

/* 初始化期致命错误停机标志:非零表示固件因配置错误停机(LOGERROR+while(1)路径均置1).
 * DISABLE_LOG_SYSTEM 关闭日志时,用调试器/Ozone观察此变量即可发现停机原因(具体位置看PC/调用栈) */
volatile uint8_t BSP_FaultCode = 0;

void BSPLogInit()
{
    SEGGER_RTT_Init();
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

