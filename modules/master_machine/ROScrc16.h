#ifndef __ROSCRC16_H
#define __ROSCRC16_H

#include <stdint.h>

// CRC16校验验证函数
uint8_t  CRC16_Verify(const uint8_t *buf, uint32_t len);
// CRC16校验追加函数
void CRC16_Append(uint8_t *buf, uint32_t len);

#endif
