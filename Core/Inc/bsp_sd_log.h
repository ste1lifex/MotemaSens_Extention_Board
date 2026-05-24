#ifndef __BSP_SD_LOG_H
#define __BSP_SD_LOG_H

#include "main.h"

typedef enum
{
    SD_LOG_OK = 0,
    SD_LOG_ERR_SD,
    SD_LOG_ERR_FULL,
    SD_LOG_ERR_IO
} SD_Log_Status_t;

/* 开机调用：从扇区 2 起覆盖写入新会话（不保留上次数据） */
SD_Log_Status_t SD_Log_Init(void);

uint8_t SD_Log_IsReady(void);

/* 写入一个 5 s 测量周期：含全部 IR/RD 采样点及 HR/SpO2/体温 */
SD_Log_Status_t SD_Log_WriteCycle(uint32_t cycle_index,
                                  uint32_t cycle_ms,
                                  int32_t hr,
                                  int8_t hr_valid,
                                  int32_t spo2,
                                  int8_t spo2_valid,
                                  float temp_c,
                                  uint8_t temp_valid,
                                  const uint32_t *ir_buf,
                                  const uint32_t *red_buf,
                                  uint16_t sample_count);

#endif /* __BSP_SD_LOG_H */
