#ifndef __BSP_GD60932_H
#define __BSP_GD60932_H

#include "main.h"

#define GD60932_I2C_ADDR_7BIT    0x18U

#define GD60932_REG_FOREHEAD     0x80U
#define GD60932_REG_WRIST        0x81U
#define GD60932_REG_READ_TEMP    0x1CU
#define GD60932_CMD_MEASURE      0x1AU
#define GD60932_CMD_OBJECT       0x1FU
#define GD60932_CMD_AMBIENT      0x1EU

typedef enum
{
    GD60932_OK = 0,
    GD60932_ERR_I2C = 1,
    GD60932_ERR_DATA = 2
} GD60932_Status_t;

GD60932_Status_t GD60932_Init(void);
/* 额头温度模式：0x80 → 0x1A → 延时 0.5s → 读 0x1C */
GD60932_Status_t GD60932_ReadBodyTemperature(float *temp_c);
GD60932_Status_t GD60932_ReadObjectTemperature(float *temp_c);
GD60932_Status_t GD60932_ReadAmbientTemperature(float *temp_c);
uint32_t GD60932_GetLastI2CError(void);
uint8_t GD60932_GetLastReadyAddr(void);

#endif
