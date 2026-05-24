#include "bsp_gd60932.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c2;

#define GD60932_I2C_TIMEOUT_MS   200U
/* 手册：触发后等待 0.5s 再读 0x1C；略加长以提高成功率 */
#define GD60932_MEASURE_WAIT_MS  600U
#define GD60932_RETRY_COUNT      5U
#define GD60932_TEMP_READ_RETRIES 16U
#define GD60932_TEMP_READ_RETRY_MS 35U
/* 手册应答占位：勿当作 0x1C 温度原始值 */
#define GD60932_ACK_FOREHEAD_U16 0x8080U
#define GD60932_ACK_WRIST_U16    0x8181U
#define GD60932_ACK_TRIGGER_U16  0x55AAU
#define GD60932_HAL_ADDR(addr7)  ((uint16_t)((addr7) << 1))

static uint32_t s_last_i2c_error = 0U;
static uint8_t s_last_ready_addr = 0U;
/* Init 时探测到的 7 位从机地址；读函数必须用此地址，不能写死 0x18 */
static uint8_t s_dev_addr7 = GD60932_I2C_ADDR_7BIT;

static GD60932_Status_t GD60932_ReadU16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef hal_st;
    uint8_t retry = 0U;
    uint16_t dev_hal = (uint16_t)(s_dev_addr7 << 1U);

    for (retry = 0U; retry < GD60932_RETRY_COUNT; retry++)
    {
        /* Mem_Read：写寄存器地址 + 重复起始 + 读 2 字节，与手册“读寄存器”时序一致 */
        hal_st = HAL_I2C_Mem_Read(&hi2c2, dev_hal, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, GD60932_I2C_TIMEOUT_MS);
        if (hal_st == HAL_OK)
        {
            *value = ((uint16_t)buf[1] << 8) | buf[0];
            return GD60932_OK;
        }

        s_last_i2c_error = HAL_I2C_GetError(&hi2c2);
    }

    return GD60932_ERR_I2C;
}

static uint8_t GD60932_TempRawIsValid(uint16_t raw)
{
    if ((raw == GD60932_ACK_FOREHEAD_U16) || (raw == GD60932_ACK_WRIST_U16) ||
        (raw == GD60932_ACK_TRIGGER_U16))
    {
        return 0U;
    }
    return 1U;
}

static GD60932_Status_t GD60932_CommandThenReadTemp(uint8_t command, float *temp_c)
{
    uint16_t ack = 0U;
    uint16_t raw = 0U;
    int16_t signed_raw = 0;
    GD60932_Status_t st = GD60932_OK;

    if (temp_c == NULL)
    {
        return GD60932_ERR_DATA;
    }

    st = GD60932_ReadU16(command, &ack);
    if (st != GD60932_OK)
    {
        return st;
    }

    HAL_Delay(GD60932_MEASURE_WAIT_MS);

    st = GD60932_ReadU16(GD60932_REG_READ_TEMP, &raw);
    if (st != GD60932_OK)
    {
        return st;
    }

    if (GD60932_TempRawIsValid(raw) == 0U)
    {
        return GD60932_ERR_DATA;
    }

    signed_raw = (int16_t)raw;
    *temp_c = (float)signed_raw / 10.0f;
    return GD60932_OK;
}

GD60932_Status_t GD60932_Init(void)
{
    uint16_t mode = 0U;
    uint8_t probe_addrs[2] = {GD60932_I2C_ADDR_7BIT, 0x19U};
    uint8_t i = 0U;
    HAL_StatusTypeDef st = HAL_ERROR;

    s_last_i2c_error = 0U;
    s_last_ready_addr = 0U;
    s_dev_addr7 = GD60932_I2C_ADDR_7BIT;

    for (i = 0U; i < 2U; i++)
    {
        st = HAL_I2C_IsDeviceReady(&hi2c2, GD60932_HAL_ADDR(probe_addrs[i]), 3, GD60932_I2C_TIMEOUT_MS);
        if (st == HAL_OK)
        {
            s_last_ready_addr = GD60932_HAL_ADDR(probe_addrs[i]);
            s_dev_addr7 = probe_addrs[i];
            break;
        }
        s_last_i2c_error = HAL_I2C_GetError(&hi2c2);
    }

    if (st != HAL_OK)
    {
        return GD60932_ERR_I2C;
    }

    /* 读取额头温度模式寄存器用于探测器件在线状态 */
    return GD60932_ReadU16(GD60932_REG_FOREHEAD, &mode);
}

GD60932_Status_t GD60932_ReadBodyTemperature(float *temp_c)
{
    uint16_t forehead_ack = 0U;
    uint16_t trigger_ack = 0U;
    uint16_t raw = 0U;
    uint8_t attempt = 0U;
    int16_t signed_raw = 0;
    GD60932_Status_t st = GD60932_OK;

    if (temp_c == NULL)
    {
        return GD60932_ERR_DATA;
    }

    /* 额头温度模式：读 0x80 → 读 0x1A → 延时 → 读 0x1C */
    st = GD60932_ReadU16(GD60932_REG_FOREHEAD, &forehead_ack);
    if (st != GD60932_OK)
    {
        return st;
    }

    st = GD60932_ReadU16(GD60932_CMD_MEASURE, &trigger_ack);
    if (st != GD60932_OK)
    {
        return st;
    }

    HAL_Delay(GD60932_MEASURE_WAIT_MS);

    for (attempt = 0U; attempt < GD60932_TEMP_READ_RETRIES; attempt++)
    {
        st = GD60932_ReadU16(GD60932_REG_READ_TEMP, &raw);
        if (st != GD60932_OK)
        {
            return st;
        }
        if (GD60932_TempRawIsValid(raw) != 0U)
        {
            break;
        }
        if ((attempt + 1U) >= GD60932_TEMP_READ_RETRIES)
        {
            return GD60932_ERR_DATA;
        }
        HAL_Delay(GD60932_TEMP_READ_RETRY_MS);
    }

    signed_raw = (int16_t)raw;
    *temp_c = (float)signed_raw / 10.0f;
    return GD60932_OK;
}

GD60932_Status_t GD60932_ReadObjectTemperature(float *temp_c)
{
    return GD60932_CommandThenReadTemp(GD60932_CMD_OBJECT, temp_c);
}

GD60932_Status_t GD60932_ReadAmbientTemperature(float *temp_c)
{
    return GD60932_CommandThenReadTemp(GD60932_CMD_AMBIENT, temp_c);
}

uint32_t GD60932_GetLastI2CError(void)
{
    return s_last_i2c_error;
}

uint8_t GD60932_GetLastReadyAddr(void)
{
    return s_last_ready_addr;
}
