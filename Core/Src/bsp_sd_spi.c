#include "bsp_sd_spi.h"
#include "spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

#define SD_SPI_TIMEOUT_MS       200U
#define SD_CMD_TIMEOUT_MS       500U
#define SD_DATA_TIMEOUT_MS      1000U

#define SD_SPI_PRESCALER_SLOW   SPI_BAUDRATEPRESCALER_64
#define SD_SPI_PRESCALER_FAST   SPI_BAUDRATEPRESCALER_2

#define SD_TOKEN_START_BLOCK    0xFEU
#define SD_TOKEN_WRITE_OK       0x05U
#define SD_TOKEN_WRITE_ERR      0x0DU

static SD_CardInfo_t s_card_info = {
    SD_TYPE_UNKNOWN,
    0U,
    0U,
    0ULL
};
static uint8_t s_sd_ready = 0U;

static void SD_SpiSetPrescaler(uint32_t prescaler)
{
    if (hspi2.Init.BaudRatePrescaler == prescaler)
    {
        return;
    }

    (void)HAL_SPI_DeInit(&hspi2);
    hspi2.Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

static uint8_t SD_SpiExchange(uint8_t tx)
{
    uint8_t rx = 0xFFU;

    if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, SD_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return 0xFFU;
    }

    return rx;
}

static void SD_SpiTx(const uint8_t *tx, uint16_t len)
{
    (void)HAL_SPI_Transmit(&hspi2, (uint8_t *)tx, len, SD_SPI_TIMEOUT_MS);
}

static void SD_SpiRx(uint8_t *rx, uint16_t len)
{
    uint8_t tx = 0xFFU;

    for (uint16_t i = 0U; i < len; i++)
    {
        rx[i] = SD_SpiExchange(tx);
    }
}

static SD_Status_t SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (SD_SpiExchange(0xFFU) == 0xFFU)
        {
            return SD_OK;
        }
    }

    return SD_ERR_TIMEOUT;
}

static void SD_Deselect(void)
{
    SD_CS_HIGH();
    (void)SD_SpiExchange(0xFFU);
}

static SD_Status_t SD_Select(void)
{
    SD_CS_LOW();
    if (SD_WaitReady(SD_CMD_TIMEOUT_MS) != SD_OK)
    {
        SD_Deselect();
        return SD_ERR_NOT_READY;
    }
    return SD_OK;
}

static uint8_t SD_ReadResponse1(void)
{
    uint8_t resp = 0xFFU;
    uint8_t retry = 0U;

    for (retry = 0U; retry < 10U; retry++)
    {
        resp = SD_SpiExchange(0xFFU);
        if ((resp & 0x80U) == 0U)
        {
            break;
        }
    }

    return resp;
}

static SD_Status_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *r1)
{
    uint8_t frame[6];

    if (SD_Select() != SD_OK)
    {
        return SD_ERR_NOT_READY;
    }

    frame[0] = 0x40U | cmd;
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)(arg);
    frame[5] = crc;
    SD_SpiTx(frame, 6U);

    if (cmd == 12U)
    {
        (void)SD_SpiExchange(0xFFU);
    }

    *r1 = SD_ReadResponse1();
    if (*r1 == 0xFFU)
    {
        SD_Deselect();
        return SD_ERR_TIMEOUT;
    }

    return SD_OK;
}

static SD_Status_t SD_SendAppCmd(uint8_t acmd, uint32_t arg, uint8_t *r1)
{
    uint8_t r1_cmd55 = 0xFFU;

    if (SD_SendCmd(55U, 0U, 0xFFU, &r1_cmd55) != SD_OK)
    {
        return SD_ERR_CMD;
    }

    return SD_SendCmd(acmd, arg, 0xFFU, r1);
}

static uint32_t SD_ParseCapacityFromCsd(const uint8_t *csd)
{
    if ((csd[0] & 0xC0U) == 0x40U)
    {
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3FU) << 16) |
                          ((uint32_t)csd[8] << 8) |
                          (uint32_t)csd[9];
        return (c_size + 1U) * 1024U;
    }

    {
        uint8_t read_bl_len = csd[5] & 0x0FU;
        uint16_t c_size = (uint16_t)(((csd[6] & 0x03U) << 10) |
                                     ((uint16_t)csd[7] << 2) |
                                     ((csd[8] & 0xC0U) >> 6));
        uint8_t c_size_mult = (uint8_t)(((csd[9] & 0x03U) << 1) | ((csd[10] & 0x80U) >> 7));
        uint32_t block_len = 1UL << read_bl_len;
        uint32_t mult = 1UL << (c_size_mult + 2U);

        return (uint32_t)(c_size + 1U) * mult * (block_len / SD_BLOCK_SIZE);
    }
}

static SD_Status_t SD_ReadDataBlock(uint8_t *buf)
{
    uint8_t token = 0xFFU;
    uint32_t start = HAL_GetTick();
    uint8_t crc_buf[2];

    while ((HAL_GetTick() - start) < SD_DATA_TIMEOUT_MS)
    {
        token = SD_SpiExchange(0xFFU);
        if (token == SD_TOKEN_START_BLOCK)
        {
            break;
        }
        if (token != 0xFFU)
        {
            SD_Deselect();
            return SD_ERR_DATA;
        }
    }

    if (token != SD_TOKEN_START_BLOCK)
    {
        SD_Deselect();
        return SD_ERR_TIMEOUT;
    }

    SD_SpiRx(buf, SD_BLOCK_SIZE);
    SD_SpiRx(crc_buf, 2U);
    SD_Deselect();
    return SD_OK;
}

static SD_Status_t SD_ReadRegister(uint8_t cmd, uint8_t *buf)
{
    uint8_t r1 = 0xFFU;

    if (SD_SendCmd(cmd, 0U, 0xFFU, &r1) != SD_OK)
    {
        return SD_ERR_CMD;
    }
    if (r1 != 0U)
    {
        SD_Deselect();
        return SD_ERR_CMD;
    }

    return SD_ReadDataBlock(buf);
}

static SD_Status_t SD_LoadCardInfo(void)
{
    uint8_t csd[SD_BLOCK_SIZE];
    SD_Status_t st = SD_ReadRegister(9U, csd);

    if (st != SD_OK)
    {
        return st;
    }

    s_card_info.block_size = SD_BLOCK_SIZE;
    s_card_info.block_count = SD_ParseCapacityFromCsd(csd);
    s_card_info.capacity_bytes = (uint64_t)s_card_info.block_count * SD_BLOCK_SIZE;
    return SD_OK;
}

void Bsp_SD_SPI_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = SD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SD_CS_PORT, &gpio);

    SD_CS_HIGH();
}

SD_Status_t SD_Init(void)
{
    uint8_t r1 = 0xFFU;
    uint32_t start = 0U;
    SD_Status_t st = SD_OK;
    uint8_t ocr[4] = {0};

    s_sd_ready = 0U;
    memset(&s_card_info, 0, sizeof(s_card_info));

    Bsp_SD_SPI_Init();
    SD_SpiSetPrescaler(SD_SPI_PRESCALER_SLOW);

    SD_CS_HIGH();
    for (uint8_t i = 0U; i < 10U; i++)
    {
        (void)SD_SpiExchange(0xFFU);
    }

    st = SD_SendCmd(0U, 0U, 0x95U, &r1);
    if ((st != SD_OK) || (r1 != 0x01U))
    {
        SD_Deselect();
        return SD_ERR_NO_CARD;
    }
    SD_Deselect();

    st = SD_SendCmd(8U, 0x000001AAU, 0x87U, &r1);
    if (st == SD_OK)
    {
        if (r1 == 0x01U)
        {
            uint8_t resp[4];
            SD_SpiRx(resp, 4U);
            SD_Deselect();

            if ((resp[2] == 0x01U) && (resp[3] == 0xAAU))
            {
                s_card_info.type = SD_TYPE_V2;
            }
            else
            {
                return SD_ERR_CMD;
            }
        }
        else if (r1 == 0x05U)
        {
            SD_Deselect();
            s_card_info.type = SD_TYPE_V1;
        }
        else
        {
            SD_Deselect();
            return SD_ERR_CMD;
        }
    }
    else
    {
        s_card_info.type = SD_TYPE_V1;
    }

    start = HAL_GetTick();
    do
    {
        if (s_card_info.type == SD_TYPE_V1)
        {
            st = SD_SendAppCmd(41U, 0U, &r1);
        }
        else
        {
            st = SD_SendAppCmd(41U, 0x40000000U, &r1);
        }

        if (st != SD_OK)
        {
            return st;
        }

        SD_Deselect();

        if (r1 == 0U)
        {
            break;
        }

        if ((HAL_GetTick() - start) > SD_CMD_TIMEOUT_MS)
        {
            return SD_ERR_TIMEOUT;
        }
    } while (1);

    if (s_card_info.type != SD_TYPE_V1)
    {
        st = SD_SendCmd(58U, 0U, 0xFFU, &r1);
        if (st != SD_OK)
        {
            return st;
        }

        SD_SpiRx(ocr, 4U);
        SD_Deselect();

        if (r1 != 0U)
        {
            return SD_ERR_CMD;
        }

        if (ocr[0] & 0x40U)
        {
            s_card_info.type = SD_TYPE_SDHC;
        }
    }
    else
    {
        st = SD_SendCmd(16U, SD_BLOCK_SIZE, 0xFFU, &r1);
        SD_Deselect();
        if ((st != SD_OK) || (r1 != 0U))
        {
            return SD_ERR_CMD;
        }
    }

    SD_SpiSetPrescaler(SD_SPI_PRESCALER_FAST);

    st = SD_LoadCardInfo();
    if (st != SD_OK)
    {
        return st;
    }

    s_sd_ready = 1U;
    return SD_OK;
}

SD_Status_t SD_ReadBlock(uint32_t block_addr, uint8_t *buf)
{
    uint8_t r1 = 0xFFU;
    SD_Status_t st = SD_OK;
    uint32_t addr = block_addr;

    if ((s_sd_ready == 0U) || (buf == NULL))
    {
        return SD_ERR_NOT_READY;
    }

    if (s_card_info.type != SD_TYPE_SDHC)
    {
        addr = block_addr * SD_BLOCK_SIZE;
    }

    st = SD_SendCmd(17U, addr, 0xFFU, &r1);
    if ((st != SD_OK) || (r1 != 0U))
    {
        SD_Deselect();
        return SD_ERR_CMD;
    }

    return SD_ReadDataBlock(buf);
}

SD_Status_t SD_WriteBlock(uint32_t block_addr, const uint8_t *buf)
{
    uint8_t r1 = 0xFFU;
    SD_Status_t st = SD_OK;
    uint32_t addr = block_addr;
    uint8_t token = 0xFFU;
    uint8_t crc_buf[2] = {0xFFU, 0xFFU};
    uint32_t start = 0U;

    if ((s_sd_ready == 0U) || (buf == NULL))
    {
        return SD_ERR_NOT_READY;
    }

    if (s_card_info.type != SD_TYPE_SDHC)
    {
        addr = block_addr * SD_BLOCK_SIZE;
    }

    st = SD_SendCmd(24U, addr, 0xFFU, &r1);
    if ((st != SD_OK) || (r1 != 0U))
    {
        SD_Deselect();
        return SD_ERR_CMD;
    }

    SD_SpiExchange(0xFFU);
    SD_SpiExchange(SD_TOKEN_START_BLOCK);
    SD_SpiTx(buf, SD_BLOCK_SIZE);
    SD_SpiTx(crc_buf, 2U);

    token = SD_SpiExchange(0xFFU);
    SD_Deselect();

    if ((token & 0x1FU) != SD_TOKEN_WRITE_OK)
    {
        return SD_ERR_DATA;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < SD_DATA_TIMEOUT_MS)
    {
        if (SD_WaitReady(10U) == SD_OK)
        {
            return SD_OK;
        }
    }

    return SD_ERR_TIMEOUT;
}

SD_Status_t SD_ReadBlocks(uint32_t block_addr, uint8_t *buf, uint32_t block_count)
{
    SD_Status_t st = SD_OK;

    if ((s_sd_ready == 0U) || (buf == NULL) || (block_count == 0U))
    {
        return SD_ERR_NOT_READY;
    }

    for (uint32_t i = 0U; i < block_count; i++)
    {
        st = SD_ReadBlock(block_addr + i, buf + (i * SD_BLOCK_SIZE));
        if (st != SD_OK)
        {
            return st;
        }
    }

    return SD_OK;
}

SD_Status_t SD_WriteBlocks(uint32_t block_addr, const uint8_t *buf, uint32_t block_count)
{
    SD_Status_t st = SD_OK;

    if ((s_sd_ready == 0U) || (buf == NULL) || (block_count == 0U))
    {
        return SD_ERR_NOT_READY;
    }

    for (uint32_t i = 0U; i < block_count; i++)
    {
        st = SD_WriteBlock(block_addr + i, buf + (i * SD_BLOCK_SIZE));
        if (st != SD_OK)
        {
            return st;
        }
    }

    return SD_OK;
}

SD_Status_t SD_GetCardInfo(SD_CardInfo_t *info)
{
    if ((info == NULL) || (s_sd_ready == 0U))
    {
        return SD_ERR_NOT_READY;
    }

    *info = s_card_info;
    return SD_OK;
}

uint8_t SD_IsReady(void)
{
    return s_sd_ready;
}
