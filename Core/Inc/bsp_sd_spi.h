#ifndef __BSP_SD_SPI_H
#define __BSP_SD_SPI_H

#include "main.h"

/* TF 卡槽：SPI2，PB12=CS，PB13=SCK，PC2=MISO，PC3=MOSI */
#define SD_CS_PORT      GPIOB
#define SD_CS_PIN       GPIO_PIN_12

#define SD_BLOCK_SIZE   512U

typedef enum
{
    SD_OK = 0,
    SD_ERR_TIMEOUT,
    SD_ERR_CMD,
    SD_ERR_DATA,
    SD_ERR_NOT_READY,
    SD_ERR_NO_CARD
} SD_Status_t;

typedef enum
{
    SD_TYPE_UNKNOWN = 0,
    SD_TYPE_V1,
    SD_TYPE_V2,
    SD_TYPE_SDHC
} SD_CardType_t;

typedef struct
{
    SD_CardType_t type;
    uint32_t block_size;
    uint32_t block_count;
    uint64_t capacity_bytes;
} SD_CardInfo_t;

void Bsp_SD_SPI_Init(void);
SD_Status_t SD_Init(void);
SD_Status_t SD_ReadBlock(uint32_t block_addr, uint8_t *buf);
SD_Status_t SD_WriteBlock(uint32_t block_addr, const uint8_t *buf);
SD_Status_t SD_ReadBlocks(uint32_t block_addr, uint8_t *buf, uint32_t block_count);
SD_Status_t SD_WriteBlocks(uint32_t block_addr, const uint8_t *buf, uint32_t block_count);
SD_Status_t SD_GetCardInfo(SD_CardInfo_t *info);
uint8_t SD_IsReady(void);

#endif /* __BSP_SD_SPI_H */
