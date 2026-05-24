#include "ff.h"
#include "diskio.h"
#include "bsp_sd_spi.h"

#define FATFS_SD_PDRV    0U

static DSTATUS s_disk_stat = STA_NOINIT;

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != FATFS_SD_PDRV)
    {
        return STA_NOINIT;
    }

    if (SD_IsReady() == 0U)
    {
        if (SD_Init() != SD_OK)
        {
            s_disk_stat = STA_NOINIT;
            return s_disk_stat;
        }
    }

    s_disk_stat = 0U;
    return s_disk_stat;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != FATFS_SD_PDRV)
    {
        return STA_NOINIT;
    }

    if (SD_IsReady() == 0U)
    {
        s_disk_stat = STA_NOINIT;
    }

    return s_disk_stat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != FATFS_SD_PDRV) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if (disk_status(pdrv) & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (SD_ReadBlocks((uint32_t)sector, buff, (uint32_t)count) != SD_OK)
    {
        return RES_ERROR;
    }

    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if ((pdrv != FATFS_SD_PDRV) || (buff == NULL) || (count == 0U))
    {
        return RES_PARERR;
    }

    if (disk_status(pdrv) & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (SD_WriteBlocks((uint32_t)sector, buff, (uint32_t)count) != SD_OK)
    {
        return RES_ERROR;
    }

    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    SD_CardInfo_t info;

    if (pdrv != FATFS_SD_PDRV)
    {
        return RES_PARERR;
    }

    if (disk_status(pdrv) & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT:
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        if (SD_GetCardInfo(&info) != SD_OK)
        {
            return RES_ERROR;
        }
        *(LBA_t *)buff = (LBA_t)info.block_count;
        return RES_OK;

    case GET_SECTOR_SIZE:
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        *(WORD *)buff = SD_BLOCK_SIZE;
        return RES_OK;

    case GET_BLOCK_SIZE:
        if (buff == NULL)
        {
            return RES_PARERR;
        }
        *(DWORD *)buff = 1U;
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
