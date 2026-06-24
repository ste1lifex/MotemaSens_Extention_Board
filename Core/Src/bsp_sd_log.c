#include "bsp_sd_log.h"
#include "bsp_sd_spi.h"
#include "algorithm.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

#define SD_LOG_FILE_PATH   "DATA.CSV"
#define SD_LOG_MKFS_WORK   4096U
/* 每 N 个周期 f_sync 一次，减轻写盘阻塞（调试数据仍逐行写入） */
#define SD_LOG_SYNC_EVERY_CYCLES  5U

static FATFS s_fatfs;
static FIL s_fil;
static uint8_t s_log_ready = 0U;
static uint8_t s_sync_cycle_count = 0U;

static SD_Log_Status_t SD_Log_MapResult(FRESULT fr)
{
    switch (fr)
    {
    case FR_OK:
        return SD_LOG_OK;
    case FR_NOT_READY:
    case FR_NO_FILESYSTEM:
        return SD_LOG_ERR_SD;
    case FR_DENIED:
        return SD_LOG_ERR_FULL;
    default:
        return SD_LOG_ERR_IO;
    }
}

static SD_Log_Status_t SD_Log_MountOrFormat(void)
{
    FRESULT fr;
    static BYTE work[SD_LOG_MKFS_WORK];
    MKFS_PARM opt = {0};

    fr = f_mount(&s_fatfs, "", 1);
    if (fr == FR_NO_FILESYSTEM)
    {
        opt.fmt = FM_FAT32;
        opt.n_fat = 1U;
        opt.align = 0U;
        opt.n_root = 0U;
        opt.au_size = 0U;

        fr = f_mkfs("", &opt, work, sizeof(work));
        if (fr != FR_OK)
        {
            return SD_Log_MapResult(fr);
        }

        fr = f_mount(&s_fatfs, "", 1);
    }

    if (fr != FR_OK)
    {
        return SD_Log_MapResult(fr);
    }

    return SD_LOG_OK;
}

SD_Log_Status_t SD_Log_Init(void)
{
    FRESULT fr;
    UINT bw = 0U;
    const char *header =
        "cycle,elapsed_ms,hr,hr_valid,spo2,spo2_valid,temp_C,temp_valid,sample_idx,ir,rd\r\n";

    s_log_ready = 0U;

    if (SD_IsReady() == 0U)
    {
        return SD_LOG_ERR_SD;
    }

    if (SD_Log_MountOrFormat() != SD_LOG_OK)
    {
        return SD_LOG_ERR_SD;
    }

    fr = f_open(&s_fil, SD_LOG_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        return SD_Log_MapResult(fr);
    }

    fr = f_write(&s_fil, header, (UINT)strlen(header), &bw);
    if ((fr != FR_OK) || (bw != strlen(header)))
    {
        (void)f_close(&s_fil);
        return SD_LOG_ERR_IO;
    }

    fr = f_sync(&s_fil);
    if (fr != FR_OK)
    {
        (void)f_close(&s_fil);
        return SD_Log_MapResult(fr);
    }

    s_log_ready = 1U;
    return SD_LOG_OK;
}

uint8_t SD_Log_IsReady(void)
{
    return s_log_ready;
}

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
                                  uint16_t sample_count)
{
    char line[96];
    FRESULT fr;
    UINT bw = 0U;
    uint16_t i;

    if (s_log_ready == 0U)
    {
        return SD_LOG_ERR_SD;
    }

    if (sample_count > BUFFER_SIZE)
    {
        sample_count = BUFFER_SIZE;
    }

    if ((sample_count == 0U) || (ir_buf == NULL) || (red_buf == NULL))
    {
        (void)snprintf(line, sizeof(line),
                       "%lu,%lu,%ld,%d,%ld,%d,%.2f,%u,,,\r\n",
                       (unsigned long)cycle_index,
                       (unsigned long)cycle_ms,
                       (long)hr,
                       (int)hr_valid,
                       (long)spo2,
                       (int)spo2_valid,
                       (double)temp_c,
                       (unsigned int)temp_valid);
        fr = f_write(&s_fil, line, (UINT)strlen(line), &bw);
        if (fr != FR_OK)
        {
            return SD_Log_MapResult(fr);
        }
    }
    else
    {
        for (i = 0U; i < sample_count; i++)
        {
            (void)snprintf(line, sizeof(line),
                           "%lu,%lu,%ld,%d,%ld,%d,%.2f,%u,%u,%lu,%lu\r\n",
                           (unsigned long)cycle_index,
                           (unsigned long)cycle_ms,
                           (long)hr,
                           (int)hr_valid,
                           (long)spo2,
                           (int)spo2_valid,
                           (double)temp_c,
                           (unsigned int)temp_valid,
                           (unsigned int)i,
                           (unsigned long)ir_buf[i],
                           (unsigned long)red_buf[i]);
            fr = f_write(&s_fil, line, (UINT)strlen(line), &bw);
            if (fr != FR_OK)
            {
                return SD_Log_MapResult(fr);
            }
        }
    }

    s_sync_cycle_count++;
    if (s_sync_cycle_count >= SD_LOG_SYNC_EVERY_CYCLES)
    {
        s_sync_cycle_count = 0U;
        fr = f_sync(&s_fil);
        return SD_Log_MapResult(fr);
    }

    return SD_LOG_OK;
}
