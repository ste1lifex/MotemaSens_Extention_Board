/**
  ******************************************************************************
  * @file    algorithm.c
  * @author  Maxim Integrated
  * @version V1.0
  * @date    2024
  * @brief   心率和血氧计算算法 - C语言版本
  ******************************************************************************
  */
#include "algorithm.h"

/* SpO2计算公式: -45.060*ratioAverage*ratioAverage + 30.354*ratioAverage + 94.845 */

/* 静态变量 */
static int32_t an_x[BUFFER_SIZE];              // ir
static int32_t an_y[BUFFER_SIZE];              // red
#if ALGO_USE_IR_BANDPASS
static float s_bp_work[BUFFER_SIZE]; /* 去直流浮点 + 带通中间量 */
#endif

/* 辅助宏 */
#define min(x, y) ((x) < (y) ? (x) : (y))

#if ALGO_USE_IR_BANDPASS
/* Butterworth bandpass 2nd order, 0.5~4.5 Hz, Fs=100 Hz（scipy.signal.butter 系数） */
static void ppg_lfilter_one_section(const float *in, float *out, int32_t len,
                                    float b0, float b1, float b2, float a1, float a2)
{
    int32_t i;
    float wim1 = 0.0f;
    float wim2 = 0.0f;
    float yim1 = 0.0f;
    float yim2 = 0.0f;

    for (i = 0; i < len; i++)
    {
        float wi = in[i];
        float yi = (b0 * wi) + (b1 * wim1) + (b2 * wim2) - (a1 * yim1) - (a2 * yim2);
        wim2 = wim1;
        wim1 = wi;
        yim2 = yim1;
        yim1 = yi;
        out[i] = yi;
    }
}

static void ppg_ir_bandpass_zero_mean(float *io, int32_t len)
{
    static float stage[BUFFER_SIZE];
    /* [b0 b1 b2 a0 a1 a2] a0=1 */
    const float sos0[6] = { 0.0133592f, 0.0267184f, 0.0133592f, 1.0f, -1.6733217f, 0.73084409f };
    const float sos1[6] = { 1.0f, -2.0f, 1.0f, 1.0f, -1.95786499f, 0.95902367f };

    ppg_lfilter_one_section(io, stage, len, sos0[0], sos0[1], sos0[2], sos0[4], sos0[5]);
    ppg_lfilter_one_section(stage, io, len, sos1[0], sos1[1], sos1[2], sos1[4], sos1[5]);
}
#endif /* ALGO_USE_IR_BANDPASS */

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length,
                                            uint32_t *pun_red_buffer, int32_t *pn_spo2,
                                            int8_t *pch_spo2_valid, int32_t *pn_heart_rate,
                                            int8_t *pch_hr_valid)
{
    uint32_t un_ir_mean;
    int32_t k, n_i_ratio_count;
    int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks;   
    int32_t an_ir_valley_locs[15] ;
    int32_t n_peak_interval_sum;
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx = 0;
    int32_t n_x_dc_max_idx = 0; 
    int32_t an_ratio[5], n_ratio_average; 
    int32_t n_nume, n_denom ;

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        un_ir_mean += pun_ir_buffer[k];
    }
    un_ir_mean = un_ir_mean / n_ir_buffer_length;

#if ALGO_USE_IR_BANDPASS
    /* 先去直流再带通（约 0.5~4.5 Hz），抑制基线漂移与高频噪声；找峰用，血氧仍用下方原始缓冲 */
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        s_bp_work[k] = (float)pun_ir_buffer[k] - (float)un_ir_mean;
    }
    ppg_ir_bandpass_zero_mean(s_bp_work, n_ir_buffer_length);
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        float v = s_bp_work[k];
        if (v > 32767.0f)
        {
            v = 32767.0f;
        }
        if (v < -32768.0f)
        {
            v = -32768.0f;
        }
        an_x[k] = -(int32_t)lroundf(v);
    }
#else
    /* remove DC and invert signal so that we can use peak detector as valley detector */
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        an_x[k] = -1 * ((int32_t)pun_ir_buffer[k] - (int32_t)un_ir_mean);
    }

    /* 4 pt Moving Average */
    for (k = 0; k < n_ir_buffer_length - MA4_SIZE; k++)
    {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int32_t)4;
    }
#endif

    /* calculate threshold */
    n_th1 = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        n_th1 += an_x[k];
    }
    n_th1 = n_th1 / n_ir_buffer_length;
#if ALGO_USE_IR_BANDPASS
    if (n_th1 < 5)
    {
        n_th1 = 5;
    }
    if (n_th1 > 400)
    {
        n_th1 = 400;
    }
#else
    if (n_th1 < 20)
    {
        n_th1 = 20;
    }
    if (n_th1 > 120)
    {
        n_th1 = 120;
    }
#endif

    for (k = 0; k < 15; k++) 
        an_ir_valley_locs[k] = 0;
    
    // since we flipped signal, we use peak detector as valley detector
    maxim_find_peaks(an_ir_valley_locs, &n_npks, an_x, n_ir_buffer_length, n_th1, 4, 15); // peak_height, peak_distance, max_num_peaks 
    
    n_peak_interval_sum = 0;
    if (n_npks >= 2)
    {
        for (k = 1; k < n_npks; k++) 
            n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = (int32_t)((FS * 60) / n_peak_interval_sum);
        *pch_hr_valid = 1;
    }
    else
    { 
        *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
        *pch_hr_valid = 0;
    }

    // load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        an_x[k] = pun_ir_buffer[k]; 
        an_y[k] = pun_red_buffer[k]; 
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = n_npks; 
    
    // using exact_ir_valley_locs, find ir-red DC and ir-red AC for SPO2 calibration an_ratio
    // finding AC/DC maximum of raw

    n_ratio_average = 0; 
    n_i_ratio_count = 0; 
    for (k = 0; k < 5; k++) 
        an_ratio[k] = 0;
    
    for (k = 0; k < n_exact_ir_valley_locs_count; k++)
    {
        if (an_ir_valley_locs[k] > n_ir_buffer_length)
        {
            *pn_spo2 = -999; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid = 0; 
            return;
        }
    }
    
    // find max between two valley locations 
    // and use an_ratio between AC component of Ir & Red and DC component of Ir & Red for SPO2 
    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++)
    {
        n_y_dc_max = -16777216; 
        n_x_dc_max = -16777216; 
        if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3)
        {
            for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++)
            {
                if (an_x[i] > n_x_dc_max) 
                {
                    n_x_dc_max = an_x[i]; 
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max) 
                {
                    n_y_dc_max = an_y[i]; 
                    n_y_dc_max_idx = i;
                }
            }
            n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]]) * (n_y_dc_max_idx - an_ir_valley_locs[k]); // red
            n_y_ac = an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]); 
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;    // subtracting linear DC components from raw 
            n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_ir_valley_locs[k]); // ir
            n_x_ac = an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]); 
            n_x_ac = an_x[n_x_dc_max_idx] - n_x_ac;      // subtracting linear DC components from raw 
            n_nume = (n_y_ac * n_x_dc_max) >> 7; // prepare X100 to preserve floating value
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0)
            {   
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom; // formula is (n_y_ac * n_x_dc_max) / (n_x_ac * n_y_dc_max);
                n_i_ratio_count++;
            }
        }
    }
    
    // choose median value since PPG signal may varies from beat to beat
    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx];

    // 使用公式直接计算SpO2: -45.060*ratioAverage*ratioAverage + 30.354*ratioAverage + 94.845
    // ratio_average范围通常在0.4到1.8之间对应SpO2 70-100%
    if (n_ratio_average > 0 && n_ratio_average < 200)
    {
        // 将整数比率转换为浮点数进行计算
        float f_ratio = (float)n_ratio_average / 100.0f;
        float f_spo2 = (-45.060f * f_ratio + 30.354f) * f_ratio + 94.845f;
        
        // 限制SpO2在合理范围内（70-100%）
        if (f_spo2 < 70.0f)
            f_spo2 = 70.0f;
        else if (f_spo2 > 100.0f)
            f_spo2 = 100.0f;
        
        *pn_spo2 = (int32_t)(f_spo2 + 0.5f); // 四舍五入到整数
        *pch_spo2_valid = 1;
    }
    else
    {
        *pn_spo2 = -999; // do not use SPO2 since signal an_ratio is out of range
        *pch_spo2_valid = 0; 
    }
}

void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size,
                     int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
{
    maxim_peaks_above_min_height(pn_locs, pn_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, pn_npks, pn_x, n_min_distance);
    *pn_npks = min(*pn_npks, n_max_num);
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x,
                                  int32_t n_size, int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size - 1)
    {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])
        { // find left edge of potential peaks
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i + n_width] && (*pn_npks) < 15)
            { // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width + 1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x,
                              int32_t n_min_distance)
{
    int32_t i, j, n_old_npks, n_dist;
    
    /* Order peaks from large to small */
    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);
    
    for (i = -1; i < *pn_npks; i++)
    {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++)
        {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorr is at index -1
            if (n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }
    
    // Resort indices into ascending order
    maxim_sort_ascend(pn_locs, *pn_npks);
}

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}


