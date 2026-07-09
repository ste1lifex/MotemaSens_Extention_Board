/**
  ******************************************************************************
  * @file    algorithm.c
  * @author  Maxim Integrated
  * @version V1.0
  * @date    2024
  * @brief   心率和血氧计算算法 - C语言版本 / HR and SpO2 algorithm - C version
  ******************************************************************************
  */
#include "algorithm.h"

/* SpO2计算公式: -45.060*ratioAverage*ratioAverage + 30.354*ratioAverage + 94.845
 * SpO2 formula: -45.060*ratioAverage^2 + 30.354*ratioAverage + 94.845 */

/* 静态变量 / Static variables */
static int32_t an_x[BUFFER_SIZE];              // ir
static int32_t an_y[BUFFER_SIZE];              // red
#if ALGO_USE_IR_BANDPASS
static float s_bp_work[BUFFER_SIZE]; /* 去直流浮点 + 带通中间量 / DC-removed float + bandpass workspace */
#endif

/* 辅助宏 / Helper macros */
#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))

/* SpO2 胸贴系数见 algorithm.h（ALGO_CHEST_MODE）
 * Chest SpO2 coefficients in algorithm.h (ALGO_CHEST_MODE) */

static void ppg_sanitize_samples(uint32_t *ir, uint32_t *red, int32_t len)
{
    int32_t k;
    uint32_t last_ir = 0U;
    uint32_t last_rd = 0U;

    for (k = 0; k < len; k++)
    {
        if (ir[k] != 0U)
        {
            last_ir = ir[k];
        }
        else if (last_ir != 0U)
        {
            ir[k] = last_ir;
        }

        if (red[k] != 0U)
        {
            last_rd = red[k];
        }
        else if (last_rd != 0U)
        {
            red[k] = last_rd;
        }
    }

    if (ir[0] == 0U)
    {
        for (k = 1; k < len; k++)
        {
            if (ir[k] != 0U)
            {
                ir[0] = ir[k];
                red[0] = red[k];
                break;
            }
        }
    }
}

static void ppg_linear_detrend(float *buf, int32_t len)
{
    int32_t k;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xy = 0.0f;
    float sum_xx = 0.0f;
    float denom;
    float slope;
    float intercept;
    float mx;

    if (len < 3)
    {
        return;
    }

    mx = (float)(len - 1) * 0.5f;
    for (k = 0; k < len; k++)
    {
        float x = (float)k - mx;
        sum_x += x;
        sum_y += buf[k];
        sum_xy += x * buf[k];
        sum_xx += x * x;
    }

    denom = ((float)len * sum_xx) - (sum_x * sum_x);
    if (denom == 0.0f)
    {
        return;
    }

    slope = (((float)len * sum_xy) - (sum_x * sum_y)) / denom;
    intercept = (sum_y - (slope * sum_x)) / (float)len;

    for (k = 0; k < len; k++)
    {
        buf[k] -= (slope * ((float)k - mx) + intercept);
    }
}

static int32_t ppg_median_int32(int32_t *vals, int32_t count)
{
    int32_t tmp[15];
    int32_t i;
    int32_t j;
    int32_t t;

    if (count <= 0)
    {
        return 0;
    }

    for (i = 0; i < count; i++)
    {
        tmp[i] = vals[i];
    }

    for (i = 1; i < count; i++)
    {
        t = tmp[i];
        for (j = i; j > 0 && t < tmp[j - 1]; j--)
        {
            tmp[j] = tmp[j - 1];
        }
        tmp[j] = t;
    }

    return tmp[count / 2];
}

static int8_t ppg_hr_from_intervals(int32_t *peak_locs, int32_t n_npks, int32_t *pn_hr)
{
    int32_t intervals[14];
    int32_t n_intervals = 0;
    int32_t k;
    int32_t median_rr;
    int32_t hr;

    if (n_npks < 2)
    {
        return 0;
    }

    for (k = 1; k < n_npks; k++)
    {
        int32_t dt = peak_locs[k] - peak_locs[k - 1];
        if ((dt >= ALGO_RR_MIN_SAMPLES) && (dt <= ALGO_RR_MAX_SAMPLES))
        {
            if (n_intervals < 14)
            {
                intervals[n_intervals++] = dt;
            }
        }
    }

    if (n_intervals < 1)
    {
        return 0;
    }

    median_rr = ppg_median_int32(intervals, n_intervals);
    if (median_rr <= 0)
    {
        return 0;
    }

    hr = (int32_t)((FS * 60) / median_rr);

#if ALGO_CHEST_MODE
    if (hr > 95)
    {
        hr = (int32_t)((FS * 60) / (median_rr * 2));
    }
    else if ((hr < 68) && (median_rr > 0))
    {
        int32_t hr2 = (int32_t)((FS * 60) * 2 / median_rr);
        if ((hr2 >= ALGO_CHEST_HR_PLAUSIBLE_LO) && (hr2 <= ALGO_CHEST_HR_PLAUSIBLE_HI))
        {
            hr = hr2;
        }
    }
#endif

    if ((hr < MIN_HR) || (hr > MAX_HR))
    {
        return 0;
    }

    *pn_hr = hr;
    return 1;
}

#if ALGO_CHEST_MODE
static int8_t ppg_hr_from_autocorr_range(const int32_t *sig, int32_t len, int32_t *pn_hr,
                                         int32_t hr_lo, int32_t hr_hi)
{
    int32_t lag;
    int32_t lo = (FS * 60) / hr_hi;
    int32_t hi = (FS * 60) / hr_lo;
    int32_t best_lag = 0;
    int64_t best_acc = -1;

    if (len < (hi + 20))
    {
        return 0;
    }

    for (lag = lo; lag <= hi; lag++)
    {
        int64_t acc = 0;
        int32_t i;
        for (i = 0; i < (len - lag); i++)
        {
            acc += (int64_t)sig[i] * (int64_t)sig[i + lag];
        }
        if (acc > best_acc)
        {
            best_acc = acc;
            best_lag = lag;
        }
    }

    if (best_lag < lo)
    {
        return 0;
    }

    *pn_hr = (int32_t)((FS * 60) / best_lag);
    if ((*pn_hr < MIN_HR) || (*pn_hr > MAX_HR))
    {
        return 0;
    }

    return 1;
}

static int8_t ppg_hr_from_autocorr(const int32_t *sig, int32_t len, int32_t *pn_hr)
{
    return ppg_hr_from_autocorr_range(sig, len, pn_hr, ALGO_CHEST_HR_AC_LO, ALGO_CHEST_HR_AC_HI);
}

static int8_t ppg_hr_in_plausible_band(int32_t hr)
{
    return (hr >= ALGO_CHEST_HR_PLAUSIBLE_LO) && (hr <= ALGO_CHEST_HR_PLAUSIBLE_HI);
}

static int8_t ppg_hr_fuse_chest(int32_t hr_peak, int8_t peak_ok, int32_t hr_ac, int8_t ac_ok,
                                int32_t *pn_hr)
{
    /* 自相关落在 70~85 优先（1.3.4：均值 76.6，略宽于 73~80）
     * Prefer autocorr HR in 70~85 (v1.3.4: mean 76.6, slightly wider than 73~80) */
    if ((ac_ok != 0) && (ppg_hr_in_plausible_band(hr_ac) != 0))
    {
        *pn_hr = hr_ac;
        return 1;
    }

    if ((peak_ok != 0) && (ac_ok != 0))
    {
        int32_t diff = hr_peak - hr_ac;
        if (diff < 0)
        {
            diff = -diff;
        }

        if (diff <= 10)
        {
            *pn_hr = (hr_peak + hr_ac) / 2;
            return 1;
        }

        if (ppg_hr_in_plausible_band(hr_ac) != 0)
        {
            *pn_hr = hr_ac;
            return 1;
        }

        if (ppg_hr_in_plausible_band(hr_peak) != 0)
        {
            *pn_hr = hr_peak;
            return 1;
        }

        *pn_hr = hr_ac;
        return 1;
    }

    if (ac_ok != 0)
    {
        *pn_hr = hr_ac;
        return (ppg_hr_in_plausible_band(hr_ac) != 0) ? 1 : 0;
    }

    if (peak_ok != 0)
    {
        *pn_hr = hr_peak;
        return (ppg_hr_in_plausible_band(hr_peak) != 0) ? 1 : 0;
    }

    return 0;
}

static float ppg_spo2_from_ratio_x100(int32_t ratio_x100)
{
    float f_ratio = (float)ratio_x100 / 100.0f;

#if ALGO_CHEST_MODE
    /* 胸贴：去趋势 MAD 之比 R 与指夹 R 不同量纲，用本地线性标定而非指夹二次曲线
     * Chest: detrended MAD ratio R differs from finger-clip R; use local linear cal, not quadratic */
    float f_spo2 = (ALGO_SPO2_CHEST_SLOPE * f_ratio) + ALGO_SPO2_CHEST_INTERCEPT;
    if (f_spo2 < ALGO_SPO2_CHEST_CLAMP_LO)
    {
        f_spo2 = ALGO_SPO2_CHEST_CLAMP_LO;
    }
    else if (f_spo2 > ALGO_SPO2_CHEST_CLAMP_HI)
    {
        f_spo2 = ALGO_SPO2_CHEST_CLAMP_HI;
    }
#else
    float f_spo2 = (-45.060f * f_ratio + 30.354f) * f_ratio + 94.845f;
    if (f_spo2 < 70.0f)
    {
        f_spo2 = 70.0f;
    }
    else if (f_spo2 > 100.0f)
    {
        f_spo2 = 100.0f;
    }
#endif

    return f_spo2;
}

/*
 * 胸贴 SpO2 比值 R（去趋势 + MAD + 通道相关门控）。
 * Chest-patch SpO2 ratio R (detrend + MAD + channel-correlation gating).
 *
 * 做法（单遍累加，无需额外大缓冲）：
 * Method (single-pass accumulation, no extra large buffer):
 *   1) DC = mean(ir), mean(rd)
 *   2) 对每通道做最小二乘线性去趋势（抑制 6s 窗内呼吸基线漂移）
 *      Least-squares linear detrend per channel (suppress breath baseline drift in 6s window)
 *   3) AC = MAD = mean(|去趋势值|)，对小幅脉动比峰谷法更抗噪
 *      AC = MAD = mean(|detrended|); more noise-robust than peak-valley for small pulsation
 *   4) 通道相关 corr(ir_dt, rd_dt)：好脉搏两通道高度同形，运动/噪声则不相关
 *      Channel corr(ir_dt, rd_dt): good pulse highly correlated; motion/noise uncorrelated
 *   5) R = (AC_rd/DC_rd)/(AC_ir/DC_ir)，并做 PI、相关、R 区间三重门控
 *      R = (AC_rd/DC_rd)/(AC_ir/DC_ir) with PI, correlation, and R-range triple gating
 *
 * 返回 1 且写出 *ratio_x100（=100*R）表示可信；否则返回 0。
 * Returns 1 and writes *ratio_x100 (=100*R) if trustworthy; else 0.
 */
static int8_t ppg_spo2_window_ratio(const uint32_t *ir, const uint32_t *rd, int32_t len,
                                    int32_t *ratio_x100)
{
    int32_t k;
    uint64_t sum_ir = 0U;
    uint64_t sum_rd = 0U;
    float mean_ir;
    float mean_rd;
    float mx;
    float xc;
    float sxy_ir = 0.0f;  /* sum(xc * ir) 用于斜率 / for slope */
    float sxy_rd = 0.0f;
    float sxx = 0.0f;     /* sum(xc^2) */
    float slope_ir;
    float slope_rd;
    float sum_abs_ir = 0.0f;
    float sum_abs_rd = 0.0f;
    float s_ii = 0.0f;    /* sum(ir_dt^2) */
    float s_rr = 0.0f;    /* sum(rd_dt^2) */
    float s_ir = 0.0f;    /* sum(ir_dt*rd_dt) */
    float mad_ir;
    float mad_rd;
    float pi_ir;
    float corr;
    float denom_corr;
    float r_val;

    if (len < 100)
    {
        return 0;
    }

    for (k = 0; k < len; k++)
    {
        sum_ir += ir[k];
        sum_rd += rd[k];
    }
    mean_ir = (float)sum_ir / (float)len;
    mean_rd = (float)sum_rd / (float)len;
    if ((mean_ir < 1000.0f) || (mean_rd < 1000.0f))
    {
        return 0;
    }

    mx = (float)(len - 1) * 0.5f;
    for (k = 0; k < len; k++)
    {
        xc = (float)k - mx;
        sxx += xc * xc;
        sxy_ir += xc * ((float)ir[k] - mean_ir);
        sxy_rd += xc * ((float)rd[k] - mean_rd);
    }
    if (sxx <= 0.0f)
    {
        return 0;
    }
    slope_ir = sxy_ir / sxx;
    slope_rd = sxy_rd / sxx;

    /* 第二遍：在线生成去趋势值，累加 MAD 与相关所需统计量
     * Second pass: online detrended values, accumulate MAD and correlation stats */
    for (k = 0; k < len; k++)
    {
        float xi;
        float yi;
        xc = (float)k - mx;
        xi = (float)ir[k] - (slope_ir * xc + mean_ir);
        yi = (float)rd[k] - (slope_rd * xc + mean_rd);
        sum_abs_ir += (xi >= 0.0f) ? xi : -xi;
        sum_abs_rd += (yi >= 0.0f) ? yi : -yi;
        s_ii += xi * xi;
        s_rr += yi * yi;
        s_ir += xi * yi;
    }

    mad_ir = sum_abs_ir / (float)len;
    mad_rd = sum_abs_rd / (float)len;
    if ((mad_ir < 1.0f) || (mad_rd < 1.0f))
    {
        return 0;
    }

    /* 灌注指数门控（过小=无脉动，过大=运动伪迹）
     * Perfusion index gating (too low=no pulse, too high=motion artifact) */
    pi_ir = mad_ir / mean_ir;
    if ((pi_ir < ALGO_SPO2_PI_MIN) || (pi_ir > ALGO_SPO2_PI_MAX))
    {
        return 0;
    }

    /* 通道相关门控：红/红外脉搏应高度同形
     * Channel correlation gating: red/IR pulse waveforms should be highly similar */
    denom_corr = s_ii * s_rr;
    if (denom_corr <= 0.0f)
    {
        return 0;
    }
    corr = s_ir / sqrtf(denom_corr);
    if (corr < ((float)ALGO_SPO2_CORR_MIN_X100 / 100.0f))
    {
        return 0;
    }

    /* R = (MAD_rd/DC_rd)/(MAD_ir/DC_ir) = (MAD_rd*DC_ir)/(MAD_ir*DC_rd) */
    r_val = (mad_rd * mean_ir) / (mad_ir * mean_rd);
    *ratio_x100 = (int32_t)(r_val * 100.0f + 0.5f);
    if ((*ratio_x100 < ALGO_SPO2_R_X100_MIN) || (*ratio_x100 > ALGO_SPO2_R_X100_MAX))
    {
        return 0;
    }

    return 1;
}
#endif

static int8_t ppg_signal_usable(const uint32_t *ir, int32_t len)
{
    int32_t k;
    uint64_t sum = 0U;
    uint32_t min_v = 0xFFFFFFFFU;
    uint32_t max_v = 0U;
    int32_t zero_cnt = 0;
    uint32_t mean;

    if (len < 100)
    {
        return 0;
    }

    for (k = 0; k < len; k++)
    {
        uint32_t v = ir[k];
        sum += v;
        if (v < min_v)
        {
            min_v = v;
        }
        if (v > max_v)
        {
            max_v = v;
        }
        if (v == 0U)
        {
            zero_cnt++;
        }
    }

    mean = (uint32_t)(sum / (uint32_t)len);
    if ((mean < 5000U) || (mean > 85000U))
    {
        return 0;
    }

    if ((zero_cnt * 100) > (len * 3))
    {
        return 0;
    }

    if (max_v > min_v)
    {
        uint32_t pp = ((max_v - min_v) * 100U) / mean;
        if (pp < 1U)
        {
            return 0;
        }
    }

    return 1;
}

#if ALGO_USE_IR_BANDPASS
/* Butterworth bandpass 2nd order, 0.5~4.5 Hz, Fs=100 Hz（scipy.signal.butter 系数）
 * Butterworth bandpass 2nd order, 0.5~4.5 Hz, Fs=100 Hz (scipy.signal.butter coeffs) */
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
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx = 0;
    int32_t n_x_dc_max_idx = 0; 
    int32_t an_ratio[5], n_ratio_average; 
    int32_t n_nume, n_denom ;
    int32_t hr_peak = 0;
    int32_t hr_ac = 0;
    int8_t hr_peak_ok = 0;
#if ALGO_CHEST_MODE
    int8_t hr_ac_ok = 0;
#endif

    *pch_hr_valid = 0;
    *pch_spo2_valid = 0;

    if (ppg_signal_usable(pun_ir_buffer, n_ir_buffer_length) == 0)
    {
        *pn_heart_rate = -999;
        *pn_spo2 = -999;
        return;
    }

    ppg_sanitize_samples(pun_ir_buffer, pun_red_buffer, n_ir_buffer_length);

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        un_ir_mean += pun_ir_buffer[k];
    }
    un_ir_mean = un_ir_mean / n_ir_buffer_length;

#if ALGO_USE_IR_BANDPASS
    /* 先去直流再带通（约 0.5~4.5 Hz），抑制基线漂移与高频噪声；找峰用，血氧仍用下方原始缓冲
     * DC remove then bandpass (~0.5~4.5 Hz) for peak find; SpO2 still uses raw buffers below */
    for (k = 0; k < n_ir_buffer_length; k++)
    {
        s_bp_work[k] = (float)pun_ir_buffer[k] - (float)un_ir_mean;
    }
    ppg_linear_detrend(s_bp_work, n_ir_buffer_length);
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
    maxim_find_peaks(an_ir_valley_locs, &n_npks, an_x, n_ir_buffer_length,
                     n_th1, ALGO_PEAK_MIN_DIST, 15);

    hr_peak_ok = ppg_hr_from_intervals(an_ir_valley_locs, n_npks, &hr_peak);

#if ALGO_CHEST_MODE
    hr_ac_ok = ppg_hr_from_autocorr(an_x, n_ir_buffer_length, &hr_ac);
    if (ppg_hr_fuse_chest(hr_peak, hr_peak_ok, hr_ac, hr_ac_ok, pn_heart_rate) != 0)
    {
        if (ppg_hr_in_plausible_band(*pn_heart_rate) != 0)
        {
            *pch_hr_valid = 1;
        }
        else if ((*pn_heart_rate >= ALGO_CHEST_HR_PLAUSIBLE_LO) &&
                 (*pn_heart_rate <= ALGO_CHEST_HR_PLAUSIBLE_HI))
        {
            *pch_hr_valid = 1;
        }
        else
        {
            *pn_heart_rate = -999;
            *pch_hr_valid = 0;
        }
    }
    else
    {
        int32_t hr_wide = 0;
        if (ppg_hr_from_autocorr_range(an_x, n_ir_buffer_length, &hr_wide,
                                      ALGO_CHEST_HR_AC_WIDE_LO,
                                      ALGO_CHEST_HR_AC_WIDE_HI) != 0)
        {
            if (ppg_hr_in_plausible_band(hr_wide) != 0)
            {
                *pn_heart_rate = hr_wide;
                *pch_hr_valid = 1;
            }
            else
            {
                *pn_heart_rate = -999;
                *pch_hr_valid = 0;
            }
        }
        else
        {
            *pn_heart_rate = -999;
            *pch_hr_valid = 0;
        }
    }
#else
    if (hr_peak_ok != 0)
    {
        *pn_heart_rate = hr_peak;
        *pch_hr_valid = 1;
    }
    else
    {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }
#endif

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

    if (n_i_ratio_count > 0)
    {
        if (n_middle_idx > 1)
        {
            n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
        }
        else
        {
            n_ratio_average = an_ratio[n_middle_idx];
        }
    }
    else
    {
        n_ratio_average = 0;
    }

#if ALGO_CHEST_MODE
    /* 胸贴：去趋势+MAD 的 R（含 PI/通道相关/区间门控），取代逐拍峰谷比
     * Chest: detrend+MAD R (PI/corr/range gating) replaces per-beat peak-valley ratio */
    if (ppg_spo2_window_ratio(pun_ir_buffer, pun_red_buffer, n_ir_buffer_length,
                               &n_ratio_average) == 0)
    {
        n_ratio_average = 0;
    }
#endif

#if ALGO_CHEST_MODE
    if ((n_ratio_average >= ALGO_SPO2_R_X100_MIN) && (n_ratio_average <= ALGO_SPO2_R_X100_MAX))
#else
    if (n_ratio_average > 0 && n_ratio_average < 200)
#endif
    {
        float f_spo2 = ppg_spo2_from_ratio_x100(n_ratio_average);

        *pn_spo2 = (int32_t)(f_spo2 + 0.5f);
#if ALGO_CHEST_MODE && ALGO_SPO2_REQUIRE_HR_VALID
        /* 可选：要求 HR 同时有效作为额外质量交叉校验（会降低覆盖率）
         * Optional: require valid HR as extra QC cross-check (reduces coverage) */
        if (*pch_hr_valid != 0)
        {
            *pch_spo2_valid = 1;
        }
        else
        {
            *pn_spo2 = -999;
            *pch_spo2_valid = 0;
        }
#else
        /* SpO2 质量已由 PI/通道相关/R 区间门控保证，与 HR 检测解耦
         * SpO2 quality ensured by PI/corr/R gating, decoupled from HR detection */
        *pch_spo2_valid = 1;
#endif
    }
    else
    {
        *pn_spo2 = -999;
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


