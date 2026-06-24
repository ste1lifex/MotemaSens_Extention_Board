#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdint.h>
#include <math.h>

#define FS 100  // 采样频率：100Hz（与硬件配置一致）
/* PPG 阶段 10s：100Hz × 10s = 1000 点（与 APP_PPG_PHASE_MS 一致）。
 * 选 10s 是因为呼吸周期约 5s，10s 恰好覆盖 ~2 个完整呼吸，
 * 使呼吸调制在 R(MAD) 中对消；离线验证窗口越长 SpO2 RMSE 越低且 6s 仍未饱和。
 * STM32F401RCT6 SRAM 64KB：8 个 BUFFER_SIZE 缓冲共约 31KB，整机约 40KB，余量充足。 */
#define BUFFER_SIZE  1000
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 // DO NOT CHANGE

/* 1=对 IR 去直流后做 IIR 带通再找峰；血氧仍用原始红/红外。0=沿用原去直流+MA4 */
#ifndef ALGO_USE_IR_BANDPASS
#define ALGO_USE_IR_BANDPASS  1
#endif

/* 心率范围定义 */
#define MAX_HR 180  // 最大心率
#define MIN_HR 40   // 最小心率

/* 1=胸贴反射模式（峰距/谐波/自相关/SpO2 偏移）；0=指夹默认 */
#ifndef ALGO_CHEST_MODE
#define ALGO_CHEST_MODE  1
#endif

/* 峰间最小样本数：约对应最高心率 FS*60/MAX_HR；胸贴用更大值抑制重搏假峰 */
#define ALGO_PEAK_MIN_DIST_DEFAULT  4
#if ALGO_CHEST_MODE
#define ALGO_PEAK_MIN_DIST          45
/* 标定会话真值 HR 73~80（DATA_chest_24052026_1.3.3） */
#define ALGO_CHEST_HR_PLAUSIBLE_LO    70
#define ALGO_CHEST_HR_PLAUSIBLE_HI    85
#define ALGO_CHEST_HR_AC_LO            73
#define ALGO_CHEST_HR_AC_HI            80
/*
 * SpO2 胸贴线性标定（基于 DATA_chest_24062026_1.3.4 + 视频参考 Omron 标定）:
 *   - R 由「线性去趋势后 MAD」之比给出: R=(MAD_rd/DC_rd)/(MAD_ir/DC_ir)
 *   - 标定会话静息真值 SpO2≈96，质量门控后 R 中位数≈0.869，离散 std≈0.034
 *   - 反推: SpO2 = SLOPE*R + INTERCEPT，令 R=0.869 -> 96.5
 *   - 离线复算（5 点中值平滑）RMSE≈0.8%，远优于原全窗 MAD 法(RMSE≈2.8%)
 * 注意: 标定为单人静息、SpO2 几乎恒定，斜率取自反射式血氧文献(约 -17)，
 *       低血氧段未经验证；仅保证静息绝对值与抑制误读，勿用于诊断低氧。
 */
#define ALGO_SPO2_CHEST_SLOPE         (-17.0f)
#define ALGO_SPO2_CHEST_INTERCEPT     (110.4f)
#define ALGO_SPO2_CHEST_CLAMP_LO      85.0f
#define ALGO_SPO2_CHEST_CLAMP_HI      100.0f
/* 去趋势 MAD 之比 R×100 合理区间（超出视为运动/噪声，SpO2 不可靠） */
#define ALGO_SPO2_R_X100_MIN          50
#define ALGO_SPO2_R_X100_MAX          160
/* 灌注指数 PI=MAD_ir/DC_ir 合理区间（胸口灌注差，过小为无脉动，过大为运动） */
#define ALGO_SPO2_PI_MIN              0.003f
#define ALGO_SPO2_PI_MAX              0.050f
/* 红/红外去趋势波形相关系数门限×100：好脉搏两通道高度同形(>0.94)，
 * 运动/噪声周期 <0.75，0.85 落在二分空档，分离干净 */
#define ALGO_SPO2_CORR_MIN_X100       85
/* 1=SpO2 还需 HR 同时有效（覆盖率约 33%）；0=与 HR 解耦，仅靠信号质量门控（约 44%） */
#ifndef ALGO_SPO2_REQUIRE_HR_VALID
#define ALGO_SPO2_REQUIRE_HR_VALID    0
#endif
#define ALGO_CHEST_HR_AC_WIDE_LO        65
#define ALGO_CHEST_HR_AC_WIDE_HI        92
#else
#define ALGO_PEAK_MIN_DIST          ALGO_PEAK_MIN_DIST_DEFAULT
#endif

/* RR 间期合法范围（样本数） */
#define ALGO_RR_MIN_SAMPLES         ((int32_t)((FS * 60) / MAX_HR))
#define ALGO_RR_MAX_SAMPLES         ((int32_t)((FS * 60) / MIN_HR))

/* 函数声明 - Maxim血氧算法 */
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, 
                                            uint32_t *pun_red_buffer, int32_t *pn_spo2, 
                                            int8_t *pch_spo2_valid, int32_t *pn_heart_rate, 
                                            int8_t *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, 
                     int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, 
                                  int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, 
                              int32_t n_min_distance);
void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

#endif /* ALGORITHM_H_ */

