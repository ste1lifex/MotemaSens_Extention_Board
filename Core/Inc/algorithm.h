#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include <stdint.h>
#include <math.h>

#define FS 100  // 采样频率：100Hz（与硬件配置一致）
/* 血氧阶段 3s 采满再算：100Hz × 3s = 300 点 */
#define BUFFER_SIZE  300
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 // DO NOT CHANGE

/* 1=对 IR 去直流后做 IIR 带通再找峰；血氧仍用原始红/红外。0=沿用原去直流+MA4 */
#ifndef ALGO_USE_IR_BANDPASS
#define ALGO_USE_IR_BANDPASS  1
#endif

/* 心率范围定义 */
#define MAX_HR 180  // 最大心率
#define MIN_HR 40   // 最小心率

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

