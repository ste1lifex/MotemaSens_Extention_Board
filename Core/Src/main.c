/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "bsp_max30102.h"
#include "bsp_spi_oled.h"
#include "bsp_gd60932.h"
#include "bsp_sd_spi.h"
#include "bsp_sd_log.h"
#include "algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 12s 周期：0~10s PPG（LED 开）→ 10~12s 关 LED，读体温、刷新显示、写 SD。
 * PPG 相位 10s 覆盖 ~2 个呼吸周期，离线验证可显著降低 SpO2 RMSE（见 algorithm.h）。 */
#define APP_MEAS_CYCLE_MS              12000U
#define APP_PPG_PHASE_MS               10000U
#define APP_POST_PHASE_MS              (APP_MEAS_CYCLE_MS - APP_PPG_PHASE_MS)
/* MAX30102 原始幅值（18 位有效）：约 <5000 偏弱，>80000 易饱和，见 bsp_max30102 注释 */
#define APP_PPG_WEAK_MAX               5000U
#define APP_PPG_STRONG_MIN             80000U
/* 红/红外 LED 电流档位(0x00~0xFF)。在 0x14 基础上再略降一档 */
#define APP_MAX30102_LED_RED           0x13U
#define APP_MAX30102_LED_IR            0x13U
/* PB6 接 3.3V 判为关机请求的去抖时间（内部下拉，悬空为低） */
#define APP_PWR_SW_DEBOUNCE_MS         80U
/* 血氧阶段结束时至少多少点才调用算法（不必凑满 BUFFER_SIZE） */
#define APP_PPG_MIN_SAMPLES            100U
/* FIFO 读循环中每隔多少点刷新一次 OLED（减轻主循环负担） */
#define APP_PPG_OLED_UPDATE_INTERVAL   8U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t g_ir_buffer[BUFFER_SIZE];
static uint32_t g_red_buffer[BUFFER_SIZE];
static int32_t g_heart_rate = 0;
static int8_t g_hr_valid = 0;
static int32_t g_spo2 = 0;
static int8_t g_spo2_valid = 0;
static float g_body_temp = 0.0f;
static uint8_t g_body_temp_valid = 0U;
static uint32_t s_meas_cycle_t0 = 0U;
static uint8_t s_cycle_led_off_done = 0U;
static uint8_t s_cycle_temp_done = 0U;
static uint8_t s_spo2_computed_this_cycle = 0U;
static uint32_t s_last_ir_sample = 0U;
static uint32_t s_pwr_high_since = 0U;
static uint8_t s_meas_resync_after_stop_wakeup = 0U;
static uint32_t s_cycle_index = 0U;
static uint16_t s_cycle_ppg_count = 0U;
static uint8_t s_cycle_sd_logged = 0U;
static uint32_t s_log_ir_snapshot[BUFFER_SIZE];
static uint32_t s_log_red_snapshot[BUFFER_SIZE];
static uint16_t s_log_ppg_snapshot = 0U;
static int32_t s_hr_fifo[HR_FIFO_SIZE];
static uint8_t s_hr_fifo_count = 0U;
static uint8_t s_hr_fifo_idx = 0U;
/* SpO2 跨周期中值平滑：胸贴单周期 R 仍有残余噪声，离线验证 5 点中值可将 RMSE 0.97->0.82 */
#define SPO2_FIFO_SIZE 5
static int32_t s_spo2_fifo[SPO2_FIFO_SIZE];
static uint8_t s_spo2_fifo_count = 0U;
static uint8_t s_spo2_fifo_idx = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_DisplayTitle(void);
static void App_UpdateLedStatusDisplay(uint8_t led_on);
static void App_UpdateDisplay(int32_t hr, int8_t hr_valid, int32_t spo2, int8_t spo2_valid, uint32_t ir_sample);
static uint8_t App_MAX30102_Probe(uint8_t *part_id);
static float App_ReadBatteryVoltage(void);
static void App_UpdateTemperatureDisplay(float temp_c, uint8_t valid);
static void App_UpdatePpgLevelDisplay(uint32_t ir_raw, uint32_t red_raw, uint8_t led_on);
static void App_ScanI2C2(void);
static void App_EnterStopFromPowerSwitch(void);
static void App_RestorePb6PowerSensePin(void);
static void App_ComputePpgMetrics(uint16_t n_samples);
static int32_t App_SmoothHeartRate(int32_t hr, int8_t hr_valid);
static int32_t App_SmoothSpo2(int32_t spo2, int8_t spo2_valid);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_RestorePb6PowerSensePin(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLDOWN;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
}

static void App_EnterStopFromPowerSwitch(void)
{
  GPIO_InitTypeDef g = {0};
  /* 下降沿唤醒：外部撤掉 3.3V 后内部下拉将 PB6 拉低 */
  g.Pin = GPIO_PIN_6;
  g.Mode = GPIO_MODE_IT_FALLING;
  g.Pull = GPIO_PULLDOWN;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);

  (void)MAX30102_SetLEDCurrent(0U, 0U);
  OLED_CLS();

  /* STOP 后 HSE/PLL 停振，唤醒须重配时钟；若板级直接断电则不会执行到这里 */
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
  HAL_ResumeTick();

  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  App_RestorePb6PowerSensePin();
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  SystemClock_Config();
  s_meas_resync_after_stop_wakeup = 1U;
}

static void App_UpdateLedStatusDisplay(uint8_t led_on)
{
  if (led_on != 0U)
  {
    OLED_ShowStr(0, 0, (unsigned char *)"LED ON            ", 1);
  }
  else
  {
    OLED_ShowStr(0, 0, (unsigned char *)"LED OFF           ", 1);
  }
}

static void App_DisplayTitle(void)
{
  OLED_CLS();
  App_UpdateLedStatusDisplay(0U);
  OLED_ShowStr(0, 2, (unsigned char *)"HR:", 1);
  OLED_ShowStr(0, 3, (unsigned char *)"SpO2:", 1);
}

static void App_UpdateDisplay(int32_t hr, int8_t hr_valid, int32_t spo2, int8_t spo2_valid, uint32_t ir_sample)
{
  char line_hr[24];
  char line_spo2[24];

  if (ir_sample < 15000U)
  {
    OLED_ShowStr(0, 6, (unsigned char *)"Place finger      ", 1);
    return;
  }

  OLED_ShowStr(0, 6, (unsigned char *)"Finger detected   ", 1);

  if ((hr_valid != 0) && (hr >= MIN_HR) && (hr <= MAX_HR))
  {
    (void)snprintf(line_hr, sizeof(line_hr), "%3ld bpm   ", (long)hr);
  }
  else
  {
    (void)snprintf(line_hr, sizeof(line_hr), "--- bpm   ");
  }

  if (spo2_valid != 0)
  {
    (void)snprintf(line_spo2, sizeof(line_spo2), "%3ld %%    ", (long)spo2);
  }
  else
  {
    (void)snprintf(line_spo2, sizeof(line_spo2), "--- %%    ");
  }

  OLED_ShowStr(24, 2, (unsigned char *)line_hr, 1);
  OLED_ShowStr(36, 3, (unsigned char *)line_spo2, 1);
}

static uint8_t App_MAX30102_Probe(uint8_t *part_id)
{
  uint8_t id = 0;
  if (MAX30102_ReadReg(REG_PART_ID, &id) != 0U)
  {
    return 0U;
  }

  *part_id = id;
  return 1U;
}

static float App_ReadBatteryVoltage(void)
{
  uint32_t sum = 0U;
  uint16_t sample = 0U;
  uint8_t i = 0U;
  float v_adc = 0.0f;

  for (i = 0U; i < 8U; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      continue;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
    {
      sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
      sum += sample;
    }

    (void)HAL_ADC_Stop(&hadc1);
  }

  sample = (uint16_t)(sum / 8U);
  v_adc = ((float)sample * 3.3f) / 4095.0f;

  /* 分压电阻 100k:100k，电池电压为采样点电压的 2 倍 */
  return v_adc * 2.0f;
}

static void App_UpdateTemperatureDisplay(float temp_c, uint8_t valid)
{
  char temp_line[24];

  if (valid != 0U)
  {
    (void)snprintf(temp_line, sizeof(temp_line), "TEMP: %.1fC   ", temp_c);
  }
  else
  {
    (void)snprintf(temp_line, sizeof(temp_line), "TEMP: --.-C   ");
  }

  OLED_ShowStr(0, 1, (unsigned char *)temp_line, 1);
}

static void App_UpdatePpgLevelDisplay(uint32_t ir_raw, uint32_t red_raw, uint8_t led_on)
{
  char line1[24];
  char line2[24];
  char ir_tag;
  char rd_tag;

  App_UpdateLedStatusDisplay(led_on);

  if (led_on == 0U)
  {
    OLED_ShowStr(0, 4, (unsigned char *)"IR: ---          ", 1);
    OLED_ShowStr(0, 5, (unsigned char *)"RD: ---          ", 1);
    return;
  }

  ir_tag = (ir_raw < APP_PPG_WEAK_MAX) ? 'L' : ((ir_raw > APP_PPG_STRONG_MIN) ? 'H' : ' ');
  rd_tag = (red_raw < APP_PPG_WEAK_MAX) ? 'L' : ((red_raw > APP_PPG_STRONG_MIN) ? 'H' : ' ');

  (void)snprintf(line1, sizeof(line1), "IR:%6lu %c", (unsigned long)ir_raw, ir_tag);
  (void)snprintf(line2, sizeof(line2), "RD:%6lu %c", (unsigned long)red_raw, rd_tag);
  OLED_ShowStr(0, 4, (unsigned char *)line1, 1);
  OLED_ShowStr(0, 5, (unsigned char *)line2, 1);
}

static int32_t App_SmoothHeartRate(int32_t hr, int8_t hr_valid)
{
  int32_t buf[HR_FIFO_SIZE];
  uint8_t i;
  uint8_t n;
  uint8_t j;
  int32_t t;

  if (hr_valid == 0)
  {
    return hr;
  }

#if ALGO_CHEST_MODE
  if ((hr < ALGO_CHEST_HR_PLAUSIBLE_LO) || (hr > ALGO_CHEST_HR_PLAUSIBLE_HI))
  {
    return hr;
  }
#endif

  s_hr_fifo[s_hr_fifo_idx] = hr;
  s_hr_fifo_idx = (uint8_t)((s_hr_fifo_idx + 1U) % HR_FIFO_SIZE);
  if (s_hr_fifo_count < HR_FIFO_SIZE)
  {
    s_hr_fifo_count++;
  }

  n = s_hr_fifo_count;
  for (i = 0U; i < n; i++)
  {
    uint8_t pos = (uint8_t)((s_hr_fifo_idx + HR_FIFO_SIZE - n + i) % HR_FIFO_SIZE);
    buf[i] = s_hr_fifo[pos];
  }

  for (i = 1U; i < n; i++)
  {
    t = buf[i];
    for (j = i; (j > 0U) && (t < buf[j - 1U]); j--)
    {
      buf[j] = buf[j - 1U];
    }
    buf[j] = t;
  }

  return buf[n / 2U];
}

static int32_t App_SmoothSpo2(int32_t spo2, int8_t spo2_valid)
{
  int32_t buf[SPO2_FIFO_SIZE];
  uint8_t i;
  uint8_t n;
  uint8_t j;
  int32_t t;

  if (spo2_valid == 0)
  {
    return spo2;
  }

  s_spo2_fifo[s_spo2_fifo_idx] = spo2;
  s_spo2_fifo_idx = (uint8_t)((s_spo2_fifo_idx + 1U) % SPO2_FIFO_SIZE);
  if (s_spo2_fifo_count < SPO2_FIFO_SIZE)
  {
    s_spo2_fifo_count++;
  }

  n = s_spo2_fifo_count;
  for (i = 0U; i < n; i++)
  {
    uint8_t pos = (uint8_t)((s_spo2_fifo_idx + SPO2_FIFO_SIZE - n + i) % SPO2_FIFO_SIZE);
    buf[i] = s_spo2_fifo[pos];
  }

  for (i = 1U; i < n; i++)
  {
    t = buf[i];
    for (j = i; (j > 0U) && (t < buf[j - 1U]); j--)
    {
      buf[j] = buf[j - 1U];
    }
    buf[j] = t;
  }

  return buf[n / 2U];
}

static void App_ComputePpgMetrics(uint16_t n_samples)
{
  int32_t hr_raw = 0;

  if (n_samples < APP_PPG_MIN_SAMPLES)
  {
    g_hr_valid = 0;
    g_spo2_valid = 0;
    return;
  }

  maxim_heart_rate_and_oxygen_saturation(g_ir_buffer, (int32_t)n_samples,
                                         g_red_buffer, &g_spo2, &g_spo2_valid,
                                         &hr_raw, &g_hr_valid);
  if (g_hr_valid != 0)
  {
    g_heart_rate = App_SmoothHeartRate(hr_raw, 1);
  }
  else
  {
    g_heart_rate = hr_raw;
  }

  if (g_spo2_valid != 0)
  {
    g_spo2 = App_SmoothSpo2(g_spo2, 1);
  }

  App_UpdateDisplay(g_heart_rate, g_hr_valid, g_spo2, g_spo2_valid, s_last_ir_sample);
  s_spo2_computed_this_cycle = 1U;
}

static void App_ScanI2C2(void)
{
  uint8_t probe_list[2] = {0x30U, 0x32U};
  uint8_t i = 0U;

  /* 仅探测体温芯片可能地址，避免全地址扫描阻塞主循环 */
  for (i = 0U; i < 2U; i++)
  {
    (void)HAL_I2C_IsDeviceReady(&hi2c2, probe_list[i], 1, 5U);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  Bsp_SPI_OLED_Init();
  OLED_Init();

  Bsp_MAX30102_Init();
  MAX30102_SetLEDCurrent(APP_MAX30102_LED_RED, APP_MAX30102_LED_IR);
  App_ScanI2C2();
  (void)GD60932_Init();
  HAL_Delay(100);

  {
    uint8_t part_id = 0;
    char line[24];

    OLED_CLS();
    OLED_ShowStr(0, 0, (unsigned char *)"Boot Self Test", 1);
    OLED_ShowStr(0, 2, (unsigned char *)"OLED: OK", 1);

    if (App_MAX30102_Probe(&part_id) != 0U)
    {
      (void)snprintf(line, sizeof(line), "MAX:OK ID=0x%02X", part_id);
      OLED_ShowStr(0, 3, (unsigned char *)line, 1);
    }
    else
    {
      OLED_ShowStr(0, 3, (unsigned char *)"MAX: FAIL", 1);
      OLED_ShowStr(0, 5, (unsigned char *)"Check PB8/PB7", 1);
      OLED_ShowStr(0, 6, (unsigned char *)"and power 3V3", 1);
    }

    if (SD_Init() == SD_OK)
    {
      SD_CardInfo_t sd_info;
      if (SD_GetCardInfo(&sd_info) == SD_OK)
      {
        uint32_t mb = (uint32_t)(sd_info.capacity_bytes / (1024ULL * 1024ULL));
        (void)snprintf(line, sizeof(line), "SD:OK %luMB", (unsigned long)mb);
      }
      else
      {
        (void)snprintf(line, sizeof(line), "SD: OK");
      }
      OLED_ShowStr(0, 4, (unsigned char *)line, 1);
      (void)SD_Log_Init();
    }
    else
    {
      OLED_ShowStr(0, 4, (unsigned char *)"SD: FAIL", 1);
    }
  }

  HAL_Delay(1200);
  App_DisplayTitle();
  App_UpdateTemperatureDisplay(0.0f, 0U);
  App_UpdatePpgLevelDisplay(0U, 0U, 0U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t red_sample = 0;
    uint32_t ir_sample = 0;
    static uint16_t sample_index = 0;
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt = 0U;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
    {
      if (s_pwr_high_since == 0U)
      {
        s_pwr_high_since = now_ms;
      }
      else if ((now_ms - s_pwr_high_since) >= APP_PWR_SW_DEBOUNCE_MS)
      {
        App_EnterStopFromPowerSwitch();
        s_pwr_high_since = 0U;
      }
    }
    else
    {
      s_pwr_high_since = 0U;
    }

    if (s_meas_resync_after_stop_wakeup != 0U)
    {
      s_meas_resync_after_stop_wakeup = 0U;
      s_meas_cycle_t0 = 0U;
      s_cycle_led_off_done = 0U;
      s_cycle_temp_done = 0U;
      s_spo2_computed_this_cycle = 0U;
      s_cycle_sd_logged = 0U;
      s_last_ir_sample = 0U;
      s_cycle_ppg_count = 0U;
      s_hr_fifo_count = 0U;
      s_hr_fifo_idx = 0U;
      /* 唤醒后重新开始测量，清空 SpO2 跨周期平滑（避免休眠前的旧值串入） */
      s_spo2_fifo_count = 0U;
      s_spo2_fifo_idx = 0U;
      sample_index = 0U;
      (void)MAX30102_SetLEDCurrent(APP_MAX30102_LED_RED, APP_MAX30102_LED_IR);
      App_DisplayTitle();
      App_UpdateTemperatureDisplay(0.0f, 0U);
      App_UpdatePpgLevelDisplay(0U, 0U, 1U);
    }

    if (s_meas_cycle_t0 == 0U)
    {
      s_meas_cycle_t0 = now_ms;
    }

    dt = now_ms - s_meas_cycle_t0;

    if (dt >= APP_MEAS_CYCLE_MS)
    {
      s_meas_cycle_t0 = now_ms;
      s_cycle_led_off_done = 0U;
      s_cycle_temp_done = 0U;
      s_spo2_computed_this_cycle = 0U;
      s_cycle_sd_logged = 0U;
      s_cycle_ppg_count = 0U;
      s_hr_fifo_count = 0U;
      s_hr_fifo_idx = 0U;
      sample_index = 0U;
      (void)MAX30102_SetLEDCurrent(APP_MAX30102_LED_RED, APP_MAX30102_LED_IR);
      dt = 0U;
      App_UpdatePpgLevelDisplay(0U, 0U, 1U);
    }

    if (dt < APP_MEAS_CYCLE_MS)
    {
      /* 0~6s：采 PPG；满 BUFFER_SIZE 可提前算；6s 关灯并快照 */
      if (s_cycle_led_off_done == 0U)
      {
        if (sample_index < BUFFER_SIZE)
        {
          uint8_t fifo_burst = 0U;
          while ((fifo_burst < 48U) && (MAX30102_CheckFIFOStatus() != 0U))
          {
            if (MAX30102_ReadFIFO(&red_sample, &ir_sample) != 0U)
            {
              break;
            }
            fifo_burst++;
            s_last_ir_sample = ir_sample;
            if (((sample_index + 1U) % APP_PPG_OLED_UPDATE_INTERVAL) == 0U)
            {
              App_UpdatePpgLevelDisplay(ir_sample, red_sample, 1U);
            }
            g_red_buffer[sample_index] = red_sample;
            g_ir_buffer[sample_index] = ir_sample;
            sample_index++;
            if (sample_index >= BUFFER_SIZE)
            {
              break;
            }
          }
        }

        if ((sample_index >= BUFFER_SIZE) && (s_spo2_computed_this_cycle == 0U))
        {
          App_ComputePpgMetrics(BUFFER_SIZE);
        }

        if (dt >= APP_PPG_PHASE_MS)
        {
          s_cycle_ppg_count = sample_index;
          if (s_spo2_computed_this_cycle == 0U)
          {
            App_ComputePpgMetrics(sample_index);
          }
          s_log_ppg_snapshot = s_cycle_ppg_count;
          if (s_log_ppg_snapshot > 0U)
          {
            (void)memcpy(s_log_ir_snapshot, g_ir_buffer,
                         (size_t)s_log_ppg_snapshot * sizeof(uint32_t));
            (void)memcpy(s_log_red_snapshot, g_red_buffer,
                         (size_t)s_log_ppg_snapshot * sizeof(uint32_t));
          }
          sample_index = 0U;
          (void)MAX30102_SetLEDCurrent(0U, 0U);
          s_cycle_led_off_done = 1U;
          App_UpdatePpgLevelDisplay(0U, 0U, 0U);
        }
      }
      /* 6~8s：关 LED 后读体温、刷新 HR/SpO2/体温、写 SD（不占用下一周期 PPG） */
      else if (dt < (APP_PPG_PHASE_MS + APP_POST_PHASE_MS))
      {
        if (s_cycle_temp_done == 0U)
        {
          if (GD60932_ReadBodyTemperature(&g_body_temp) == GD60932_OK)
          {
            g_body_temp_valid = 1U;
          }
          else
          {
            g_body_temp_valid = 0U;
          }
          App_UpdateTemperatureDisplay(g_body_temp, g_body_temp_valid);
          App_UpdateDisplay(g_heart_rate, g_hr_valid, g_spo2, g_spo2_valid, s_last_ir_sample);
          s_cycle_temp_done = 1U;
        }

        if ((s_cycle_temp_done != 0U) && (s_cycle_sd_logged == 0U))
        {
          if (SD_Log_IsReady() != 0U)
          {
            (void)SD_Log_WriteCycle(s_cycle_index,
                                    APP_MEAS_CYCLE_MS,
                                    g_heart_rate,
                                    g_hr_valid,
                                    g_spo2,
                                    g_spo2_valid,
                                    g_body_temp,
                                    g_body_temp_valid,
                                    s_log_ir_snapshot,
                                    s_log_red_snapshot,
                                    s_log_ppg_snapshot);
            s_cycle_index++;
          }
          s_cycle_sd_logged = 1U;
        }
      }
    }

    {
      static uint32_t last_batt_ms = 0U;
      if ((now_ms - last_batt_ms) >= 500U)
      {
        char batt_line[24];
        float batt_v = App_ReadBatteryVoltage();
        (void)snprintf(batt_line, sizeof(batt_line), "BAT: %.2fV   ", batt_v);
        OLED_ShowStr(0, 7, (unsigned char *)batt_line, 1);
        last_batt_ms = now_ms;
      }
    }

    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
