# MotemaSens Extension Board — Firmware

**固件版本：v1.3**

基于 **STM32F401RCT6** 的健康监测扩展板固件：通过 **MAX30102** 采集 PPG 信号计算心率与血氧，通过 **GD60932** 非接触式红外测温，在 **SPI OLED** 上显示测量结果与电池电压，并通过 **TF 卡（FAT32）** 记录每周期测量数据。

- **硬件版本**：MotemaSens Extension Board V1.0  
- **远程仓库**：[github.com/ste1lifex/MotemaSens_Extention_Board](https://github.com/ste1lifex/MotemaSens_Extention_Board)

---

## 功能概览

| 功能 | 说明 |
|------|------|
| 心率 (HR) | Maxim PPG 算法，100 Hz 采样，每周期 300 点（约 3 s） |
| 血氧 (SpO₂) | 与心率共用红/红外 FIFO 数据 |
| 体温 | GD60932 额头模式，LED 关闭后测量，避免 PPG 光干扰 |
| 电池电压 | ADC1 采样 + 100k:100k 分压，约每 500 ms 刷新 |
| 显示 | 128×64 SPI OLED，开机自检与各通道状态 |
| TF 卡 | FAT32 根目录 `DATA.CSV`，每 5 s 写入 HR/SpO₂/体温及全部 IR/RD 采样 |
| 低功耗 | PB6 接高电平（去抖 80 ms）进入 **STOP**；撤掉高电平后下降沿唤醒 |

### 测量周期（5 s）

```
0 ───────────── 3 s ──── 4 s ──── 5 s ──► 下一周期
│  MAX30102 LED 开，采集 PPG   │ LED 关 │ 读体温 │ 写 SD / 重置
│  满 300 点即计算 HR / SpO₂   │ 等待 1s│ GD60932│
```

- 血氧阶段：红/红外 LED 电流默认 `0x13`（可在 `main.c` 中 `APP_MAX30102_LED_*` 调整）  
- PPG 幅值提示：IR/RD 原始值 &lt; 5000 标 `L`（偏弱），&gt; 80000 标 `H`（易饱和）  
- 手指检测：IR 幅值 &gt; 15000 视为已放置手指  

---

## 硬件与接口

| 模块 | 器件 | 总线 / 引脚 |
|------|------|-------------|
| MCU | STM32F401RCT6 (LQFP64) | — |
| 血氧心率 | MAX30102 | I2C1 — PB8 SCL, PB7 SDA |
| 红外测温 | GD60932 | I2C2 — 7 位地址 0x18（初始化时探测 0x30 / 0x32） |
| 显示屏 | SPI OLED | SPI1 — PA2 RES, PA3 DC, PA4 CS, PA5 SCK, PA7 MOSI |
| TF 卡槽 | microSD（SPI） | SPI2 — PB12 CS, PB13 SCK, PC2 MISO, PC3 MOSI |
| 调试串口 | USART1 | PA9 TX, PA10 RX（115200，可按工程配置） |
| 电池检测 | ADC1 IN1 | PA1，分压后 ×2 得电池电压 |
| 电源 / 关机 | GPIO 输入 | PB6，内部下拉；高电平请求进入 STOP |

> 芯片手册等 PDF 文档保留在本地，**不纳入 Git**（见 `.gitignore`）。

---

## 软件架构

```
program/
├── Core/
│   ├── Inc/
│   └── Src/
│       ├── main.c
│       ├── algorithm.c/h
│       ├── bsp_max30102.c/h
│       ├── bsp_gd60932.c/h
│       ├── bsp_spi_oled.c/h
│       ├── bsp_sd_spi.c/h       # TF 卡 SPI 块设备
│       ├── bsp_sd_log.c/h       # FAT32 CSV 日志
│       └── fatfs_diskio.c       # FatFs 底层磁盘接口
├── Middlewares/Third_Party/FatFs/src/   # ChaN FatFs R0.16
├── Drivers/                   # STM32 HAL + CMSIS
├── MDK-ARM/
├── MotemaSens_Extention_Board_V1.0.ioc
└── README.md
```

### 主要源文件

- **`main.c`**：5 s 测量周期、OLED、电池电压、STOP 唤醒、SD 日志触发  
- **`algorithm.c`**：Maxim 心率 / 血氧算法（可选 `ALGO_USE_IR_BANDPASS`）  
- **`bsp_sd_spi.c`**：`SD_Init`、块读写  
- **`bsp_sd_log.c`**：挂载 FAT32、创建/追加 `DATA.CSV`  
- **`fatfs_diskio.c`**：FatFs 与 `bsp_sd_spi` 桥接  

### SD 卡数据日志（`DATA.CSV`）

| 项目 | 说明 |
|------|------|
| 文件 | 卡根目录 **`DATA.CSV`**，读卡器插入电脑即可用 Excel 打开 |
| 时机 | 每 5 s 周期结束（体温读完、下一周期开始前） |
| 覆盖 | **上电/复位**后重建文件，不保留上次会话 |
| 无 FAT | 自动 `f_mkfs` 格式化为 FAT32（**会清空整卡**） |

**CSV 列名：**

`cycle,elapsed_ms,hr,hr_valid,spo2,spo2_valid,temp_C,temp_valid,sample_idx,ir,rd`

- 采满时**每个周期约 300 行**（每行一个 PPG 点，`hr`/`spo2`/`temp` 列重复便于筛选）  
- Excel 中按 `cycle` 列筛选可查看单周期波形  

#### 512 MB 卡能存多久？

每周期约 **16.5 KB**（300 行 × ~55 字节/行）：

| 卡容量（标称） | 约可存时长（5 s/周期） |
|----------------|------------------------|
| 512 MB | **约 43 小时（约 1.8 天）** |
| 512 MiB | **约 45 小时（约 1.9 天）** |

### OLED 界面（运行态）

| 行 | 内容 |
|----|------|
| 0 | `LED ON` / `LED OFF` |
| 1 | 体温 |
| 2 | 心率 |
| 3 | 血氧 |
| 4–5 | IR / RD 原始值 |
| 6 | 手指检测 |
| 7 | 电池电压 |

---

## 版本记录

### v1.3（当前）

- **新增** FatFs R0.16 + `fatfs_diskio`，TF 卡 **FAT32** 文件 `DATA.CSV`  
- **新增** `bsp_sd_log`：每 5 s 写入 HR、SpO₂、体温及**全部 IR/RD 采样**（开机覆盖）  
- **修复** 3 s 关灯时 `sample_index` 被清零导致 CSV 中 `ir`/`rd` 为空；改用 `s_cycle_ppg_count` 保存本周期采样数  
- **新增** `SD_WriteBlocks`；`fatfs_diskio` 先包含 `ff.h` 以通过 Keil 编译  

### v1.1

- **新增** TF 卡 SPI 驱动 `bsp_sd_spi`（SPI2，软件片选 PB12）  
- **调整** OLED：第一行 LED 状态；取消 “Health Monitor” 与第四行 “LED off”  
- **调整** 开机自检显示 SD 容量  

### v1.0

- 初始功能：MAX30102、GD60932、SPI OLED、5 s 测量周期、STOP 低功耗  

---

## 开发环境

| 工具 | 版本 / 说明 |
|------|-------------|
| [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) | 6.16.x（与 `.ioc` 一致） |
| [Keil MDK-ARM](https://www.keil.com/) | ARM Compiler 5，目标 `STM32F401RCTx` |
| 调试器 | ST-LINK（SWD：PA13 / PA14） |

### 编译与烧录

1. 打开 `MDK-ARM/MotemaSens_Extention_Board_V1.0.uvprojx`  
2. 安装 **Keil.STM32F4xx_DFP** 器件包  
3. **Build** → **Download**  

**CubeMX 注意**：SPI2 使用 **Software NSS**；勿将 PB12 配为硬件 NSS。

---

## Git 使用

```bash
git add .
git commit -m "描述本次修改"
git push
```

已忽略 Keil 编译产物及本地 PDF 资料。

---

## 免责声明

- 心率、血氧、体温结果仅供 **原型验证与教学演示**，不可替代医疗器械。  
- `Drivers/` 中 HAL/CMSIS 版权归 **STMicroelectronics** 所有；PPG 算法来源于 **Maxim Integrated** 参考实现。

---

## 作者

[ste1lifex](https://github.com/ste1lifex)
