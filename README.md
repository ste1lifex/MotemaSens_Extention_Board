# MotemaSens Extension Board — Firmware

基于 **STM32F401RCT6** 的健康监测扩展板固件：通过 **MAX30102** 采集 PPG 信号计算心率与血氧，通过 **GD60932** 非接触式红外测温，并在 **SPI OLED** 上显示测量结果与电池电压。

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
| 低功耗 | PB6 接高电平（去抖 80 ms）进入 **STOP**；撤掉高电平后下降沿唤醒 |

### 测量周期（5 s）

```
0 ───────────── 3 s ──── 4 s ──── 5 s ──► 下一周期
│  MAX30102 LED 开，采集 PPG   │ LED 关 │ 读体温 │ 重置
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
| 调试串口 | USART1 | PA9 TX, PA10 RX（115200，可按工程配置） |
| 电池检测 | ADC1 IN1 | PA1，分压后 ×2 得电池电压 |
| 电源 / 关机 | GPIO 输入 | PB6，内部下拉；高电平请求进入 STOP |

> 芯片手册等 PDF 文档保留在本地，**不纳入 Git**（见 `.gitignore`）。

---

## 软件架构

```
program/
├── Core/
│   ├── Inc/          # 应用头文件
│   └── Src/
│       ├── main.c           # 测量状态机、显示、电源管理
│       ├── algorithm.c/h    # Maxim 心率 / 血氧算法
│       ├── bsp_max30102.c/h
│       ├── bsp_gd60932.c/h
│       └── bsp_spi_oled.c/h
├── Drivers/          # STM32 HAL + CMSIS（CubeMX 生成）
├── MDK-ARM/          # Keil µVision 工程
├── MotemaSens_Extention_Board_V1.0.ioc   # STM32CubeMX 配置
└── README.md
```

### 主要源文件

- **`main.c`**：5 s 测量周期、OLED 界面、电池电压、STOP 唤醒同步  
- **`algorithm.c`**：基于 Maxim 参考实现；可选 IR 带通（`ALGO_USE_IR_BANDPASS`）  
- **`bsp_*.c`**：各外设底层驱动  

---

## 开发环境

| 工具 | 版本 / 说明 |
|------|-------------|
| [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) | 6.16.x（与 `.ioc` 一致） |
| [Keil MDK-ARM](https://www.keil.com/) | ARM Compiler 5，目标 `STM32F401RCTx` |
| 调试器 | ST-LINK（SWD：PA13 / PA14） |

### 编译与烧录

1. 用 Keil 打开 `MDK-ARM/MotemaSens_Extention_Board_V1.0.uvprojx`  
2. 确认已安装 **Keil.STM32F4xx_DFP** 器件包  
3. 选择目标 **MotemaSens_Extention_Board_V1.0**，执行 **Build**  
4. 连接 ST-LINK，**Download** 到 Flash  

修改引脚或外设时，建议先改 `MotemaSens_Extention_Board_V1.0.ioc`，再 **Generate Code**，并合并 `USER CODE` 区域内的自定义逻辑。

---

## Git 使用

```bash
git add .
git commit -m "描述本次修改"
git push
```

已忽略 Keil 编译产物（`*.o`、`*.hex`、`*.uvguix.*` 等）及本地 PDF 资料，避免仓库膨胀。

---

## 免责声明

- 心率、血氧、体温结果仅供 **原型验证与教学演示**，不可替代医疗器械。  
- `Drivers/` 中 HAL/CMSIS 版权归 **STMicroelectronics** 所有；PPG 算法逻辑来源于 **Maxim Integrated** 参考实现，请遵循相应许可使用。

---

## 作者

[ste1lifex](https://github.com/ste1lifex)
