/**
  ******************************************************************************
  * @file    bsp_max30102.c
  * @author  STM32
  * @version V1.0
  * @date    2024
  * @brief   MAX30102血氧心率传感器驱动 - STM32F10x标准库版本
  *          MAX30102 SpO2/HR sensor driver - STM32F10x stdlib port
  * @note    使用 I2C1（PB8-SCL, PB7-SDA）；PB1 为 INT（若使用）
  *          Uses I2C1 (PB8-SCL, PB7-SDA); PB1 is INT (if used)
  ******************************************************************************
  */
#include "bsp_max30102.h"
#include "i2c.h"
#include "gpio.h"

extern I2C_HandleTypeDef hi2c1;

#define MAX30102_I2C_TIMEOUT_MS 100U

/**
  * @brief  写MAX30102寄存器 / Write MAX30102 register
  * @param  reg_addr: 寄存器地址 / Register address
  * @param  reg_data: 寄存器数据 / Register data
  * @retval 0-成功, 1-失败 / 0-success, 1-fail
  */
uint8_t MAX30102_WriteReg(uint8_t reg_addr, uint8_t reg_data)
{
    if (HAL_I2C_Mem_Write(&hi2c1,
                          MAX30102_I2C_WRITE_ADDR,
                          reg_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          &reg_data,
                          1,
                          MAX30102_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  读MAX30102寄存器 / Read MAX30102 register
  * @param  reg_addr: 寄存器地址 / Register address
  * @param  reg_data: 寄存器数据指针 / Register data pointer
  * @retval 0-成功, 1-失败 / 0-success, 1-fail
  */
uint8_t MAX30102_ReadReg(uint8_t reg_addr, uint8_t *reg_data)
{
    if (HAL_I2C_Mem_Read(&hi2c1,
                         MAX30102_I2C_WRITE_ADDR,
                         reg_addr,
                         I2C_MEMADD_SIZE_8BIT,
                         reg_data,
                         1,
                         MAX30102_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  读取FIFO数据 / Read FIFO data
  * @param  red_led: 红光LED数据指针 / Red LED data pointer
  * @param  ir_led: 红外LED数据指针 / IR LED data pointer
  * @retval 0-成功, 1-失败 / 0-success, 1-fail
  */
uint8_t MAX30102_ReadFIFO(uint32_t *red_led, uint32_t *ir_led)
{
    uint8_t data[6];
    uint8_t temp;
    uint32_t un_temp;
    
    *red_led = 0;
    *ir_led = 0;
    
    /* 读取并清除状态寄存器 / Read and clear status registers */
    MAX30102_ReadReg(REG_INTR_STATUS_1, &temp);
    MAX30102_ReadReg(REG_INTR_STATUS_2, &temp);
    
    if (HAL_I2C_Mem_Read(&hi2c1,
                         MAX30102_I2C_WRITE_ADDR,
                         REG_FIFO_DATA,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         6,
                         MAX30102_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return 1;
    }
    
    /* 组合红光LED数据 / Assemble red LED data */
    un_temp = (uint32_t)data[0];
    un_temp <<= 16;
    *red_led += un_temp;
    un_temp = (uint32_t)data[1];
    un_temp <<= 8;
    *red_led += un_temp;
    *red_led += (uint32_t)data[2];
    *red_led &= 0x03FFFF;  // Mask MSB [23:18]
    
    /* 组合红外LED数据 / Assemble IR LED data */
    un_temp = (uint32_t)data[3];
    un_temp <<= 16;
    *ir_led += un_temp;
    un_temp = (uint32_t)data[4];
    un_temp <<= 8;
    *ir_led += un_temp;
    *ir_led += (uint32_t)data[5];
    *ir_led &= 0x03FFFF;  // Mask MSB [23:18]
    
    return 0;
}

/**
  * @brief  简单延时函数 / Simple delay
  * @param  ms: 延时毫秒数 / Delay in milliseconds
  * @retval 无 / None
  */
static void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  复位MAX30102 / Reset MAX30102
  * @param  无 / None
  * @retval 0-成功, 1-失败 / 0-success, 1-fail
  */
uint8_t MAX30102_Reset(void)
{
    if(MAX30102_WriteReg(REG_MODE_CONFIG, 0x40) != 0)
        return 1;
    return 0;
}

/**
  * @brief  初始化MAX30102 / Initialize MAX30102
  * @param  无 / None
  * @retval 无 / None
  */
void Bsp_MAX30102_Init(void)
{
    /* 复位传感器 / Reset sensor */
    MAX30102_Reset();
    Delay_ms(100);
    
    /* 配置寄存器 / Configure registers */
    MAX30102_WriteReg(REG_INTR_ENABLE_1, 0xC0);  // INTR setting
    MAX30102_WriteReg(REG_INTR_ENABLE_2, 0x00);
    MAX30102_WriteReg(REG_FIFO_WR_PTR, 0x00);    // FIFO_WR_PTR[4:0]
    MAX30102_WriteReg(REG_OVF_COUNTER, 0x00);    // OVF_COUNTER[4:0]
    MAX30102_WriteReg(REG_FIFO_RD_PTR, 0x00);    // FIFO_RD_PTR[4:0]
    MAX30102_WriteReg(REG_FIFO_CONFIG, 0x0F);    // sample avg = 1, fifo rollover=false, fifo almost full = 17
    MAX30102_WriteReg(REG_MODE_CONFIG, 0x03);     // 0x03 for SpO2 mode
    MAX30102_WriteReg(REG_SPO2_CONFIG, 0x27);    // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    MAX30102_WriteReg(REG_LED1_PA, 0x1F);        // ~5mA LED1 (降低电流以减小信号值 / lower current to reduce signal)
    MAX30102_WriteReg(REG_LED2_PA, 0x1F);        // ~5mA LED2 (降低电流以减小信号值 / lower current to reduce signal)
    MAX30102_WriteReg(REG_PILOT_PA, 0x7F);       // Choose value for ~ 25mA for Pilot LED
}

/**
  * @brief  设置LED电流强度 / Set LED current
  * @param  led1_current: LED1电流值 (0x00-0xFF, 0x1F约5mA, 0x24约7mA, 0x2F约9.5mA)
  *                       LED1 current (0x00-0xFF, 0x1F~5mA, 0x24~7mA, 0x2F~9.5mA)
  * @param  led2_current: LED2电流值 (0x00-0xFF, 0x1F约5mA, 0x24约7mA, 0x2F约9.5mA)
  *                       LED2 current (0x00-0xFF, 0x1F~5mA, 0x24~7mA, 0x2F~9.5mA)
  * @retval 0-成功, 1-失败 / 0-success, 1-fail
  * @note   如果信号值过大(>80000)，降低电流值；如果信号值过小(<5000)，增加电流值
  *         If signal too high (>80000), lower current; if too low (<5000), increase current
  *         推荐值范围：0x15(3.3mA) ~ 0x2F(9.5mA)
  *         Recommended range: 0x15 (3.3mA) ~ 0x2F (9.5mA)
  */
uint8_t MAX30102_SetLEDCurrent(uint8_t led1_current, uint8_t led2_current)
{
    if (MAX30102_WriteReg(REG_LED1_PA, led1_current) != 0)
        return 1;
    if (MAX30102_WriteReg(REG_LED2_PA, led2_current) != 0)
        return 1;
    return 0;
}

/**
  * @brief  外部中断初始化 (PB1) / External interrupt init (PB1)
  * @param  无 / None
  * @retval 无 / None
  */
void MAX30102_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/**
  * @brief  检查FIFO状态（轮询方式）/ Check FIFO status (polling)
  * @param  无 / None
  * @retval 0-无数据, 1-有数据 / 0-no data, 1-has data
  */
uint8_t MAX30102_CheckFIFOStatus(void)
{
    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    uint8_t status_reg = 0;
    
    /* 读取中断状态寄存器1检查FIFO状态 / Read INTR status 1 for FIFO state */
    if (MAX30102_ReadReg(REG_INTR_STATUS_1, &status_reg) == 0)
    {
        /* 检查FIFO几乎满标志 (bit 5) 或其他数据就绪标志
         * Check FIFO almost-full (bit 5) or other data-ready flags */
        if ((status_reg & 0xE0) != 0)
        {
            return 1;  // 有数据 / Has data
        }
    }
    
    /* 通过比较FIFO读写指针判断是否有数据 / Compare FIFO read/write pointers */
    if (MAX30102_ReadReg(REG_FIFO_WR_PTR, &write_ptr) == 0)
    {
        if (MAX30102_ReadReg(REG_FIFO_RD_PTR, &read_ptr) == 0)
        {
            /* 提取低5位（FIFO指针只有5位）/ Mask lower 5 bits (FIFO ptr is 5-bit) */
            write_ptr = write_ptr & 0x1F;
            read_ptr = read_ptr & 0x1F;
            
            /* 如果读写指针不同，说明有数据 / Different pointers mean data available */
            if (write_ptr != read_ptr)
            {
                return 1;  // 有数据 / Has data
            }
        }
    }
    
    return 0;  // 无数据 / No data
}
