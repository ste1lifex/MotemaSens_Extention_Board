#ifndef __BSP_MAX30102_H
#define __BSP_MAX30102_H

#include "main.h"

/* MAX30102 I2C地址 / MAX30102 I2C address */
#define MAX30102_I2C_WRITE_ADDR  0xAE  // 写地址 / Write address
#define MAX30102_I2C_READ_ADDR  0xAF  // 读地址 / Read address

/* 寄存器地址 / Register addresses */
#define REG_INTR_STATUS_1    0x00
#define REG_INTR_STATUS_2    0x01
#define REG_INTR_ENABLE_1    0x02
#define REG_INTR_ENABLE_2    0x03
#define REG_FIFO_WR_PTR      0x04
#define REG_OVF_COUNTER      0x05
#define REG_FIFO_RD_PTR      0x06
#define REG_FIFO_DATA        0x07
#define REG_FIFO_CONFIG      0x08
#define REG_MODE_CONFIG      0x09
#define REG_SPO2_CONFIG      0x0A
#define REG_LED1_PA          0x0C
#define REG_LED2_PA          0x0D
#define REG_PILOT_PA         0x10
#define REG_MULTI_LED_CTRL1  0x11
#define REG_MULTI_LED_CTRL2  0x12
#define REG_TEMP_INTR        0x1F
#define REG_TEMP_FRAC        0x20
#define REG_TEMP_CONFIG      0x21
#define REG_PROX_INT_THRESH  0x30
#define REG_REV_ID           0xFE
#define REG_PART_ID          0xFF

/* 函数声明 / Function declarations */
void Bsp_MAX30102_Init(void);
uint8_t MAX30102_WriteReg(uint8_t reg_addr, uint8_t reg_data);
uint8_t MAX30102_ReadReg(uint8_t reg_addr, uint8_t *reg_data);
uint8_t MAX30102_ReadFIFO(uint32_t *red_led, uint32_t *ir_led);
uint8_t MAX30102_Reset(void);
void MAX30102_EXTI_Init(void);  // 外部中断初始化 / External interrupt init
uint8_t MAX30102_CheckFIFOStatus(void);  // 检查FIFO状态 / Check FIFO status
uint8_t MAX30102_SetLEDCurrent(uint8_t led1_current, uint8_t led2_current);  // 设置LED电流强度 / Set LED current

#endif /* __BSP_MAX30102_H */
