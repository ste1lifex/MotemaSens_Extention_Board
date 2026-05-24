#ifndef __BSP_SPI_OLED_H
#define __BSP_SPI_OLED_H

/* 包含头文件 */
#include "main.h"

/* 引脚定义 - 根据实际引脚分配表配置 */
#define OLED_CS_PORT      GPIOA
#define OLED_CS_PIN       GPIO_PIN_4
#define OLED_DC_PORT      GPIOA
#define OLED_DC_PIN       GPIO_PIN_3
#define OLED_RES_PORT     GPIOA
#define OLED_RES_PIN      GPIO_PIN_2

/* 宏定义 */
#define OLED_CS_LOW()     HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET)
#define OLED_CS_HIGH()    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET)
#define OLED_DC_LOW()     HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_DC_HIGH()    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_RES_LOW()    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_RESET)
#define OLED_RES_HIGH()   HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_SET)

/* 函数声明 */
void Bsp_SPI_OLED_Init(void);
void OLED_Init(void);
void OLED_Fill(unsigned char fill_Data);   // 全屏填充
void OLED_CLS(void);                        // 清屏
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

#endif /* __BSP_SPI_OLED_H */

