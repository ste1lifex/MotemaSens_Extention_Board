/**
  ******************************************************************************
  * @file     bsp_spi_oled.c
  * @author   embedfire
  * @version  V1.0
  * @date     2024
  * @brief    SPI OLED应用接口 - STM32F10x标准库版本
  *           SPI OLED application interface - STM32F10x stdlib port
  * @note     引脚分配: PA2-RES, PA3-DC, PA4-CS, PA5-SCK, PA7-MOSI
  *           Pinout: PA2-RES, PA3-DC, PA4-CS, PA5-SCK, PA7-MOSI
  ******************************************************************************
  */
#include "bsp_spi_oled.h"
#include "codetab.h"
#include "spi.h"
#include "gpio.h"

/**
  * @brief  简单延时函数 / Simple delay
  * @param  ms: 延时毫秒数 / Delay in milliseconds
  * @retval 无 / None
  */
extern SPI_HandleTypeDef hspi1;

static void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  SPI和GPIO初始化 / SPI and GPIO init
  * @param  无 / None
  * @note   无 / None
  * @retval 无 / None
  */
void Bsp_SPI_OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* CS/DC/RES 由软件控制 / CS/DC/RES under software control */
    GPIO_InitStruct.Pin = OLED_CS_PIN | OLED_DC_PIN | OLED_RES_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(OLED_CS_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_SET);
}

/**
  * @brief  SPI发送一个字节 / SPI transmit one byte
  * @param  data: 要发送的数据 / Data to send
  * @retval 无 / None
  */
static void SPI_WriteByte(uint8_t data)
{
    (void)HAL_SPI_Transmit(&hspi1, &data, 1, 100);
}

 /**
  * @brief  WriteCmd向OLED写命令 / Write command to OLED
  * @param  Command: 命令字 / Command byte
  * @retval 无 / None
  */
static void WriteCmd(unsigned char Command)
{
    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET);
    SPI_WriteByte(Command);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
}

 /**
  * @brief  WriteDat向OLED写数据 / Write data to OLED
  * @param  Data: 数据 / Data byte
  * @retval 无 / None
  */
static void WriteDat(unsigned char Data)
{
    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET);
    SPI_WriteByte(Data);
    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET);
}

 /**
  * @brief  OLED_Init初始化OLED / Initialize OLED
  * @param  无 / None
  * @retval 无 / None
  */
void OLED_Init(void)
{
    /* 硬件复位 / Hardware reset */
    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_RESET);
    Delay_ms(100);
    HAL_GPIO_WritePin(OLED_RES_PORT, OLED_RES_PIN, GPIO_PIN_SET);
    Delay_ms(100);
    
    /* 初始化序列 / Initialization sequence */
    WriteCmd(0xAE); //display off
    WriteCmd(0x20); //Set Memory Addressing Mode	
    WriteCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    WriteCmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
    WriteCmd(0xc8); //Set COM Output Scan Direction
    WriteCmd(0x00); //---set low column address
    WriteCmd(0x10); //---set high column address
    WriteCmd(0x40); //--set start line address
    WriteCmd(0x81); //--set contrast control register
    WriteCmd(0xff); //亮度调节 0x00~0xff / Brightness 0x00~0xff
    WriteCmd(0xa1); //--set segment re-map 0 to 127
    WriteCmd(0xa6); //--set normal display
    WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
    WriteCmd(0x3F); //
    WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    WriteCmd(0xd3); //-set display offset
    WriteCmd(0x00); //-not offset
    WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
    WriteCmd(0xf0); //--set divide ratio
    WriteCmd(0xd9); //--set pre-charge period
    WriteCmd(0x22); //
    WriteCmd(0xda); //--set com pins hardware configuration
    WriteCmd(0x12);
    WriteCmd(0xdb); //--set vcomh
    WriteCmd(0x20); //0x20,0.77xVcc
    WriteCmd(0x8d); //--set DC-DC enable
    WriteCmd(0x14); //
    WriteCmd(0xaf); //--turn on oled panel
}

 /**
  * @brief  OLED_SetPos设置坐标（内部函数）/ Set position (internal)
  * @param  x,列x位置 / Column x
  *         y,页y位置 / Page y
  * @retval 无 / None
  */
static void OLED_SetPos(unsigned char x, unsigned char y)
{ 
    WriteCmd(0xb0+y);
    WriteCmd(((x&0xf0)>>4)|0x10);
    WriteCmd((x&0x0f)|0x01);
}

 /**
  * @brief  OLED_Fill填充整个屏幕 / Fill entire screen
  * @param  fill_Data:要填充的数据 / Fill pattern
  * @retval 无 / None
  */
void OLED_Fill(unsigned char fill_Data)
{
    unsigned char m,n;
    for(m=0;m<8;m++)
    {
        WriteCmd(0xb0+m);
        WriteCmd(0x00);
        WriteCmd(0x10);
        for(n=0;n<128;n++)
        {
            WriteDat(fill_Data);
        }
    }
}

 /**
  * @brief  OLED_CLS清屏 / Clear screen
  * @param  无 / None
  * @retval 无 / None
  */
void OLED_CLS(void)
{
    OLED_Fill(0x00);
}

 /**
  * @brief  OLED_ON点亮OLED显示/睡眠唤醒 / Turn OLED on / wake from sleep
  * @param  无 / None
  * @retval 无 / None
  */
void OLED_ON(void)
{
    WriteCmd(0X8D);
    WriteCmd(0X14);
    WriteCmd(0XAF);
}

 /**
  * @brief  OLED_OFF关闭OLED显示 / Turn OLED off
  * @param  无 / None
  * @retval 无 / None
  */
void OLED_OFF(void)
{
    WriteCmd(0X8D);
    WriteCmd(0X10);
    WriteCmd(0XAE);
}

 /**
  * @brief  OLED_ShowStr显示字符串 / Display ASCII string
  * @param  x,y : 起始坐标(x:0~127, y:0~7); / Start (x:0~127, y:0~7)
  *         ch[] : 要显示的字符串; / String to display
  *         TextSize : 字符大小(1:6*8 ; 2:8*16) / Size (1:6x8, 2:8x16)
  * @retval 无 / None
  */
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
    unsigned char c = 0,i = 0,j = 0;
    switch(TextSize)
    {
        case 1:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<6;i++)
                    WriteDat(F6x8[c][i]);
                x += 6;
                j++;
            }
        }break;
        case 2:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<8;i++)
                    WriteDat(F8X16[c*16+i]);
                OLED_SetPos(x,y+1);
                for(i=0;i<8;i++)
                    WriteDat(F8X16[c*16+i+8]);
                x += 8;
                j++;
            }
        }break;
    }
}

 /**
  * @brief  OLED_ShowCN显示汉字 / Display Chinese glyph
  * @param  x,y: 起始坐标(x:0~127, y:0~7); / Start (x:0~127, y:0~7)
  *         N:汉字在codetab.h中的序号 / Glyph index in codetab.h
  * @retval 无 / None
  */
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
    unsigned char wm=0;
    unsigned int  adder=32*N;
    OLED_SetPos(x , y);
    for(wm = 0;wm < 16;wm++)
    {
        WriteDat(F16x16[adder]);
        adder += 1;
    }
    OLED_SetPos(x,y + 1);
    for(wm = 0;wm < 16;wm++)
    {
        WriteDat(F16x16[adder]);
        adder += 1;
    }
}

 /**
  * @brief  OLED_DrawBMP显示BMP位图 / Draw BMP bitmap
  * @param  x0,y0 :起始坐标(x0:0~127, y0:0~7); / Start (x0:0~127, y0:0~7)
  *         x1,y1 : 对角(右下角)坐标(x1:1~128,y1:1~8) / Bottom-right (x1:1~128, y1:1~8)
  * @retval 无 / None
  */
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
    unsigned int j=0;
    unsigned char x,y;

    if(y1%8==0)
        y = y1/8;
    else
        y = y1/8 + 1;
    for(y=y0;y<y1;y++)
    {
        OLED_SetPos(x0,y);
        for(x=x0;x<x1;x++)
        {
            WriteDat(BMP[j++]);
        }
    }
}
