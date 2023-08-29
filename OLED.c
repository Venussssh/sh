#include "OLED.h"
#include "stdio.h"
#include <stdarg.h>
#include <string.h>
#include "i2c.h"
#include "OLED_Font.h"

/* Private SSD1306 structure */

/* Private variable */
// static SSD1306_t SSD1306;





void SSD1306_Write_Cmd(uint8_t cmd)
{
    SSD1306_I2C_Write(0x00, cmd);
}

void SSD1306_Write_Data(uint8_t data)
{
    SSD1306_I2C_Write(0x40, data);
}

/* 打开显示 */
void SSD1306_ON(void) 
{
    SSD1306_Write_Cmd(0x8D);
    SSD1306_Write_Cmd(0x14);
    SSD1306_Write_Cmd(0xAF);
}

/* 关闭显示 */
void SSD1306_OFF(void) 
{
    SSD1306_Write_Cmd(0x8D);
    SSD1306_Write_Cmd(0x10);
    SSD1306_Write_Cmd(0xAE);
}

/* 更新屏幕显示，当我们把显示缓冲区数据改写了之后，必须调用这个函数才能在屏幕更新显示 */
// void SSD1306_UpdateScreen(void) 
// {
//     uint8_t page;

//     for (page = 0; page < 8; page++) 
//     {
//         SSD1306_Write_Cmd(0xB0 + page);
//         SSD1306_Write_Cmd(0x00);
//         SSD1306_Write_Cmd(0x10);

//         /* Write multi data */
//         SSD1306_Write_MultiData(&SSD1306_Buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
//     }
// }




void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	SSD1306_Write_Cmd(0xB0 | Y);					//设置Y位置
	SSD1306_Write_Cmd(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	SSD1306_Write_Cmd(0x00 | (X & 0x0F));			//设置X位置低4位
}


void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		SSD1306_Write_Data(F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		SSD1306_Write_Data(F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			SSD1306_Write_Data(0x00);
		}
	}
}


/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}



/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}


/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}




// * void OLED_ShowFNum()  显示任意浮点数，参考中景园OLED_ShowNum()函数
//  * x , y :  起点坐标	 
//  * Fnum  :  要显示的浮点数
//  * size1 :  字体大小	 		
//  * mode  :  0,反色显示;1,正常显示
//  * @作 者 :  Guard_Byte
//  ***************************************************************/
void OLED_ShowFNum(uint8_t x,uint8_t y,float Fnum)
{
	uint8_t Data[]= " ";                             //创建目标数组，用来存放转换后的字符数据 
    sprintf(Data,"%.2f",Fnum);                       //保留小数点后3位小数，打印到Data数组中
	OLED_ShowString(x,y,Data);            //调用OLED字符串显示函数，在OLED屏上显示
}






void SSD1306_Init(void) 
{
    /* A little delay */
    HAL_Delay(1000);

    /* Init LCD */
    SSD1306_Write_Cmd(0xAE); //display off
    SSD1306_Write_Cmd(0x20); //Set Memory Addressing Mode
    SSD1306_Write_Cmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    SSD1306_Write_Cmd(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
    SSD1306_Write_Cmd(0xC8); //Set COM Output Scan Direction
    SSD1306_Write_Cmd(0x00); //---set low column address
    SSD1306_Write_Cmd(0x10); //---set high column address
    SSD1306_Write_Cmd(0x40); //--set start line address
    SSD1306_Write_Cmd(0x81); //--set contrast control register
    SSD1306_Write_Cmd(0xFF);
    SSD1306_Write_Cmd(0xA1); //--set segment re-map 0 to 127
    SSD1306_Write_Cmd(0xA6); //--set normal display
    SSD1306_Write_Cmd(0xA8); //--set multiplex ratio(1 to 64)
    SSD1306_Write_Cmd(0x3F); //
    SSD1306_Write_Cmd(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    SSD1306_Write_Cmd(0xD3); //-set display offset
    SSD1306_Write_Cmd(0x00); //-not offset
    SSD1306_Write_Cmd(0xD5); //--set display clock divide ratio/oscillator frequency
    SSD1306_Write_Cmd(0xF0); //--set divide ratio
    SSD1306_Write_Cmd(0xD9); //--set pre-charge period
    SSD1306_Write_Cmd(0x22); //
    SSD1306_Write_Cmd(0xDA); //--set com pins hardware configuration
    SSD1306_Write_Cmd(0x12);
    SSD1306_Write_Cmd(0xDB); //--set vcomh
    SSD1306_Write_Cmd(0x20); //0x20,0.77xVcc
    SSD1306_Write_Cmd(0x8D); //--set DC-DC enable
    SSD1306_Write_Cmd(0x14); //
    SSD1306_Write_Cmd(0xAF); //--turn on SSD1306 panel

    /* Clear screen */
    OLED_Clear();

    
}
