#ifndef __OLED_H__
#define __OLED_H__

#include <stdbool.h>
#include <stdint.h>




/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;




#define SSD1306_ADDR 0x78
#define SSD1306_CMD 0x00
#define SSD1306_DATA 0x40
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_ROW_NUM 8   // 每页行数
#define SSD1306_PAGE_NUM 8  // 总页数
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

typedef enum {
    SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
    SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR_t;



typedef enum { FONT_6x8 = 0, FONT_6X16 = 1 } SSD1306_FONT;

typedef enum {
  BLACK = 0,
  WHITE = 1,
} SSD1306_COLOR;



void SSD1306_Init(void);

void SSD1306_OFF(void);
void SSD1306_ON(void);


void SSD1306_Write_Data(uint8_t data);
void SSD1306_Write_Cmd(uint8_t cmd);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t mode);
uint32_t OLED_Pow(uint32_t X, uint32_t Y);

void OLED_ShowFNum(uint8_t x,uint8_t y,float Fnum);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);


#endif

