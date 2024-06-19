#ifndef __LCD_7735_H
#define __LCD_7735_H		

#include "lcd_api.h"
#include "rgb_color.h"

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned long  u32;
typedef char  i8;
typedef short i16;
typedef long  i32;


//-----------------OLED端口定义---------------- 
#ifdef __cplusplus
extern "C" {
#endif

#define RGB_ORDER       1 // 0: RGB, 1: BGR
#define HAS_BLK_CNTL    0

#define LCD_WIDTH 160
#define LCD_HEIGHT 80

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


extern Lcd_Oper  lcd_st7735;


#ifdef __cplusplus
}
#endif

#endif
