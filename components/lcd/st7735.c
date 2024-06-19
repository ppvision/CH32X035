#include "./st7735.h"
#include "lcd_api.h"

#if 1

static int _rotation = 3;  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏



#define USE_ZJY_LCD  1


// static void LCD_ShowString(Hw_SPI *spi,u16 x,u16 y,const u8 *p,u16 mod,u16 color);
// static void LCD_ShowChar(Hw_SPI *spi,u16 x,u16 y,u8 str,u16 color);
// static void LCD_Circle(Hw_SPI *spi,u16 x0,u16 y0,u8 r,u16 color);
// static void LCD_DrawRectangle(Hw_SPI *spi,u16 x1, u16 y1, u16 x2, u16 y2,u16 color);
// static void LCD_DrawLine(Hw_SPI *spi,u16 x1,u16 y1,u16 x2,u16 y2,u16 color);
// static void LCD_Fill(Hw_SPI *spi,u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
// static void LCD_DrawPoint(Hw_SPI *spi,u16 x,u16 y,u16 color);
// static void LCD_SetRotation(Hw_SPI *spi,u8 rot);
// static void LCD_Clear(Hw_SPI *spi,u16 Color);
// static void LCD_Init(Hw_SPI *spi);
// static void LCD_Address_Set(Hw_SPI *spi,u16 x1,u16 y1,u16 x2,u16 y2);
// static void LCD_reset(Hw_SPI *spi);
// static void LCD_ShowPicture(Hw_SPI *spi,u16 x1, u16 y1, u16 x2, u16 y2, u8 *image);
// static void LCD_DrawCircle(Hw_SPI *spi,u16 x0,u16 y0,u8 r,u16 color);




/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/

static void LCD_Address_Set(Hw_SPI *spi,u16 x1,u16 y1,u16 x2,u16 y2)
{
   #if 1
      static int offset_x = 1; //1
      static int offset_y = 26;
   #else
      static int offset_x = 0; //1   -- 0
      static int offset_y = 24;

   #endif


   if(_rotation==0 ||_rotation==1)
   {
      lcd_send_cmd(spi,0x2a);//列地址设置
      lcd_send_word(spi,x1+offset_y);
      lcd_send_word(spi,x2+offset_y);
      lcd_send_cmd(spi,0x2b);//行地址设置
      lcd_send_word(spi,y1+offset_x);
      lcd_send_word(spi,y2+offset_x);
      lcd_send_cmd(spi,0x2c);//储存器写
   }
   else if(_rotation==2 || _rotation==3)
   {
      lcd_send_cmd(spi,0x2a);//列地址设置
      lcd_send_word(spi,x1+offset_x);
      lcd_send_word(spi,x2+offset_x);
      lcd_send_cmd(spi,0x2b);//行地址设置
      lcd_send_word(spi,y1+offset_y);
      lcd_send_word(spi,y2+offset_y);
      lcd_send_cmd(spi,0x2c);//储存器写
   }
   
}

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void LCD_SetRotation(Hw_SPI *spi,u8 rot)
{
   _rotation = rot & 0x3;
   // Memory data access control (MADCTL)
   lcd_send_cmd(spi,0x36);
   if(_rotation==0)   lcd_send_data(spi,0x00 | (RGB_ORDER<<3));
   else if(_rotation==1)lcd_send_data(spi,0xC0 | (RGB_ORDER<<3));
   else if(_rotation==2)lcd_send_data(spi,0x70 | (RGB_ORDER<<3));
   else lcd_send_data(spi,0xA0 | (RGB_ORDER<<3));
}

static void LCD_Clear(Hw_SPI *spi,u16 Color);

/******************************************************************************
      函数说明：LCD初始化函数
      入口数据：无
      返回值：  无
******************************************************************************/
static void LCD_Init(Hw_SPI *spi)
{
#if HAS_BLK_CNTL
   gpio_init(PIN_LCD_BLK);
   gpio_set_dir(PIN_LCD_BLK, GPIO_OUT);
#endif
   lcd_reset(spi);
   const uint8_t init2[]={
      0x11,TFT_INIT_DELAY,120,// turn off sleep mode
      0x20,0,//Display inversion
      0xB1,3,0x05,0x3C,0x3C,
      0xB2,3,0x05,0x3C,0x3C,
      0xB3,6,0x05,0x3C,0x3C,0x05,0x3C,0x3C,// Set the frame frequency of the Partial mode/ full colors
      0xB4,1,0x03,

      0xC0,3,0xAB,0x0B,0x04,//AVDD GVDD
      0xC1,1,0xC5,
      0xC2,2,0x0D,0x00,//Normal Mode
      0xC3,2,0x8D,0x6A,//Normal Mode
      0xC4,2,0x8D,0xEE,
      0xC5,1,0x0F,/*VCOM*/

      0xE0,16,0x07,0x0E,0x08,0x07,0x10,0x07,0x02,0x07,0x09,0x0F,0x25,0x36,0x00,0x08,0x04,0x10,
      0xE1,16,0x0A,0x0D,0x08,0x07,0x0F,0x07,0x02,0x07,0x09,0x0F,0x25,0x35,0x00,0x09,0x04,0x10,
      0xFC,1,0x80,   
      0x3A,1,0x05,// 16-bit/pixel
      0,
   };
   #if 0
   const uint8_t init1[]={
      0x11,TFT_INIT_DELAY,100,// turn off sleep mode
      0x21,0,
      0xB1,3,0x05,0x3A,0x3A,// Set the frame frequency of the full colors normal mode
      0xB2,3,0x05,0x3A,0x3A,// Set the frame frequency of the Idle mode
      0xB3,6,0x05,0x3A,0x3A,0x05,0x3A,0x3A,// Set the frame frequency of the Partial mode/ full colors
      0xB4,1,0x03,

      0xC0,3,0x62,0x02,0x04,
      0xC1,1,0xC0,
      0xC2,2,0x0D,0x00,
      0xC3,2,0x8D,0x6A,
      0xC4,2,0x8D,0xEE,
      0xC5,1,0x0E,/*VCOM*/

      0xE0,16,0x10,0x0E ,0x02 ,0x03 ,0x0E ,0x07 ,0x02 ,0x07 ,0x0A ,0x12 ,0x27 ,0x37 ,0x00 ,0x0D ,0x0E ,0x10,
      0xE1,16,0x10,0x0E ,0x03 ,0x03 ,0x0F ,0x06 ,0x02 ,0x08 ,0x0A ,0x13 ,0x26 ,0x36 ,0x00 ,0x0D ,0x0E ,0x10,
      0x3A,1,0x05,// 16-bit/pixel
      0,
   };
   #endif
   lcd_send_cmds(spi,init2);
   LCD_SetRotation(spi,_rotation);
   // LCD_Clear(spi,WHITE);
   // BACK_COLOR=WHITE;
   lcd_send_cmd(spi,0x29); // Display On
}


static u16 LCD_W()
{
   if (_rotation == 0 || _rotation == 1) {
      return LCD_HEIGHT;//80
   } else {
      return LCD_WIDTH;
   }
}

static u16 LCD_H()
{
   if (_rotation == 0 || _rotation == 1) {
      return LCD_WIDTH;
   } else {
      return LCD_HEIGHT;
   }
}

/******************************************************************************
      函数说明：LCD清屏函数
      入口数据：无
      返回值：  无
******************************************************************************/
static void LCD_Clear(Hw_SPI *spi,u16 Color)
{
   u16 i,j;    
   LCD_Address_Set(spi,0,0,LCD_W()-1,LCD_H()-1);
   for(i=0;i<LCD_W();i++)
   {
      for (j=0;j<LCD_H();j++)
      {
         lcd_send_word(spi,Color);
      }
   }
//   lcd_send_cmd(spi,0x2c);
}



/******************************************************************************
      函数说明：LCD显示汉字
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
static void LCD_DrawPoint(Hw_SPI *spi,u16 x,u16 y,u16 color)
{
   LCD_Address_Set(spi,x,y,x,y);//设置光标位置 
   lcd_send_word(spi,color);

} 

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
      返回值：  无
******************************************************************************/
static void LCD_Fill(Hw_SPI *spi,u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
   u16 i,j; 
   LCD_Address_Set(spi,xsta,ysta,xend,yend);      //设置光标位置 
   for(i=ysta;i<=yend;i++)
   {                                               
      for(j=xsta;j<=xend;j++) lcd_send_word(spi,color);//设置光标位置        
   }                     
}



static void LCD_DrawLine(Hw_SPI *spi,u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
   u16 t; 
   int xerr=0,yerr=0,delta_x,delta_y,distance;
   int incx,incy,uRow,uCol;
   delta_x=x2-x1; //计算坐标增量 
   delta_y=y2-y1;
   uRow=x1;//画线起点坐标
   uCol=y1;
   if(delta_x>0)incx=1; //设置单步方向 
   else if (delta_x==0)incx=0;//垂直线 
   else {incx=-1;delta_x=-delta_x;}
   if(delta_y>0)incy=1;
   else if (delta_y==0)incy=0;//水平线 
   else {incy=-1;delta_y=-delta_y;}
   if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
   else distance=delta_y;
   for(t=0;t<distance+1;t++)
   {
      LCD_DrawPoint(spi,uRow,uCol,color);//画点
      xerr+=delta_x;
      yerr+=delta_y;
      if(xerr>distance)
      {
         xerr-=distance;
         uRow+=incx;
      }
      if(yerr>distance)
      {
         yerr-=distance;
         uCol+=incy;
      }
   }
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
      返回值：  无
******************************************************************************/
static void LCD_DrawRectangle(Hw_SPI *spi,u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
   LCD_DrawLine(spi,x1,y1,x2,y1,color);
   LCD_DrawLine(spi,x1,y1,x1,y2,color);
   LCD_DrawLine(spi,x1,y2,x2,y2,color);
   LCD_DrawLine(spi,x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
      返回值：  无
******************************************************************************/
static void LCD_DrawCircle(Hw_SPI *spi,u16 x0,u16 y0,u8 r,u16 color)
{
   int a,b;
   // int di;
   a=0;b=r;   
   while(a<=b)
   {
      LCD_DrawPoint(spi,x0-b,y0-a,color);             //3           
      LCD_DrawPoint(spi,x0+b,y0-a,color);             //0           
      LCD_DrawPoint(spi,x0-a,y0+b,color);             //1                
      LCD_DrawPoint(spi,x0-a,y0-b,color);             //2             
      LCD_DrawPoint(spi,x0+b,y0+a,color);             //4               
      LCD_DrawPoint(spi,x0+a,y0-b,color);             //5
      LCD_DrawPoint(spi,x0+a,y0+b,color);             //6 
      LCD_DrawPoint(spi,x0-b,y0+a,color);             //7
      a++;
      if((a*a+b*b)>(r*r))//判断要画的点是否过远
      {
         b--;
      }
   }
}


/******************************************************************************
      函数说明：显示字符
      入口数据：x,y    起点坐标
                num    要显示的字符
                mode   1叠加方式  0非叠加方式
      返回值：  无
******************************************************************************/
static void LCD_ShowChar(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
   uint32_t i, b, j;
   LCD_Address_Set(spi,x, y, x + font.width - 1, y + font.height - 1);
   for (i = 0; i < font.height; i++) {
      b = font.data[(ch - 32) * font.height + i];
      for (j = 0; j < font.width; j++) {
         if ((b << j) & 0x8000) {
            lcd_send_word(spi,color);
         }
         else {
            lcd_send_word(spi,color);
         }
      }
   }
}


static void LCD_ShowString(Hw_SPI *spi,uint16_t x, uint16_t y, const char *str,FontDef font,uint16_t color, uint16_t bgcolor)
{
   while (*str) {
      if (x + font.width >= LCD_W()) {
         x = 0;
         y += font.height;
         if (y + font.height >= LCD_H()) {
            break;
         }
         if (*str == ' ') {
            // skip spaces in the beginning of the new line
            str++;
            continue;
         }
      }
      LCD_ShowChar(spi,x, y, *str, font, color, bgcolor);
      x += font.width;
      str++;
   }
}


/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
static void LCD_ShowPicture(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
   if ((x >= LCD_W()) || (y >= LCD_H()))
      return;
   if ((x + w - 1) >= LCD_W())
      return;
   if ((y + h - 1) >=  LCD_H())
      return;

   LCD_Address_Set(spi,x, y, x + w , y + h );
   lcd_send_buffer(spi,(uint8_t *)data, sizeof(uint16_t) * w * h);
}

static void SendData(Hw_SPI *spi,uint8_t* data,size_t len){
   lcd_send_buffer(spi,(uint8_t *)data, len);
}

static void LCD_test(Hw_SPI *spi){

    LCD_Clear(spi,BLACK);
    LCD_Clear(spi,BLUE);
    LCD_Clear(spi,BRED);
    LCD_Clear(spi,GRED);
    LCD_Clear(spi,GBLUE);
    LCD_Clear(spi,RED);
    LCD_Clear(spi,MAGENTA);
    LCD_Clear(spi,GREEN);
    LCD_Clear(spi,CYAN);
    LCD_Clear(spi,YELLOW);
    LCD_Clear(spi,BROWN);
    LCD_Clear(spi,GRAY);
    LCD_Clear(spi,BRRED);

}

static void LCD_InvertColors(Hw_SPI *spi,uint8_t  invert){
    lcd_send_cmd(spi,invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
}

 Lcd_Oper  lcd_st7735 = {
    .init          = LCD_Init,
    .reset         = lcd_reset,
    .SetRotation   = LCD_SetRotation,
    .Fill          = LCD_Fill,
    .DrawPixel     = LCD_DrawPoint,
    .DrawLine      = LCD_DrawLine,
    .DrawRectangle = LCD_DrawRectangle,
    .DrawCircle    = LCD_DrawCircle,
    .DrawImage     = LCD_ShowPicture,
    .DrawChar      = LCD_ShowChar,
    .DrawString    = LCD_ShowString,
    .SetAddress    = LCD_Address_Set,
    .SendData      = SendData,
    .test          = LCD_test,
    .clear         = LCD_Clear,
    .invert        = LCD_InvertColors,
 };


#endif

