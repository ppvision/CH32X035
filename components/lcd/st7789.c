#include "st7789.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_Test(Hw_SPI *spi);
void ST7789_TearEffect(Hw_SPI *spi,uint8_t tear);
void ST7789_DrawFilledCircle(Hw_SPI *spi,int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7789_DrawFilledTriangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawTriangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ST7789_DrawFilledRectangle(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_DrawImage(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void ST7789_DrawPixel_4px(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color);
void ST7789_Init(Hw_SPI *spi);
static void ST7789_SetAddressWindow(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);


void ST7789_DrawPixel(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color);
void ST7789_Fill(Hw_SPI *spi,uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);
void ST7789_DrawLine(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ST7789_DrawRectangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_DrawCircle(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void ST7789_WriteChar(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_WriteString(Hw_SPI *spi,uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7789_SetRotation(Hw_SPI *spi,uint8_t m);
void ST7789_Fill_Color(Hw_SPI *spi,uint16_t color);
void ST7789_InvertColors(Hw_SPI *spi,uint8_t invert);


void ST7789_SetRotation(Hw_SPI *spi,uint8_t m)
{
	lcd_send_cmd(spi,ST7789_MADCTL);	// MADCTL
    lcd_send_cmd(spi,0x2a);
	switch (m) {
	case 0:
		lcd_send_data(spi,ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		break;
	case 1:
		lcd_send_data(spi,ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	case 2:
		lcd_send_data(spi,ST7789_MADCTL_RGB);
		break;
	case 3:
		lcd_send_data(spi,ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	default:
		break;
	}
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;
	
	/* Column Address set */
	lcd_send_cmd(spi,ST7789_CASET);
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};

		lcd_send_buffer(spi,data, sizeof(data));
	}

	/* Row Address set */
	lcd_send_cmd(spi,ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		lcd_send_buffer(spi,data, sizeof(data));
	}
	/* Write to RAM */
	lcd_send_cmd(spi,ST7789_RAMWR);
}

/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init(Hw_SPI *spi)
{
	#ifdef USE_DMA
		memset(disp_buf, 0, sizeof(disp_buf));
	#endif
		
	lcd_reset(spi);


	const uint8_t init1[]={
    	0x01,TFT_INIT_DELAY,120,//Software reset
    	0x11,TFT_INIT_DELAY,120,//Sleep exit 
    	0xf0,1,0xC3,
    	0xf0,1,0x96,
    	0x36,1,0x48,
    	0x3a,1,0x55,
    	0xb4,1,0x01,
    	0xb6,1,0x80, 0x02 ,0x3B,
    	0xE8,8,0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33,
    	0xC1,1,0x06,
    	0xC2,1,0xa7,
    	0xC5,1,0x18,
    	0xC5,1+TFT_INIT_DELAY,0x18,120,
    	0xE0,14,0xF0,0x09 ,0x0b ,0x06 ,0x04 ,0x15 ,0x2F ,0x54 ,0x42 ,0x3C ,0x17 ,0x14 ,0x18 ,0x1B,
    	0xE1,14+TFT_INIT_DELAY,0xE0,0x09 ,0x0B ,0x06 ,0x04 ,0x03 ,0x2B ,0x43 ,0x42 ,0x3B ,0x16 ,0x14 ,0x17 ,0x1B,120,
    	0xF0,1,0x3C,
    	0xF0,1,0x69,
    	0x29,0,
    	0
    };
    lcd_send_cmds(spi,init1);
#if 0		
    lcd_send_cmd(spi,ST7789_COLMOD);		//	Set color mode
    lcd_send_byte(spi,ST7789_COLOR_MODE_16bit);
  	lcd_send_cmd(spi,0xB2);				//	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		lcd_send_buffer(spi,data, sizeof(data));
	}
	ST7789_SetRotation(spi,ST7789_ROTATION);	//	MADCTL (Display Rotation)
	
	/* Internal LCD Voltage generator settings */
    lcd_send_cmd(spi,0XB7);				//	Gate Control
    lcd_send_byte(spi,0x35);			//	Default value
    lcd_send_cmd(spi,0xBB);				//	VCOM setting
    lcd_send_byte(spi,0x19);			//	0.725v (default 0.75v for 0x20)
    lcd_send_cmd(spi,0xC0);				//	LCMCTRL
    lcd_send_byte(spi,0x2C);			//	Default value
    lcd_send_cmd(spi,0xC2);				//	VDV and VRH command Enable
    lcd_send_byte(spi,0x01);			//	Default value
    lcd_send_cmd(spi,0xC3);				//	VRH set
    lcd_send_byte(spi,0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
    lcd_send_cmd(spi,0xC4);				//	VDV set
    lcd_send_byte(spi,0x20);			//	Default value
    lcd_send_cmd(spi,0xC6);				//	Frame rate control in normal mode
    lcd_send_byte(spi,0x0F);			//	Default value (60HZ)
    lcd_send_cmd(spi,0xD0);				//	Power control
    lcd_send_byte(spi,0xA4);			//	Default value
    lcd_send_byte(spi,0xA1);			//	Default value
	/**************** Division line ****************/

	lcd_send_cmd(spi,0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		lcd_send_buffer(spi,data, sizeof(data));
	}

    lcd_send_cmd(spi,0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		lcd_send_buffer(spi,data, sizeof(data));
	}

#endif
    lcd_send_cmd(spi,ST7789_INVON);		//	Inversion ON
	lcd_send_cmd(spi,ST7789_SLPOUT);	//	Out of sleep mode
  	lcd_send_cmd(spi,ST7789_NORON);		//	Normal Display on
  	lcd_send_cmd(spi,ST7789_DISPON);	//	Main screen turned on	

	sleep_ms(50);
	ST7789_Fill_Color(spi,BLACK);				//	Fill with Black.
}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(Hw_SPI *spi,uint16_t color)

{
	uint16_t i;
	ST7789_SetAddressWindow(spi,0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);

	#ifdef USE_DMA
		for (i = 0; i < ST7789_HEIGHT / HOR_LEN; i++)
		{
			memset(disp_buf, color, sizeof(disp_buf));
			lcd_send_buffer(spi,disp_buf, sizeof(disp_buf));
		}
	#else
		uint16_t j;
		for (i = 0; i < ST7789_WIDTH; i++)
				for (j = 0; j < ST7789_HEIGHT; j++) {
					uint8_t data[] = {color >> 8, color & 0xFF};
					lcd_send_buffer(spi,data, sizeof(data));
				}
	#endif
}

/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */

void ST7789_DrawPixel(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color)
{
	if ((x < 0) || (x >= ST7789_WIDTH) ||
		 (y < 0) || (y >= ST7789_HEIGHT))	return;
	
	ST7789_SetAddressWindow(spi,x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};
	lcd_send_buffer(spi,data, sizeof(data));
}

/**
 * @brief Fill an Area with single color
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill(Hw_SPI *spi,uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
	if ((xEnd < 0) || (xEnd >= ST7789_WIDTH) ||
		 (yEnd < 0) || (yEnd >= ST7789_HEIGHT))	return;
	uint16_t i, j;
	ST7789_SetAddressWindow(spi,xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++)
		for (j = xSta; j <= xEnd; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			lcd_send_buffer(spi,data, sizeof(data));
		}
}

/**
 * @brief Draw a big Pixel at a point
 * @param x&y -> coordinate of the point
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel_4px(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color)
{
	if ((x <= 0) || (x > ST7789_WIDTH) ||
		 (y <= 0) || (y > ST7789_HEIGHT))	return;
	ST7789_Fill(spi,x - 1, y - 1, x + 1, y + 1, color);
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
         {
	uint16_t swap;
    uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            ST7789_DrawPixel(spi,y0, x0, color);
        } else {
            ST7789_DrawPixel(spi,x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Draw a Rectangle with single color
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param color -> color of the Rectangle line
 * @return none
 */
void ST7789_DrawRectangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	ST7789_DrawLine(spi,x1, y1, x2, y1, color);
	ST7789_DrawLine(spi,x1, y1, x1, y2, color);
	ST7789_DrawLine(spi,x1, y2, x2, y2, color);
	ST7789_DrawLine(spi,x2, y1, x2, y2, color);
}

/** 
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void ST7789_DrawCircle(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(spi,x0, y0 + r, color);
	ST7789_DrawPixel(spi,x0, y0 - r, color);
	ST7789_DrawPixel(spi,x0 + r, y0, color);
	ST7789_DrawPixel(spi,x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(spi,x0 + x, y0 + y, color);
		ST7789_DrawPixel(spi,x0 - x, y0 + y, color);
		ST7789_DrawPixel(spi,x0 + x, y0 - y, color);
		ST7789_DrawPixel(spi,x0 - x, y0 - y, color);

		ST7789_DrawPixel(spi,x0 + y, y0 + x, color);
		ST7789_DrawPixel(spi,x0 - y, y0 + x, color);
		ST7789_DrawPixel(spi,x0 + y, y0 - x, color);
		ST7789_DrawPixel(spi,x0 - y, y0 - x, color);
	}
}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))
		return;
	if ((x + w - 1) >= ST7789_WIDTH)
		return;
	if ((y + h - 1) >= ST7789_HEIGHT)
		return;

	ST7789_SetAddressWindow(spi,x, y, x + w - 1, y + h - 1);
	lcd_send_buffer(spi,(uint8_t *)data, sizeof(uint16_t) * w * h);
}

/**
 * @brief Invert Fullscreen color
 * @param invert -> Whether to invert
 * @return none
 */
void ST7789_InvertColors(Hw_SPI *spi,uint8_t invert)
{
	lcd_send_cmd(spi,invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
}

/** 
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param color -> color of the char
 * @param bgcolor -> background color of the char
 * @return  none
 */
void ST7789_WriteChar(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
	uint32_t i, b, j;
	ST7789_SetAddressWindow(spi,x, y, x + font.width - 1, y + font.height - 1);

	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) {
			if ((b << j) & 0x8000) {
				uint8_t data[] = {color >> 8, color & 0xFF};
				lcd_send_buffer(spi,data, sizeof(data));
			}
			else {
				uint8_t data[] = {bgcolor >> 8, bgcolor & 0xFF};
				lcd_send_buffer(spi,data, sizeof(data));
			}
		}
	}
}

/** 
 * @brief Write a string 
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param color -> color of the string
 * @param bgcolor -> background color of the string
 * @return  none
 */
void ST7789_WriteString(Hw_SPI *spi,uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	while (*str) {
		if (x + font.width >= ST7789_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7789_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		ST7789_WriteChar(spi,x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
}

/** 
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void ST7789_DrawFilledRectangle(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	uint8_t i;
	/* Check input parameters */
	if (x >= ST7789_WIDTH ||
		y >= ST7789_HEIGHT) {
		/* Return error */
		return;
	}
	/* Check width and height */
	if ((x + w) >= ST7789_WIDTH) {
		w = ST7789_WIDTH - x;
	}
	if ((y + h) >= ST7789_HEIGHT) {
		h = ST7789_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		ST7789_DrawLine(spi,x, y + i, x + w, y + i, color);
	}
}

/** 
 * @brief Draw a Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the lines
 * @return  none
 */
void ST7789_DrawTriangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	/* Draw lines */
	ST7789_DrawLine(spi,x1, y1, x2, y2, color);
	ST7789_DrawLine(spi,x2, y2, x3, y3, color);
	ST7789_DrawLine(spi,x3, y3, x1, y1, color);
}

/** 
 * @brief Draw a filled Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the triangle
 * @return  none
 */
void ST7789_DrawFilledTriangle(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	}
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	}
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	}
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7789_DrawLine(spi,x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

/** 
 * @brief Draw a Filled circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle
 * @return  none
 */
void ST7789_DrawFilledCircle(Hw_SPI *spi,int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(spi,x0, y0 + r, color);
	ST7789_DrawPixel(spi,x0, y0 - r, color);
	ST7789_DrawPixel(spi,x0 + r, y0, color);
	ST7789_DrawPixel(spi,x0 - r, y0, color);
	ST7789_DrawLine(spi,x0 - r, y0, x0 + r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawLine(spi,x0 - x, y0 + y, x0 + x, y0 + y, color);
		ST7789_DrawLine(spi,x0 + x, y0 - y, x0 - x, y0 - y, color);

		ST7789_DrawLine(spi,x0 + y, y0 + x, x0 - y, y0 + x, color);
		ST7789_DrawLine(spi,x0 + y, y0 - x, x0 - y, y0 - x, color);
	}
}


/**
 * @brief Open/Close tearing effect line
 * @param tear -> Whether to tear
 * @return none
 */
void ST7789_TearEffect(Hw_SPI *spi,uint8_t tear)
{
	lcd_send_cmd(spi,tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
}


/** 
 * @brief A Simple test function for ST7789
 * @param  none
 * @return  none
 */
void ST7789_Test(Hw_SPI *spi)
{
	ST7789_Fill_Color(spi,WHITE);
	sleep_ms(1000);
	ST7789_WriteString(spi,10, 20, "Speed Test", Font_11x18, RED, WHITE);
	sleep_ms(1000);
	ST7789_Fill_Color(spi,CYAN);
    sleep_ms(500);
	ST7789_Fill_Color(spi,RED);
    sleep_ms(500);
	ST7789_Fill_Color(spi,BLUE);
    sleep_ms(500);
	ST7789_Fill_Color(spi,GREEN);
    sleep_ms(500);
	ST7789_Fill_Color(spi,YELLOW);
    sleep_ms(500);
	ST7789_Fill_Color(spi,BROWN);
    sleep_ms(500);
	ST7789_Fill_Color(spi,DARKBLUE);
    sleep_ms(500);
	ST7789_Fill_Color(spi,MAGENTA);
    sleep_ms(500);
	ST7789_Fill_Color(spi,LIGHTGREEN);
    sleep_ms(500);
	ST7789_Fill_Color(spi,LGRAY);
    sleep_ms(500);
	ST7789_Fill_Color(spi,LBBLUE);
    sleep_ms(500);
	ST7789_Fill_Color(spi,WHITE);
	sleep_ms(500);

	ST7789_WriteString(spi,10, 10, "Font test.", Font_16x26, GBLUE, WHITE);
	ST7789_WriteString(spi,10, 50, "Hello Steve!", Font_7x10, RED, WHITE);
	ST7789_WriteString(spi,10, 75, "Hello Steve!", Font_11x18, YELLOW, WHITE);
	ST7789_WriteString(spi,10, 100, "Hello Steve!", Font_16x26, MAGENTA, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Rect./Line.", Font_11x18, YELLOW, BLACK);
	ST7789_DrawRectangle(spi,30, 30, 100, 100, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Filled Rect.", Font_11x18, YELLOW, BLACK);
	ST7789_DrawFilledRectangle(spi,30, 30, 50, 50, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Circle.", Font_11x18, YELLOW, BLACK);
	ST7789_DrawCircle(spi,60, 60, 25, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Filled Cir.", Font_11x18, YELLOW, BLACK);
	ST7789_DrawFilledCircle(spi,60, 60, 25, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Triangle", Font_11x18, YELLOW, BLACK);
	ST7789_DrawTriangle(spi,30, 30, 30, 70, 60, 40, WHITE);
	sleep_ms(1000);

	ST7789_Fill_Color(spi,RED);
	ST7789_WriteString(spi,10, 10, "Filled Tri", Font_11x18, YELLOW, BLACK);
	ST7789_DrawFilledTriangle(spi,30, 30, 30, 70, 60, 40, WHITE);
	sleep_ms(1000);

	//	If FLASH cannot storage anymore datas, please delete codes below.
	ST7789_Fill_Color(spi,WHITE);
	// ST7789_DrawImage(0, 0, 128, 128, (uint16_t *)saber);
	sleep_ms(3000);
}




// typedef struct {
//    void (*init)(Hw_SPI *spi);
//    void (*reset)(Hw_SPI *spi);

//    void (*FillColor)(Hw_SPI *spi,uint16_t color);

//    void (*SetRotation)(Hw_SPI *spi,uint8_t m);
//    void (*InvertColors)(Hw_SPI *spi,uint16_t color);
//    void (*DrawPixel)(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color);
//    void (*Fill)(Hw_SPI *spi,uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);
//    void (*DrawLine)(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
//    void (*DrawRectangle)(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
//    void (*DrawCircle)(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
//    void (*DrawImage)(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
//    void (*DrawChar)(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
//    void (*DrawString)(Hw_SPI *spi,uint16_t x, uint16_t y, const char *str,FontDef font,uint16_t color, uint16_t bgcolor);
//    void (*test)();

//    void (*init_spi)(spi_inst_t * inst,int clk, int mosi,int miso,int cs, int dc);
// }Lcd_Oper;

 Lcd_Oper  lcd_st7789 = {
    .init          = ST7789_Init,
    .reset         = lcd_reset,
    .invert        = ST7789_InvertColors,
    .SetRotation   = ST7789_SetRotation,
    .Fill          = ST7789_Fill,
    .DrawPixel     = ST7789_DrawPixel,
    .DrawLine      = ST7789_DrawLine,
    .DrawRectangle = ST7789_DrawRectangle,
    .DrawCircle    = ST7789_DrawCircle,
    .DrawImage     = ST7789_DrawImage,
    .DrawChar      = ST7789_WriteChar,
    .DrawString    = ST7789_WriteString,
    .FillColor     = ST7789_Fill_Color,
 };


