//
// Created by Bright on 2023/8/21.
//

#ifndef PICOXTOOLS_UART_LCD_H
#define PICOXTOOLS_UART_LCD_H
#include <stdint.h>
#include "fonts.h"
#include "ch32x035.h"

#define spi_inst_t SPI_TypeDef


typedef struct Hw_SPI_t{
   spi_inst_t * inst;
   int clk_pin;
   int mosi_pin;
   int miso_pin;
   int cs_pin;
   int dc_pin;
   int rst_pin;
   int freq;

   void (*init_spi)(struct Hw_SPI_t * inst,uint32_t freq,uint8_t cpol,uint8_t cpha,int dma);

   void (* select)(struct Hw_SPI_t *spi ,uint8_t val);
   void (* setDc)(struct Hw_SPI_t *spi ,uint8_t val);
   void (* send_cmd)(struct Hw_SPI_t *spi ,uint8_t cmd);
   void (* send_data)(struct Hw_SPI_t *spi ,uint8_t val);
   void (* send_byte)(struct Hw_SPI_t *spi ,uint8_t val);
   void (* send_word)(struct Hw_SPI_t *spi ,uint16_t val);
   void (* send_buffer)(struct Hw_SPI_t *spi ,uint8_t *buff, size_t buff_size);
   void (* spi_write)(struct Hw_SPI_t *spi ,uint8_t *buff, size_t buff_size);
   int  dma_tx;
   int  useDMA;
//   dma_channel_config   spi_cfg;
}Hw_SPI;

void lcd_send_cmds (Hw_SPI *spi,const uint8_t *addr);
#define TFT_INIT_DELAY 0x80 // Not used unless commandlist invoked


void lcd_select(Hw_SPI *spi,uint8_t val);
void lcd_set_dc(Hw_SPI *spi,uint8_t val);
void lcd_send_cmd(Hw_SPI *spi,uint8_t val);
void lcd_send_data(Hw_SPI *spi,uint8_t val);
void lcd_send_word(Hw_SPI *spi,uint16_t val);
void lcd_send_buffer(Hw_SPI *spi,uint8_t* data,size_t len);
void lcd_reset(Hw_SPI *spi);

#define lcd_send_byte lcd_send_data

typedef struct {
   void (*init)(Hw_SPI *spi);
   void (*reset)(Hw_SPI *spi);

   void (*FillColor)(Hw_SPI *spi,uint16_t color);

   void (*SetRotation)(Hw_SPI *spi,uint8_t m);
   void (*invert)(Hw_SPI *spi,uint8_t t_f);
   void (*SetAddress)(Hw_SPI *spi,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
   void (*DrawPixel)(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t color);
   void (*Fill)(Hw_SPI *spi,uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);
   void (*DrawLine)(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
   void (*DrawRectangle)(Hw_SPI *spi,uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
   void (*DrawCircle)(Hw_SPI *spi,uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
   void (*DrawImage)(Hw_SPI *spi,uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
   void (*DrawChar)(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
   void (*DrawString)(Hw_SPI *spi,uint16_t x, uint16_t y, const char *str,FontDef font,uint16_t color, uint16_t bgcolor);
   void (*SendData)(Hw_SPI *spi,uint8_t* data,size_t len);
   void (*test)(Hw_SPI *spi);
   void (*clear)(Hw_SPI *spi,uint16_t color);
   void (*init_spi)(spi_inst_t * inst,int clk, int mosi,int miso,int cs, int dc,int dma);
}Lcd_Oper;

extern Hw_SPI g_lcd_spi;


#endif // PICOXTOOLS_UART_LCD_H
