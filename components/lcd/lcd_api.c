//
// Created by Bright on 2023/8/21.
//

#include "lcd_api.h"
#define LCD_GPIO_PORT GPIOB

#define LCD_RESET_PORT GPIOB
#define LCD_RESET_PIN  GPIO_Pin_0

#define LCD_SWT_PORT GPIOB
#define LCD_SWT_PIN  GPIO_Pin_1

#define LCD_SELECT_PIN  GPIO_Pin_4

void lcd_select(Hw_SPI *spi,uint8_t val){
    if(!spi) return;
    if(spi->cs_pin ) {
        GPIO_WriteBit(LCD_GPIO_PORT, LCD_SELECT_PIN, val);
    }
}
void LCD_SPI_Init(){
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(LCD_GPIO_PORT, LCD_SELECT_PIN, 1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 |GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}



void lcd_set_dc(Hw_SPI *spi,uint8_t val){
   if(!spi) return;
   if(spi->dc_pin ) GPIO_WriteBit(LCD_GPIO_PORT, LCD_SWT_PIN, val);
}

void lcd_send_cmd(Hw_SPI *spi,uint8_t val){
   if(!spi) return;
   lcd_set_dc(spi,0);
   if(spi->spi_write)spi->spi_write(spi, &val, 1);
}

void lcd_send_data(Hw_SPI *spi,uint8_t val){
   if(!spi) return;
   lcd_set_dc(spi,1);
   if(spi->spi_write)spi->spi_write(spi, &val, 1);
}

void lcd_send_word(Hw_SPI *spi,uint16_t val){
   if(!spi) return;
   lcd_set_dc(spi,1);
#if 1
   uint8_t H = val >> 8;
   uint8_t L = val & 0xff;
   if(spi->spi_write){
       spi->spi_write(spi, &H, 1);
       spi->spi_write(spi, &L, 1);
   }
#else
   uint16_t word = (val >> 8)|((val & 0xff) <<8);
   if(spi->spi_write) spi->spi_write(spi, (uint8_t *)&word, 2);
#endif
}

void lcd_send_buffer(Hw_SPI *spi,uint8_t* data,size_t len){
   if(!spi) return;
   lcd_set_dc(spi,1);
   if(spi->spi_write)spi->spi_write(spi, data, len);
}


static u8 SPI1_ReadWriteByte(spi_inst_t *pSpi,u8 TxData)
{
    u8 i = 0;
    while(SPI_I2S_GetFlagStatus(pSpi, SPI_I2S_FLAG_TXE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }
    SPI_I2S_SendData(pSpi, TxData);
    i = 0;
    while(SPI_I2S_GetFlagStatus(pSpi, SPI_I2S_FLAG_RXNE) == RESET)
    {
        i++;
        if(i > 200)
            return 0;
    }
    return SPI_I2S_ReceiveData(pSpi);
}

void spi_write(Hw_SPI *spi,uint8_t* data,size_t len){
    GPIO_WriteBit(LCD_GPIO_PORT, LCD_SELECT_PIN, 0);
    for(size_t i =0;i<len;i++)
        SPI1_ReadWriteByte(spi->inst, data[i]);
    GPIO_WriteBit(LCD_GPIO_PORT, LCD_SELECT_PIN, 1);
}

void lcd_spi_config(Hw_SPI *spi,uint32_t freq,uint8_t cpol,uint8_t cpha,int useDMA)
{
    LCD_SPI_Init();
    SPI_InitTypeDef  SPI_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;

    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);

}


void lcd_reset(Hw_SPI *spi){
//   if(spi->rst_pin)
   {
       GPIO_WriteBit(LCD_GPIO_PORT, LCD_RESET_PIN, 0);
       Delay_Ms(200);
       GPIO_WriteBit(LCD_GPIO_PORT, LCD_RESET_PIN, 1);
       Delay_Ms(20);
   }
}
spi_inst_t * inst;
int clk_pin;
int mosi_pin;
int miso_pin;
int cs_pin;
int dc_pin;
int rst_pin;
int freq;

Hw_SPI g_lcd_spi ={
   SPI1,
   5,//PA
   7,//PA
   6,//PA
   4,//PB4
   1,//PB1
   0,//reset
   (48 * 1000*1000),
   lcd_spi_config,
   lcd_select,
   lcd_set_dc,
   lcd_send_cmd,
   lcd_send_data,
   lcd_send_data,
   lcd_send_word,
   lcd_send_buffer,
   spi_write,
};



//#define SendCmd(cmd,buffer) lcd_send_cmd(spi,buffer[0]); lcd_send_buffer(spi,buffer+1,sizeof(buffer)-1)
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))
void lcd_send_cmds (Hw_SPI *spi,const uint8_t *addr)
{
  uint8_t  cmd;
  uint8_t  numArgs;
  uint8_t  ms;
  cmd = pgm_read_byte(addr++);   // Number of commands to follow
  while (cmd)                  // For each command...
  {
    lcd_send_cmd(spi,cmd); // Read, issue command
    numArgs = pgm_read_byte(addr++);     // Number of args to follow
    ms = numArgs & TFT_INIT_DELAY;       // If hibit set, delay follows args
    numArgs &= ~TFT_INIT_DELAY;          // Mask out delay bit

    while (numArgs--)                    // For each argument...
    {
      lcd_send_byte(spi,pgm_read_byte(addr++));  // Read, issue argument
    }
    if (ms)
    {
      ms = pgm_read_byte(addr++);        // Read post-command delay time (ms)
        Delay_Ms( (ms==255 ? 500 : ms) );
    }
    cmd = pgm_read_byte(addr++);
  }
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
//    void (*DrawImage)(Hw_SPI *spi,uint16_t x0,  uint16_t w, uint16_t h, const uint16_t *data);
//    void (*DrawChar)(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
//    void (*DrawString)(Hw_SPI *spi,uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
//    void (*test)();

//    void (*init_spi)(spi_inst_t * inst,int clk, int mosi,int miso,int cs, int dc);
// }Lcd_Oper;



