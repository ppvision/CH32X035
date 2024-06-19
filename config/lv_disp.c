/**
 * @file lv_port_disp_templ.c
 *
 */
#define  sleep_ms Delay_Ms

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1


#define LCD_WR_DATA8(x) VSPI_data_queue_data_8bit(x)
#define LCD_WR_REG(x)   VSPI_data_queue_REG_8bit(x)

// /******************************************************************************
//       函数说明：设置起始和结束地址
//       入口数据：x1,x2 设置列的起始和结束地址
//                 y1,y2 设置行的起始和结束地址
//       返回值：  无
// ******************************************************************************/
// void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
// {
//     VSPI_data_queue_REG_8bit(0x2a); //列地址设置
//     VSPI_data_queue_data_32bit(x1 >> 8, x1, x2 >> 8, x2);
//     VSPI_data_queue_REG_8bit(0x2b); //行地址设置
//     VSPI_data_queue_data_32bit(y1 >> 8, y1, y2 >> 8, y2);
//     VSPI_data_queue_REG_8bit(0x2c); //储存器写


//     LCD_WR_REG(0x2a);//列地址设置
//         LCD_WR_DATA(x1+26);
//         LCD_WR_DATA(x2+26);
        
//         LCD_WR_REG(0x2b);//行地址设置
//         LCD_WR_DATA(y1+1);
//         LCD_WR_DATA(y2+1);
//         LCD_WR_REG(0x2c);//储存器写
// }

/*16bit rgb 565*/
//IRAM_ATTR void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
//{
//    uint32_t siofs = 240 * 2 * 10; // 同时刷多少行240 *2（8bit）*10行
//    uint32_t siof = 240 * 1 * 10;  // 因为数组是16位的，但字节指向的地址
//    uint32_t size = (x2 + 1 - x1) * (y2 + 1 - y1) * 2;
//    LCD_Address_Set(x1, y1, x2, y2);
//    uint32_t send_cnt = size / siofs;  // 需要整行刷新多少行
//    uint32_t send_cnt2 = size % siofs; // 整行刷不完还剩多少字节
//    if (size == 0)
//    {
//        ESP_LOGE(TAG, "size为0,跳出了");
//        return;
//    }
//    for (int i = 0; i < send_cnt; i++)
//    {
//        if ((i + 1) == send_cnt && send_cnt2 == 0)
//        {
//            VSPI_data_queue(&color[i * siof], siof * 16, SPI_WRITE_DATA_OVER);
//        }
//        else
//        {
//            VSPI_data_queue(&color[i * siof], siof * 16, SPI_WRITE_DATA);
//        }
//    }
//    if (send_cnt2 != 0)
//    {
//        VSPI_data_queue(&color[send_cnt * siof], send_cnt2 * 8, SPI_WRITE_DATA_OVER);
//    }
//}


/*********************
 *      INCLUDES
 *********************/
#include "lv_disp.h"
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/

#if  1
#define MY_DISP_HOR_RES    160
#define MY_DISP_VER_RES    80
#else
#define MY_DISP_HOR_RES    240
#define MY_DISP_VER_RES    135
#endif

#ifndef MY_DISP_HOR_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen width, default value 320 is used for now.
    #define MY_DISP_HOR_RES    320
#endif

#ifndef MY_DISP_VER_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen height, default value 240 is used for now.
    #define MY_DISP_VER_RES    240
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */
#define BUFFER_TYPE  1

#if BUFFER_TYPE == 1
    /* Example for 1) */
    #define BUF_LINES 20
    static lv_disp_draw_buf_t draw_buf_dsc_1;
    static lv_color_t buf_1[MY_DISP_HOR_RES * BUF_LINES];                          /*A buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, MY_DISP_HOR_RES * BUF_LINES);   /*Initialize the display buffer*/
#elif BUFFER_TYPE == 2
    /* Example for 2) */
    static lv_disp_draw_buf_t draw_buf_dsc_2;
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * 10];                        /*An other buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/
#elif BUFFER_TYPE == 3
    /* Example for 3) also set disp_drv.full_refresh = 1 below*/
    static lv_disp_draw_buf_t draw_buf_dsc_3;
    static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*A screen sized buffer*/
    static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*Another screen sized buffer*/
    lv_disp_draw_buf_init(&draw_buf_dsc_3, buf_3_1, buf_3_2,
                          MY_DISP_VER_RES * LV_VER_RES_MAX);   /*Initialize the display buffer*/
#endif
    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_1;

    /*Required for Example 3)*/
    //disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
#include "lcd_api.h"
#include "st7735.h"
#include "st7789.h"
/*Initialize your display and the required peripherals.*/

 //配置DMA


static void disp_init(void)
{

    g_lcd_spi.init_spi(&g_lcd_spi,g_lcd_spi.freq,0,0,0);
    lcd_st7735.init(&g_lcd_spi);
    lcd_st7735.clear(&g_lcd_spi,0xFFFF);
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}


static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
         int Y = area->y2 - area->y1;
         int X = area->x2 - area->x1;
         Y += 1;
         // X += 1;
         lcd_st7735.DrawImage(&g_lcd_spi,area->x1,area->y1,X,Y, (uint16_t *)&(color_p->full));
    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}
#include <stdio.h>
#if 0
 lv_init();
    lv_port_disp_init();

    // disp_disable_update();

    lv_demo_benchmark_set_finished_cb(&on_benchmark_finished);
    lv_demo_benchmark_set_max_speed(true);
    lv_demo_benchmark();



    repeating_timer_t timer;
    // negative timeout means exact delay (rather than delay between callbacks)
    if (!add_repeating_timer_us(1000*10, timer_callback, NULL, &timer)) {
        printf("Failed to add timer\n");
        return 1;
    }

    while(1) {
      lv_timer_handler();
      sleep_ms(1);
    }

    static lv_obj_t *default_src;
    default_src = lv_scr_act();
//获取默认屏幕
    lv_obj_t * label = lv_label_create(default_src);
    lv_label_set_text(label, "hello_1vg18");
    lv_obj_center(label);
/*Add a label to the button*/
/*Set the labels text*/


    while(1) {
      lv_timer_handler();
      sleep_ms(15);
    }
#endif


/* Global Variable */
void SysTick_Handler(void) __attribute__((interrupt()));
u32 counter=0;

void SYSTICK_Init_Config(u64 ticks)
{
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = ticks;
    SysTick->CTLR =0xF;

    NVIC_SetPriority(SysTicK_IRQn, 15);
    NVIC_EnableIRQ(SysTicK_IRQn);
}

static int is_fished = 0;
void SysTick_Handler(void)
{
    printf("welcome to WCH\r\n");
    SysTick->SR = 0;
    counter++;
    printf("Counter:%d\r\n",counter);
    if(!is_fished)lv_tick_inc(10);
}


void stop_lvgl_test(){
    is_fished = true;
}


typedef void finished_cb_t(void);
void lv_demo_benchmark(void);
void lv_demo_benchmark_close(void);
void lv_demo_benchmark_set_finished_cb(finished_cb_t * finished_cb);

/**
 * Make the benchmark work at the highest frame rate
 * @param en true: highest frame rate; false: default frame rate
 */
void lv_demo_benchmark_set_max_speed(bool en);





static void on_benchmark_finished(void)
{
    disp_enable_update();
    stop_lvgl_test();
}
void lvgl_test(){
    is_fished = false;
    // lv_init();
    lv_port_disp_init();

    lv_demo_benchmark_set_finished_cb(&on_benchmark_finished);
    lv_demo_benchmark_set_max_speed(true);
    lv_demo_benchmark();

    // negative timeout means exact delay (rather than delay between callbacks)
    /*if (!add_repeating_timer_us(1000*10, timer_callback, NULL, &timer)) {
        return ;
    }*/
    while(!is_fished) {
      lv_timer_handler();
      sleep_ms(1);
    }
    stop_lvgl_test();
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
