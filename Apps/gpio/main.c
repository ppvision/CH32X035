/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *GPIO routine:
 *PA0 push-pull output.
 *
 ***Only PA0--PA15 and PC16--PC17 support input pull-down.
 */

#include "debug.h"
#include "lcd_api.h"
#include "rgb_color.h"
/* Global define */

/* Global Variable */

/* Enable global interrupt and configure privileged mode */

void EnableGlobalInt(uint32_t uvalue) {
//    uint32_t uvalue = 88;
    __asm volatile("csrw  mstatus, %0"::"r"(uvalue));
}
/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
//"WCH-Interrupt-fast"

void EXTI7_0_IRQHandler(void) __attribute__((interrupt()));
void EXTI15_8_IRQHandler(void) __attribute__((interrupt()));

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 Handler.
 *
 * @return  none
 */

extern int flash_test(void);
extern Lcd_Oper  lcd_st7735;

extern Hw_SPI g_lcd_spi;
extern FontDef Font_7x10;

uint16_t bg_color[] ={
0x0000,//#define BLACK
0x001F,//#define BLUE
0XF81F,//#define BRED
0XFFE0,//#define GRED
0X07FF,//#define GBLUE
0xF800,//#define RED
0xF81F,//#define MAGENTA
0x07E0,//#define GREEN
0x7FFF,//#define CYAN
0xFFE0,//#define YELLOW
};
int nCount = sizeof (bg_color)/sizeof (bg_color[0]);
int nColorIdex = 0;
void EXTI7_0_IRQHandler(void)
{
    lcd_st7735.clear(&g_lcd_spi,bg_color[nColorIdex++%nCount]);
    if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
    {
//        printf("Run at EXTI\r\n");
        EXTI_ClearITPendingBit(EXTI_Line0);     /* Clear Flag */
    }
    if(EXTI_GetITStatus(EXTI_Line6)!=RESET)
    {
        printf("Run at EXTI6\r\n");
        EXTI_ClearITPendingBit(EXTI_Line6);     /* Clear Flag */
    }
}

void EXTI15_8_IRQHandler(void) {
    lcd_st7735.clear(&g_lcd_spi,bg_color[++nColorIdex%nCount]);
    if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
    {
        printf("Run at EXTI_8\r\n");
        EXTI_ClearITPendingBit(EXTI_Line8);     /* Clear Flag */
    }
}

void EXTI0_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


#if 1
    /* GPIOB GPIO_BTN_DWN---->  */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#endif

#if 1
    /* GPIOB GPIO_BTN_UP ----> EXTI_Line8 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_8_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}


void GPIO_Toggle_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //SPI_CS2
//    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */


extern void LCD_SPI_Init();
extern void lcd_spi_config(Hw_SPI *spi,uint32_t freq,uint8_t cpol,uint8_t cpha,int useDMA);

int main(void)
{
    u8 i = 0;
    BitAction val=Bit_SET;
    EnableGlobalInt(0x88);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("GPIO Toggle TEST\r\n");
    GPIO_Toggle_INIT();
    EXTI0_INT_INIT();
//    GPIO_WriteBit(GPIOB, GPIO_Pin_3, 1);
    int loop = 0;
    g_lcd_spi.inst = SPI1;

    g_lcd_spi.init_spi(&g_lcd_spi,g_lcd_spi.freq,0,0,0);
    lcd_st7735.init(&g_lcd_spi);
    lcd_st7735.clear(&g_lcd_spi,0xFFFF);
//  lcd_st7735.test(&g_lcd_spi);
//    flash_test();
    while(1)
    {
        Delay_Ms(500);
        val = (i++ %2)?Bit_SET:Bit_RESET;
        GPIO_WriteBit(GPIOA, GPIO_Pin_0|GPIO_Pin_1,val);
        GPIO_WriteBit(GPIOC, GPIO_Pin_0|GPIO_Pin_3, !val);
        printf("Loop:%d\r\n",loop++);
    }
}
