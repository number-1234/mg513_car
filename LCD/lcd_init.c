#include "lcd_init.h"
#include "ti_msp_dl_config.h"
#include "sys/sys.h"

void LCD_GPIO_Init(void)
{
    /* SPI 由 SysConfig 初始化，此处只配 GPIO */
    DL_GPIO_initDigitalOutput(LCD_RES_IOMUX);  DL_GPIO_setPins(LCD_PORT, LCD_RES_PIN);
    DL_GPIO_initDigitalOutput(LCD_DC_IOMUX);   DL_GPIO_setPins(LCD_PORT, LCD_DC_PIN);
    DL_GPIO_initDigitalOutput(LCD_CS_IOMUX);   DL_GPIO_setPins(LCD_PORT, LCD_CS_PIN);
    DL_GPIO_initDigitalOutput(LCD_BLK_IOMUX);  DL_GPIO_clearPins(LCD_PORT, LCD_BLK_PIN);
    DL_GPIO_enableOutput(LCD_PORT, LCD_RES_PIN | LCD_DC_PIN | LCD_CS_PIN | LCD_BLK_PIN);
}

void LCD_Writ_Bus(u8 dat) 
{	
    LCD_CS_Clr();
    DL_SPI_transmitData8(SPI_LCD_INST, dat);
    while(DL_SPI_isBusy(SPI_LCD_INST));
    LCD_CS_Set();
}

void LCD_WR_DATA8(u8 dat) { LCD_Writ_Bus(dat); }

void LCD_WR_DATA(u16 dat)
{
    LCD_Writ_Bus(dat>>8);
    LCD_Writ_Bus(dat);
}

void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+0); LCD_WR_DATA(x2+0);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+0); LCD_WR_DATA(y2+0);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+0); LCD_WR_DATA(x2+0);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+80); LCD_WR_DATA(y2+80);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+0); LCD_WR_DATA(x2+0);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+0); LCD_WR_DATA(y2+0);
        LCD_WR_REG(0x2c);
    }
    else
    {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1+80); LCD_WR_DATA(x2+80);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1+0); LCD_WR_DATA(y2+0);
        LCD_WR_REG(0x2c);
    }
}

void LCD_Init(void)
{
    LCD_GPIO_Init();
    LCD_RES_Clr(); delay_ms(20);
    LCD_RES_Set(); delay_ms(20);
    LCD_BLK_Set();

    LCD_WR_REG(0x11); delay_ms(120);
    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
    else LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x05);
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x0C);LCD_WR_DATA8(0x0C);LCD_WR_DATA8(0x00);LCD_WR_DATA8(0x33);LCD_WR_DATA8(0x33);
    LCD_WR_REG(0xB7); LCD_WR_DATA8(0x35);
    LCD_WR_REG(0xBB); LCD_WR_DATA8(0x35);
    LCD_WR_REG(0xC0); LCD_WR_DATA8(0x2C);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xC6); LCD_WR_DATA8(0x0F);
    LCD_WR_REG(0xD0); LCD_WR_DATA8(0xA4);LCD_WR_DATA8(0xA1);
    LCD_WR_REG(0xD6); LCD_WR_DATA8(0xA1);
    LCD_WR_REG(0xE0); LCD_WR_DATA8(0xF0);LCD_WR_DATA8(0x04);LCD_WR_DATA8(0x07);LCD_WR_DATA8(0x09);LCD_WR_DATA8(0x0A);LCD_WR_DATA8(0x06);LCD_WR_DATA8(0x2B);LCD_WR_DATA8(0x2B);LCD_WR_DATA8(0x36);LCD_WR_DATA8(0x15);LCD_WR_DATA8(0x14);LCD_WR_DATA8(0x28);LCD_WR_DATA8(0x30);
    LCD_WR_REG(0xE1); LCD_WR_DATA8(0xF0);LCD_WR_DATA8(0x09);LCD_WR_DATA8(0x0C);LCD_WR_DATA8(0x06);LCD_WR_DATA8(0x15);LCD_WR_DATA8(0x07);LCD_WR_DATA8(0x3C);LCD_WR_DATA8(0x43);LCD_WR_DATA8(0x47);LCD_WR_DATA8(0x07);LCD_WR_DATA8(0x0E);LCD_WR_DATA8(0x17);LCD_WR_DATA8(0x30);
    LCD_WR_REG(0x21);
    LCD_WR_REG(0x29);
}
