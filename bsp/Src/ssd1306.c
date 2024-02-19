/*
 * ssd1306.c
 *
 *  Created on: Feb 9, 2024
 *      Author: andrew.purves
 */

#include "ssd1306.h"
#include "stm32l07xx.h"
#include <string.h>
#include "font.c"

#define SSD1306_MY_ADDR                         0x61
#define SSD1306_SLAVE_ADDR                      0x3C

#define SSD1306_SCREEN_WIDTH                    128
#define SSD1306_SCREEN_HEIGHT                   32
#define SSD1306_BUFFER_SIZE                     (SSD1306_SCREEN_WIDTH * ((SSD1306_SCREEN_HEIGHT + 7) / 8))
#define SSD1306_FRAME_SIZE                      32

#define SSD1306_CHARGE_PUMP                     0x8D
#define SSD1306_COLUMN_ADDR                     0x21
#define SSD1306_COM_SCAN_DEC                    0xC8
#define SSD1306_COMMAND_CONTROL_BYTE            0x00
#define SSD1306_DATA_CONTROL_BYTE               0x40
#define SSD1306_DEACTIVATE_SCROLL               0x2E
#define SSD1306_DISPLAY_ALL_ON_RESUME           0xA4
#define SSD1306_DISPLAY_OFF                     0xAE
#define SSD1306_DISPLAY_ON                      0xAF
#define SSD1306_MEMORY_MODE                     0x20
#define SSD1306_PAGE_ADDR                       0x22
#define SSD1306_SET_COM_PINS                    0xDA
#define SSD1306_SET_CONTRAST                    0x81
#define SSD1306_SET_DISPLAY_CLOCK_DIV           0xD5
#define SSD1306_SET_DISPLAY_OFFSET              0xD3
#define SSD1306_SET_MULTIPLEX                   0xA8
#define SSD1306_SET_PRECHARGE                   0xD9
#define SSD1306_SEG_REMAP                       0xA0
#define SSD1306_SET_START_LINE                  0x40
#define SSD1306_SET_VCOM_DETECT                 0xDB

static void I2C1_GPIOInits(void);
static void I2C1_Inits(void);
static void SSD1306_SendCommandList(uint8_t* commandList, uint8_t numBytes);
static void SSD1306_PrintChar(char c, uint8_t x, uint8_t y, uint8_t fontSize);
static void SSD1306_DrawRect(uint8_t x, uint8_t width, uint8_t y, uint8_t height);

static I2C_Handle_t i2c1Handle;
static uint8_t* buffer;

/*
//  * PC0-> SCL
//  * PC1 -> SDA
 * PB8-> SCL
 * PB9 -> SDA
 */
static void I2C1_GPIOInits(void)
{
	GPIO_Handle_t i2cPins;

	// i2cPins.pGPIOx = GPIOC;
	i2cPins.pGPIOx = GPIOB;
	i2cPins.pinConfig.pinMode = GPIO_MODE_ALTFN;
	i2cPins.pinConfig.pinOPType = GPIO_OP_TYPE_OD;
	i2cPins.pinConfig.pinPuPdControl = GPIO_PIN_PU;
	// i2cPins.pinConfig.pinAltFunMode = 7;
	i2cPins.pinConfig.pinAltFunMode = 4;
	i2cPins.pinConfig.pinSpeed = GPIO_SPEED_HIGH;

	// sda
	// i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_1;
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&i2cPins);

	// scl
	// i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_0;
	i2cPins.pinConfig.pinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&i2cPins);
}

static void I2C1_Inits(void)
{
	i2c1Handle.pI2Cx = I2C1;
	i2c1Handle.config.ownAddress = SSD1306_MY_ADDR;
	i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_SM;
	// i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_FM;

	I2C_Init(&i2c1Handle);
}

static void SSD1306_SendCommandList(uint8_t* commandList, uint8_t numBytes)
{
    I2C_MasterSendData(&i2c1Handle, commandList, numBytes, SSD1306_SLAVE_ADDR);
}

static void SSD1306_PrintChar(char c, uint8_t x, uint8_t y, uint8_t fontSize)
{
    for (int8_t i = 0; i < FONT_WIDTH; i++) {
        uint8_t line = font[c * FONT_WIDTH + i];
        for (int8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1)
            {
                if (fontSize == 1)
                {
                    SSD1306_DrawPixel(x + j, y + i, SSD1306_WHITE);
                }
                else
                {
                    SSD1306_DrawRect(x + j * fontSize, fontSize, y + i * fontSize, fontSize);
                }
            }
        }
    }
}

static void SSD1306_DrawRect(uint8_t x, uint8_t width, uint8_t y, uint8_t height)
{
    for (uint8_t i = x; i < x + width; i++)
    {
        for (uint8_t j = y; j < y + height; j++)
        {
            SSD1306_DrawPixel(i, j, SSD1306_WHITE);
        }
    }
}

void SSD1306_Init(uint8_t displayMode)
{
    I2C1_GPIOInits();
    I2C1_Inits();
	I2C_PeripheralControl(I2C1, ENABLE);
    
    // send a dummy byte because I2C1 messes up the first address phase it sends
    static const uint8_t dummyByte = 0;
    I2C_MasterSendData(&i2c1Handle, &dummyByte, 1, SSD1306_SLAVE_ADDR);
    delay(10);
    
    const uint8_t init[] = {
        SSD1306_COMMAND_CONTROL_BYTE,
        SSD1306_DISPLAY_OFF,
        SSD1306_SET_DISPLAY_CLOCK_DIV,
        0x80,                                   // the suggested ratio 0x80
        SSD1306_SET_MULTIPLEX,
        SSD1306_SCREEN_HEIGHT - 1,
        SSD1306_SET_DISPLAY_OFFSET,
        0x0,                                    // no offset
        SSD1306_SET_START_LINE | 0x0,           // line #0
        SSD1306_CHARGE_PUMP,
        0x14,                                   // internal display voltage source
        SSD1306_MEMORY_MODE,
        0x00,                                   // 0x0 act like ks0108
        SSD1306_SEG_REMAP | 0x1,
        SSD1306_COM_SCAN_DEC,
        SSD1306_SET_COM_PINS,
        0x02,
        SSD1306_SET_CONTRAST,
        0x8F,
        SSD1306_SET_PRECHARGE,
        0xF1,
        SSD1306_SET_VCOM_DETECT,
        0x40,
        SSD1306_DISPLAY_ALL_ON_RESUME,
        displayMode,
        SSD1306_DEACTIVATE_SCROLL,
        SSD1306_DISPLAY_ON
    };
    SSD1306_SendCommandList(&init, sizeof(init));

    buffer = (uint8_t*) malloc(SSD1306_BUFFER_SIZE);
    SSD1306_ClearDisplay();
}

void SSD1306_ClearDisplay(void)
{
    memset(buffer, 0, SSD1306_BUFFER_SIZE);
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if ((y >= 0) && (y < SSD1306_SCREEN_WIDTH) && (x >= 0) && (x < SSD1306_SCREEN_HEIGHT))
    {
        switch (color)
        {
        case SSD1306_WHITE:
            REG_SET_BIT(buffer[y + (x / 8) * SSD1306_SCREEN_WIDTH], x & 7);
            break;
        case SSD1306_BLACK:
            REG_CLEAR_BIT(buffer[y + (x / 8) * SSD1306_SCREEN_WIDTH], x & 7);
            break;
        case SSD1306_INVERSE:
            REG_FLIP_BIT(buffer[y + (x / 8) * SSD1306_SCREEN_WIDTH], x & 7);
            break;
        }
    }
}

void SSD1306_Display()
{
    static const uint8_t dlist1[] = {
        SSD1306_COMMAND_CONTROL_BYTE,
        SSD1306_PAGE_ADDR,
        0,                                      // Page start address
        0xFF,                                   // Page end (not really, but works here)
        SSD1306_COLUMN_ADDR,
        0,
        SSD1306_SCREEN_WIDTH - 1
    };                                          // Column start address
    SSD1306_SendCommandList(dlist1, sizeof(dlist1));

    uint16_t count = SSD1306_BUFFER_SIZE;
    uint8_t *ptr = buffer;

    uint8_t controlByte = SSD1306_DATA_CONTROL_BYTE;
    uint8_t bufferPortion[SSD1306_FRAME_SIZE + 1];
    bufferPortion[0] = controlByte;
    while (count > SSD1306_FRAME_SIZE)
    {
        for (uint8_t i = 0; i < SSD1306_FRAME_SIZE; i++)
        {
            bufferPortion[i + 1] = *(ptr + i);
        }
        I2C_MasterSendData(&i2c1Handle, &bufferPortion, SSD1306_FRAME_SIZE + 1, SSD1306_SLAVE_ADDR);
        count -= SSD1306_FRAME_SIZE;
        ptr += SSD1306_FRAME_SIZE;
    }
    
    for (uint8_t i = 0; i < count; i++)
    {
        bufferPortion[i + 1] = *(ptr + i);
    }
    I2C_MasterSendData(&i2c1Handle, &bufferPortion, count, SSD1306_SLAVE_ADDR);
}

void SSD1306_ReadBuffer(uint8_t* buf)
{
    uint8_t controlByte = SSD1306_DATA_CONTROL_BYTE;
    uint8_t dummyRead;
    I2C_MasterSendData(&i2c1Handle, &controlByte, 1, SSD1306_SLAVE_ADDR);
    I2C_MasterReceiveData(&i2c1Handle, &dummyRead, 1, SSD1306_SLAVE_ADDR);
    
    uint16_t count = SSD1306_BUFFER_SIZE - 1;
    uint8_t *ptr = buf;
    while (count > SSD1306_FRAME_SIZE)
    {
        I2C_MasterSendData(&i2c1Handle, &controlByte, 1, SSD1306_SLAVE_ADDR);
        I2C_MasterReceiveData(&i2c1Handle, ptr, SSD1306_FRAME_SIZE, SSD1306_SLAVE_ADDR);
        count -= SSD1306_FRAME_SIZE;
        ptr += SSD1306_FRAME_SIZE;
    }
    I2C_MasterReceiveData(&i2c1Handle, ptr, count, SSD1306_SLAVE_ADDR);
}

void SSD1306_Print(const char* text, uint32_t length, uint8_t fontSize)
{
    SSD1306_ClearDisplay();
    const uint8_t charsPerLine = SSD1306_SCREEN_WIDTH / (FONT_WIDTH * fontSize + 1);

    uint8_t lineNumber = 0;
    uint8_t columnNumber = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        if (i == charsPerLine || text[i] == '\n')
        {
            columnNumber = 0;
            lineNumber++;
        }
        if (text[i] == '\n')
        {
            continue;
        }
        SSD1306_PrintChar(
            text[i],
            lineNumber * (FONT_HEIGHT * fontSize + 1),
            columnNumber * (FONT_WIDTH * fontSize + 1),
            fontSize
        );
        columnNumber++;
    }
}
