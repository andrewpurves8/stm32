/*
 * am2320.c
 *
 *  Created on: Feb 16, 2024
 *      Author: andrew.purves
 */

#include "am2320.h"
#include <string.h>
#include "font.c"

// datasheet specifies that 0xB8 should be sent for write, but address is expected to be the 7 MSBs
#define AM2320_SLAVE_ADDR		  				0x5C
#define AM2320_MY_ADDR                         0x61
#define AM2320_READ_COMMAND						0x03
#define AM2320_TEMPERATURE_ADDR					0x02
#define AM2320_TEMPERATURE_LEN					2
#define AM2320_HUMIDITY_ADDR					0x00
#define AM2320_HUMIDITY_LEN						2

// request to sensor contains function code + starting address + number of registers
#define AM2320_DATA_LEN_SEND					3
// sensor sends function code + number of bytes + requested data (TEMPERATURE_LEN + HUMIDITY_LEN) + CRC code
#define AM2320_DATA_LEN_RECEIVE_TEMP_AND_HUM	(3 + AM2320_TEMPERATURE_LEN + AM2320_HUMIDITY_LEN)
#define AM2320_DATA_LEN_RECEIVE_TEMP			(3 + AM2320_TEMPERATURE_LEN)
#define AM2320_DATA_LEN_RECEIVE_HUM			    (3 + AM2320_HUMIDITY_LEN)

static void I2C1_GPIOInits(void);
static void I2C1_Inits(void);

static I2C_Handle_t i2c1Handle;

// rcv buffer
static uint8_t rcvBuf[AM2320_DATA_LEN_RECEIVE_TEMP_AND_HUM];
static uint8_t sendData[AM2320_DATA_LEN_SEND] = {AM2320_READ_COMMAND, AM2320_HUMIDITY_ADDR, AM2320_TEMPERATURE_LEN + AM2320_HUMIDITY_LEN};

static uint8_t dataToRead = AM2320_TEMPERATURE_AND_HUMIDITY;
static uint8_t dataLenReceive = AM2320_DATA_LEN_RECEIVE_TEMP_AND_HUM;
static float temperature = 0.f;
static float humidity = 0.f;

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
	i2c1Handle.config.ownAddress = AM2320_MY_ADDR;
	i2c1Handle.config.sclSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&i2c1Handle);
}

void AM2320_Init(uint8_t requestedData)
{
	I2C1_GPIOInits();

	// i2c peripheral configuration
	I2C1_Inits();

	// i2c IRQ configurations
	I2C_IrqInterruptConfig(IRQ_NO_I2C1, ENABLE);

	// enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);
    
    dataToRead = requestedData;
    switch (dataToRead)
    {
        case AM2320_TEMPERATURE_AND_HUMIDITY:
            dataLenReceive = AM2320_DATA_LEN_RECEIVE_TEMP_AND_HUM;
            sendData[1] = AM2320_HUMIDITY_ADDR;
            sendData[2] = AM2320_TEMPERATURE_LEN + AM2320_HUMIDITY_LEN;
            break;
        case AM2320_TEMPERATURE:
            dataLenReceive = AM2320_DATA_LEN_RECEIVE_TEMP;
            sendData[1] = AM2320_TEMPERATURE_ADDR;
            sendData[2] = AM2320_TEMPERATURE_LEN;
            break;
        case AM2320_HUMIDITY:
            dataLenReceive = AM2320_DATA_LEN_RECEIVE_HUM;
            sendData[1] = AM2320_HUMIDITY_ADDR;
            sendData[2] = AM2320_HUMIDITY_LEN;
            break;
        default:
            break;
    }
}

void AM2320_RequestData()
{
    static const uint8_t dummyByte = 0;
    // wake sensor
    while (I2C_MasterSendDataIT(&i2c1Handle, &dummyByte, 1, AM2320_SLAVE_ADDR) != I2C_READY);
    delay(10);
    while (I2C_MasterSendDataIT(&i2c1Handle, &sendData, AM2320_DATA_LEN_SEND, AM2320_SLAVE_ADDR) != I2C_READY);
    delay(10);
    while (I2C_MasterReceiveDataIT(&i2c1Handle, &rcvBuf, dataLenReceive, AM2320_SLAVE_ADDR) != I2C_READY);
    delay(10);
}

float AM2320_GetTemperature()
{
    return temperature;
}

float AM2320_GetHumidity()
{
    return humidity;
}

void AM2320_IrqHandling()
{
	I2C_IrqHandling(&i2c1Handle);
}

uint8_t AM2320_HandleInterruptEvent(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{
	static uint8_t nackReceived = RESET;
	static uint8_t humidityHighByte = 0;
	static uint8_t humidityLowByte = 0;
	static uint8_t temperatureHighByte = 0;
	static uint8_t temperatureLowByte = 0;
    if (pI2CHandle->slaveAddress == AM2320_SLAVE_ADDR)
    {
        if (appEv == I2C_EV_RX_CMPLT)
        {
            switch (dataToRead)
            {
                case AM2320_TEMPERATURE_AND_HUMIDITY:
                    humidityHighByte = rcvBuf[2];
                    humidityLowByte = rcvBuf[3];
                    humidity = (humidityHighByte << 8 | humidityLowByte) / 10.f;
                    temperatureHighByte = rcvBuf[4];
                    temperatureLowByte = rcvBuf[5];
                    temperature = (temperatureHighByte << 8 | temperatureLowByte) / 10.f;
                    break;
                case AM2320_TEMPERATURE:
                    temperatureHighByte = rcvBuf[2];
                    temperatureLowByte = rcvBuf[3];
                    temperature = (temperatureHighByte << 8 | temperatureLowByte) / 10.f;
                    break;
                case AM2320_HUMIDITY:
                    humidityHighByte = rcvBuf[2];
                    humidityLowByte = rcvBuf[3];
                    humidity = (humidityHighByte << 8 | humidityLowByte) / 10.f;
                    break;
                default:
                    break;
            }

            return AM2320_DATA_READY;
        }
        else if (appEv == I2C_EV_STOP)
        {
            // the stop after the NACK caused by sensor wakeup must be ignored
            if (nackReceived)
            {
                nackReceived = RESET;
            }
            else
            {
                I2C_CloseSendData(pI2CHandle);
            }
        }
        else if (appEv == I2C_ERROR_NACK)
        {
            nackReceived = SET;
            I2C_CloseSendData(pI2CHandle);
            I2C_GenerateStopCondition(I2C1);
        }
    }
    return AM2320_DATA_NOT_READY;
}
