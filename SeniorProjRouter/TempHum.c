/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
This code utilizes the I2C protocol by opening the I2C driver and transmitting commands to the third party device (in this case I'm using the Adafruit HTU21D-F breakout board).
This is done by using the correct slave address (0x40), and connecting the pins correctly from the Adafruit device to the CC2650 launchpad.
When the I2C driver is initialized and starts transmission, the TxBuffer contains the command and retrieves the data for temperature
and humidity.
*/

/* XDCtools Header files */
#include <xdc/std.h>
#include <stdio.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>
/* Board Header files */
#include "Board.h"
#include <stdint.h>
#include "TempHum.h"

struct point test;
struct TempHum                                                               //Struct TempHum (buffers that are used throughout)
{
    uint8_t RxBuffer[2];
    uint8_t TxBuffer[3];
    float Temp[1];
    float Humidity[1];

};

#define TASKSTACKSIZE   768
#define BufferSize 100

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* Pin driver handle */
#include <ti/drivers/i2c/I2CCC26XX.h>

/***** Prototypes *****/


const float OffSet = -46.84;
const float MultValue = 175.72;
const float DivValue = 65536;






void I2CBusProj(UArg arg0, UArg arg1)                                        //Start of I2C
{
    test.h = (char*) malloc(1);                                              //Allocate one byte to the heap and return pointer
    test.t = (char*) malloc(1);                                              //Allocate one byte to the heap and return pointer
    struct TempHum p;                                                        //Declare struct in I2CBusProj
//  Display_Params D_params;                                                 //Setup LCD display
 // Display_Params_init(&D_params);
 // D_params.lineClearMode = DISPLAY_CLEAR_BOTH;
 // Display_Handle hDisplayLcd = Display_open(Display_Type_LCD, &D_params);

    I2C_Handle      handle;                                                  //I2C initialization
    I2C_Params      params;
    I2C_Transaction i2cTransaction;

    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;                                             //Frequency set to 400kHz
    handle = I2C_open(Board_I2C, &params);
    if (!handle)
    {
        System_printf("I2C did not open");
    }

    Task_sleep(10000000/ Clock_tickPeriod);                                  //Delay before I2C transfer

    while(1)
    {
        Task_sleep(10000000/ Clock_tickPeriod);

        p.TxBuffer[0] = 0xE3;                                           //Send command Trigger Temperature Measurement to obtain temperature data

        i2cTransaction.slaveAddress = 0x40;                             //Set slave address of HTU21D(F) sensor
        i2cTransaction.writeBuf = p.TxBuffer;                           //Set write buffer as TxBuffer
        i2cTransaction.writeCount = 1;                                  //Send one byte of data (0xE3)
        i2cTransaction.readBuf = p.RxBuffer;                            //Set read buffer as RxBuffer
        i2cTransaction.readCount = 2;                                   //Read in two bytes of data for temperature
        I2C_transfer(handle, &i2cTransaction);                          //Start transfer of bus
        Temperature(&p);                                       //Call Temperature function to convert data to Fahrenheit
      //  Display_print1(hDisplayLcd, 4, 2, "TEMP: %d (F)", p.Temp[0]);   //Display Fahrenheit data on LCD
        *test.t =  p.Temp[0];
        Task_sleep(10000000/ Clock_tickPeriod);

        p.TxBuffer[0] = 0xE5;                                           //Send command Trigger Temperature Measurement to obtain temperature data

        i2cTransaction.slaveAddress = 0x40;                                    //Set slave address of HTU21D(F) sensor
        i2cTransaction.writeBuf = p.TxBuffer;                                  //Set write buffer as TxBuffer
        i2cTransaction.writeCount = 1;                                         //Send one byte of data (0xE3)
        i2cTransaction.readBuf = p.RxBuffer;                                   //Set read buffer as RxBuffer
        i2cTransaction.readCount = 2;                                          //Read in two bytes of data for temperature
        I2C_transfer(handle, &i2cTransaction);                                 //Start transfer of bus
        Humidity(&p);                                                 //Call Temperature function to convert data to Fahrenheit
     //   Display_print2(hDisplayLcd, 5, 2, "Hum:  %d%c", p.Humidity[0], '%');   //Display Fahrenheit data on LCD
        *test.h =  p.Humidity[0];

        Task_sleep(10000000/ Clock_tickPeriod);                                  //Delay before I2C transfer
        free(test.h);                                                            //Free allocated memory and allow OS to use it
        free(test.t);                                                            //This is important to avoid memory leaks!
    }

}

/*
 *  ======== initialization of TempHum ========
 */
void TempHum_init(void)
{
    PIN_Handle ledPinHandle;
    Task_Params taskParams;

    /* Call board init functions */
    Board_initI2C();
    // Board_initSPI();
    // Board_initUART();
    // Board_initWatchdog();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)I2CBusProj, &taskParams, NULL);
}


void Temperature(struct TempHum * b)                     //Converts data to Celsius, then converts Celsius to Fahrenheit
{
    unsigned int tVal;
    unsigned int val2;
    unsigned int val;
    float temp;
                                                         //We are using two bytes of data that we read from I2C, we need to shift the first byte
                                                         //and add the second byte to that value
    val = b->RxBuffer[0] << 8;                           //Shift data by 8 bits (1 byte) e.g., 60 (0110 0000) would be shifted 0110 0000 0000 0000
    val2 = b->RxBuffer[1];                               //No shift required e.g., if value is 50 decimal, then in binary it would be 0000 0000 0101 0000
    tVal = val + val2;                                   //Add both values (first and second byte)


    temp = OffSet + (MultValue * (tVal / DivValue));     //Convert data to celsius value (-46.85 + 175.72*(S_temp/2^16)
    temp = (temp * (1.8) + 32);                          //Convert C to F
    b->Temp[0] = temp;                                   //Store result in memory
}

void Humidity(struct TempHum * b)                     //Converts data to Celsius, then converts Celsius to Fahrenheit
{
    unsigned int tVal;
    unsigned int val2;
    unsigned int val;
    float temp;
                                                         //We are using two bytes of data that we read from I2C, we need to shift the first byte
                                                         //and add the second byte to that value
    val = b->RxBuffer[0] << 8;                           //Shift data by 8 bits (1 byte) e.g., 60 (0110 0000) would be shifted 0110 0000 0000 0000
    val2 = b->RxBuffer[1];                               //No shift required e.g., if value is 50 decimal, then in binary it would be 0000 0000 0101 0000
    tVal = val + val2;                                   //Add both values (first and second byte)


    temp = -6 + (125 * (tVal / DivValue));               //Convert data to decimal humidity value (-6 + 125*(S_hum/2^16)
    b->Humidity[0] = temp;                               //Store result in memory
}
