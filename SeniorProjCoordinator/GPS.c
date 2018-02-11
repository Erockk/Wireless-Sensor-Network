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
 *  ======== uartecho.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdio.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>
#include <GPS.h>


#define TASKSTACKSIZE     768
#define bSize   200
#define packet  5
#define time    6
#define status  1
#define LatDeg  2
#define LongDeg 3
#define DMM     6   //DMM degress and decimal minutes



struct pointer test;


struct Buffer
{
    unsigned char rxBuffer[bSize];
    char comparePacket[packet];
    char Time[time];
    char Status[status];
    char NS[status];
    char EW[status];

    char LatitudeDeg[LatDeg];
    char LatitudeDMM[DMM];               // DMM degrees and decimal minutes
    char LongitudeDeg[LongDeg];
    char LongitudeDMM[DMM];
};

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 *
 */
Void GPS(UArg arg0, UArg arg1)
{


    struct Buffer p;
    bool done = false;
    bool comp = false;
    //char ret;
    int i = 0;

    //UART_CONFIG_WLEN_MASK = 0x08;
   // unsigned char p.rxBuffer[200];
    //int ret;
    UART_Handle handle;
    UART_Params uartParams;
   Display_Params params;
   Display_Params_init(&params);
   params.lineClearMode = DISPLAY_CLEAR_BOTH;
   Display_Handle hDisplayLcd = Display_open(Display_Type_LCD, &params);


    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;      //Not used, but Data is not processed mode
    uartParams.readDataMode = UART_DATA_BINARY;       //Data is not processed mode
    uartParams.readReturnMode = UART_RETURN_FULL;     //Receive return mode / UART_RETURN_FULL unblock/callback when buffer is full
    uartParams.readEcho = UART_ECHO_OFF;              //Echo received data back (false)
    uartParams.baudRate = 9600;                       //Set the baud rate of the GPS
    handle = UART_open(Board_UART0, &uartParams);     //

    if (handle == NULL) {
        System_abort("Error opening the UART");
    }

   //UART_write(uart, echoPrompt, sizeof(echoPrompt));


     /*
      *  ======== Parse ========
      *  Simple function for parsing data
      */
   // th.t2 = (char*) malloc(1);


        test.t = (char*) malloc(1);
        test.h = (char*) malloc(1);
    while (1)
    {
       UART_read(handle, p.rxBuffer, 200);                                      //Read in data from UART into rxBuffer (100 byte buffer)
       for(i = 0; i < 200; i++)                                                 //Iterate through buffer with for loop
       {
           if (p.rxBuffer[i] == '$')                                            //Start printing at the beginning of packet '$'
           {
               while(done == false)                                             //This while loop will iterate the buffer until a newline character is found
               {
                   comp = comparePackets(&p.rxBuffer, i);                       //Compares packets with "GPRMC", if true, then print to LCD
                   if (comp == true)
                   {
                       Time(&p.rxBuffer, i);                                    //This function creates an array with data for TIME
                       Status(&p.rxBuffer, i);
                       Display_print5(hDisplayLcd, 1, 2, "Packet: %c%c%c%c%c",  //This function prints the packet type
                       p.comparePacket[0], p.comparePacket[1],
                       p.comparePacket[2], p.comparePacket[3],
                       p.comparePacket[4]);

                       Display_print2(hDisplayLcd, 3, 2, "HR: %c%c", p.Time[0], p.Time[1]); //This function prints the Hour
                       Display_print2(hDisplayLcd, 4, 2, "MIN:%c%c", p.Time[2], p.Time[3]); //This function prints the Minutes
                       Display_print2(hDisplayLcd, 5, 2, "SEC:%c%c", p.Time[4], p.Time[5]); //This function prints the Seconds
                       Task_sleep(1000 * (1000/Clock_tickPeriod));              //Delay (1000 * (1000 / 48,000,000))

                       Display_print1(hDisplayLcd, 6, 2, "Status: %c", p.Status[0]); //This function prints the Status
                       if(p.Status[0] == 'A')                                   //Do not parse data if data is invalid! 'V' void
                       {
                           Latitude(&p.rxBuffer, i);
                           Longitude(&p.rxBuffer, i);
                           NorthSouth(&p.rxBuffer, i);
                           EastWest(&p.rxBuffer, i);
                           Display_print5(hDisplayLcd, 7, 0, "Lat:%c%c Long:%c%c%c", p.LatitudeDeg[0], //This function prints latitude/longitude degrees
                           p.LatitudeDeg[1], p.LongitudeDeg[0], p.LongitudeDeg[1], p.LongitudeDeg[2]);


                           Display_print2(hDisplayLcd, 8, 0, "N/S:%c   E/W:%c", p.NS[0], p.EW[0]); //This function prints NS/EW

                           Display_print5(hDisplayLcd, 9, 0, "LatDMM:%c%c%c%c%c", p.LatitudeDMM[0], //This function prints DMM
                                                    p.LatitudeDMM[1], p.LatitudeDMM[2], p.LatitudeDMM[3], p.LatitudeDMM[4]);

                           Display_print5(hDisplayLcd, 10, 0, "LongDMM:%c%c%c%c%c", p.LongitudeDMM[0], //This function prints DMM
                                                                               p.LongitudeDMM[1], p.LongitudeDMM[2], p.LongitudeDMM[3], p.LongitudeDMM[4]);
                       //    Display_print2(hDisplayLcd, 11, 0, "LatM:%c LongM:%c ", p.LongitudeDMM[5], p.LatitudeDMM[5]); //This function prints NS/EW
                       }
                       else
                       {
                           Display_print0(hDisplayLcd, 7, 0, "Lat:NA Long:NA");            //Data not available!!
                           Display_print0(hDisplayLcd, 8, 0, "N/S:NA  E/W:NA");
                           Display_print0(hDisplayLcd, 9, 0, "LaDMM:NA");
                           Display_print0(hDisplayLcd, 10, 0, "LoDMM:NA");
                       //    Display_print0(hDisplayLcd, 11, 0, "LatM:NA LongM:NA");
                       }
                       comp = false;                                               //Reset comp variable
                   }
                   if (p.rxBuffer[i] == 10)                                     //If newline character is found, break loop and read in new data
                   {
                    done = true;                                                //Break while loop by setting done = to true
                    i = 200;                                                    //Break for loop
                   }
                i++;                                                            //Increment i to iterate through buffer
               }
           }
        done = false;                                                           //Reset done variable to false
       }

     //  Task_sleep(1000 * (1000/Clock_tickPeriod));
       if (*test.t != 0)
       {
           Display_print3(hDisplayLcd, 11, 2, "T: %d H: %d%c", *test.t, *test.h, '%');
      //     free(t.t);
       }


    }

}

/*
 *  ======== main ========
 */
void GPS_init(void)
{
    Task_Params taskParams;

    /* Construct BIOS objects */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)GPS, &taskParams, NULL);

}

bool comparePackets(struct Buffer * b, int k)
{
   char cmp[] = {'G','P','R','M','C'};
   int i = 0;

   for(i = 0; i < 5; i++)
   {
       k++;
       b->comparePacket[i] = b->rxBuffer[k];
   }
   int y;
   y = strncmp(cmp, b->comparePacket, 5);
   if (y == 0)
   {
      return true;
   }
   return false;
}

void Time(struct Buffer * b, int k)
{
int i = 0;
k += 7;
while(b->comparePacket[k] != ',' && i != 7)
{
    b->Time[i] = b->rxBuffer[k];
    i++;
    k++;
}
}

void Status(struct Buffer * b, int k)
{
int i = 0;
k += 18;                                        //Point at byte element 18 on the packet
while(b->comparePacket[k] != ',' && i < 1)
{
    b->Status[i] = b->rxBuffer[k];
    i++;
    k++;
}
}

void Latitude(struct Buffer * b, int k)
{
int i = 0;
int j = 0;
k += 20;                                        //Point at byte element 18 on the packet
while(b->comparePacket[k] != ',' && i < 10)
{
    if (i < 2)
    {
    b->LatitudeDeg[i] = b->rxBuffer[k];
    i++;
    k++;

    }
    else if (2 <= i < 10)
    {

        if (b->rxBuffer[k] != '.')
        {
            b->LatitudeDMM[j] = b->rxBuffer[k];
                i++;
                j++;
        }
                k++;
    }
}
}


void Longitude(struct Buffer * b, int k)
{
int i = 0;
int j = 0;
k += 32;                                        //Point at byte element 18 on the packet
while(b->comparePacket[k] != ',' && i < 10)
{
    if (i < 3)
    {
    b->LongitudeDeg[i] = b->rxBuffer[k];
    i++;
    k++;

    }
    else if (3 <= i < 10)
    {
        if (b->rxBuffer[k] != '.')
        {
            b->LongitudeDMM[j] = b->rxBuffer[k];
                i++;
                j++;
        }
                k++;
    }
}
}

void NorthSouth(struct Buffer * b, int k)
{
int i = 0;
k += 30;                                        //Point at byte element 18 on the packet
while(b->comparePacket[k] != ',' && i < 1)
{
    b->NS[i] = b->rxBuffer[k];
    i++;
    k++;
}
}

void EastWest(struct Buffer * b, int k)
{
int i = 0;
k += 43;                                        //Point at byte element i + 43 on the packet
while(b->comparePacket[k] != ',' && i < 1)
{
    b->EW[i] = b->rxBuffer[k];
    i++;
    k++;
}
}


