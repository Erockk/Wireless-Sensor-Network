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

/***** Includes *****/

#include <xdc/std.h>
#include <stdio.h>
#include <xdc/runtime/System.h>
#include <stdint.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* Drivers */
#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"
#include "CoordinatorRFTask.h"
#include "CoordinatorTask.h"
#include "RadioProtocol.h"
//#include "GPS.h"



/***** Defines *****/
#define COORDINATOR_TASK_STACK_SIZE 1024
#define COORDINATOR_TASK_PRIORITY   3

#define COORDINATOR_EVENT_ALL                         0xFFFFFFFF
#define COORDINATOR_EVENT_NEW_ADC_SENSOR_VALUE    (uint32_t)(1 << 0)

#define COORDINATOR_MAX_NODES 7

#define COORDINATOR_DISPLAY_LINES 8

/***** Type declarations *****/
struct AdcSensorNode {
    uint8_t address;
    uint16_t latestAdcValue;
    uint8_t button;
    int8_t latestRssi;
};




/***** Variable declarations *****/
static Task_Params coordinatorTaskParams;
Task_Struct coordinatorTask;    /* not static so you can see in ROV */
static uint8_t coordinatorTaskStack[COORDINATOR_TASK_STACK_SIZE];
Event_Struct coordinatorEvent;  /* not static so you can see in ROV */
static Event_Handle coordinatorEventHandle;


static struct AdcSensorNode latestActiveAdcSensorNode;
struct AdcSensorNode knownSensorNodes[COORDINATOR_MAX_NODES];
static struct AdcSensorNode* lastAddedSensorNode = knownSensorNodes;
static Display_Handle hDisplayLcd;
static Display_Handle hDisplaySerial;
//struct TempHum test;



/***** Prototypes *****/
static void coordinatorTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union CoordinatorPacket* packet, int8_t rssi);
static uint8_t isKnownNodeAddress(uint8_t address);
static void addNewNode(struct AdcSensorNode* node);
static void updateNode(struct AdcSensorNode* node);


/***** Function definitions *****/
void CoordinatorTask_init(void) {

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&coordinatorEvent, &eventParam);
    coordinatorEventHandle = Event_handle(&coordinatorEvent);

    /* Create the concentrator radio protocol task */

      Task_Params_init(&coordinatorTaskParams);
      coordinatorTaskParams.arg0 = 1000000 / Clock_tickPeriod;
      coordinatorTaskParams.stackSize = COORDINATOR_TASK_STACK_SIZE;
      coordinatorTaskParams.stack = &coordinatorTaskStack;
      coordinatorTaskParams.priority = COORDINATOR_TASK_PRIORITY;
      Task_construct(&coordinatorTask, (Task_FuncPtr)coordinatorTaskFunction, &coordinatorTaskParams, NULL);

}


static void coordinatorTaskFunction(UArg arg0, UArg arg1)
{

    /* Enter main task loop */
    /* Register a packet received callback with the radio task */
       CoordinatorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);
       while(1) {

           /* Wait for event */
           uint32_t events = Event_pend(coordinatorEventHandle, 0, COORDINATOR_EVENT_ALL, BIOS_WAIT_FOREVER);

           /* If we got a new ADC sensor value */
           if(events & COORDINATOR_EVENT_NEW_ADC_SENSOR_VALUE) {
               /* If we knew this node from before, update the value */
               if(isKnownNodeAddress(latestActiveAdcSensorNode.address)) {
                   updateNode(&latestActiveAdcSensorNode);
               }
               else {
                   /* Else add it */
                   addNewNode(&latestActiveAdcSensorNode);
               }

               /* Update the values on the LCD */
               Task_sleep(10000000/ Clock_tickPeriod);

           }
       }
}

static void packetReceivedCallback(union CoordinatorPacket* packet, int8_t rssi)
{
    /* If we recived an ADC sensor packet, for backward compatibility */
    if (packet->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
    {
        /* Save the values */
        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
        latestActiveAdcSensorNode.latestAdcValue = packet->adcSensorPacket.adcValue;
        latestActiveAdcSensorNode.button = 0; //no button value in ADC packet
        latestActiveAdcSensorNode.latestRssi = rssi;

        Event_post(coordinatorEventHandle, COORDINATOR_EVENT_NEW_ADC_SENSOR_VALUE);
    }
    /* If we recived an DualMode ADC sensor packet*/
    else if(packet->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
    {

        /* Save the values */
        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
        latestActiveAdcSensorNode.latestAdcValue = packet->dmSensorPacket.adcValue;
        latestActiveAdcSensorNode.button = packet->dmSensorPacket.button;
        latestActiveAdcSensorNode.latestRssi = rssi;

        Event_post(coordinatorEventHandle, COORDINATOR_EVENT_NEW_ADC_SENSOR_VALUE);
    }
}

static uint8_t isKnownNodeAddress(uint8_t address) {
    uint8_t found = 0;
    uint8_t i;
    for (i = 0; i < COORDINATOR_MAX_NODES; i++)
    {
        if (knownSensorNodes[i].address == address)
        {
            found = 1;
            break;
        }
    }
    return found;
}

static void updateNode(struct AdcSensorNode* node) {
    uint8_t i;
    for (i = 0; i < COORDINATOR_MAX_NODES; i++) {
        if (knownSensorNodes[i].address == node->address)
        {
            knownSensorNodes[i].latestAdcValue = node->latestAdcValue;
            knownSensorNodes[i].latestRssi = node->latestRssi;
            knownSensorNodes[i].button = node->button;
            break;
        }
    }
}

static void addNewNode(struct AdcSensorNode* node) {
    *lastAddedSensorNode = *node;

    /* Increment and wrap */
    lastAddedSensorNode++;
    if (lastAddedSensorNode > &knownSensorNodes[COORDINATOR_MAX_NODES-1])
    {
        lastAddedSensorNode = knownSensorNodes;
    }
}
