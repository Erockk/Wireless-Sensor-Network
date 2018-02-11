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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdlib.h>

#include "CoordinatorRFTask.h"

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include "easylink/EasyLink.h"
#include "RadioProtocol.h"
#include "CoordinatorRFTask.h"
#include "CoordinatorTask.h"
#include "GPS.h"



/***** Defines *****/
#define COORDINATOR_TASK_STACK_SIZE 1024
#define COORDINATOR_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                  0xFFFFFFFF
#define RADIO_EVENT_VALID_PACKET_RECEIVED      (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED (uint32_t)(1 << 1)
#define COORDINATOR_ACTIVITY_LED Board_LED0


/***** Variable declarations *****/
Event_Struct radioOperationEvent;
Task_Struct coordinatorRFTask;
static Event_Handle radioOperationEventHandle;
static uint8_t coordinatorTaskStack[COORDINATOR_TASK_STACK_SIZE];
Event_Struct coordinatorOperationEvent;  /* not static so you can see in ROV */

static CoordinatorRadio_PacketReceivedCallback packetReceivedCallback;
static union CoordinatorPacket latestRxPacket;
static EasyLink_TxPacket txPacket;
static struct AckPacket ackPacket;  //Found in RadioProtocol.h
static uint8_t coordinatorAddress; // 0x00
static int8_t latestRssi;



/***** Prototypes *****/
static void coordinatorRFTaskFunction(UArg arg0, UArg arg1);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void notifyPacketReceived(union CoordinatorPacket* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);
static void PrintPacketData(EasyLink_RxPacket * rxPacket);




/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
struct pointer test;

/* Configure LED Pin */
PIN_Config ledPinTable[] = {
     COORDINATOR_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


void CoordinatorRFTask_init(void)
{
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    Task_Params coordinatorRFTaskParams;

    /* Call board init functions */
    //Board_initI2C();
    // Board_initSPI();
    // Board_initUART();
    // Board_initWatchdog();

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Construct coordinator Task  thread */
    Task_Params_init(&coordinatorRFTaskParams);
    coordinatorRFTaskParams.arg0 = 1000000 / Clock_tickPeriod;
    coordinatorRFTaskParams.stackSize = COORDINATOR_TASK_STACK_SIZE;
    coordinatorRFTaskParams.stack = &coordinatorTaskStack;
    coordinatorRFTaskParams.priority = COORDINATOR_TASK_PRIORITY;
    Task_construct(&coordinatorRFTask, (Task_FuncPtr)coordinatorRFTaskFunction, &coordinatorRFTaskParams, NULL);

}

void CoordinatorRadioTask_registerPacketReceivedCallback(CoordinatorRadio_PacketReceivedCallback callback)
{
    packetReceivedCallback = callback;
}


void coordinatorRFTaskFunction(UArg arg0, UArg arg1)
{

    /* Initialize EasyLink */
    if(EasyLink_init(RADIO_EASYLINK_MODULATION) != EasyLink_Status_Success)
    {
          System_abort("EasyLink_init failed");
    }

    /* Set concentrator address */;
    coordinatorAddress = RADIO_COORDINATOR_ADDRESS;            //Coordinator address is 0x00
    EasyLink_enableRxAddrFilter(&coordinatorAddress, 1, 1);     //Filters addresses that are in table, by using 0x00, other addresses can't be added

    /* Set up Ack packet */
    /*
    AckPacket is in RadioProtocol.h and contains only a header
    (which contains a source address and packet type). We now
    proceed creating the Ack packet below
    */

    ackPacket.header.sourceAddress = coordinatorAddress;            //First byte is the address 0x00
    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;     //Second byte of the header is the packet type

    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }


    while (1)
    {


        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER); //wait until state changes

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {

            /* Send ack packet to node that sent packet to coordinator */
            sendAck(latestRxPacket.header.sourceAddress);

            /* Call packet received callback */
            notifyPacketReceived(&latestRxPacket);

            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }

            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, COORDINATOR_ACTIVITY_LED,
                    !PIN_getOutputValue(COORDINATOR_ACTIVITY_LED));
        }

        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }
        }

    }



}



static void sendAck(uint8_t latestSourceAddress) {

    /* Set destinationAdress, but use EasyLink layers destination adress capability */
    txPacket.dstAddr[0] = latestSourceAddress;

    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
    txPacket.len = sizeof(ackPacket);

    /* Send packet  */
    if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }
}

static void notifyPacketReceived(union CoordinatorPacket* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}


static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    union CoordinatorPacket * tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success)
    {
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRssi = (int8_t)rxPacket->rssi;

        /* Check that this is a valid packet */
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);

        /* If this is a known packet */
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
        {
            /* Save packet */
            memcpy((void*)&latestRxPacket, &rxPacket->payload, sizeof(struct AdcSensorPacket));

            /* Signal packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
        {
            /* Save packet */
            memcpy((void*)&latestRxPacket, &rxPacket->payload, sizeof(struct DualModeSensorPacket));

            /* Signal packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);

            if (rxPacket->payload[13] != 0)                     //The 13th byte of the payload holds the temperature data.
                   {
                       *test.t = rxPacket->payload[13];
                   }
            if (rxPacket->payload[14] != 0)
            {
                       *test.h = rxPacket->payload[14];         //The 14th byte of the payload holds the humidity data.
            }

        }
        else
        {
            /* Signal invalid packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
    }
    else
    {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }

  // *p.t = rxPacket->payload[13];
   //p.t = rxPacket->payload[14];

}



