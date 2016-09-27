/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensor.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "sensor.h"
#include "callback.h"
#include "debug.h"
#include "timers.h"
#include "queue.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

SENSOR_DATA sensorData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SENSOR_Initialize ( void )

  Remarks:
    See prototype in sensor.h.
 */


void Transmit_Packaging(msg_header *header, short *data, msg_footer *footer){
    *header.source = ROVER_1;
    *header.destination = ROVER_2;
    *header.sequence = ir_tx_sequence;
    *data = sensorData.distance;

    ir_tx_checksum = ir_tx_checksum + *header.source + *header.destination + *header.sequence;
    ir_tx_checksum += *data;
    ir_tx_checksum += ir_tx_checksum;

    *footer.checksum = ir_tx_checksum;

    ir_tx_sequence++;
}

void IR_Receive_Packaging(msg_header *header, short *data, msg_footer *footer){
    ir_rx_checksum = *header.source + *header.destination + *header.sequence;
    ir_rx_checksum += *data;
    ir_rx_checksum += *footer.checksum;

    /*
    if (ir_rx_checksum != ir_rx_msg_footer.checksum){
        // if the checksums do not match -- bad data was received
        error_LEDS();
    }
    if (rx_sequence != ir_rx_msg_header.sequence){
        // if the sequence data does not match -- data was dropped somewhere
        error_LEDS();
    }
      */
    ir_rx_sequence++;
}


void SENSOR_ADC_Average(void){
    int i;
    sensorData.dataReady = true;
    
    for (i = 0; i < ADC_NUM_SAMPLE_PER_AVERAGE; i++){
        sensorData.potValue += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
    }
    sensorData.potValue = sensorData.potValue / ADC_NUM_SAMPLE_PER_AVERAGE;
    sensorData.distance = 31 - (sensorData.potValue * ((double) 30 / 1024));    
    
    xQueueSendFromISR(sensorQueue, &sensorData.distance, NULL);
    Transmit_Packaging(&ir_msg_header, &ir_msg_data, &ir_msg_footer);
}


void SENSOR_ADC_AverageOnLEDs(void){
    sensorData.potValue >>= 8;
    switch (sensorData.potValue){
        case 0b00:
        {
#ifdef DEBUG
            dbgOutputLoc(ADC0);
#endif
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 1);

            break;
        }
        case 0b01:
        {
#ifdef DEBUG
            dbgOutputLoc(ADC1);
#endif
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 0);

            break;
        }
        case 0b10:
        {
#ifdef DEBUG
            dbgOutputLoc(ADC2);
#endif
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 0);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 1);

            break;
        }
        case 0b11:
        {
#ifdef DEBUG
            dbgOutputLoc(ADC3);
#endif
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 0);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 0);

            break;
        }
        default:
        {
            break;
        }
    }
}


void SENSOR_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sensorData.state = SENSOR_STATE_INIT;
    sensorData.dataReady = false;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void SENSOR_Tasks ( void )

  Remarks:
    See prototype in sensor.h.
 */

void SENSOR_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( sensorData.state )
    {
        /* Application's initial state. */
        case SENSOR_STATE_INIT:
        {
            bool appInitialized = true;
            sensorQueue = xQueueCreate(10, sizeof(char));
            if (sensorQueue == NULL){
                error_LEDS();
            }
            
            sensorTimer = xTimerCreate("sensorTimer", (100 / portTICK_PERIOD_MS), pdTRUE, (void *) 0, vSensorTimerCallback);
            if(sensorTimer == NULL){
                error_LEDS();
            }
            if (xTimerStart(sensorTimer, 5) != pdPASS){
                error_LEDS();
            }
       
        
            if (appInitialized)
            {
                DRV_ADC_Open();
                sensorData.state = SENSOR_STATE_IDLE;
            }
            break;
        }

        case SENSOR_STATE_IDLE:
        {
            if (sensorData.dataReady){
                sensorData.state = SENSOR_STATE_DONE_CONVERTING;
            }
        
            break;
        }
        
        case SENSOR_STATE_DONE_CONVERTING:
        {
            if (sensorData.dataReady == true){
                SENSOR_ADC_AverageOnLEDs();
            }
            sensorData.dataReady = false;
            sensorData.state = SENSOR_STATE_IDLE;
            
            if (xQueueReceive(sensorQueue, &sensorDataReceived, portMAX_DELAY)){
                dbgOutputVal(ir_msg_header.source);
                dbgOutputVal(ir_msg_header.destination);
                dbgOutputVal(ir_msg_header.sequence);
                dbgOutputVal((ir_msg_header.sequence >> 8));
                dbgOutputVal(ir_msg_data);
                dbgOutputVal((ir_msg_data >> 8));
                dbgOutputVal(ir_msg_footer.checksum);
                dbgOutputVal((ir_msg_footer.checksum >> 8));
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
