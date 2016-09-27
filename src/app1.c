/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app1.c

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


#include "app1.h"
#include "debug.h"
#include "timers.h"
#include "callback.h"
#include "queue.h"
#include "app1_public.h"

#define UART
//#define DEBUG


void error_LEDS(){
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 0);
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 0);
    xTimerStop(my_timer, 0);
    halt();
}

void flash_LEDS(){
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
}


int appSendTimerValToMsgQ(unsigned int millisecondsElapsed){
    
#ifdef DEBUG
    dbgOutputLoc(CALLBACK);
#endif
        
    if (*name_current == NULL){
        name_current = name_original;
    }
    
    DRV_USART_BUFFER_HANDLE rx_buf_handle;
    DRV_USART_BufferAddRead(my_usart, &rx_buf_handle, &wifly_rx_msg_header, 4);
    DRV_USART_BufferAddRead(my_usart, &rx_buf_handle, &wifly_rx_msg_data, 2);
    DRV_USART_BufferAddRead(my_usart, &rx_buf_handle, &wifly_rx_msg_footer, 2);
    
    
    xQueueSendFromISR(myQueue, name_current, NULL);
    
#ifdef DEBUG
    dbgOutputLoc(Q_WRITE);
#endif
    
    name_current++;
}



void Transmit_Packaging(msg_header *header, short *data, msg_footer *footer){
    *header.source = ROVER_1;
    *header.destination = ROVER_2;
    *header.sequence = tx_sequence;

    tx_checksum = tx_checksum + *header.source + *header.destination + *header.sequence;
    tx_checksum += *data;
    tx_checksum += tx_checksum;

    *footer.checksum = tx_checksum;

    tx_sequence++;
}

void Receive_Packaging(msg_header * header, short * data, msg_footer *footer){
    rx_checksum = *header.source + *header.destination + *header.sequence;
    rx_checksum += *data;
    rx_checksum += *footer.checksum;

    /*
    if (rx_checksum != wifly_rx_msg_footer.checksum){
        // if the checksums do not match -- bad data was received
        error_LEDS();
    }
    if (rx_sequence != wifly_rx_msg_header.sequence){
        // if the sequence data does not match -- data was dropped somewhere
        error_LEDS();
    }
      */
    rx_sequence++;
}



void APP1_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    init_callback();
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_C, 0x0002);
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_A, 0x0008);
    
#ifdef DEBUG    
    dbgOutputLoc(INIT);
#endif    
    
    my_usart = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);    
    app1Data.state = APP1_STATE_INIT;
    
}


void APP1_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( app1Data.state )
    {
        /* Application's initial state. */
        case APP1_STATE_INIT:
        {
            bool appInitialized = true;
            tx_sequence = 0;
            tx_checksum = 0;
            rx_sequence = 0;
            rx_checksum = 0;
            
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 1, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, 3, 1);
            
#ifdef DEBUG
            dbgOutputLoc(STATE_INIT);
#endif
            
            name_original = "Jisu Park Sean Kim Brian Worek Sachin Yadav ";
            name_current = name_original;
            
            myQueue = xQueueCreate(10, sizeof(char));
            if (myQueue == NULL){
                error_LEDS();
            }
   
            my_timer = xTimerCreate("my_timer", (50 / portTICK_PERIOD_MS), pdTRUE, (void *) 0, vTimerCallback);
            if (my_timer == NULL){
                error_LEDS();
            }
            if (xTimerStart(my_timer, 5) != pdPASS){
                error_LEDS();
            }   // check if actually started

            
            if (appInitialized)
            {
                app1Data.state = APP1_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP1_STATE_SERVICE_TASKS:
        {
#ifdef DEBUG
            dbgOutputLoc(STATE_RUN);
#endif            
            
            DRV_USART_BUFFER_HANDLE buf_handle;            
            
            if (xQueueReceive(myQueue, &rx, portMAX_DELAY)){
//                DRV_USART_BufferAddWrite(my_usart, &buf_handle, &rx, 1);
                
                // TODO: have to check if buf_handle comes back invalid 
                DRV_USART_BufferAddWrite(my_usart, &buf_handle, &wifly_tx_msg_header, 4);
                DRV_USART_BufferAddWrite(my_usart, &buf_handle, &wifly_tx_msg_data, 2);
            
                DRV_USART_BufferAddWrite(my_usart, &buf_handle, &wifly_tx_msg_footer, 2);
            }
            
            if (buf_handle == DRV_USART_BUFFER_HANDLE_INVALID){
                dbgOutputLoc(8);

            }


#ifdef DEBUG
            dbgOutputLoc(Q_READ);
#endif
            
#ifdef UART           

            
#endif
            
#ifdef DEBUG            
            dbgOutputVal(rx);
            dbgOutputLoc(DEBUG_WRITE);
#endif
           
            break;
        }    

        default:
        {
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */
