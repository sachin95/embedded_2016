/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app1.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP1_H
#define _APP1_H


#define IR_LENGTH                       2
#define PIXY_LENGTH                     20
#define WIFLY_LENGTH                    2
#define UART_TO_CONTROL_LENGTH          2
#define SENSOR_TO_CONTROL_LENGTH        4
#define CONTROL_TO_MOTOR_LENGTH         1


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "timers.h"
#include "queue.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 


typedef enum{
    ROVER_1=0,
    ROVER_2,
    IR,
    PIXY,
    MOTOR_CTL,
    SENSOR_TO_CONTROL,
    UART_TO_CONTROL,
    UART_TO_SENSOR,
    CONTROL_TO_MOTOR,
} MESSAGE_SOURCE_DESTINATION;
    
    
struct msg_header{
    char source;
    char destination;
    short sequence;
}wifly_tx_msg_header, wifly_rx_msg_header, ir_msg_header, ir_rx_msg_header;


struct msg_footer{
    short checksum;
}wifly_tx_msg_footer, wifly_rx_msg_footer, ir_msg_footer, ir_rx_msg_footer;

typedef enum
{
	/* Application's state machine's initial state. */
	APP1_STATE_INIT=0,
	APP1_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} APP1_STATES;


typedef struct
{
    /* The application's current state */
    APP1_STATES state;

    /* TODO: Define any additional data used by the application. */

} APP1_DATA;

APP1_DATA app1Data;
TimerHandle_t my_timer;
QueueHandle_t myQueue;
DRV_HANDLE my_usart;

//msg_header wifly_msg_header;
//msg_footer wifly_msg_footer;
short wifly_tx_msg_data, wifly_rx_msg_data;
short ir_msg_data, ir_rx_msg_data;

char* name_original;
char* name_current;
char rx;

short tx_sequence, rx_sequence;
short tx_checksum, rx_checksum;
short ir_tx_sequence, ir_rx_sequence;
short ir_tx_checksum, ir_rx_checksum;


void APP1_Initialize ( void );
void error_LEDS();

void Transmit_Packaging(msg_header *header, short *data, msg_footer *footer);
void Receive_Packaging(msg_header *header, short *data, msg_footer *footer);


void APP1_Tasks( void );


#endif /* _APP1_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

