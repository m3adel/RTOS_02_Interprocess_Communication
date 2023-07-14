/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include <limits.h>
#include <semphr.h>
#include <string.h>

/*-----------------------------------------------------------*/

/* Priorities for the application tasks. */
#define BTN1_TASK_PRIORITY				2
#define BTN2_TASK_PRIORITY				2
#define PERIODIC_TASK_PRIORITY		0
#define UART_TASK_PRIORITY				3

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

void Btn1_Task( void *pvParameters );
void Btn2_Task( void *pvParameters );
void Periodic_Transmitter( void *pvParameters );
void Uart_Receiver( void *pvParameters );
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
TaskHandle_t PeriodicTransmitterHandler = NULL;
TaskHandle_t UartReceiverHandler = NULL;
TaskHandle_t Btn1_Task_Handler = NULL;
TaskHandle_t Btn2_Task_Handler = NULL;
QueueHandle_t EventQueue;
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

		EventQueue = xQueueCreate(10, sizeof(const char *));
    /* Create Tasks here */
	xTaskCreate( 
								Btn1_Task										, 
								"Btn1_Task"										,
								configMINIMAL_STACK_SIZE						,
								NULL											,
								BTN1_TASK_PRIORITY								,
								&Btn1_Task_Handler								);
	
	xTaskCreate( 
								Btn2_Task										, 
								"Btn2_Task"										,
								configMINIMAL_STACK_SIZE						,
								NULL											,
								BTN2_TASK_PRIORITY								,
								&Btn2_Task_Handler								);


								 
								 
	xTaskCreate( 
								Periodic_Transmitter							,
								"PeriodicTransmitter"							,
								configMINIMAL_STACK_SIZE						,
								NULL											,
								UART_TASK_PRIORITY								,
								&PeriodicTransmitterHandler						);	
								 
								 
	xTaskCreate( 
								Uart_Receiver									,
								"UARTreceiver"									,
								configMINIMAL_STACK_SIZE						,
								NULL											,
								PERIODIC_TASK_PRIORITY							,
								&UartReceiverHandler							);

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}

/*-----------------------------------------------------------*/
void Btn1_Task( void *pvParameters )
{
		uint8_t u8_PressFlag = pdFALSE;
		uint8_t ButtonState;
    for(;;)
			{
				ButtonState = GPIO_read(PORT_0,PIN0);  
        if (ButtonState == pdTRUE && u8_PressFlag == pdFALSE) 
					{
						const char *eventString = "\n BUTTON1: Rising Edge \n";
            xQueueSend(EventQueue, &eventString, portMAX_DELAY);
            u8_PressFlag = pdTRUE;
					} 
				else if (ButtonState == pdFALSE && u8_PressFlag == pdTRUE) 
					{
						const char *eventString = "\n BUTTON1: Falling Edge \n";
            xQueueSend(EventQueue, &eventString, portMAX_DELAY);
						u8_PressFlag = pdFALSE;
					}
				else
				{
					//do nothin
				}
					vTaskDelay(50);
    }
}

/*-----------------------------------------------------------*/
void Btn2_Task( void *pvParameters )
{
		uint8_t u8_PressFlag = pdFALSE;
		uint8_t ButtonState;
    for(;;)
			{
				ButtonState = GPIO_read(PORT_0,PIN1);  
        if (ButtonState == pdTRUE && u8_PressFlag == pdFALSE) 
					{
						const char *eventString = "\n BUTTON2: Rising Edge \n";
            xQueueSend(EventQueue, &eventString, portMAX_DELAY);
            u8_PressFlag = pdTRUE;
					} 
				else if (ButtonState == pdFALSE && u8_PressFlag == pdTRUE) 
					{
						const char *eventString = "\n BUTTON2: Falling Edge \n";
            xQueueSend(EventQueue, &eventString, portMAX_DELAY);
						u8_PressFlag = pdFALSE;
					}
				else
				{
					//do nothing
				}
				vTaskDelay(50);
    }
}

/*-----------------------------------------------------------*/
void Periodic_Transmitter(void *pvParameters) {

    for(;;) 
				{
					const char *eventString = "Periodic_String  ";
					xQueueSend(EventQueue, &eventString, portMAX_DELAY);
					vTaskDelay(100);
				}
}

/*--------------------------------------------------------------*/
void Uart_Receiver(void *pvParameters) {
    const char *eventString;
    for(;;) 
			{
				if (xQueueReceive(EventQueue, &eventString, portMAX_DELAY)) 
				{
						vSerialPutString ((const signed char*)eventString, strlen(eventString));
				}
				vTaskDelay(20);
			}
}

/*--------------------------------------------------------------*/

