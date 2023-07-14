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
#include "semphr.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Priorities for the application tasks. */
#define UART1_TaskA_PRIORITY		3
#define UART2_TaskB_PRIORITY		2

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

void UART1_TaskA( void *pvParameters );
void UART2_TaskB( void *pvParameters );

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
SemaphoreHandle_t UART_Semaphore = NULL;
TaskHandle_t UART1_TaskA_Handler = NULL;
TaskHandle_t UART2_TaskB_Handler = NULL;
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	UART_Semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive( UART_Semaphore );
	
    /* Create Tasks here */
	xTaskCreate( 
								UART1_TaskA											, 
								"UART1_Task"										,
								configMINIMAL_STACK_SIZE							,
								NULL												,
								UART1_TaskA_PRIORITY								,
								&UART1_TaskA_Handler								);
	xTaskCreate( 
								UART2_TaskB											, 
								"UART2_TaskB"										,
								configMINIMAL_STACK_SIZE							,
								NULL												,
								UART2_TaskB_PRIORITY								,
								&UART2_TaskB_Handler								);


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
void UART1_TaskA( void *pvParameters )
{
  /* Local variable to be used as index. */	
	uint8_t i = 0;
	
	for( ;; )
	{

		if ( xSemaphoreTake( UART_Semaphore, portMAX_DELAY ) == pdTRUE )
		{
			vSerialPutString( ( signed char * ) "***** TASK A *****\r\n", 22 );			
			vTaskDelay( 5 );
			for ( i = 0; i < 10; i++ )
			{
				xSerialPutChar( i + '0' );
				vSerialPutString( ( signed char * ) "-string\r\n", 11 );				
				vTaskDelay( 2 );
			}
			xSemaphoreGive( UART_Semaphore );
		}
		else
		{
			//nothing
		}
		
		vTaskDelay( 100 );
	}
}
/*-----------------------------------------------------------*/
void UART2_TaskB( void *pvParameters )
{
	uint8_t j = 0;
	uint32_t k = 0;
	for( ;; )
	{
if ( xSemaphoreTake( UART_Semaphore, portMAX_DELAY ) == pdTRUE )
		{
			vSerialPutString( ( signed char * ) "***** TASK B ******\r\n", 22 );			
			vTaskDelay( 5 );
			for ( j = 0; j < 10; j++ )
			{
				xSerialPutChar( j + '0' );
				vSerialPutString( ( signed char * ) "-string\r\n", 11 );
				for ( k = 0; k < 100000; k++ )
				{
					// Nothing
				}      
			}
			xSemaphoreGive( UART_Semaphore );
		}
		else
		{
			//nothing
		}
		vTaskDelay( 500 );		
	}
}

/*-----------------------------------------------------------*/

