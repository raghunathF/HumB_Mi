/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains the source code for initializing the UART module.		|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include "app_uart.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "HummingbirdBitPinout.h"

/************************************************************************/

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
#define PIN_UNUSED		0xFFFFFFFF
#define PIN_UNUSED		0xFFFFFFFF

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define MAX_DATA_LEN                                        25

#define LEN_LRS_COMMAND                                     3
#define LEN_SETALL_COMMAND1                                  19
#define LEN_CALIBRATE_COMMAND																0
#define LEN_BUZZER_COMMAND																	4
#define LEN_LEDARRAY_COMMAND																19
#define LEN_READ_COMMAND																		1

#define SETALL_COMMAND																			0xCA
#define SPI_MASK 																						0xC0
/************************************************************************/

/************************************************************************/
/******************   Variables        **********************************/
/************************************************************************/
APP_TIMER_DEF(refreshTimer);
static uint8_t index 			= 0;
extern uint8_t UARTRingBuffer[255];
extern volatile uint8_t UARTTailPointer ;
extern volatile uint8_t UARTHeadPointer ; 
/************************************************************************/

/************************************************************************
Timer to reset the serial data in case of data corruption
************************************************************************/
static void startTimer()
{
	app_timer_start(refreshTimer, APP_TIMER_TICKS(20, 0), NULL);
}
/************************************************************************/

/************************************************************************/
static void stopTimer()
{
	app_timer_stop(refreshTimer);
}
/************************************************************************/
	

/************************************************************************
Ring buffer to store the data length of the commands are different 
************************************************************************/
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[MAX_DATA_LEN];
    
		static uint8_t bytesLeft  = 0;
		uint8_t i =0;
    //uint32_t       err_code;
		uint8_t firstByte = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
						if(index == 0)
						{
							startTimer();
							firstByte = data_array[index];
							if( (firstByte & SPI_MASK) == SPI_MASK )
							{
									if(firstByte == SETALL_COMMAND)
									{
											firstByte = 'A';
									}
									else
									{
											firstByte = 'a';
									}
							}
							switch(firstByte)
							{
								//Length of smaller commands to read individual data for hummingbird bit
								case 'a':
									bytesLeft = LEN_LRS_COMMAND;
									break;
								//Length of set all command
								case 'A':
									bytesLeft = LEN_SETALL_COMMAND1;
									break;
								//Length of calibrate
								case 'C':
									bytesLeft = LEN_CALIBRATE_COMMAND;
									break;
								//Length of buzzer
								case 'B':
									bytesLeft = LEN_BUZZER_COMMAND;
									break;
								//Length of LED ARRAY
								case 'l':
									bytesLeft = LEN_LEDARRAY_COMMAND;
									break;
								//Length of read
								case 'R':
									bytesLeft = LEN_READ_COMMAND;
									break;
								default:
									bytesLeft = 0;
									break;
							}
						}
						//read all the data and store in a global variable
            index++;
						if(bytesLeft == 0)
						{
							stopTimer();
							for(i=0;i<index;i++)
							{
								UARTRingBuffer[UARTHeadPointer] = data_array[i];
								UARTHeadPointer = UARTHeadPointer + 1;
							}
							index = 0;
							
						}
						else
						{
							bytesLeft--;
						}
						
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/************************************************************************/
void sendUART(uint8_t* inputData , uint8_t length)
{
	uint8_t i =0;
	for(i=0;i<length;i++)
	{
		app_uart_put(inputData[i]);
	}
}
/************************************************************************/

/************************************************************************/
void UARTInit()
{
	uint32_t    err_code;
  const app_uart_comm_params_t comm_params =
   {
        RX_MICRO,
        TX_MICRO,
        PIN_UNUSED,
        PIN_UNUSED,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
   };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
	 APP_ERROR_CHECK(err_code);
}
/************************************************************************/

/************************************************************************
Refresh the data of the ring buffer in case of a failure to receive
complete length of the data
************************************************************************/
static void refreshTimerHandler(void * p_context)
{
		index = 0;
		UARTHeadPointer = 0;
		UARTTailPointer = 0;
}
/************************************************************************/

/************************************************************************/
void createRefreshTimer()
{
		app_timer_create(&refreshTimer,APP_TIMER_MODE_SINGLE_SHOT,refreshTimerHandler);
}
/************************************************************************/

/************************************************************************/
void UARTTXRXInit()
{
	  nrf_gpio_cfg_input(RX_MICRO, NRF_GPIO_PIN_PULLUP); //This some how has a lot of infulence
		UARTInit();
	  nrf_gpio_cfg_input(RX_MICRO, NRF_GPIO_PIN_PULLUP);
		createRefreshTimer();
}
/************************************************************************/
