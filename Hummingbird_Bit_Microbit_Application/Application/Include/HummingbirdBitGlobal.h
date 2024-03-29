/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/

/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdlib.h>
#include <stdio.h>
/************************************************************************/

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         11                                           /**< Size of timer operation queues. */

#define CHROMEBOOK 

#define HUMMINGBIRDBIT									1
#define MICROBIT												0

#define ADVERTISING_MODE								0x00                                        
#define BLUETOOTH_MODE 									0x01
#define UART_MODE												0x02

#define SOUND_STARTUP         					1 
#define SOUND_CONNECTION      					2
#define SOUND_DISCONNECTION   					3

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define LED3_TEST 											18                                           /**Debug test pins **/ //LED3   
#define LED2_TEST 											1																						//LED2

#define SENSORS_LENGTH  								5

#define HUMMINGBIRD_BIT   							0
#define MICRO_BIT 			  							1

#ifdef  CHROMEBOOK
#define MICRO_FIRMWARE_VERSION  				0x52
#else
#define MICRO_FIRMWARE_VERSION  				0x02
#endif

#define NXP 												1
#define LS													2

#define REFRESH_SYMBOL              0x155555

#define BROKEN											2 
#define FAILURE											0
#define SUCCESS											1


/************************************************************************/
