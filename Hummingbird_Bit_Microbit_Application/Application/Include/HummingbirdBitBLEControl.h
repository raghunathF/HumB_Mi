/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/

/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
extern volatile bool spi_xfer_done;
extern uint8_t INITIAL_NAME[2];
extern volatile bool broadcast_flag ;
extern volatile uint8_t tail_pointer;
extern volatile uint8_t head_pointer; 
extern uint8_t buffer_data[255];
extern bool app_selected;
extern uint16_t transmit_length;
extern uint8_t    sensor_outputs[20];
extern uint8_t prev_state;
extern bool flash_mb ;
/************************************************************************/

/************************************************************************/
/******************     DEFINES        **********************************/
/************************************************************************/
#define SYMBOL_STRING_MASK       									0x80
#define SYMBOL 									 									0x80
#define LENGTH_STRING_MASK       									0x1F
#define SCROLL_FLASH_MASK       									0x40
#define FLASH                    									0x40
#define SAMD_MINIMUM_FIRMWARE_VERSION 						0x01   

//Commands opcode
#define SETALL_SPI                                0xCA
#define SET_LEDARRAY                              0xCC
#define SET_LED_2                                 0xC1
#define SET_LED_3                                 0xC2
#define SET_BUZZER                                0xCD
#define SET_CALIBRATE                             0xCE
#define SET_FIRMWARE                              0xCF
#define STOP_ALL                                  0xCB
#define BROADCAST                                 'b'
#define MICRO_IO                                  0x90

#define LENGTH_SETALL_SPI                         13
#define LENGTH_OTHER_SPI                          4

#define HARDWARE_VERSION				                  0x01
#define MICRO_FIRMWARE_VERSION                    0x01

#define SENSOR_SEND_LENGTH                				14

#define HUMMINGBIRD_BIT                           0
#define MICRO_BIT 			                          1

#define LOWER_LIMIT_BATTERY_MB_HB									0x20
#define UPPER_LIMIT_BATTERY_MB_HB									0xFF
/************************************************************************/

/************************************************************************/
/******************     Prototypes     **********************************/
/************************************************************************/
void uart_spi_bridge(void);
void check_update_name(void);
void check_update_name_disconnect(void);
void init_broadcast_timer(void);
void  broadcast_stop(void);
void broadcast_timer_start(void);
/************************************************************************/
