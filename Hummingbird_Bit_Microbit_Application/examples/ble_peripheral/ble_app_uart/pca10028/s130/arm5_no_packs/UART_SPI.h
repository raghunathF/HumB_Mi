/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*/
/************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
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
/************************************************************************/

void uart_spi_bridge();
void check_update_name();
void check_update_name_disconnect();
void init_broadcast_timer();
void  broadcast_stop();
void broadcast_timer_start();
/************************************************************************/
