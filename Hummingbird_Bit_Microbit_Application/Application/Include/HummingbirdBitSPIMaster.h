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
#include <stdbool.h>
#include <stdlib.h>
/************************************************************************/


/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
#define SPI_SS_PIN				16      //P
#define SPI_MISO_PIN			22
#define SPI_MOSI_PIN			21
#define SPI_SCK_PIN				23
#define INITIAL_LENGTH    19
/************************************************************************/


/************************************************************************/
/******************     Prototpes      **********************************/
/************************************************************************/
extern uint8_t input_micro_packet[20];
void transfer_data_sensor(uint16_t transmit_length);
void transfer_data(uint16_t transmit_length,uint8_t* data_send);
void read_sensor_HB(void);
void read_sensor_MB(void);
void SPI_init(void);
/************************************************************************/



