/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "bsp.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "sdk_config.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"

#include "HummingbirdBitSPIMaster.h"
#include "HummingbirdBitUARTControl.h"
/************************************************************************/

/************************************************************************/
/******************     Define         **********************************/
/************************************************************************/
#define SPI_INSTANCE  0 /**< SPI instance index. */
#define TEST_PIN_1 18
#define TEST_PIN_2 1
/************************************************************************/

/************************************************************************/
/******************    Variables       **********************************/
/************************************************************************/
nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

extern uint8_t temp_p_data[20];
static uint8_t       m_rx_buf[20];           /**< RX buffer. */

extern uint8_t    input_micro_packet[20];
extern uint8_t    sensor_outputs[20];
extern uint8_t    temp_p_sensor_data[20];

extern  volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
/************************************************************************/

//SPI callback 
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
		spi_xfer_done = true;
}
/************************************************************************/

void read_sensor_HB()
{

	int i =0;
	for(i=0;i<4;i++)
	{
			sensor_outputs[i] = m_rx_buf[i];                 //Read HummingBird Bit inputs

	}
}
/************************************************************************/

void read_sensor_MB()
{
	int i =0;
	for(i=4;i<14;i++)
	{
		sensor_outputs[i] = input_micro_packet[i-4];    //Read micro bits inputs
	}	
}
/************************************************************************/

//SPI transfer
void transfer_data(uint16_t transmit_length,uint8_t* data_send)
{
	spi_xfer_done = false;
	memset(m_rx_buf, 0, transmit_length);
	nrf_drv_spi_transfer(&spi, data_send, transmit_length, m_rx_buf, transmit_length);
}
/************************************************************************/

/************************************************************************/
//Initializing the values of SPI transfer buffer
void initSPISendData()
{
	  uint8_t i =0;
	  for(i=0;i<18;i++)
	  {
			temp_p_data[i]  = 0xCA + i ;   //Starts with set all 
		}
}
/************************************************************************/


/************************************************************************/
void SPI_init()
{
	 
	  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
	  //Initilizing the data to be sent by SPI Master
	  initSPISendData();
}
/************************************************************************/
