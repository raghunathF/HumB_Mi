/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : Reads the values collected from UART service and then
							 pushes the values to SAMD through SPI.
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdio.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
/************************************************************************/

#include "HummingbirdBitSensors.h"
#include "HummingbirdBitLEDS.h"
#include "HummingbirdBitLEDArray.h"
#include "HummingbirdBitFlash.h"
#include "HummingbirdBitMagnetometer.h"
#include "HummingbirdBitAccelerometer.h"
#include "HummingbirdBitCalibration.h"
#include "HummingbirdBitGlobal.h"
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
extern const nrf_drv_twi_t m_twi;
/* Indicates if operation on TWI has ended. */
extern volatile bool m_xfer_done;
extern const nrf_drv_twi_t m_twi;
extern uint8_t calibrate_feedback; 
extern uint8_t accl_mag_chip;
/************************************************************************/




/************************************************************************/
/* TWI instance. */
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
/************************************************************************/
/**
* @brief TWI initialization with 100K frequency.
*/
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_micro_config = {
       .scl                = MICRO_SCL_PIN,
       .sda                = MICRO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_micro_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
}
/************************************************************************/

/************************************************************************/
static void reset_i2c()
{
			ret_code_t err_code;
			const nrf_drv_twi_config_t twi_micro_config = 
			{
       .scl                = MICRO_SCL_PIN,
       .sda                = MICRO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = true
      };
			
			//Remove TWI
			nrf_drv_twi_disable(&m_twi);
			nrf_drv_twi_uninit(&m_twi);
			
			//Intialize TWI
			err_code = nrf_drv_twi_init(&m_twi, &twi_micro_config, twi_handler, NULL);
			APP_ERROR_CHECK(err_code);
			nrf_drv_twi_enable(&m_twi);
}
/************************************************************************/

/************************************************************************
To differentiate between old and new microbot and upate a global regarding the same.
First check if the ol version is present by send the command to read the device ID if present you will recevie a
appropriate value if not the I2C is stuck so have  to come out of the loop using a timeout and repeat
the same for the other version of micro bit , we use this only by reading the magentometer.
************************************************************************/
static uint8_t read_who_am_i()
{
	  //Read the accel_mag who_am_i register on the micro_bit V1.3
	  uint8_t reg_mag[1]   =  { MAG_REG_WHO_AM_I };
		uint8_t reg_ls[1]   =   { LS_MAG_REG_WHO_AM_I };
		uint8_t who_am_i[1];
		uint8_t accl_mag_id = 0;
		uint8_t count_timeout = 0;
		uint8_t next_step = true;
	  ret_code_t err_code;

		//Read axis
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, reg_mag, 1, true); //Write to the magnetometer of TX
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false)
		{
			count_timeout++;
			if(count_timeout > TIMEOUT_I2C)                   //Time out
			{
				 reset_i2c();                                   //Reset I2C
				 count_timeout = 0;
			   next_step = false;
				 m_xfer_done = true;
			}
			nrf_delay_ms(5);
		}
		//If we have written successfully
    if(next_step == true)
		{
			m_xfer_done = false;
			err_code = nrf_drv_twi_rx(&m_twi, MAG_ADDR, who_am_i, LENGTH_READ_WHO_AM_I);
			APP_ERROR_CHECK(err_code);
			while (m_xfer_done == false);
		}
	  count_timeout = 0;
	  next_step = true;
		
		//Now verify the if the ID mathces
		if(who_am_i[0] == MAG_WHO_AM_I_VALUE)
		{
			 accl_mag_id = 1;
		}
		else
		{
			//Read axis
			m_xfer_done = false;
			err_code = nrf_drv_twi_tx(&m_twi,LS_MAG_ADDR, reg_ls, 1, true);
			APP_ERROR_CHECK(err_code);
			
			while (m_xfer_done == false)
			{
				nrf_delay_ms(5);
				count_timeout++;
				if(count_timeout > TIMEOUT_I2C)
				{
					 reset_i2c();
					 count_timeout = 0;
					 next_step = false;
				}
			}
			if(next_step == true)
			{
				
				m_xfer_done = false;
				err_code = nrf_drv_twi_rx(&m_twi, LS_MAG_ADDR, who_am_i, LENGTH_READ_WHO_AM_I);
				APP_ERROR_CHECK(err_code);
				while (m_xfer_done == false);
			}
			if(who_am_i[0] == LS_MAG_WHO_AM_I_VALUE)
			{
				accl_mag_id = 2;
			}
		}
		
		return accl_mag_id;	 
		//Read the accel_mag who_am_i register on the micro_bit V1.5
}
/************************************************************************/

/************************************************************************
Quick access to know whether the microbit used is a new/old
************************************************************************/
uint8_t check_accel_mag()
{
	uint8_t accl_mag_type = 0;
	uint8_t accl_mag_found = false;
	while(accl_mag_found == false)
	{
		accl_mag_type = read_who_am_i();
		if((accl_mag_type == NXP) || (accl_mag_type == LS))
		{
			accl_mag_found = true;
		}
	}
	
	return accl_mag_type; 
}
/************************************************************************/


/************************************************************************
Read sensor data magentometer
************************************************************************/
void check_read_sensor_data_mag()
{
	switch(accl_mag_chip)
	{
		case LS:
			read_sensor_data_mag_ls();
			return;
		case NXP:
			read_sensor_data_mag();
			return;
		default:
			return;
	 }
}



/************************************************************************
Configure interrupt pins for new accel
************************************************************************/
void set_interrupt_pins_mma_ls()
{
	//Set pinin for MMA-- interrupt--1
	nrf_gpio_cfg_input(ACCL_INT_1_PIN_LS,NRF_GPIO_PIN_PULLDOWN);
}

/************************************************************************
Configure interrupt pins for new mag
************************************************************************/
void set_interrupt_pins_mag_ls()
{
	//Set pinin for MAG-- interrupt--1
	nrf_gpio_cfg_input(MAG_INT_1_PIN_LS,NRF_GPIO_PIN_PULLDOWN);
}


/************************************************************************
Configure interrupt pins for old accelerometer
************************************************************************/
void set_interrupt_pins_mma()
{
	
	//Set pinin for MMA-- interrupt--1
	//set pinin for MMA-- interrupt--2
	nrf_gpio_cfg_input(ACCL_INT_1_PIN,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(ACCL_INT_2_PIN,NRF_GPIO_PIN_PULLDOWN);
	
}

/************************************************************************
Configure interrupt pins for old magentometer
************************************************************************/
void set_interrupt_pins_mag()
{
	//Set pinin for MAG-- interrupt--1
	nrf_gpio_cfg_input(MAG_INT_1_PIN,NRF_GPIO_PIN_PULLDOWN);
}



/************************************************************************/
void init_mma_mag()
{
		set_interrupt_pins_mma();
	  set_interrupt_pins_mag();
}

/************************************************************************/
void init_mma_mag_ls()
{
		set_interrupt_pins_mma_ls();
	  set_interrupt_pins_mag_ls();
}

/************************************************************************/
void init_buttons()
{
   	//Initialize the input buttons
	  nrf_gpio_cfg_input(BUTTONA_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(BUTTONB_PIN,NRF_GPIO_PIN_NOPULL);		
}
/************************************************************************/


/************************************************************************/
void read_buttons()
{
	  volatile uint8_t buttona =  0;
	  volatile uint8_t buttonb = 0;
		buttona = nrf_gpio_pin_read(BUTTONA_PIN);
	  buttona = buttona << 4;
	  buttonb = nrf_gpio_pin_read(BUTTONB_PIN);
	  buttonb = buttonb << 5;
	  input_micro_packet[3] =   (input_micro_packet[3] & ~0x30 )  | calibrate_feedback | buttona | buttonb;
}
/************************************************************************/

/************************************************************************/

void verify_data_packet()
{
	switch(accl_mag_chip)
	{
		case LS:
			read_mag_packet_ls();
		  read_mma_packet_ls();
			break;
		case NXP:
			read_mma_packet();
	    read_mag_packet();
			break;
		default:
			break;
	}
	read_buttons();
}
/************************************************************************/


/************************************************************************/
void read_data_packet()
{
	switch(accl_mag_chip)
	{
		case LS:
			read_sensor_data_mma_ls();
			read_sensor_data_mag_ls();
			break;
		case NXP:
			read_sensor_data_mma();
			read_sensor_data_mag();
			break;
		default:
			break;
		
	}
	verify_data_packet();
}
/************************************************************************/





/************************************************************************/
void  init_microbit_sensors(void)
{
	  //Button A and Button B on the micro::Bit
	  init_buttons();
	  //Initialize the I2C 
	  twi_init();
		//LED_micro_control(CORRECT_LEDARRAY,BRIGHTNESS);
	  accl_mag_chip = check_accel_mag();
		//LED_micro_control(CORRECT_LEDARRAY,BRIGHTNESS);
	  //Support for two different chips
		switch( accl_mag_chip )
		{
			case LS:
				//initialize the IO  pins for the interrupts
				init_mma_mag_ls();
			  //Set up the Accelerometer for the LS part
			  MMA_set_mode_ls();
			  //Set up the Magnetometer for the LS part
				MAG_set_mode_ls();
				return;
			case NXP:
				//Set I2C and interrupt pins not really used
				init_mma_mag();
				//Initialization I2C initialization
				MAG_set_mode();
				MMA_set_mode();
				//extension_set_mode();
				return;
			default:
				return;
		}
	  
}
/************************************************************************/


/** @} */

/************************************************************************
To test the extension port with a tri color sensor
************************************************************************/
/*
static void extension_set_mode(void)
{
	
	ret_code_t err_code;
	nrf_delay_ms(1000);
	//MAG
	uint8_t ext_reg_1[2]   = {0x07, 0x09};
	
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, EXT_ADDR, ext_reg_1, sizeof(ext_reg_1), false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);
	
	
  uint8_t reg[1]   = { EXT_REG_HW_LSB };
	//Read axis
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, EXT_ADDR, reg, 1, true);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, EXT_ADDR, read_axis_mma , LENGTH_AXIS_DATA);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}
*/
