/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : Reads the values collected from UART service and then
							 pushes the values to SAMD through SPI.
*/
/************************************************************************/
#include <stdio.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "micro_bit_sensors.h"
#include "microbit_LEDS.h"
#include "f_save.h"

/************************************************************************/
#define CORRECT_LEDARRAY						0x022a200
#define WRONG_LEDARRAY				  		0x1151151
#define MAG_REG_OFFSET_X_MSB				0x09
#define LENGTH_OFFSET_DATA          6
#define MIN_THRESHOLD_CALIB         700
#define EXT_REG_HW_LSB 							0x00
#define EXT_ADDR       							0x49
/************************************************************************/
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static uint8_t calibrate_feedback = 0; 
static int16_t calibrate_mag[300][3];
/************************************************************************/
/* TWI instance. */
static uint8_t read_axis_mma[LENGTH_AXIS_DATA] ;
static uint8_t read_shake_mma[LENGTH_SHAKE_DATA];
static uint8_t read_axis_mag[LENGTH_AXIS_DATA] ;
static uint8_t read_temp_mag[LENGTH_TEMP_DATA];
volatile static int16_t offset_x,offset_y,offset_z  = 0;
static int16_t scale_x, scale_y,scale_z = 0;
static bool calib_mag = false;
uint8_t read_offset_mag[LENGTH_OFFSET_DATA] ;

/************************************************************************/
void send_I2C_mag(uint8_t const* data,uint8_t length)
{
		ret_code_t err_code;
	  m_xfer_done    = false;
		err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, data, length , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/************************************************************************
Initial setting of the magnetometer values , receive 16 bits. 
************************************************************************/
void MAG_set_mode(void)
{
	  ret_code_t err_code;
	  
	  //MAG
		uint8_t mag_reg_1[2]   = {MAG_REG_CTRL1, STANDBY_MODE_MAG};
		uint8_t mag_reg_2[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
		uint8_t mag_reg_3[2]   = {MAG_REG_CTRL1, ACTIVE_MODE_MAG};
		
		send_I2C_mag(mag_reg_1,sizeof(mag_reg_1));
		send_I2C_mag(mag_reg_2,sizeof(mag_reg_2));
		send_I2C_mag(mag_reg_3,sizeof(mag_reg_3));
		
	
}
/************************************************************************/


/************************************************************************
To test the extension port with a tri color sensor
************************************************************************/
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


/************************************************************************/
void send_I2C_acc(uint8_t const* data,uint8_t length)
{
		ret_code_t err_code;
	  m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MMA_ADDR, data, sizeof(data), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


/************************************************************************
Initializing Accelerometer
************************************************************************/
void MMA_set_mode(void)
{
	
    ret_code_t err_code;
	  
	  //MMA
	  uint8_t accl_reg_1[2]   = {MMA_REG_CTRL1, STANDBY_MODE};
		uint8_t accl_reg_2[2]   = {MMA_REG_CTRL2, CTRL2_VALUE};
		uint8_t accl_reg_3[2]   = {MMA_REG_CTRL3, CTRL3_VALUE};
		uint8_t accl_reg_4[2]   = {MMA_REG_CTRL4, CTRL4_VALUE};
		uint8_t accl_reg_5[2]   = {MMA_REG_CTRL5, CTRL5_VALUE};
		uint8_t accl_reg_6[2]   = {MMA_REG_XYZ, XYZ_VALUE};
		uint8_t accl_reg_7[2]   = {MMA_REG_SHAKE, SHAKE_VALUE};
		uint8_t accl_reg_8[2]   = {MMA_REG_SHAKE_THRES, SHAKE_THRES_VALUE};
		uint8_t accl_reg_9[2]   = {MMA_REG_SHAKE_COUNT, SHAKE_COUNT_VALUE};
		uint8_t accl_reg_10[2]  = {MMA_REG_CTRL1, ACTIVE_MODE};
	  
		send_I2C_acc( accl_reg_1,sizeof(accl_reg_1));
		send_I2C_acc( accl_reg_2,sizeof(accl_reg_2));
		send_I2C_acc( accl_reg_3,sizeof(accl_reg_3));
		send_I2C_acc( accl_reg_4,sizeof(accl_reg_4));
		send_I2C_acc( accl_reg_5,sizeof(accl_reg_5));
		send_I2C_acc( accl_reg_6,sizeof(accl_reg_6));
		send_I2C_acc( accl_reg_7,sizeof(accl_reg_7));
		send_I2C_acc( accl_reg_8,sizeof(accl_reg_8));
		send_I2C_acc( accl_reg_9,sizeof(accl_reg_9));
		send_I2C_acc( accl_reg_10,sizeof(accl_reg_10));
		
		
}

/************************************************************************/
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
* @brief TWI initialization.
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
static void read_sensor_data_mma()
{		

		uint8_t reg[2]   = { MMA_REG_STATUS , STANDBY_MODE };
	  ret_code_t err_code;
		
		//Read axis
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MMA_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		
		
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MMA_ADDR, read_axis_mma, LENGTH_AXIS_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
		//Shake Status
		reg[0]   = MMA_REG_SHAKE_STATUS;
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MMA_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		
		
		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MMA_ADDR, read_shake_mma, LENGTH_SHAKE_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		

}



/************************************************************************/
static void read_sensor_data_mag()
{		

		uint8_t reg[2]   = { MAG_REG_STATUS , STANDBY_MODE };
	  ret_code_t err_code;
		
		//Read axis
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAG_ADDR, read_axis_mag, LENGTH_AXIS_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
	

		reg[0]   = MAG_REG_TEMP;
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		 m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAG_ADDR, read_temp_mag, LENGTH_TEMP_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
}

/************************************************************************/
void read_verify_offset_mag()
{		

		uint8_t reg[2]   = { MAG_REG_OFFSET_X_MSB , STANDBY_MODE };
	  ret_code_t err_code;
		//Verify offset
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAG_ADDR, read_offset_mag, LENGTH_OFFSET_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		nrf_delay_ms(5);
}



/************************************************************************/
void set_interrupt_pins_mma()
{
	
	//Set pinin for MMA-- interrupt--1
	//set pinin for MMA-- interrupt--2
	nrf_gpio_cfg_input(ACCL_INT_1_PIN,NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(ACCL_INT_2_PIN,NRF_GPIO_PIN_PULLDOWN);
	
}

/************************************************************************/
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
    twi_init();
}

/************************************************************************/
void init_buttons()
{

   	//Initialize the input buttons
	  nrf_gpio_cfg_input(BUTTONA_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(BUTTONB_PIN,NRF_GPIO_PIN_NOPULL);	
		
}

/************************************************************************/
void read_mma_packet()
{
	  static  uint8_t prev_input_micro_packet[19];
	  uint8_t i =0;
		if((read_axis_mma[0]&0x08))
		{		
				for(i=0;i<3;i++)
				{
					input_micro_packet[i] = read_axis_mma[i*2 + 1];
				}
				
				if((read_shake_mma[0]&0xAA) > 1)
				{
					input_micro_packet[3] = 1;
				}
				else
				{
					input_micro_packet[3] = 0;
				}
				
				for(i=0;i<=3;i++)
				{
					prev_input_micro_packet[i] = input_micro_packet[i];
				}
				
				
		}
		else
		{
			  for(i=0;i<=3;i++)
				{
					input_micro_packet[i] = prev_input_micro_packet[i];
				}
			
		}
}
/************************************************************************/

void read_mag_packet()
{	
	static  uint8_t prev_input_micro_packet[19];
	uint8_t i =0;
	if((read_axis_mag[0]&0x08))
		{		
				for(i=1;i<=6;i++)
				{
					input_micro_packet[i+3]  =  read_axis_mag[i];
				}
		
			  //Update the previous
				
				for(i=4;i<=9;i++)
				{
					prev_input_micro_packet[i] = input_micro_packet[i];
				}
				
		}
		else
		{
				for(i=4;i<=9;i++)
				{
					input_micro_packet[i] = prev_input_micro_packet[i];
				}
		}
}
/************************************************************************/
void read_buttons()
{
	  uint8_t buttona =  0;
	  uint8_t buttonb = 0;
		buttona = nrf_gpio_pin_read(BUTTONA_PIN);
	  buttona = buttona << 4;
	  buttonb = nrf_gpio_pin_read(BUTTONB_PIN);
	  buttonb = buttonb << 5;
	  input_micro_packet[3] =   input_micro_packet[3]  | calibrate_feedback | buttona | buttonb;
}
/************************************************************************/
void verify_data_packet()
{
	  read_mma_packet();
	  read_mag_packet();
	  read_buttons();	
}
/************************************************************************/
void read_data_packet()
{
		read_sensor_data_mma();
    read_sensor_data_mag();
	  verify_data_packet();
}

/************************************************************************
Writes offset od X,Y,Z into the magnetometer after calibration or at the 
startup
************************************************************************/
void write_offset(uint8_t* offset)
{
	  uint32_t err_code = 0;
	  uint8_t mag_reg_2[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	  uint8_t accl_reg_offset[7] = {MAG_REG_OFFSET_X_MSB , offset[0] , offset[1] , offset[2], offset[3] , offset[4] , offset[5]  }; 
	  m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, accl_reg_offset, sizeof(accl_reg_offset), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, mag_reg_2, sizeof(mag_reg_2), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}
    
/************************************************************************
Compass calibration increments the LED periodically 50msec and gets the
magmetometer values in X,Y,Z and finds the max,min value of the received value 
.If the difference between X,Y,Z min and X,Y,Z max is greater than 700 
and half amount of time is passed then tick mark is displayed and if not cross mark
is displayed.
************************************************************************/
void calibrate_compass()
{
	
	uint16_t i,j = 0;
	uint8_t brightness 				 					 = 0x55;  //currently not using
	volatile uint32_t led_array 				 = 0x00000000;
	
	uint32_t loop_count 			 = 300;
	uint32_t no_of_leds 			 = 25;
	uint32_t led_update 			 = loop_count/no_of_leds;
	
	uint32_t err_code = 0;
	static uint8_t offset[6] ;
	bool result_calib = false;
	
	int16_t min_x =  32765;
	int16_t min_y =  32765;
	int16_t min_z =  32765;
	int16_t max_x = -32765;
	int16_t max_y = -32765;
	int16_t max_z = -32765;
	
	
	uint8_t mag_reg_1[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE_2};
	uint8_t mag_reg_2[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	
	for(i=0;i<6;i++)
	{
		offset[i]  = 0;
	}
	
	write_offset(offset);
	nrf_delay_ms(100);
	read_verify_offset_mag();
	nrf_delay_ms(50);
	//Indication of start with LED array

	for(i=0;i<loop_count;i++)
	{
		read_sensor_data_mag();
		for(j=0;j<3;j++)
		{
				calibrate_mag[i][j]   =  read_axis_mag[j*2 + 1];
			  calibrate_mag[i][j]   =  calibrate_mag[i][j]<<8;
				calibrate_mag[i][j]  |=  read_axis_mag[j*2 + 2];
		}
		nrf_delay_ms(50);
	  
		if( i%(loop_count/no_of_leds) == 0 )
		{
			led_array  = led_array | 0x01;
			LED_micro_control(led_array,brightness);
			led_array  = led_array << 1;
		}
		
		//X-- axis
		if(min_x > calibrate_mag[i][0])
		{
			min_x  = calibrate_mag[i][0];
		}
		if(max_x < calibrate_mag[i][0])
		{
			max_x  = calibrate_mag[i][0];
		}
		
		//Y-- axis
		if(min_y > calibrate_mag[i][1])
		{
			min_y  = calibrate_mag[i][1];
		}
		if(max_y < calibrate_mag[i][1])
		{
			max_y  = calibrate_mag[i][1];
		}
		//Z-- axis
		if(min_z > calibrate_mag[i][2])
		{
			min_z  = calibrate_mag[i][2];
		}
		if(max_z < calibrate_mag[i][2])
		{
			max_z  = calibrate_mag[i][2];
		}
		
		if((abs(max_x - min_x) > MIN_THRESHOLD_CALIB) && (abs(max_x - min_x) > MIN_THRESHOLD_CALIB) && (abs(max_x - min_x) > MIN_THRESHOLD_CALIB) )
		{
			
			if(i>(loop_count/2))
			{
				result_calib= true;
				break;
			}
			
		}
	}
	
	//Tick Mark
	if(result_calib == true)
	{
		calibrate_feedback = 0x04;
		//Hard iron offset
		offset_x = ((max_x + min_x)/2);
		offset_y = ((max_y + min_y)/2);
		offset_z = ((max_z + min_z)/2);
		
		offset_x = offset_x <<1; 
		offset_y = offset_y <<1; 
		offset_z = offset_z <<1; 
		
		//write offsets into the magentometer
		//x
		offset[0]  = (offset_x & 0xFF00) >> 8;
		offset[1]  =  offset_x & 0x00FE;
		
		//y
		offset[2]  = (offset_y & 0xFF00) >> 8;
		offset[3]  = offset_y & 0x00FE;
		
		
		//z
		offset[4]  = (offset_z & 0xFF00) >> 8;
		offset[5]  = offset_z & 0x00FE;
		
		
		//Check if the values lie in the range 
		
		read_verify_offset_mag();
		write_offset(offset);
		nrf_delay_ms(100);
		write_calib_values(offset);
		read_verify_offset_mag();
		
		LED_micro_control(CORRECT_LEDARRAY,brightness);
	  nrf_delay_ms(1000);
	  LED_micro_control(0x00000000,brightness);
	}
	else
	{
		calibrate_feedback = 0x08;
		LED_micro_control(WRONG_LEDARRAY,brightness);
	  nrf_delay_ms(1000);
	  LED_micro_control(0x00000000,brightness);
	}
}



/************************************************************************/
void  init_microbit_sensors(void)
{
	 
	  //Button A and Button B on the micro::Bit
	  init_buttons();
	  //Set I2C and interrupt pins not really used
		init_mma_mag();
		
	  //Initialization I2C initialization
	  MAG_set_mode();
    MMA_set_mode();
	  //extension_set_mode();
    
}

/** @} */