/************************************************************************/
/************************************************************************/
/************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "HummingbirdBitGlobal.h"
#include "HummingbirdBitSensors.h"

extern uint8_t accl_mag_chip;
extern nrf_drv_twi_t m_twi;
extern volatile bool m_xfer_done;

extern uint8_t read_axis_mag[LENGTH_AXIS_DATA] ;
static uint8_t read_temp_mag[LENGTH_TEMP_DATA];
static uint8_t read_offset_mag[LENGTH_OFFSET_DATA] ;

/************************************************************************/
uint8_t find_version()
{
	uint8_t version= 0;
	version = accl_mag_chip;
	return version;
}

/************************************************************************
Writes offset of X,Y,Z into the magnetometer after calibration or at the 
startup
************************************************************************/
void write_offset(uint8_t* offset)
{
	  uint32_t err_code = 0;
	  uint8_t mag_reg_2[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	  uint8_t mag_reg_offset[7] = {MAG_REG_OFFSET_X_MSB , offset[0] , offset[1] , offset[2], offset[3] , offset[4] , offset[5]  }; 
	  m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, mag_reg_offset, sizeof(mag_reg_offset), false);
    APP_ERROR_CHECK(err_code);	
    while (m_xfer_done == false);
		/*
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, mag_reg_2, sizeof(mag_reg_2), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
	  */
}


/************************************************************************
Writes offset od X,Y,Z into the magnetometer after calibration or at the 
startup
************************************************************************/
void write_offset_ls(uint8_t* offset)
{
	  uint32_t err_code = 0;
	  //change_MSB_LSB(offset);
	  //uint8_t mag_reg_2_ls[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	  uint8_t mag_reg_offset[7] = {MAG_REG_OFFSET_X_LSB_LS , offset[0] , offset[1] , offset[2], offset[3] , offset[4] , offset[5]  }; 
	  m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MAG_ADDR, mag_reg_offset, sizeof(mag_reg_offset), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		//change_MSB_LSB(offset);
		/*
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, mag_reg_2_ls, sizeof(mag_reg_2_ls), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		*/
}

/************************************************************************/
void convert_axis_scale()
{
	int16_t temp = 0;
	uint8_t i =0;
	for(i=0;i<3;i++)
	{
		temp = (read_axis_mag[2*i+1]<<8) & 0xFF00;
		temp = temp | (read_axis_mag[2*i + 2] & 0x00FF);
		temp = (temp * 3)/2 ;
		if(i != 1)
		{
			temp = -1 * temp;
		}
		read_axis_mag[2*i + 1] = (temp & 0xFF00) >> 8;
		read_axis_mag[2*i + 2] = (temp & 0x00FF);
	}
	
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
void read_verify_offset_mag_ls()
{		

		uint8_t reg[2]   = { MAG_REG_OFFSET_X_LSB_LS , STANDBY_MODE };
	  ret_code_t err_code;
		//Verify offset
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LS_MAG_ADDR, read_offset_mag, LENGTH_OFFSET_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		nrf_delay_ms(5);
}

/************************************************************************/
void send_I2C_mag(uint8_t const* data,uint8_t length)
{
		ret_code_t err_code;
	  m_xfer_done    = false;
		err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, data, length , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/************************************************************************/
void send_I2C_mag_ls(uint8_t const* data,uint8_t length)
{
		ret_code_t err_code;
	  m_xfer_done    = false;
		err_code = nrf_drv_twi_tx(&m_twi, LS_MAG_ADDR, data, length , false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


/************************************************************************
Initial setting of the magnetometer values , receive 16 bits. 
************************************************************************/
//Later Updates : Write at once
void MAG_set_mode(void)
{
	  
	  //MAG
		uint8_t mag_reg_1[2]   = {MAG_REG_CTRL1, STANDBY_MODE_MAG};
		uint8_t mag_reg_2[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE}; //Set 10Hz
		uint8_t mag_reg_3[2]   = {MAG_REG_CTRL1, ACTIVE_MODE_MAG};
		
		send_I2C_mag(mag_reg_1,sizeof(mag_reg_1));
		send_I2C_mag(mag_reg_2,sizeof(mag_reg_2));
		send_I2C_mag(mag_reg_3,sizeof(mag_reg_3));
		
}
/************************************************************************/


/************************************************************************
Initial setting of the magnetometer values , receive 16 bits. 
************************************************************************/
//Later Updates : Write at once
void MAG_set_mode_ls(void)
{
	  
	  //MAG
		uint8_t mag_reg_1[2]   = {MAG_CFG_REG_A_LS, CFG_A_VALUE_S};
		uint8_t mag_reg_2[2]   = {MAG_CFG_REG_B_LS, CFG_B_VALUE_S};
		uint8_t mag_reg_3[2]   = {MAG_CFG_REG_C_LS, CFG_C_VALUE_S};
		
		
		send_I2C_mag_ls(mag_reg_1,sizeof(mag_reg_1));
		send_I2C_mag_ls(mag_reg_2,sizeof(mag_reg_2));
		send_I2C_mag_ls(mag_reg_3,sizeof(mag_reg_3));
	
}
/************************************************************************/



/************************************************************************/
void read_sensor_data_mag_ls()
{		

		uint8_t reg[2]   = { MAG_REG_STATUS_LS , STANDBY_MODE };
	  ret_code_t err_code;
		
		//Read axis
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LS_MAG_ADDR, read_axis_mag, LENGTH_AXIS_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		//change_MSB_LSB(read_axis_mag);
		
}


/************************************************************************/
void check_write_offset(uint8_t* offset)
{
	
	switch(accl_mag_chip)
	{
		case LS:
			
			write_offset_ls(offset);
			nrf_delay_ms(100);
			read_verify_offset_mag_ls();
			nrf_delay_ms(50);
			break;
		
		case NXP:
			
			write_offset(offset);
			nrf_delay_ms(100);
			read_verify_offset_mag();
			nrf_delay_ms(50);
			break;
		default:
			break;
	 }

}


/************************************************************************/
 void read_sensor_data_mag()
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
	
		/*
		reg[0]   = MAG_REG_TEMP;
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MAG_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		 m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAG_ADDR, read_temp_mag, LENGTH_TEMP_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		*/
}

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
void read_mag_packet_ls()
{
	uint8_t i =0;
	convert_axis_scale();
	for(i=1;i<=6;i++)
	{
		input_micro_packet[i+3]  =  read_axis_mag[i];
	}
}
