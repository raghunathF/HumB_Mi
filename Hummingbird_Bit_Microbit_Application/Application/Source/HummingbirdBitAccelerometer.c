/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains the source code for initializing all the modules.		|
*|                  |         Does a regular check for presence of data through BLE or UART. 					|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/* Please reffer to the License folder in the repository of the source folder */
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "HummingbirdBitSensors.h"
/************************************************************************/

/************************************************************************/
/******************     Variables       **********************************/
/************************************************************************/
static uint8_t read_axis_mma[LENGTH_AXIS_DATA] ;
static uint8_t read_shake_mma[LENGTH_SHAKE_DATA];

extern const nrf_drv_twi_t m_twi;
extern volatile bool m_xfer_done;
/************************************************************************/

void convert_axis_mma_ls()
{
	int8_t temp = 0;
	
	temp = read_axis_mma[1];
	temp = -temp ;
	read_axis_mma[1] = temp;
}

/************************************************************************/
void read_mma_packet_ls()
{
	  
	  uint8_t i =0;	
		convert_axis_mma_ls();
		for(i=0;i<3;i++)
		{
			input_micro_packet[i] = read_axis_mma[i*2 + 1];
		}
		
		if((read_shake_mma[0]&0x0A) > 0)
		{
			input_micro_packet[3] = 1;
		}
		else
		{
			input_micro_packet[3] = 0;
		}
}

/************************************************************************/
void send_I2C_acc_ls(uint8_t const* data,uint8_t length)
{
	  volatile uint8_t test = 0; 
		ret_code_t err_code;
	  m_xfer_done = false;
	  test = length;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MMA_ADDR, data, length, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


/************************************************************************/
void send_I2C_acc(uint8_t const* data,uint8_t length)
{
		ret_code_t err_code;
	  m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, MMA_ADDR, data, length, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}


/************************************************************************
Initializing Accelerometer
************************************************************************/
//Later Updates : Write at once
void MMA_set_mode(void)
{
	
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


/************************************************************************
Initializing Accelerometer
************************************************************************/
//Later Updates : Write at once
void MMA_set_mode_ls(void)
{
	  //MMA
		uint8_t accl_reg_2[2]   = {MMA_REG_CTRL4_LS, CTRL4_VALUE_LS};
		uint8_t accl_reg_3[2]   = {MMA_REG_SHAKE_THS_LS, SHAKE_TH_LS};
		uint8_t accl_reg_4[2]   = {MMA_REG_CTRL1_LS, ACTIVE_MODE_LS};
		uint8_t accl_reg_5[2]   = {MMA_REG_SHAKE_CONFIG_LS, XYZ_SHAKE_ENABLE_LS};
		
		
		send_I2C_acc_ls( accl_reg_4,sizeof(accl_reg_4));
		send_I2C_acc_ls( accl_reg_2,sizeof(accl_reg_2));
		send_I2C_acc_ls( accl_reg_3,sizeof(accl_reg_3));
		send_I2C_acc_ls( accl_reg_5,sizeof(accl_reg_5));
		
}

/************************************************************************/
void read_sensor_data_mma()
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
void read_sensor_data_mma_ls()
{		
		uint8_t reg[2]   = { MMA_REG_STATUS_LS , STANDBY_MODE };
	  ret_code_t err_code;
		
		//Read axis
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MMA_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
		
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LS_MMA_ADDR, read_axis_mma, LENGTH_AXIS_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
		
		
		//Shake Status
		//reg[0]   = MMA_REG_SHAKE_STATUS_LS;
		reg[0]     =  MMA_REG_SHAKE_STATUS_LS;  
		m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LS_MMA_ADDR, reg, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
				
		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LS_MMA_ADDR, read_shake_mma, LENGTH_SHAKE_DATA);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);
}
/************************************************************************/

/************************************************************************
Read the X,Y,Z and status register values. Based on status register value
you can know if they value collected is corrupt.
************************************************************************/
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
