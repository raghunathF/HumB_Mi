/************************************************************************/
/*
*Author    : Raghunathreddy Jangam
*Last Edit : 8/7/2018
*Decription: This driver helps to store the calibrated values of magnetometer 
							in flash 
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "fstorage.h"
#include "app_util_platform.h"

#include "HummingbirdBitSensors.h"
#include "HummingbirdBitMagnetometer.h"
#include "HummingbirdBitFlash.h"
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
static uint8_t fs_callback_flag;
/************************************************************************/


/************************************************************************/
static void fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result)
{
		if (result == FS_SUCCESS)
		{
				fs_callback_flag = 0;
		}
}

/************************************************************************/
FS_REGISTER_CFG(fs_config_t fs_config) =
{
			.callback  = fs_evt_handler, // Function for event callbacks.
			.num_pages = NUM_PAGES,      // Number of physical flash pages required.
	    .priority  = 0xFE            // Priority for flash usage.
};

/************************************************************************/
void init_fstorage()
{
		fs_ret_t ret = fs_init();
}

/************************************************************************
Erase a page with the specific start address
************************************************************************/
void fstorage_erase()
{
		fs_callback_flag = 1;
		fs_erase(&fs_config, fs_config.p_start_addr, 1, NULL);
}

/************************************************************************/
uint32_t fstorage_read(uint32_t addr_no)
{
	  static volatile uint32_t flash_data = 0;
		flash_data = *(fs_config.p_start_addr + addr_no);
	  return flash_data;
}
/************************************************************************/

void fstorage_write(uint32_t* data_output, uint32_t addr_no )
{
	  
	  fs_callback_flag = 1;
		fs_store(&fs_config, fs_config.p_start_addr + addr_no, data_output, 1,NULL);      //Write data to memory address 0x0003F000. Check it with command: nrfjprog --memrd 0x0003F000 --n 16
}
/************************************************************************
If the flash value is read is 0x55555555 , this means that there is calibration
already done.
************************************************************************/
bool check_calibrate()
{
	bool calibrate_flag = false;
	if(fstorage_read(0) == 0x55555555)
	{
		calibrate_flag = true;
	}
	return calibrate_flag;
}
/************************************************************************/

uint16_t* read_calib_values()
{
	static uint16_t offset_values[3];
	uint8_t i =0;
	for(i=1;i<4;i++)
	{
		offset_values[i-1] = fstorage_read(i);
	}
	return offset_values;
}


/************************************************************************
This functions is generally used after calibration is done so that values
can be stored in the flash.
************************************************************************/

void write_calib_values(volatile uint8_t* offset)
{
	uint8_t i =0;
	static uint32_t offset_32[3];
	static uint32_t calibrate_pos = 0x55555555;
	fstorage_erase();
	nrf_delay_ms(10);
	fstorage_write(&calibrate_pos, 0);
	nrf_delay_ms(10);
	for(i=1;i<4;i++)
	{
		offset_32[i-1]     = (uint32_t)offset[2*(i-1)]<<8;
		offset_32[i-1]    |= (uint32_t)offset[(i*2)-1];
		fstorage_write(&offset_32[i-1], i);
		nrf_delay_ms(10);
	}
}

/************************************************************************
Check if the magentometer is already calibrated by looking the values in 
the flash if it is calibrated pick out the values from the flash and write
into the magnetometer offset
************************************************************************/
void start_check_update_calibrate()
{
	uint16_t* offset_values = NULL;
	static uint8_t  offset[6] ;
	uint8_t i =0;
	bool calibrate_flag = false;
	init_fstorage();
	calibrate_flag = check_calibrate();
	if(calibrate_flag == true)
	{
		offset_values = read_calib_values();
		for(i=0;i<3;i++)
		{
			offset[i*2]   =  (offset_values[i] & 0xFF00)>>8;
			offset[i*2+1] =  (offset_values[i] & 0x00FF);
		}
		check_write_offset(offset);
	}
	
}












