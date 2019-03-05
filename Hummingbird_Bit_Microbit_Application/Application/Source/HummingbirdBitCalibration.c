/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains the source code for calibration of magnetometer .		|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/* Please reffer to the License folder in the repository of the source folder */
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include  <stdint.h>
#include  <stdbool.h>
#include  "nrf_delay.h"

#include  "HummingbirdBitSensors.h"
#include  "HummingbirdBitMagnetometer.h"
#include  "HummingbirdBitLEDArray.h"
#include  "HummingbirdBitGlobal.h"
#include  "HummingbirdBitFlash.h"
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
static int16_t calibrate_mag[300][3];
extern uint8_t read_axis_mag[LENGTH_AXIS_DATA] ;
extern uint8_t calibrate_feedback;
extern uint8_t accl_mag_chip;

extern volatile bool    calibrationFlag;
/************************************************************************/

/************************************************************************
Compass calibration increments the LED periodically 50msec and gets the
magmetometer values in X,Y,Z and finds the max,min value of the received value.
If the difference between X,Y,Z min and X,Y,Z max is greater than 700 
and half amount of time is passed then tick mark is displayed and if not cross mark
is displayed.
************************************************************************/
uint8_t calibrate_compass()
{
	volatile uint32_t led_array 				 = 0x00000000;
	
	//uint32_t err_code 									 = 0;
	uint16_t i,j 												 = 0;
	uint8_t brightness 				 					 = 0x55;  //currently not using
	uint8_t calibrationStatus            = FAILURE;
	
	int16_t min_x					=     32765;
	int16_t min_y					=     32765;
	int16_t min_z					=     32765;
	int16_t max_x					=    -32765;
	int16_t max_y					=    -32765;
	int16_t max_z					=    -32765;
	uint32_t loop_count 			 	= 	 300;
	uint32_t no_of_leds 			 	= 	 25;
	//uint32_t led_update 			 	= 	loop_count/no_of_leds;
	
	
	static uint8_t offset[6] ;
	static int16_t offset_x,offset_y,offset_z  = 0;
	bool result_calib = false;
	
	//uint8_t mag_reg_1[2]   		= {MAG_REG_CTRL2, MAG_CTRL2_VALUE_2};
	//uint8_t mag_reg_2[2]   		= {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	
	//uint8_t mag_reg_1_ls[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE_2};
	//uint8_t mag_reg_2_ls[2]   = {MAG_REG_CTRL2, MAG_CTRL2_VALUE};
	
	for(i=0;i<6;i++)
	{
		offset[i]  = 0;
	}
	
	check_write_offset(offset);
	nrf_delay_ms(50);
	
	
	//Indication of start with LED array
	calibrationFlag = true;
	for(i=0;i<loop_count;i++)
	{
		check_read_sensor_data_mag();
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
		
		if(calibrationFlag == false)
		{
			calibrationStatus  = BROKEN;
			result_calib= false;
			break;
		}
		
		if(accl_mag_chip == NXP)
		{
			//Check if the maximum settings have been reached
			if((abs(max_x - min_x) > MIN_THRESHOLD_CALIB) && (abs(max_y - min_y) > MIN_THRESHOLD_CALIB) && (abs(max_z - min_z) > MIN_THRESHOLD_CALIB) )
			{
				
				if(i>(loop_count/2))
				{
					result_calib= true;
					break;
				}
				
			}
		}
		else
		{
			//Check if the maximum settings have been reached
			if((abs(max_x - min_x) > MIN_THRESHOLD_CALIB_LS) && (abs(max_y - min_y) > MIN_THRESHOLD_CALIB_LS) && (abs(max_z - min_z) > MIN_THRESHOLD_CALIB_LS) )
			{
				if(i>(loop_count/2))
				{
					result_calib= true;
					break;
				}
			 }
		 }
				
			
	}
	
	//Tick Mark
	if(result_calib == true)
	{
		calibrationStatus  = SUCCESS;
		calibrate_feedback = 0x04;
		//Hard iron offset
		offset_x = ((max_x + min_x)/2);
		offset_y = ((max_y + min_y)/2);
		offset_z = ((max_z + min_z)/2);
		
		switch(accl_mag_chip)
		{
			case LS:
				//x
				offset[1]  = (offset_x & 0xFF00) >> 8;
				offset[0]  =  offset_x & 0x00FF;
				//y
				offset[3]  = (offset_y & 0xFF00) >> 8;
				offset[2]  = offset_y & 0x00FF;
				//z
				offset[5]  = (offset_z & 0xFF00) >> 8;
				offset[4]  = offset_z & 0x00FF;
				break;
			
			case NXP:	
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
				break;
			
			default:	
				break;
		}
		
		//Check if the values lie in the range 
		//read_verify_offset_mag();
		check_write_offset(offset);
		nrf_delay_ms(50);
		write_calib_values(offset);
		
		/*
		write_offset(offset);
		nrf_delay_ms(100);
		*/
		
		//read_verify_offset_mag();
		
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
	calibrationFlag = false;
	
	return calibrationStatus;

}
