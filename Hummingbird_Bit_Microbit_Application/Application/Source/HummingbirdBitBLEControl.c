/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : Reads the values collected from UART service and then
							 if necessary pushes the values to SAMD through SPI.
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include "nrf_delay.h"
#include "ble_nus.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "app_timer.h"

#include "HummingbirdBitBLEControl.h"
#include "HummingbirdBitSPIMaster.h"
#include "HummingbirdBitLEDS.h"
#include "HummingbirdBitSensors.h"
#include "HummingbirdBitBuzzer.h"
#include "HummingbirdBitLEDArray.h"
#include "HummingbirdBitIO.h"
#include "HummingbirdBitMagnetometer.h"
#include "HummingbirdBitCalibration.h"
#include "HummingbirdBitGlobal.h"
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
APP_TIMER_DEF(broadcast_timer_id);
extern ble_nus_t   m_nus;
extern uint8_t currentConnectionMode;
static uint8_t samd_firmware_version = 0;
volatile bool sensor_data_ready = false;
uint8_t flash_data[20]; 
/************************************************************************/

/************************************************************************
Send fake firmware number , this is used so that old app partialy works 
with the HummingbirdBit mimicing the Hummingbird Duo
************************************************************************/
static void send_fake_firmware_number()
{
	
	uint32_t err_code;
	uint8_t fake_firmware_number[5] = {0x03,0x00,0x02,0x03,0x61}; //Mimic the version number of Hummingbird Duo
	err_code = ble_nus_string_send(&m_nus, fake_firmware_number, 5);
	if (err_code != NRF_ERROR_INVALID_STATE)
	{
			APP_ERROR_CHECK(err_code);
	}
}


/************************************************************************
Send firmware version number of the micro bit , SAMD and also harware version 
number of microbit.
************************************************************************/
static void send_firmware_number()
{
	
	uint32_t err_code;
	uint8_t hardware_version_number = 0;
	hardware_version_number = find_version(); 
	uint8_t firmware_number[3] = {hardware_version_number ,MICRO_FIRMWARE_VERSION,samd_firmware_version};
	err_code = ble_nus_string_send(&m_nus, firmware_number, 3);
	if (err_code != NRF_ERROR_INVALID_STATE)
	{
			APP_ERROR_CHECK(err_code);
	}
}


/************************************************************************
This is where sensor data is sent has a notification to the central device
************************************************************************/
static void transmit_ble_data()
{
	ble_nus_string_send(&m_nus, sensor_outputs, SENSOR_SEND_LENGTH);
}

/************************************************************************
Read the firmware version from the SAMD in the initial state and store it.
************************************************************************/
void read_firmware_SAMD()
{
	static uint8_t SAMDFirmwareCommand[4]= { 0x8C, 0xFF, 0xFF, 0xFF};
	nrf_delay_ms(50);
	transfer_data(LENGTH_OTHER_SPI,SAMDFirmwareCommand);
	nrf_delay_ms(10);
  read_sensor_HB();
	samd_firmware_version = sensor_outputs[3];
}


/************************************************************************
To check if the micro bit is connected to the HummingBird Bit using the 
sensor 4 value which is connected to the power of the HummingbirdBit. This
is during the initialization. 
************************************************************************/
void check_update_name()
{
	static uint8_t temp[13]= { 0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}; //Initializing Array
	//To send values after SAMD is out of bootloader mode
	nrf_delay_ms(2000);   //2 seconds delay
	transfer_data(LENGTH_SETALL_SPI , temp);
	nrf_delay_ms(100);
	read_sensor_HB();
	if((sensor_outputs[3]>LOWER_LIMIT_BATTERY_MB_HB) && (sensor_outputs[3] < UPPER_LIMIT_BATTERY_MB_HB))
	{
		INITIAL_NAME[0] = 'B';
		INITIAL_NAME[1] = 'B';   
		prev_state      =  HUMMINGBIRD_BIT ;
		hummingbit_pwm_init();
	}
	else
	{
		prev_state = MICRO_BIT;
		microbit_pwm_init(); 
	}
	read_firmware_SAMD();
}

//Future Change
/************************************************************************
To check if the micro bit is connected to the HummingBird Bit using the 
sensor 4 value which is connected to the power of the HummingbirdBit. This is
a regular check.
************************************************************************/
void check_update_name_disconnect()
{
	static uint8_t temp[13]= { 0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}; //Initializing Array
	transfer_data(LENGTH_SETALL_SPI , temp);
	nrf_delay_ms(100);
	read_sensor_HB();
	if((sensor_outputs[3]>0x20) && (sensor_outputs[3] < 0xFF))
	{
		INITIAL_NAME[0] = 'B';
		INITIAL_NAME[1] = 'B';
		prev_state      =  0 ;
		hummingbit_pwm_init();
	}
	else
	{
		prev_state = 1;
		microbit_pwm_init(); 
	}
	
	read_firmware_SAMD();
}


/************************************************************************
Notifications are sent in regular periods using this callback.
************************************************************************/
void broadcast_timer_handler()
{
	if(sensor_data_ready)
	{
		transmit_ble_data();
		sensor_data_ready = false;
	}	
}



/************************************************************************
Broadcast timer is initiated
************************************************************************/
void init_broadcast_timer(void)
{
	 volatile uint32_t err_code = 0;
	 err_code = app_timer_create(&broadcast_timer_id,APP_TIMER_MODE_REPEATED,broadcast_timer_handler);
   APP_ERROR_CHECK(err_code);
	 
}

/************************************************************************
Broadcast timer will be called every 30msecs
************************************************************************/
void broadcast_start()
{
	volatile uint32_t err_code = 0;
	err_code = app_timer_start(broadcast_timer_id, APP_TIMER_TICKS(30, 0), NULL);
	APP_ERROR_CHECK(err_code);
	broadcast_flag = true;
}

/************************************************************************
Broadcast timer will stop sending notifiactionsat ~30 msec
************************************************************************/
void broadcast_stop()
{
	volatile uint32_t err_code = 0;
	err_code = app_timer_stop(broadcast_timer_id);
	APP_ERROR_CHECK(err_code);
	broadcast_flag = false;
}


/************************************************************************
UART SPI bridge helps to decode the protocol and assigns the command to
the respective function.
************************************************************************/
void uart_spi_bridge()
{
	uint32_t temp_1 = 0;
	uint32_t temp   = 0;
	uint32_t time_us = 0;
	uint8_t i,k = 0;
	uint8_t length_string = 0;
	uint8_t led2 = 0;
	uint8_t led3 = 0;
	uint8_t length = 0;
	uint16_t time_ms = 0;
	uint16_t frequency =0;
	uint16_t  mode = 0;
	static uint8_t current_command[20];
	static uint8_t sensor_command[5] = {0xAA,0xBB,0xCC,0xDD,0xEE};
	uint16_t next_states[3];
	static uint32_t sensor_data_output_sum = 0;
	static uint16_t count_MB_HB_check = 0;
	static  uint8_t current_state = 0;

	
	//Check if the tail pointer and head pointer match for the ring buffer
	if(tail_pointer != head_pointer)
	{
		 //Store the value of data received 
		 length	= buffer_data[head_pointer];
		 k=0;
		 for(i= head_pointer+1 ;i<= head_pointer + length;i++)
		 {
			current_command[k] = buffer_data[i];
			k++;
		 }
		 head_pointer = head_pointer + length + 1;
		 switch(current_command[0])
		 {
			 //Setting all outputs at once
			 case SETALL_SPI:
					//Take what microbit needs
					//LED2 , LED3 are controlled by microbit
					led2  						= current_command[13] ;
					led3 							= current_command[14] ;
					
					time_us           = current_command[15];
					time_us           = time_us << 8;
					time_us          |= current_command[16];
				 
					time_ms           = current_command[17];
					time_ms           = time_ms << 8;
					time_ms          |= current_command[18];
					
					nrf_delay_ms(1);
			 
					//SPI transfer to SAMD
					transfer_data(LENGTH_SETALL_SPI,current_command);
					
					//Control the LEDs
					LED_HB_control(LED2 ,led2);
					LED_HB_control(LED3 ,led3);
					
					//Control Buffer 
					if(time_ms > 0)
					{
							buzzer_HB_control(time_us , time_ms);
					}
					break;
			//Changing the inputs mode based on the requested command
			case MICRO_IO:
				
				frequency     	  =  ((uint16_t)current_command[1]<<8 | current_command[2]);
				time_ms           =  ((uint16_t)current_command[3]<<8 | current_command[5]);
			  for(i=0;i<3;i++)
				{
					mode = current_command[4] & (0x03 << (4 - 2*i)) ; 
					mode = mode << (4 + 2*i);
					next_states[i] = mode | current_command[5+i] ;
				}
				microbit_io_pwm_main(next_states,frequency,time_ms);
				break;
			//Start/Stop broadcasting
			case BROADCAST:
				if(current_command[1] == 'g')
				{
					 //Start timer
					 broadcast_start();
					
				}
				else if(current_command[1] == 's')
				{
					 //Stop Timer
					 broadcast_stop();
				}
				break;
			//Either Flash letters on the LED Screen or make a symbol on the screen
			case SET_LEDARRAY:
				//Flash Symbol
				if((current_command[1] & SYMBOL_STRING_MASK) == SYMBOL )
				{
					//Data parsing
					for(i = 2; i<6;i++)
					{
						temp_1 = current_command[i];
						temp   = temp | temp_1<<(24-8*(i-2));
					}
					if(flash_mb == true)
					{
						flash_mb  = false;
						stop_flashing_timer();
					}
					LED_micro_control(temp,0x55);
				}
				else 
				{
					if(	current_command[1] == 0x00)
					{
							stop_LEDarray_display();
					}
					else
					{
						length_string = current_command[1] & LENGTH_STRING_MASK;
						if(length_string > 0)
						{
								if((current_command[1] & SCROLL_FLASH_MASK) == FLASH )
								{
									//LED_Flash
									for(i=0;i<length_string;i++)
									{
										 flash_data[i]= current_command[i+2];
									}	
									update_flash_array(flash_data,length_string);
									start_flashing_string();			
								}
						 }
					}
				}
				break;
			//Control LED2
			case  SET_LED_2:
				LED_HB_control(LED2 ,current_command[1]);
				break;
			//Control LED3
			case SET_LED_3:
				LED_HB_control(LED3 ,current_command[1]);
				break;
			//Switch off LEDS , LED Array , stop buxxer . Send Stop command to SAMD
			case STOP_ALL:
				//broadcast_flag = false;
			  stop_LEDarray_display();
				if(prev_state == HUMMINGBIRD_BIT)
				{
					buzzer_HB_control(0,0);
					LED_HB_control(LED2 ,0);
					LED_HB_control(LED3 ,0);
					nrf_delay_ms(1);
					transfer_data(LENGTH_OTHER_SPI,current_command);
				}
				break;
			//Control buzzer
			case SET_BUZZER:
				temp              = current_command[1];
				temp              = temp << 8;
				temp             |= current_command[2];
				time_ms           = current_command[3];
				time_ms           = time_ms << 8;
				time_ms          |= current_command[4];
				buzzer_HB_control(temp , time_ms);
				break;
			//Send firmware version number to BLE transmit.
			case  SET_FIRMWARE:
			  send_firmware_number();
			  break;
			//Start Calibraion process
			case SET_CALIBRATE:
				calibrate_compass();
				break;
			//Any other command only send it to SAMD
			default:
				nrf_delay_ms(1);
				transfer_data(LENGTH_OTHER_SPI,current_command);
				break;
		 }
		 //resetting ring buffer to avoid data overbound issues
		 if(head_pointer == tail_pointer)
		 {
			 head_pointer = 0;
			 tail_pointer = 0;
		 }
			 
	}
	
	else 
	{
		//check if the device is in broadcast mode
		if(broadcast_flag == true)
		{
			nrf_delay_ms(10);
			transfer_data(LENGTH_OTHER_SPI ,sensor_command);
			read_data_packet();
			//If the device is in Microbit Pin 0 , Pin 1 , Pin 2 are made the sensors
			if(prev_state == MICRO_BIT)
			{
					read_sensors();
			}
			else
			{
					read_sensor_HB();
			}
			//Read accel , magnetometer , button data
			read_sensor_MB();
			sensor_data_ready = true;
		}
		else
		{
			//Check if there is a change in the device that is connected
			if(currentConnectionMode == ADVERTISING_MODE )
			{
				//nrf_delay_ms(50);
				transfer_data(LENGTH_OTHER_SPI ,sensor_command);
				read_data_packet();
				read_sensor_HB();
				read_sensor_MB();
				//sensor_outputs[3]= filter_voltage_output((uint16_t)sensor_outputs[3]);
				sensor_data_output_sum += sensor_outputs[3];
				count_MB_HB_check++;
				if(count_MB_HB_check == 100)
				{
					sensor_data_output_sum = sensor_data_output_sum/100;
					count_MB_HB_check = 0;
					if(sensor_data_output_sum > 250)
					{
							current_state = 1;// Microbit
					}
					else if((sensor_data_output_sum > 80) && (sensor_data_output_sum < 230) )
					{
							current_state = 0;//Hummingbit
					}
					if(current_state != prev_state)
					{
							LED_micro_control(REFRESH_SYMBOL,0x55);
							nrf_delay_ms(250);
							sd_nvic_SystemReset();
					}
					sensor_data_output_sum = 0;
				}
			
			 }
		   			 
		}
	}		
}
