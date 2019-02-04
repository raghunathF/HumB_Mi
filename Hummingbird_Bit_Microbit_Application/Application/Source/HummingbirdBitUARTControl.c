/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains the source code of how the UART data is controlled		|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include "ble_advertising.h"
#include "app_uart.h"
#include "nrf_delay.h"

#include "HummingbirdBitLEDS.h"
#include "HummingbirdBitBuzzer.h"
#include "HummingbirdBitSPIMaster.h"
#include "HummingbirdBitLEDArray.h"
#include "HummingbirdBitSensors.h"
#include "HummingbirdBitGlobal.h"
#include "HummingbirdBitBuzzer.h"
#include "HummingbirdBitUART.h"
#include "HummingbirdBitSound.h"
#include "HummingbirdBitMagnetometer.h"
#include "HummingbirdBitCalibration.h"
/************************************************************************/

/************************************************************************/
/******************     Variables      **********************************/
/************************************************************************/
extern uint8_t UARTRingBuffer[255];
extern volatile uint8_t UARTTailPointer ;
extern volatile uint8_t UARTHeadPointer ; 
extern uint8_t    sensor_outputs[20];
extern uint8_t currentConnectionMode;
extern bool flash_mb ;
extern uint8_t  sound_effect;
/************************************************************************/

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
#define LED1_COMMAND																				0xC0
#define LED2_COMMAND																				0xC1
#define LED3_COMMAND																				0xC2

#define ORB1_COMMAND																				0xC4
#define ORB2_COMMAND																				0xC5

#define SERVO1_COMMAND																			0xC6
#define SERVO2_COMMAND																			0xC7
#define SERVO3_COMMAND																			0xC8
#define SERVO4_COMMAND																			0xC9
#define SETALL_COMMAND                                      0xCA
#define STOPALL_COMMAND                                     0xCB

#define CALIBRATE_COMMAND																		'C'
#define BUZZER_COMMAND																			'B'
#define SETLED_COMMAND                                      'l'
#define READ_COMMAND                                        'R'
#define START_CONNECTION_COMMAND														'o'
#define STOP_CONNECTION_COMMAND															'c'
#define SENSOR_COMMAND																			's'
#define ACCL_COMMAND																				'a'
#define MAG_COMMAND																					'm'
#define FIRMWARE_COMMAND																		'f'
#define COMPLETE_COMMAND																		'C'


#define LEN_ORB_COMMAND																			4
#define LEN_SERVO_COMMAND																		2
#define LEN_CALIBRATE_COMMAND																1
#define LEN_BUZZER_COMMAND																	5
#define LEN_SETALL_COMMAND																	20
#define LEN_LEDARRAY_COMMAND																20
#define LEN_READ_COMMAND																		2
#define MAX_DATA_LEN                                        25
#define LEN_LRS_COMMAND																			4
#define LEN_SENSOR_ACC																			4
#define LEN_SENSOR_MAG																			6
#define LEN_SETLED_COMMAND																	20
#define LEN_SENSOR_SAMD																			4
#define LEN_READ_DATA																				6
#define LEN_SENSOR_ALL																			14
#define LEN_READ_ALL																				14
#define LENGTH_SETALL_SPI        														13
#define MICRO_FIRMWARE_VERSION  														0x01
#define SYMBOL_STRING_MASK       														0x80
#define SYMBOL 									 														0x80
#define LENGTH_STRING_MASK       														0x1F
#define SCROLL_FLASH_MASK       														0x40
#define FLASH                    														0x40
/************************************************************************/

/************************************************************************
Read the remaining bytes from the ring buffer
************************************************************************/
void receiveRemainingBytes(uint8_t* buffer , uint8_t length)
{
	uint8_t i =0;
	for(i=0;i<length;i++)
	{
			buffer[i] = UARTRingBuffer[UARTTailPointer];
			UARTTailPointer++;
	}

}
/************************************************************************/

/************************************************************************
Stop All to stop the hummingbird buzzer , LED Array , LEDs , humming bird Bit 
************************************************************************/
void stopAll()
{
	  static uint8_t stopAllBuffer[4] = {0xCB,0xFF,0xFF,0xFF};
		buzzer_HB_control(0,0);
		stop_LEDarray_display();
		LED_HB_control(LED2 ,0);
		LED_HB_control(LED3 ,0);
		transfer_data(LEN_LRS_COMMAND,stopAllBuffer);
}
/************************************************************************/

/************************************************************************
Read the firmware version of SAMD
************************************************************************/
uint8_t readFirmwareSAMD()
{
	uint8_t samdFirmwareVersion = 0;
	static uint8_t temp[4]= { 0x8C, 0xFF, 0xFF, 0xFF};
	transfer_data(LEN_LRS_COMMAND,temp);
  read_sensor_HB();
	samdFirmwareVersion = sensor_outputs[3];
	return samdFirmwareVersion;
}
/************************************************************************/

/************************************************************************
Check if the device connected is just micro bit or is it hummingbird bit 
and find the version number.
************************************************************************/
void sendInitialData()
{
	uint8_t samdFirmwareVersion = 0;
	uint8_t HummingBitState     = 0;
	uint8_t hardware_version_number = 0;
		uint8_t initialBuffer[4] = {hardware_version_number ,MICRO_FIRMWARE_VERSION, samdFirmwareVersion, HummingBitState};

	hardware_version_number = find_version();
  samdFirmwareVersion = readFirmwareSAMD();
	if(samdFirmwareVersion != 255)
	{
		HummingBitState  = true;
	}
	else
	{
		HummingBitState  = false;
	}
	sendUART(initialBuffer , LEN_LRS_COMMAND);
}
/************************************************************************/

/************************************************************************
Set buzzer to appropriate Us(Frequency) and Ms(Beats)
************************************************************************/
void setBuzzer(uint8_t* tempBuffer)
{
	  volatile uint16_t tempUS,tempMS = 0;
		tempUS              = tempBuffer[0];
		tempUS              = tempUS << 8;
		tempUS             |= tempBuffer[1];
		tempMS              = tempBuffer[2];
		tempMS              = tempMS << 8;
		tempMS             |= tempBuffer[3];
		if(tempMS > 0)
		{
			buzzer_HB_control(tempUS , tempMS);
		}
}

/************************************************************************
Flash or display the symbol 
************************************************************************/
void setLEDArray(uint8_t* receiveBuffer)
{
		uint8_t i =0;
		uint32_t temp_1,temp_2 = 0;
	  uint8_t  length_string = 0;
		static 	 uint8_t flashData[20];
	  //Check if a symbol is requested 
		if((receiveBuffer[1] & SYMBOL_STRING_MASK) == SYMBOL )
		{
				for(i = 2; i<6;i++)
				{
						temp_1 = receiveBuffer[i];
						temp_2   = temp_2 | temp_1<<(24-8*(i-2));
				}
				if(flash_mb == true)
				{
							flash_mb  = false;
							stop_flashing_timer();
				}
				LED_micro_control(temp_2,0x55);
		 }
		 else 
		 {
			  //Flash letters
				 if(receiveBuffer[1] == 0x00)
				 {
						stop_LEDarray_display();
				 }
				 else
				 {
					  //Find the length
						length_string = receiveBuffer[1] & LENGTH_STRING_MASK;
						if(length_string > 0)
						{
								if((receiveBuffer[1] & SCROLL_FLASH_MASK) == FLASH )
								{
									//LED_Flash
									for(i=0;i<length_string;i++)
									{
										 flashData[i]= receiveBuffer[i+2];
									}	
									update_flash_array(flashData,length_string);
									start_flashing_string();			
								 }
							}
					}
			}
}


/************************************************************************
See whats received in ring buffer if you receive something read the
entire command
************************************************************************/
void check_UART()
{
	static uint8_t receiveBuffer[20];

	uint8_t firstByte = 0;
	uint8_t i = 0;
	uint32_t err_code = 0;
	volatile uint8_t portNumber = 0;
	volatile uint8_t intensity = 0;
	uint8_t  temp_transmit[4] = {0x55,0x66,0x77,0x88} ;
  static uint8_t temp_receive[20] ;

	if(UARTHeadPointer != UARTTailPointer)
	{
		firstByte = UARTRingBuffer[UARTTailPointer];
		//UARTTailPointer++;
		switch(firstByte)
		{
			case LED1_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case LED2_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				portNumber = firstByte & 0x0F;
				intensity  = receiveBuffer[1] ;
			  portNumber = portNumber + '1';
				LED_HB_control(portNumber , intensity);
				break;
			case LED3_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				portNumber = firstByte & 0x0F;
				intensity  = receiveBuffer[1] ;
			  portNumber = portNumber + '1';
				LED_HB_control(portNumber , intensity);
				break;
			case ORB1_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case ORB2_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case SERVO1_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case SERVO2_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case SERVO3_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case SERVO4_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			case CALIBRATE_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_CALIBRATE_COMMAND);
				calibrate_compass();
				break;
			case BUZZER_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_BUZZER_COMMAND);
			  setBuzzer(&receiveBuffer[1]);
				break;
			case	SETALL_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_SETALL_COMMAND);
				//SPI transfer to SAMD
				transfer_data(LENGTH_SETALL_SPI, receiveBuffer); 
				//Microbit peripherals
			  LED_HB_control(LED2 ,receiveBuffer[13]);
				LED_HB_control(LED3 ,receiveBuffer[14]);
				setBuzzer(&receiveBuffer[15]);
				break;
			case	SETLED_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_SETLED_COMMAND);
			  setLEDArray(receiveBuffer);
				break;
		  case  STOPALL_COMMAND:
				stopAll();
				receiveRemainingBytes(receiveBuffer,LEN_LRS_COMMAND);
				transfer_data(LEN_LRS_COMMAND,receiveBuffer);
				break;
			
			case READ_COMMAND:
				receiveRemainingBytes(receiveBuffer,LEN_READ_COMMAND);
				switch(receiveBuffer[1])
				{
					case START_CONNECTION_COMMAND:
						
					   //stop advertising
						if(currentConnectionMode != UART_MODE)
						{
							currentConnectionMode = UART_MODE;
							//stop all
							stopAll();
							stop_advertising_flashing = true;
					    sound_effect  = SOUND_DISCONNECTION;
							sd_ble_gap_adv_stop();
							//Send info about the version number and check if the what state the device is connected in.
							sendInitialData();
							//buzzer_bluetooth_connection();
							//Changing states would require power cycle.
						}
						break;
					case STOP_CONNECTION_COMMAND:
						//stop all
						
						//start advertising
						if(currentConnectionMode != ADVERTISING_MODE)
						{
							stopAll();
					    start_advertising_flashing = true;
					    sound_effect  = SOUND_CONNECTION;
							currentConnectionMode = ADVERTISING_MODE;
							err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
							APP_ERROR_CHECK(err_code);
						}
						//Send info regarding successfull reception
						break;
					case SENSOR_COMMAND:
						//transfer_data(LEN_LRS_COMMAND, temp);
					  read_sensor_HB();
						//transfer_data(LEN_LRS_COMMAND,temp_transmit);
					  for(i=0;i<LEN_SENSOR_SAMD;i++)
						{
							temp_receive[i] = sensor_outputs[i];
						}
						temp_receive[4] = 0;
						temp_receive[5] = 0;
						sendUART(temp_receive, LEN_READ_DATA);
						break;
					case ACCL_COMMAND:
						read_sensor_MB();
						//transfer_data(LEN_LRS_COMMAND,temp_transmit);
					  for(i=4;i<LEN_SENSOR_ACC+4;i++)
						{
							temp_receive[i-4] = sensor_outputs[i];
						}
						temp_receive[4] = 0;
						temp_receive[5] = 0;
						sendUART(temp_receive, LEN_READ_DATA);
						break;
					case MAG_COMMAND:
						read_sensor_MB();
						//transfer_data(LEN_LRS_COMMAND,temp_transmit);
					  for(i=8;i<LEN_SENSOR_MAG+8;i++)
						{
							temp_receive[i] = sensor_outputs[i];
						}
						sendUART(temp_receive, LEN_READ_DATA);
						break;
					case FIRMWARE_COMMAND:
						sendInitialData();
						break;
					case COMPLETE_COMMAND:
						read_sensor_HB();
						read_sensor_MB();
						for(i=0;i<LEN_SENSOR_ALL;i++)
						{
							temp_receive[i] = sensor_outputs[i];
						}
						sendUART(temp_receive, LEN_READ_ALL);
						break;
					default:
						break;
					
				}
			default:
				break;
		}
		if(UARTHeadPointer == UARTTailPointer)
		{
			 UARTHeadPointer = 0;
			 UARTTailPointer  = 0;
		}
	}
	else
	{	
			if(currentConnectionMode == UART_MODE)
			{
				nrf_delay_ms(1);
				transfer_data(LEN_LRS_COMMAND, temp_transmit);
				read_data_packet();
				read_sensor_HB();
				read_sensor_MB();
				nrf_delay_ms(1);
			}
	}
	
}
