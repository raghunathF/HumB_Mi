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
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ble_gap.h"

#include "HummingbirdBitLEDMapping.h"
#include "HummingbirdBitRudeWords.h"
#include "HummingbirdBitBLEControl.h"
/************************************************************************/

extern uint8_t initials_name[4];
extern uint8_t INITIAL_NAME[2];
extern char 	DEVICE_NAME [20];
/************************************************************************/
/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
char convert_ascii(uint8_t input)
{
	char output;
	if(input <=9)
	{
		output = input + 0x30;
	}
	else
	{
		output = input + 0x37;
	}
	return output;
}
/************************************************************************/


/************************************************************************/
/**@brief 		Function for setting the device name  it either starts with HB or MB 
 *            the next five characters are taken from the last five hex values of MAC address
 *						followed by Null.
 */

void set_devicename_array()
{
		ble_gap_addr_t 					mac;
    sd_ble_gap_address_get(&mac);
		
	  DEVICE_NAME[0] = INITIAL_NAME[0];
		DEVICE_NAME[1] = INITIAL_NAME[1];
	  DEVICE_NAME[2] = convert_ascii((mac.addr[2]&0x0F));
	
	
		DEVICE_NAME[3] = (mac.addr[1]&0xF0);
		DEVICE_NAME[3] = convert_ascii(DEVICE_NAME[3]>>4);
		
	
		DEVICE_NAME[4] = convert_ascii(mac.addr[1]&0x0F);
		DEVICE_NAME[5] = (mac.addr[0]& 0xF0);
		DEVICE_NAME[5] = convert_ascii(DEVICE_NAME[5]>>4);
		
		DEVICE_NAME[6] = convert_ascii(mac.addr[0]&0x0F);
		DEVICE_NAME[7] = '\0';
}
/************************************************************************/


/************************************************************************/
void update_name_disconnect()
{
	  check_update_name_disconnect();
	  set_devicename_array();
	  ble_gap_conn_sec_mode_t 	sec_mode;	
	  sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *) DEVICE_NAME,strlen(DEVICE_NAME));	
}
/************************************************************************/


/************************************************************************/
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
bool rude_word_check()
{
	int i=0;
	for(i=0;i<sizeof(first_letter);i++)
	{
		if(initials_name[0] == first_letter[i] )
		{
			if(initials_name[1] == second_letter[i])
			{
				if(initials_name[2] == third_letter[i] )
				{
					return true;
				}
			}
		}
	}
	return false;
}
/************************************************************************/

/************************************************************************/
// Use the Mac address to find the first three letters of the fancy name to print on the LED screen
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
void getInitials_fancyName()
{
	  bool rude_word = true;
	  ble_gap_addr_t 					mac;
    sd_ble_gap_address_get(&mac);
		uint32_t temp = 0;
	  uint16_t tempCount =0;
	  uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0; 	
    temp  |= (uint32_t)(mac.addr[2]&0x0F) << 16;	
	  temp  |= (uint32_t)mac.addr[1] << 8;
	  temp  |= (uint32_t)mac.addr[0];
	
		mod16  =  temp%16;
	  top8   =  temp%256;
	  mid6   =  (temp/256)%64;
    bot6   =	(temp/256)/64;
		
		initials_name[0] = name_first[top8 + mod16];
		initials_name[1] = name_second[mid6 + mod16];
		initials_name[2] = name_third[bot6 + mod16];
	
		while(rude_word == true)
	  {
			rude_word = rude_word_check();
			if(rude_word == true)
			{
				initials_name[0] = name_first[top8 + mod16];
				initials_name[1] = name_second[(mid6 + mod16 + tempCount)%512];
				initials_name[2] = name_third[bot6 + mod16];
				
			}
			tempCount++;
		}
}
/************************************************************************/
