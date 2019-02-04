/************************************************************************/
/*
*Author    : Raghunathreddy Jangam
*Last Edit : 8/7/2018
*Decription: Connection and disconnection sounds 
*/
/************************************************************************/

/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include "nrf_delay.h"
#include "HummingbirdBitBuzzer.h"
#include "HummingbirdBitGlobal.h"
#include "HummingbirdBitBLEControl.h"
#include "HummingbirdBitNaming.h"
/************************************************************************/

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
extern uint8_t prev_state;
extern uint8_t sound_effect;
/************************************************************************/

/************************************************************************/
//Credit to Allison & Bryce for composing this tune
void buzzer_bluetooth_connection(void)
{
	
	  buzzer_HB_control(3039,100);   //e4
	  nrf_delay_ms(100);
	  buzzer_HB_control(1912,100);   //c5
		nrf_delay_ms(100);
	  buzzer_HB_control(1703,100);   //d5
		nrf_delay_ms(100);
	  buzzer_HB_control(1351,100);   //g5
		nrf_delay_ms(100);
}
/************************************************************************/

/************************************************************************/
void buzzer_bluetooth_disconnection(void)
{
	  buzzer_HB_control(1702,100);   //d5
	  nrf_delay_ms(100);
	  buzzer_HB_control(2024,100);   //b4
	  nrf_delay_ms(100);
	  buzzer_HB_control(2551,100);   //g4
	  nrf_delay_ms(100);
	  buzzer_HB_control(3816,100);   //c4
	  nrf_delay_ms(100);

}	
/************************************************************************/

/************************************************************************/
//Check if the sound effect needs to be used based on connection and disconnection.
void check_sound()
{
	//if(prev_state == HUMMINGBIRDBIT)
	//{
		if( sound_effect > 0)
		{
			switch (sound_effect)
			{ 
				case SOUND_CONNECTION:
					buzzer_bluetooth_connection();                         //Connection to the bluetooth sound
					break;
				case SOUND_DISCONNECTION:
					buzzer_bluetooth_disconnection();                      //Disconnection to the bluetooth sound
					break;
				default: 
					break;
			}
			update_name_disconnect();
			sound_effect = 0;
			
		}
	//}
}
/************************************************************************/

