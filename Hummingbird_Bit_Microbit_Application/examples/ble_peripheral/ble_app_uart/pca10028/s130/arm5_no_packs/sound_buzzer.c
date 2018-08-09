/************************************************************************/
/*
*Author    : Raghunathreddy Jangam
*Last Edit : 8/7/2018
*Decription: Connection and disconnection sounds 
*/
/************************************************************************/

#include "buzzer.h"
#include "nrf_delay.h"

//Credit to Allison & Bryce for composing this tune
void buzzer_bluetooth_connection()
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


void buzzer_bluetooth_disconnection()
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