/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains source code for controlling the IO functionalites.		|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/* Please reffer to the License folder in the repository of the source folder */
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include "app_pwm.h"
#include "nrf_drv_gpiote.h"
#include "nrf_adc.h"
#include "nrf_drv_adc.h"
#include "app_timer.h"
#include "nrf_delay.h"

#include "HummingbirdBitPinout.h"
#include "HummingbirdBitLEDS.h"
#include "HummingbirdBitBuzzer.h"
#include "HummingbirdBitIO.h"
/************************************************************************/

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
extern bool 	buzzer_progress ;
static const  uint8_t PIN_NO[NO_MICRO_PINS]       = 	PIN_NOS;
static const  uint8_t PIN_ADC[NO_MICRO_PINS]      = 	PIN_ADCS;
/************************************************************************/

typedef struct microbit_pin
{
	bool    	 input;
	bool       pwm_led;
	bool       pwm_buzzer;
	uint8_t    pwm_value;
	uint16_t   buzz_freq_micro;
	uint16_t    buzz_time_milli;
	uint8_t    pwm_number;
	uint8_t    channel_number;
	uint16_t   current_state;
}microbit_pin;

microbit_pin MB_pin[3];

/************************************************************************
Configure pin to defaults using 
************************************************************************/
void pin_default(uint8_t pin_no)
{
		MB_pin[pin_no].pwm_led   		= false ;
		MB_pin[pin_no].input 				= false ;
	  MB_pin[pin_no].pwm_buzzer   = false ;
}

/************************************************************************/
void reset_PWM()
{
			buzzer_progress = false;
			app_pwm1_reset();
			app_pwm2_reset();
}

/************************************************************************/
void PWM_three_LED()
{
		volatile uint32_t value;
	  volatile ret_code_t err_code;
		PWM1_2LEDS(PIN_MICROBIT_1 , PIN_MICROBIT_2 ,MB_pin[1].pwm_value , MB_pin[2].pwm_value );
		PWM2_1LED(MB_pin[0].pwm_value);
	
	
}


/************************************************************************/
void PWM_two_LED(uint8_t pin1 , uint8_t pin2 , uint8_t pwm_value_1 , uint8_t pwm_value_2)
{
		PWM1_2LEDS(pin1 , pin2 , pwm_value_1 , pwm_value_2 );
}


/************************************************************************/
void PWM_one_LED(uint8_t pin1 , uint8_t pwm_value)
{
		PWM1_1LED(pin1 , pwm_value );
}



/************************************************************************/
void change_dutycycle(uint8_t pwm_module_number , uint8_t channel_number ,uint8_t pwm_value )
{
	 switch (pwm_module_number)
	 {
		 case PWM1_MODULE:
			 pwm1_change_dutycycle(channel_number , pwm_value);
			 break;
		 case PWM2_MODULE:
			 pwm2_change_dutycycle(channel_number , pwm_value);
			 break;
		 default:
			 break;
	 }
		
}


/************************************************************************/
void config_adc()
{

    nrf_adc_config_t adc_config = NRF_ADC_CONFIG_DEFAULT;
		
		// Configure ADC
    adc_config.resolution = NRF_ADC_CONFIG_RES_8BIT;
	  nrf_adc_configure(&adc_config);
	 
}
/************************************************************************/

/***********************************************************************
Remembering the previous state of the pin as the pin is expected to change at any point, this 
would lead resetting PWM modules and remebering which PWM the pin is associated with is important
***********************************************************************/
void microbit_io_pwm_main(uint16_t* next_pin_state, uint16_t frequency , uint16_t time_ms)
{
	  volatile uint8_t					mode_current				= 0;
		volatile uint8_t					mode_next    				= 0;
		volatile uint8_t					count_input  				= 0;
	  bool 											led_change_value  	= false; 
		uint8_t 									pin_pwm_led[3];
		uint8_t 									pin_pwm_value[3];
	  uint8_t 									pin_pwm_index[3];	
		uint8_t 									i            				= 0;
		uint8_t 									mode_change[4]			={0x00,0x00,0x00,0x00};
		uint8_t 									count_pwm_led 			= 0;
		uint8_t 									count_pwm_buzzer 		= 0;
		uint8_t 									increment         	= 0;
		uint16_t 									value_current 			= 0;
		uint16_t 									value_next    			= 0;
		
		//Read the IO command determine what modes have changed and how many
		for(i=0;i<3;i++)
		{
			mode_current   = (uint8_t)((MB_pin[i].current_state & MODE_MASK)>>8) ;
			mode_next      = (uint8_t)((next_pin_state[i] & 0x0300)>>8);
			value_current  = MB_pin[i].current_state & VALUE_MASK;
			value_next     = next_pin_state[i] & VALUE_MASK;
			//If current state is not equal to previous stae i.e when who some requested for change in pin functionalities.
			if(mode_current != mode_next)
			{
				pin_default(i);
				mode_change[mode_current] = true;
				mode_change[mode_next] 		= true;
				//Update the data structre accordingto the data parsed
				switch(mode_next)
				{
					case PIN_INPUT:
						MB_pin[i].input        = true;
						count_input++;
						break;
					case PIN_PWM_LED:
						MB_pin[i].pwm_value 	 = next_pin_state[i] & 0x00FF;
						MB_pin[i].pwm_led      = true;
						
						break;
					case PIN_PWM_BUZZER:
						if(i == 0)
						{
							MB_pin[i].buzz_freq_micro 	 		= frequency;		
							MB_pin[i].buzz_time_milli 	 		= time_ms;	
							MB_pin[i].pwm_buzzer     			  = true;
						}
						break;
					default:
						break;
				}
			}
			//Though not same state if you are using buzzer you should update the buzzer immediately and udate th data structure
			else if(MB_pin[i].pwm_buzzer == true)
			{
						if(i == 0)
						{
							MB_pin[i].buzz_freq_micro 	 		= frequency;		
							MB_pin[i].buzz_time_milli 	 		= time_ms;	
							if(MB_pin[i].buzz_time_milli > 0)
							{	
								buzzer_HB_control(MB_pin[0].buzz_freq_micro,MB_pin[0].buzz_time_milli);
							}
						}
			}
			//If the mode is smae and but the value associated with it is not.
			else if(value_current != value_next)
			{
				if(MB_pin[i].pwm_led == true)
				{
					MB_pin[i].pwm_value 		= value_next;
					led_change_value = true;
				}
			}
			//Store the state
			MB_pin[i].current_state         = next_pin_state[i];
		}
		
		//if input mode set is true
		if(mode_change[PIN_INPUT] == true )
		{
			nrf_drv_adc_uninit();
			NRF_ADC->CONFIG =  NRF_ADC->CONFIG & ~ADC_CONFIG_PSEL_Msk;
			config_adc();
		}
		//PWM value change
		if((led_change_value == true) && (mode_change[PIN_PWM_LED_CHANGE] == false) &&(mode_change[PIN_PWM_BUZZER_CHANGE] == false))
		{
			for(i=0;i<3;i++)
			{
					if( MB_pin[i].pwm_led == true )
					{		
						//MB_pin[i].current_state = next_pin_state[i];
						change_dutycycle(MB_pin[i].pwm_number  , MB_pin[i].channel_number , MB_pin[i].pwm_value);
					}
			}
		}
		//check if the modes have changed
		if((mode_change[PIN_PWM_LED_CHANGE] == true)||(mode_change[PIN_PWM_BUZZER_CHANGE] == true))
		{
			//Reset the PWM module
			reset_PWM();
			
			//count the number of LEDS pins and buxxer pins
			for(i=0;i<NO_MICRO_PINS;i++)
			{
				if(MB_pin[i].pwm_buzzer == true)
				{
						count_pwm_buzzer++;
				}
				else if(MB_pin[i].pwm_led == true)
				{
					  pin_pwm_led[increment] = PIN_NO[i];
					  pin_pwm_value[increment] = MB_pin[i].pwm_value;
					  pin_pwm_index[increment] = i;
						increment++;
						count_pwm_led++;
				}
			}
			
			//If there is a change in LED mode and buzzer mode
			if((count_pwm_led + count_pwm_buzzer)>0)
			{
				//If all the three pins are used for LEDS
				if(count_pwm_led == 3)
				{
						PWM_three_LED();
						MB_pin[0].pwm_number 		 = 		 PWM2_MODULE;
						MB_pin[0].channel_number =     CHANNEL0;
						MB_pin[1].pwm_number 		 = 		 PWM1_MODULE;
						MB_pin[1].channel_number =     CHANNEL0;
					  MB_pin[2].pwm_number 		 = 		 PWM1_MODULE;
						MB_pin[2].channel_number =     CHANNEL1;
				}
				else 
				{
					//If one pin is used for buzzer
					if(count_pwm_buzzer>0)
					{
						buzzer_HB_control(MB_pin[0].buzz_freq_micro,MB_pin[0].buzz_time_milli);
						MB_pin[0].pwm_number 		 = 		 PWM2_MODULE;
						MB_pin[0].channel_number =     CHANNEL0;
					}
					//If two pins are used for LEDS
					if(count_pwm_led == 2)
					{
						PWM_two_LED(pin_pwm_led[0],pin_pwm_led[1], pin_pwm_value[0] , pin_pwm_value[1]);
						MB_pin[pin_pwm_index[0]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[0]].channel_number = CHANNEL0;
						MB_pin[pin_pwm_index[1]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[1]].channel_number = CHANNEL1;
					}
					//If one pin is used for PWM
					else if(count_pwm_led == 1)
					{
						PWM_one_LED(pin_pwm_led[0], pin_pwm_value[0]);
						MB_pin[pin_pwm_index[0]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[0]].channel_number = CHANNEL0;
					}
				}
			}
			
		}
		
}


/***********************************************************************
Read the sensors attached to the microbit 
***********************************************************************/
void read_sensors()
{
	 volatile uint8_t	i =0; 
	 for(i=0;i<NO_MICRO_PINS;i++)
	 {
		 if(MB_pin[i].input == true)
		 {
			 sensor_outputs[i] = nrf_adc_convert_single(PIN_ADC[i]);
		 }
		 else
		 {
			 sensor_outputs[i] = 255; //if no sensor is attached
		 }
   }
}

/***********************************************************************
Reset the PWM and congigire ADC . Make sure all the software initializers
are made to default
***********************************************************************/
void microbit_pwm_init()
{
	uint8_t i =0;
	reset_PWM();
	nrf_gpio_cfg_output(BUZZER_PIN);
	nrf_gpio_pin_clear(BUZZER_PIN);
	config_adc();
	for(i=0;i<3;i++)
	{
		pin_default(i);
		MB_pin[i].pwm_led 				  = true ;
		MB_pin[i].pwm_value       = 0;
		MB_pin[i].current_state   =     0x0000000000;
	} 
	MB_pin[0].pwm_number 		 = 		 PWM2_MODULE;
	MB_pin[0].channel_number =     CHANNEL0;
	MB_pin[1].pwm_number 		 = 		 PWM1_MODULE;
	MB_pin[1].channel_number =     CHANNEL0;
	MB_pin[2].pwm_number 		 = 		 PWM1_MODULE;
	MB_pin[2].channel_number =     CHANNEL1;
	
	PWM_three_LED();
}
/************************************************************************/

