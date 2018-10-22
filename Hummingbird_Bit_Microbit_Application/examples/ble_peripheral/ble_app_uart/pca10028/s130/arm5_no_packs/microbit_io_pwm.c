/************************************************************************/
#include "app_pwm.h"
#include "HM_pin.h"
#include <stdint.h>
#include <string.h>
#include "HM_LEDS.h"
#include "nrf_delay.h"
#include "buzzer.h"
#include  "micro_pin.h"
#include "nrf_drv_gpiote.h"
#include "nrf_adc.h"
#include "nrf_drv_adc.h"
#include "app_timer.h"
#include "HM_LEDS.h"
#include "microbit_io_pwm.h"
/************************************************************************/

#define NO_PWM                 0
#define PWM1_CH1_LED           1
#define PWM1_CH1_BUZZER				 2
#define PWM1_CH2_LED					 3
#define PWM1_CH2_BUZZER        4
#define PWM2_CH1_LED           5
#define PWM2_CH1_BUZZER        6
#define PWM2_CH2_LED           7
#define PWM2_CH2_BUZZER        8


#define NO_MICRO_PINS          3

#define MODE_MASK							 0x0300
#define VALUE_MASK						 0x00FF

#define PIN_PWM_LED						0
#define PIN_INPUT							1
#define PIN_PWM_BUZZER				2


#define PIN_PWM_LED_CHANGE    0
#define PIN_PWM_BUZZER_CHANGE 2

#define PWM1_MODULE						1
#define PWM2_MODULE						2
#define CHANNEL0							0
#define CHANNEL1							1

#define ADC_PIN_0							4
#define ADC_PIN_1							3
#define ADC_PIN_2							2

#define PIN_NOS 							 {PIN_MICROBIT_0  ,PIN_MICROBIT_1  ,PIN_MICROBIT_2 }
#define PIN_ADCS							 {NRF_ADC_CONFIG_INPUT_4, NRF_ADC_CONFIG_INPUT_3 , NRF_ADC_CONFIG_INPUT_2}

static const  uint8_t PIN_NO[NO_MICRO_PINS]       = 	PIN_NOS;
static const  uint8_t PIN_ADC[NO_MICRO_PINS]      = 	PIN_ADCS;

extern bool buzzer_progress ;


/************************************************************************/
//APP_PWM_INSTANCE(PWM1,1);
//APP_PWM_INSTANCE(PWM2,2);


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

/************************************************************************/
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
		uint32_t value;
	  ret_code_t err_code;
		PWM1_2LEDS(pin1 , pin2 , pwm_value_1 , pwm_value_2 );
	
}


/************************************************************************/
void PWM_one_LED(uint8_t pin1 , uint8_t pwm_value)
{
		uint32_t value;
	  ret_code_t err_code;
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
/*
void buzzer_control(uint16_t buzzer_value, uint16_t milli_secs)
{

  	volatile ret_code_t err_code;
	  volatile int32_t time_period = 0;	
	  if(buzzer_progress == true)
		{
			app_timer_stop(micro_buzzer);
			app_pwm_disable(&PWM2);
	    nrf_drv_gpiote_out_task_disable(BUZZER_PIN); 
			err_code = app_pwm_uninit(&PWM2);
			nrf_gpio_cfg_output(BUZZER_PIN);
			nrf_gpio_pin_clear(BUZZER_PIN);
		}
	  if(milli_secs > 0 )
	  {
		  err_code = app_timer_start(micro_buzzer, APP_TIMER_TICKS(milli_secs, 0), NULL);
	  }
	  if((buzzer_value >= 50) && (buzzer_value <= 50000))
	  {	
			app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(buzzer_value, BUZZER_PIN); //
			pwm2_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
			err_code = app_pwm_init(&PWM2,&pwm2_cfg,pwm_ready_callback_buzzer);
			app_pwm_enable(&PWM2);
			while (app_pwm_channel_duty_set(&PWM2, 0, 50) == NRF_ERROR_BUSY);
			while (!pwm_updated_buzzer);
			buzzer_progress = true;
	  }
	  
}
*/

/************************************************************************/
void config_adc()
{

    ret_code_t ret_code;
    nrf_adc_config_t adc_config = NRF_ADC_CONFIG_DEFAULT;
		
		// Configure ADC
    adc_config.resolution = NRF_ADC_CONFIG_RES_8BIT;
	  nrf_adc_configure(&adc_config);
	 
}
/************************************************************************/
void microbit_io_pwm_main(uint16_t* next_pin_state, uint16_t frequency , uint16_t time_ms)
{
		bool analog_pin_change = false;
		bool pwm_pin_change    = false;
		bool output_pin_change = false;
	  bool led_change_value  = false; 
		uint8_t pin_pwm_led[3];
		uint8_t pin_pwm_value[3];
	  uint8_t pin_pwm_index[3];
	
	  volatile uint8_t mode_current = 0;
		volatile uint8_t mode_next    = 0;
		uint16_t value_current = 0;
		uint16_t value_next    = 0;
		uint8_t i            = 0;
		uint8_t mode_change[4]={0x00,0x00,0x00,0x00};
		uint8_t count_pwm_led 		= 0;
		volatile uint8_t count_input  = 0;
	  uint8_t count_output 			= 0;
		uint8_t count_pwm_buzzer 	= 0;
		uint8_t increment         = 0;

		for(i=0;i<3;i++)
		{
			mode_current   = (uint8_t)((MB_pin[i].current_state & MODE_MASK)>>8) ;
			mode_next      = (uint8_t)((next_pin_state[i] & 0x0300)>>8);
			value_current  = MB_pin[i].current_state & VALUE_MASK;
			value_next     = next_pin_state[i] & VALUE_MASK;
			if(mode_current != mode_next)
			{
				pin_default(i);
				mode_change[mode_current] = true;
				mode_change[mode_next] 		= true;
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
			else if(MB_pin[i].pwm_buzzer == true)
			{
						if(i == 0)
						{
							MB_pin[i].buzz_freq_micro 	 		= frequency;		
							MB_pin[i].buzz_time_milli 	 		= time_ms;	
							//MB_pin[i].current_state         = next_pin_state[i];
							if(MB_pin[i].buzz_time_milli > 0)
							{	
								buzzer_HB_control(MB_pin[0].buzz_freq_micro,MB_pin[0].buzz_time_milli);
							}
						}
			}
			else if(value_current != value_next)
			{
				if(MB_pin[i].pwm_led == true)
				{
					MB_pin[i].pwm_value 		= value_next;
					led_change_value = true;
				}
			}
			MB_pin[i].current_state         = next_pin_state[i];
		}
		
		if(mode_change[PIN_INPUT] == true )
		{
			nrf_drv_adc_uninit();
			NRF_ADC->CONFIG =  NRF_ADC->CONFIG & ~ADC_CONFIG_PSEL_Msk;
			config_adc();
		}
		
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
		//check if the values have changed
		
		if((mode_change[PIN_PWM_LED_CHANGE] == true)||(mode_change[PIN_PWM_BUZZER_CHANGE] == true))
		{
			//Reset the PWM module
			reset_PWM();
			
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
			
			if((count_pwm_led + count_pwm_buzzer)>0)
			{
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
					if(count_pwm_buzzer>0)
					{
						buzzer_HB_control(MB_pin[0].buzz_freq_micro,MB_pin[0].buzz_time_milli);
						MB_pin[0].pwm_number 		 = 		 PWM2_MODULE;
						MB_pin[0].channel_number =     CHANNEL0;
					}
					if(count_pwm_led == 2)
					{
						PWM_two_LED(pin_pwm_led[0],pin_pwm_led[1], pin_pwm_value[0] , pin_pwm_value[1]);
						MB_pin[pin_pwm_index[0]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[0]].channel_number = CHANNEL0;
						MB_pin[pin_pwm_index[1]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[1]].channel_number = CHANNEL1;
					}
					else if(count_pwm_led == 1)
					{
						PWM_one_LED(pin_pwm_led[0], pin_pwm_value[0]);
						MB_pin[pin_pwm_index[0]].pwm_number     = PWM1_MODULE;
						MB_pin[pin_pwm_index[0]].channel_number = CHANNEL0;
					}
				}
			}
			
		}
		else
		{

		}
		
		//Analog Change
		
		//PWM Change
		
		
		//Output Change
		
		
		//No change in mode change in values i.e PWM and Output
}

/************************************************************************/
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
			 sensor_outputs[i] = 255;
		 }
   }
}
/************************************************************************/
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

