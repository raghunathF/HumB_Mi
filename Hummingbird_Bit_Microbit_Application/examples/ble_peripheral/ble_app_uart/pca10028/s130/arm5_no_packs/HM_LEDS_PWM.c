/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : PWM module initialzation for LED2,LED3
							
*/
/************************************************************************/
#include "app_pwm.h"
#include "HM_pin.h"

#include <stdint.h>
#include <string.h>
#include "HM_LEDS.h"
#include "nrf_delay.h"

/************************************************************************/
#define INIT_LED1_VALUE 0xBBUL
#define INIT_LED2_VALUE 0x11UL
/************************************************************************/
static volatile  bool pwm_updated_led1 = false;
static volatile  bool pwm_updated_led2 = false;

APP_PWM_INSTANCE(PWM1,1);

/************************************************************************/
void pwm_ready_callback_led1(uint32_t pwm_id)    // PWM callback function
{
    pwm_updated_led1 = true;
}


/************************************************************************
Use PWM modules to control the LEDs and frquency fo the PWM wave -- 122Hz
************************************************************************/
static void PWM_init()
{
	  volatile uint32_t value;
	  volatile ret_code_t err_code;
		app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(8192L, LED2_PIN, LED3_PIN); //122 Hz, 8192 u sec
	  
	  pwm1_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  pwm1_cfg.pin_polarity[1]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback_led1);
    APP_ERROR_CHECK(err_code);
	  app_pwm_enable(&PWM1);
}

/************************************************************************/
void LED_HB_control(uint16_t led_no , uint8_t led_value)
{
	  led_value = ((led_value*NRF_MAX_VALUE) / APP_MAX_VALUE);
		switch(led_no)
		{
			case LED2:
					while (app_pwm_channel_duty_set(&PWM1, 0, led_value) == NRF_ERROR_BUSY);
					while (!pwm_updated_led1);
					break;
			case LED3:
					while (app_pwm_channel_duty_set(&PWM1, 1, led_value) == NRF_ERROR_BUSY);
					while (!pwm_updated_led1);
					break;
			default:
					break;
		}
}

/************************************************************************/
static void LEDS_init()
{
	volatile uint16_t temp_test= 0;
	
	while (app_pwm_channel_duty_set(&PWM1, 0, 10) == NRF_ERROR_BUSY);
	while (!pwm_updated_led1);
	
	
	while (app_pwm_channel_duty_set(&PWM1, 1, 90) == NRF_ERROR_BUSY);
	while (!pwm_updated_led1);
}
/************************************************************************/
void LEDS_PWM_init()
{
	 
	  PWM_init();
	  LEDS_init();
	  /* Initialize and enable PWM. */
    
}

