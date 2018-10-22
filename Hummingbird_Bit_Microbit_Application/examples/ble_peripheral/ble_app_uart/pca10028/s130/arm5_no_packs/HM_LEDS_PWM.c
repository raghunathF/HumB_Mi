/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : PWM module initialzation for LED2,LED3
							
*/
/************************************************************************/
#include "app_pwm.h"
#include "micro_pin.h"
#

#include <stdint.h>
#include <string.h>
#include "HM_LEDS.h"
#include "nrf_delay.h"
#include "microbit_io_pwm.h"
//#include "global_pwm.h"

/************************************************************************/
#define INIT_LED1_VALUE 0xBBUL
#define INIT_LED2_VALUE 0x11UL
/************************************************************************/
static volatile  bool pwm1_updated = false;

APP_PWM_INSTANCE(PWM1,1);

void app_pwm1_reset()
{
	app_pwm_disable(&PWM1);
	app_pwm_uninit(&PWM1);
}


/************************************************************************/
void pwm1_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm1_updated = true;
}
/************************************************************************/

void PWM1_2LEDS(uint8_t pin1 , uint8_t pin2 ,uint8_t pwm_value1 , uint8_t pwm_value2)
{
		uint16_t temp1 =0;
		app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(8192L, pin1, pin2); //122 Hz, 8192 u sec
	 
	  pwm1_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  pwm1_cfg.pin_polarity[1]  = APP_PWM_POLARITY_ACTIVE_HIGH;
		
	  app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback);
	  app_pwm_enable(&PWM1);
		
		pwm1_updated = false;
	  temp1     = (pwm_value1 * 100 )/255 ;
		while (app_pwm_channel_duty_set(&PWM1, 0, temp1) == NRF_ERROR_BUSY);
		while (!pwm1_updated);
	
	
	  pwm1_updated = false;
	  temp1     = (pwm_value2 * 100 )/255 ;
		while (app_pwm_channel_duty_set(&PWM1, 1, temp1) == NRF_ERROR_BUSY);
		while (!pwm1_updated);
	
}



void PWM1_1LED(uint8_t pin1 ,uint8_t  pwm_value )
{
	  uint16_t temp =0;
	  app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(8192L, pin1); 
	  pwm1_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  app_pwm_init(&PWM1, &pwm1_cfg, pwm1_ready_callback);
	  app_pwm_enable(&PWM1);
		pwm1_updated = false;
	  temp     = (pwm_value * 100 )/255 ;
		while (app_pwm_channel_duty_set(&PWM1, 0, temp) == NRF_ERROR_BUSY);
		while (!pwm1_updated);
}
	
		
void pwm1_change_dutycycle(uint8_t channel_number , uint8_t pwm_value)
{
	  uint16_t temp =0;
		pwm1_updated = false;
		temp     = (pwm_value * 100 )/255 ;
		while (app_pwm_channel_duty_set(&PWM1, channel_number, temp) == NRF_ERROR_BUSY)
		{
			//nrf_drv_timer_clear(PWM1.p_timer);
		}
		//nrf_drv_timer_compare(p_instance->p_timer
	  //app_pwm_channel_duty_set(&PWM1, channel_number, temp);

		while (!pwm1_updated);
}



/************************************************************************
Unintialize PWM modules
************************************************************************/
static void PWM_uninit_HB_LEDS()
{
	  app_pwm_disable(&PWM1);
}
/************************************************************************/




/************************************************************************
Use PWM modules to control the LEDs and frquency fo the PWM wave -- 122Hz
************************************************************************/
static void PWM_init_HB_LEDS()
{
	  volatile uint32_t value;
	  volatile ret_code_t err_code;
		app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(8192L, LED2_PIN, LED3_PIN); //122 Hz, 8192 u sec
	  pwm1_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  pwm1_cfg.pin_polarity[1]  = APP_PWM_POLARITY_ACTIVE_HIGH;
	  err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm1_ready_callback);
    APP_ERROR_CHECK(err_code);
	  app_pwm_enable(&PWM1);
}

/************************************************************************/
void LED_HB_control(uint16_t led_no , uint8_t led_value)
{
	  pwm1_updated = false;
	  led_value = ((led_value*NRF_MAX_VALUE) / APP_MAX_VALUE);
		switch(led_no)
		{
			case LED2:
					while (app_pwm_channel_duty_set(&PWM1, 0, led_value) == NRF_ERROR_BUSY);
					while (!pwm1_updated);
					break;
			case LED3:
					while (app_pwm_channel_duty_set(&PWM1, 1, led_value) == NRF_ERROR_BUSY);
					while (!pwm1_updated);
					break;
			default:
					break;
		}
}

/************************************************************************/
static void LEDS_init()
{
	volatile uint16_t temp_test= 0;
	
	pwm1_updated = false;
	while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);
	while (!pwm1_updated);
	
	pwm1_updated = false;
	while (app_pwm_channel_duty_set(&PWM1, 1, 0) == NRF_ERROR_BUSY);
	while (!pwm1_updated);
}

void hummingbit_pwm_init()
{
	reset_PWM();
	LEDS_PWM_init();
}

/************************************************************************/
void LEDS_PWM_init()
{
		
	  PWM_init_HB_LEDS();
	  LEDS_init();
	  /* Initialize and enable PWM. */
}

