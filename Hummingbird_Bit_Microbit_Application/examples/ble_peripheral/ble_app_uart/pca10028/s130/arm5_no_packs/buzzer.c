/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : PWM module initialzation for buzzer
							
*/
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
#include "app_timer.h"
/************************************************************************/
APP_TIMER_DEF(micro_buzzer);
APP_PWM_INSTANCE(PWM2,2);

#define CORRECT_LEDARRAY				0x022a200
#define INIT_BUZZER_VALUE 			0x00UL
/************************************************************************/
extern bool buzzer_progress; 
static bool buzzer_emergency_stop = false; 

static volatile  bool pwm2_updated = false;
/************************************************************************/

void pwm2_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm2_updated = true;
}

/************************************************************************/
static void micro_buzzer_timer_handler(void * p_context)
{
	    uint32_t err_code = 0;
			app_pwm_disable(&PWM2);
			nrf_drv_gpiote_out_task_disable(BUZZER_PIN); 
			err_code = app_pwm_uninit(&PWM2);
			nrf_gpio_cfg_output(BUZZER_PIN);
			nrf_gpio_pin_clear(BUZZER_PIN);
	    buzzer_progress = false;
}

void  PWM2_1LED(uint8_t pwm_value)
{
		uint16_t temp = 0;
		temp     = (pwm_value * 100 )/255 ;
	  app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(8192L, PIN_MICROBIT_0);		
		pwm2_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
		
	
		app_pwm_init(&PWM2, &pwm2_cfg, pwm2_ready_callback);
	
		app_pwm_enable(&PWM2);
		
	  pwm2_updated = false;
		while (app_pwm_channel_duty_set(&PWM2, 0, temp) == NRF_ERROR_BUSY);
		while(!pwm2_updated);
	
}


void pwm2_change_dutycycle(uint8_t channel_number , volatile uint8_t pwm_value)
{
		uint16_t temp =0;
		pwm2_updated = false;
		temp     = (pwm_value * 100 )/255 ;
		while (app_pwm_channel_duty_set(&PWM2, channel_number, temp) == NRF_ERROR_BUSY);
		while (!pwm2_updated);
}

	

void app_pwm2_reset()
{
	app_timer_stop(micro_buzzer);
	app_pwm_disable(&PWM2);
	app_pwm_uninit(&PWM2);
}

/************************************************************************/
static void buzzer_PWM_init()
{
  	volatile ret_code_t err_code;
	  err_code = app_timer_create(&micro_buzzer,APP_TIMER_MODE_SINGLE_SHOT,micro_buzzer_timer_handler);
    APP_ERROR_CHECK(err_code);

}

/************************************************************************
If buzzer is still in progess stop the ongoing process and initialize 
the buzzer with the current PWM.Accept the values of timeperiods which are 
greater than 50us and less than 50000us .
************************************************************************/
void buzzer_HB_control(uint16_t buzzer_value, uint16_t milli_secs)
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
			pwm2_updated = false;
			app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(buzzer_value, BUZZER_PIN); //
			pwm2_cfg.pin_polarity[0]  = APP_PWM_POLARITY_ACTIVE_HIGH;
			err_code = app_pwm_init(&PWM2,&pwm2_cfg,pwm2_ready_callback);
			app_pwm_enable(&PWM2);
			while (app_pwm_channel_duty_set(&PWM2, 0, 50) == NRF_ERROR_BUSY);
			//app_pwm_channel_duty_set(&PWM2, 0, 50);
			while (!pwm2_updated);
			buzzer_progress = true;
	  }
	  
}

/************************************************************************/
void init_timer_buzzer()
{
	  //LED_pins_init();
	  nrf_gpio_cfg_output(BUZZER_PIN);
		nrf_gpio_pin_clear(BUZZER_PIN);
	  buzzer_PWM_init();
	  /* Initialize and enable PWM. */    
}



