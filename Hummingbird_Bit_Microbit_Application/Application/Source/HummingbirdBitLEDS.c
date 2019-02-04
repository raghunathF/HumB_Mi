/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_timer.h"
#include "nrf_gpio.h"

#include  "HummingbirdBitPinout.h"
/************************************************************************/

// Timeout handler for repeated timer for LED2
static void HM_LED2_timer_handler(void * p_context)
{
	//Switch off the LED2
	
}


// Timeout handler for repeated timer for LED3
static void HM_LED3_timer_handler(void * p_context)
{
	//Switch off the LED3 
	
}

// Timeout handler for repeated timer for overflow
static void overflow_timer_handler(void * p_context)
{
	//switch on the LEDS
	
}


void create_timers_HM_LEDS()
{
	  uint32_t err_code;
    // Create timers
	  APP_TIMER_DEF(HM_LED2_timer_id);
	  APP_TIMER_DEF(HM_LED3_timer_id);
	  APP_TIMER_DEF(overflow_timer_id);
    //LED2
	  err_code = app_timer_create(&HM_LED2_timer_id,APP_TIMER_MODE_REPEATED,HM_LED2_timer_handler);
    APP_ERROR_CHECK(err_code);
	  //LED3
	  err_code = app_timer_create(&HM_LED3_timer_id,APP_TIMER_MODE_REPEATED,HM_LED3_timer_handler);
    APP_ERROR_CHECK(err_code);
	  //overflow
	  err_code = app_timer_create(&overflow_timer_id,APP_TIMER_MODE_REPEATED,overflow_timer_handler);
    APP_ERROR_CHECK(err_code);
		
}

void init_timer_HM_LEDS()
{
	 create_timers_HM_LEDS();
}

void init_pins_HM_LEDS()
{
	nrf_gpio_cfg_output_high_drive(LED2_PIN);
	nrf_gpio_cfg_output_high_drive(LED3_PIN);
	nrf_gpio_pin_clear(LED2_PIN);
	nrf_gpio_pin_clear(LED3_PIN);
}

void init_HM_LEDS()
{
	init_pins_HM_LEDS();
	init_timer_HM_LEDS();
}
