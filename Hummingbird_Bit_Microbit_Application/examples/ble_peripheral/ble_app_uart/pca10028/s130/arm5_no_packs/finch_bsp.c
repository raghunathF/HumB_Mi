#include "finch_pin.h"
#include "finch_bsp.h"
#include "nrf_delay.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>



/*
void init_finch_LED(void)
{
	nrf_gpio_cfg_output_high_drive(R__RGB__R_PIN );
	nrf_gpio_cfg_output_high_drive(R__RGB__G_PIN );
	nrf_gpio_cfg_output_high_drive(R__RGB__B_PIN );
	nrf_gpio_pin_clear(R__RGB__R_PIN );
	nrf_gpio_pin_clear(R__RGB__G_PIN );
	nrf_gpio_pin_clear(R__RGB__G_PIN );
}

void LED_on(uint8_t pin , uint8_t intensity)
{
	if(intensity > 0 )
	{
		nrf_gpio_pin_clear(pin);
	}
	else
	{
		nrf_gpio_pin_set(pin);
	}
}

void LED_control(uint8_t red, uint8_t green , uint8_t blue)
{
	LED_on(R__RGB__R_PIN,red);
	LED_on(R__RGB__G_PIN,green);
	LED_on(R__RGB__B_PIN,blue);
}

*/