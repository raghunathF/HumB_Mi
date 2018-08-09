/************************************************************************/
/*
*Author      : Raghunath Jangam                                         
*Last edit   : 8/7/2018
*Description : Microbit LED array driver
*/
/************************************************************************/
#include "micro_pin.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "microbit_LEDS.h"
#include "ledarray_map.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/************************************************************************/
APP_TIMER_DEF(micro_LED_timer_id);
APP_TIMER_DEF(micro_LED_flash);
/************************************************************************/
static const  uint8_t microbit_leds[TOTAL_RC]     								= ALL_LED;
static const  uint8_t rows_leds[TOTAL_LEDS]    								= LEDS_ROWS;
static const  uint8_t columns_leds[TOTAL_LEDS] 								= LEDS_COLUMNS;
static const  uint32_t All_LEDbitmap[127]                   	= all_bitmap;

static bool micro_led_enable = false;
static uint8_t temp_value_bitmap = 0;

static uint32_t micro_led_value = 0 ;
static bool advertising_string = true;

extern bool flash_mb;

uint32_t current_flash_bitmap[40]; 
uint8_t length_flash = 0;

typedef struct led_address
{
	uint8_t row;
	uint8_t column;

}led_address;

led_address LED_ARRAY[TOTAL_LEDS];

/************************************************************************/
void start_flashing_timer()
{
	  uint32_t err_code = 0;
		err_code = app_timer_start(micro_LED_flash, APP_TIMER_TICKS(300, 0), NULL); // Start the timer and callback is called every 300ms
		APP_ERROR_CHECK(err_code);
	  flash_mb = true;              
}

/************************************************************************/
void stop_flashing_timer()
{
	  flash_mb = false;
	  app_timer_stop(micro_LED_flash);
}

//Used during the Flash string command
/************************************************************************/
void update_flash_array(uint8_t* char_array , uint8_t length )
{
	uint8_t i = 0;
	for(i=0;i<length;i++)
	{
		current_flash_bitmap[2*i]        = get_LEDbitmap(char_array[i]);       //Get teh bit map of the appropriate character
		current_flash_bitmap[2*i+1]      = 0x00000000;                         //Blank after every letter
	}
	current_flash_bitmap[length*2]     = 0x00000000;                         //Blank after every word
	current_flash_bitmap[length*2+1]   = 0x00000000;                         //Blank after every word
	length_flash = length*2 + 2;
	temp_value_bitmap = 0;
	
}


/************************************************************************/
void start_flashing_string()
{
	
	start_flashing_timer();
	LED_array_enable = true;
}

/************************************************************************/
void LED_start_flashing(uint8_t length)
{

	update_flash_array(initials_name, length);
	start_flashing_timer();
	LED_array_enable = true;

}

/************************************************************************/
void start_LEDarray_advertising()
{
	initials_name[3] = 0x00;
	update_flash_array(initials_name,4);           //Grab the intials for the advertisiment
	start_flashing_timer();
	advertising_string = true;
	LED_array_enable   = true;
}

/************************************************************************
Switch off the LEDs in the LED array
************************************************************************/
void switch_off_leds()
{
	//Switch off ROW1
	uint8_t i =0;
	for(i=0;i<TOTAL_C;i++)
	{
			nrf_gpio_pin_set(microbit_leds[i]);
	}
	for(i=TOTAL_C;i<TOTAL_C+3;i++)
	{
			nrf_gpio_pin_clear(microbit_leds[i]);
	}
	
}


/************************************************************************/
void stop_LEDarray_display()
{
	
	//Disable the timer
	if(flash_mb == true)
	{
		app_timer_stop(micro_LED_timer_id);
		app_timer_stop(micro_LED_flash);
	}
	else if(micro_led_enable == true)
	{
		app_timer_stop(micro_LED_timer_id);
	}
	
	micro_led_enable = false;
	LED_array_enable = false;
	advertising_string = false;
	temp_value_bitmap = 0;
	switch_off_leds();
	
}

/************************************************************************/
uint32_t get_LEDbitmap(uint8_t single_character)
{
	uint32_t LED_bitmap = 0;
	LED_bitmap       = All_LEDbitmap[single_character];
	return LED_bitmap;
}

/************************************************************************/
void init_led_address(void)
{
	int i=0;
	
	for(i=0;i<TOTAL_LEDS ;i++)
	{
		LED_ARRAY[i].row    = rows_leds[i];
		LED_ARRAY[i].column = columns_leds[i];
	}
}

/************************************************************************/
void init_LEDs_micro(void)
{
  int i=0;
	for(i=0;i< TOTAL_RC;i++)
	{
		if(i < TOTAL_C )
		{
			nrf_gpio_cfg_output_high_drive(microbit_leds[i]);
		  nrf_gpio_pin_set(microbit_leds[i]);
		}
		else
		{
			nrf_gpio_cfg_output(microbit_leds[i]);
		  nrf_gpio_pin_clear(microbit_leds[i]);
		}
	}
	init_led_address();
}


/************************************************************************
This is where all the action takes place, in the first part of the program
calculate what are the coloms to be switched on/off for the respective 
rows. Second part of the program where we acttually change the output based
on the first part of the program
************************************************************************/
void set_all_led_micro(uint32_t  set_led , uint8_t row, bool update_led_value)
{
	//Find columns which require 
	int i =0;
	
	static int array_leds_set[TOTAL_LEDS];
	static int LED_ROW_1[10];
	static int LED_ROW_2[10];
	static int LED_ROW_3[10];
	
	static int led_row_1_count = 0;
	static int led_row_2_count = 0;
	static int led_row_3_count = 0;
	
	if(update_led_value == true)
	{
		update_led_value = false;
		led_row_1_count = 0;
	  led_row_2_count = 0;
	  led_row_3_count = 0;
		for(i=0;i<TOTAL_LEDS;i++)
		{
			 array_leds_set[i]= 0;
		}
		for(i=0;i<10;i++)
		{
			 LED_ROW_1[i] = 0;
			 LED_ROW_2[i] = 0;
			 LED_ROW_3[i] = 0;
		}
		for(i=0;i<TOTAL_LEDS;i++)
		{
			 array_leds_set[i]= ((set_led >> i) & (0x00000001));
		}
	
		for(i=0;i<TOTAL_LEDS;i++)
		{
				if(array_leds_set[i] == 1)
				{
					if(LED_ARRAY[i].row == 1 )
					{
						LED_ROW_1[led_row_1_count] = i;
						led_row_1_count++;
					}
					else if(LED_ARRAY[i].row == 2 )
					{
						LED_ROW_2[led_row_2_count] = i;
						led_row_2_count++;
					}
					else if(LED_ARRAY[i].row == 3 )
					{
						LED_ROW_3[led_row_3_count] = i;
						led_row_3_count++;
					}
				 }
		}
	}
	
	

	switch(row)
	{
		case 0:
			//Switch on ROW1
			nrf_gpio_pin_set (microbit_leds[TOTAL_C-1+ROW_1_Value]);
			for(i=0;i<led_row_1_count;i++)
			{
				nrf_gpio_pin_clear(microbit_leds[LED_ARRAY[LED_ROW_1[i]].column - 1]);
			}
			break;
		case 1:
			switch_off_leds();
		  break;
		case 2:
			//Switch on ROW2
			nrf_gpio_pin_set (microbit_leds[TOTAL_C-1+ROW_2_Value]);
			for(i=0;i<led_row_2_count;i++)
			{
				nrf_gpio_pin_clear(microbit_leds[LED_ARRAY[LED_ROW_2[i]].column - 1]);
			}
			break;
		case 3:
			switch_off_leds();
		  break;
		case 4:
			//Switch on ROW3
			nrf_gpio_pin_set (microbit_leds[TOTAL_C-1+ROW_3_Value]);
			for(i=0;i<led_row_3_count;i++)
			{
				nrf_gpio_pin_clear(microbit_leds[LED_ARRAY[LED_ROW_3[i]].column - 1]);
			}
			break;
		case 5:
			switch_off_leds();
		  break;
		
		default:
				//Switch off ROW1
			break;	
	  }
			
}

/************************************************************************/
// Timeout handler for repeated timer for LED
static void micro_LED_timer_handler(void * p_context)
{
	//Check the row it has to update
	//Get the Global value of the micro_LED_update, if its on first row now
	//Switch off all LEDS
	//Update the row corresponding
	static uint8_t current_row = 0;
	static uint32_t temp_micro_led_value = 0;
	static uint32_t prev_temp_micro_led_value = 0;
	static bool update_led_value = false;
	update_led_value = false;
	if(current_row == 6 )
	{
		current_row = 0;
	}
	if(current_row == 0)
	{
		temp_micro_led_value     = micro_led_value ; // Global value 
		if(temp_micro_led_value != prev_temp_micro_led_value)
		{
		     update_led_value = true;
			   prev_temp_micro_led_value = temp_micro_led_value;
		}
	}
	
	set_all_led_micro(temp_micro_led_value, current_row,update_led_value);
	current_row++;
}

/************************************************************************
If the LED array value is not zero then start the timer which changes between 
3 different rows every 3 mseconds. 

************************************************************************/
void LED_micro_control(uint32_t led_value,uint8_t brightness)
{
		uint32_t err_code;
	  micro_led_value = led_value;

		if(led_value == 0)
		{
			if(micro_led_enable == true)
			{
				if(flash_mb == false)
				{
					switch_off_leds();
					app_timer_stop(micro_LED_timer_id);
					micro_led_enable = false;
				}
			}
		}
		else
		{
			if(micro_led_enable == false)
			{
				err_code = app_timer_start(micro_LED_timer_id, APP_TIMER_TICKS(3, 0), NULL);
				APP_ERROR_CHECK(err_code);
				micro_led_enable = true;
			}
		}
		 
		
	  
}

/************************************************************************
Timeout handler for repeated timer for LED. Print the characters and
all the characters are printed
************************************************************************/
static void micro_LED_flash_handler(void * p_context)
{
	
	if(LED_array_enable == true)
	{
		LED_micro_control(current_flash_bitmap[temp_value_bitmap],0x55);
	}
	temp_value_bitmap++;
  if(temp_value_bitmap >= length_flash )
	{
		temp_value_bitmap = 0;
		if(advertising_string == false)
		{
			stop_LEDarray_display();
		}
	}	
}

/************************************************************************/
//Main function which initialises all the LED array timers
void init_micro_LEDs()
{
		uint32_t err_code;
    // Create timers
	  init_LEDs_micro();
	  err_code = app_timer_create(&micro_LED_timer_id,APP_TIMER_MODE_REPEATED,micro_LED_timer_handler);
    APP_ERROR_CHECK(err_code);
	  err_code = app_timer_create(&micro_LED_flash,APP_TIMER_MODE_REPEATED,micro_LED_flash_handler);
    APP_ERROR_CHECK(err_code);
}












