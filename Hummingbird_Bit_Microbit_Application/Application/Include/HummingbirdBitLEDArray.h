/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************    Includes        **********************************/
/************************************************************************/
#include <stdbool.h>
#include <stdlib.h>
/************************************************************************/

/************************************************************************/
/******************    Defines         **********************************/
/************************************************************************/
#define TOTAL_ALPHABETS 26
/************************************************************************/

/************************************************************************/
/******************   Variables        **********************************/
/************************************************************************/
extern bool LED_array_enable;
uint32_t get_LEDbitmap(uint8_t single_character);
extern uint8_t initials_name[4];
extern bool start_advertising_flashing ;
extern bool stop_advertising_flashing  ;
/************************************************************************/

/************************************************************************/
/******************   Prototypes       **********************************/
/************************************************************************/
void stop_flashing_timer(void);
void start_LEDarray_advertising(void);
void stop_LEDarray_display(void);
void init_micro_LEDs(void);
void update_flash_array(uint8_t* char_array , uint8_t length );
void LED_micro_control(uint32_t led_value,uint8_t brightness);
void LED_start_flashing(uint8_t length);
void start_flashing_string(void);
void check_flashing(void);
/************************************************************************/