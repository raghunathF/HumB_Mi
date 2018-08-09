#include <stdbool.h>
#include <stdlib.h>

#define TOTAL_ALPHABETS 26

extern bool LED_array_enable;
uint32_t get_LEDbitmap(uint8_t single_character);
extern uint8_t initials_name[4];


void stop_flashing_timer();
void start_LEDarray_advertising();
void stop_LEDarray_display();
void init_micro_LEDs();
void update_flash_array(uint8_t* char_array , uint8_t length );
void LED_micro_control(uint32_t led_value,uint8_t brightness);
void LED_start_flashing(uint8_t length);
void start_flashing_string();

