#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void init_LEDs(void);
void test_LED_1(void);
void test_LED_2(int);
void test_LED_3(void);
void test_LED_4(void);

void happy_ledscreen(void);
void sad_ledscreen(void);
void LED_control(uint8_t red, uint8_t green , uint8_t blue);
void set_all_led_once(uint32_t);
void init_finch_LED();
void init_buzzer();

