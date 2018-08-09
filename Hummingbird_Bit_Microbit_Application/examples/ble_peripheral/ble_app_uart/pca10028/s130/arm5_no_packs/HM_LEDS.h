#include <stdint.h>
#include <string.h>



#define NRF_MAX_VALUE 100
#define APP_MAX_VALUE 255

#define LED2 0x32
#define LED3 0x33

void LEDS_PWM_init();
void LED_HB_control(uint16_t led_no , uint8_t led_value);

