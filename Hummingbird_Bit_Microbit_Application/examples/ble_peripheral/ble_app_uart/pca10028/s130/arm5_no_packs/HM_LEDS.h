#include <stdint.h>
#include <string.h>



#define NRF_MAX_VALUE 100
#define APP_MAX_VALUE 255

#define LED2 0x32
#define LED3 0x33

void LEDS_PWM_init();
void LED_HB_control(uint16_t led_no , uint8_t led_value);
void hummingbit_pwm_init();

void pwm1_change_dutycycle(uint8_t channel_number , uint8_t pwm_value);
void PWM1_1LED(uint8_t pin1 ,uint8_t  pwm_value );
void PWM1_2LEDS(uint8_t pin1 , uint8_t pin2 ,uint8_t pwm_value1 , uint8_t pwm_value2);
void app_pwm1_reset();

