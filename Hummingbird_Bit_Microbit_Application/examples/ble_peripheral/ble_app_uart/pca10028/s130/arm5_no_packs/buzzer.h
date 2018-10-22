
#include <stdint.h>
#include <string.h>

#define NRF_MAX_VALUE_BUZZER      50000UL
#define NRF_MIN_VALUE_BUZZER      50UL
#define APP_MAX_VALUE_BUZZER      255UL
#define MUL_FACTOR                (NRF_MAX_VALUE_BUZZER - NRF_MIN_VALUE_BUZZER)/APP_MAX_VALUE
void init_timer_buzzer();
void buzzer_HB_control(uint16_t buzzer_value, uint16_t milli_secs);

void app_pwm2_reset();
void pwm2_change_dutycycle(uint8_t channel_number , uint8_t pwm_value);
void  PWM2_1LED(uint8_t pwm_value);