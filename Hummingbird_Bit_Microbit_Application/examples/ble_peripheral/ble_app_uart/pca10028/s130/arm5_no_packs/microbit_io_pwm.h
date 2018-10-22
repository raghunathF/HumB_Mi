void reset_PWM();
void read_sensors();
void microbit_io_pwm_main(uint16_t* next_pin_state, uint16_t frequency  , uint16_t time_us );
void microbit_pwm_init();
extern uint8_t    sensor_outputs[20];