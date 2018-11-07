/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************    Includes        **********************************/
/************************************************************************/
#include <stdint.h>
/************************************************************************/

/************************************************************************/
/******************    Define          **********************************/
/************************************************************************/
#define NO_PWM                 0
#define PWM1_CH1_LED           1
#define PWM1_CH1_BUZZER				 2
#define PWM1_CH2_LED					 3
#define PWM1_CH2_BUZZER        4
#define PWM2_CH1_LED           5
#define PWM2_CH1_BUZZER        6
#define PWM2_CH2_LED           7
#define PWM2_CH2_BUZZER        8

#define NO_MICRO_PINS          3

#define MODE_MASK							 0x0300
#define VALUE_MASK						 0x00FF

#define PIN_PWM_LED						0
#define PIN_INPUT							1
#define PIN_PWM_BUZZER				2


#define PIN_PWM_LED_CHANGE    0
#define PIN_PWM_BUZZER_CHANGE 2

#define PWM1_MODULE						1
#define PWM2_MODULE						2
#define CHANNEL0							0
#define CHANNEL1							1

#define ADC_PIN_0							4
#define ADC_PIN_1							3
#define ADC_PIN_2							2

#define PIN_NOS 							 {PIN_MICROBIT_0  ,PIN_MICROBIT_1  ,PIN_MICROBIT_2 }
#define PIN_ADCS							 {NRF_ADC_CONFIG_INPUT_4, NRF_ADC_CONFIG_INPUT_3 , NRF_ADC_CONFIG_INPUT_2}


extern uint8_t    sensor_outputs[20];
/************************************************************************/

/************************************************************************/
/******************    Prototype       **********************************/
/************************************************************************/
void reset_PWM(void);
void read_sensors(void);
void microbit_io_pwm_main(uint16_t* next_pin_state, uint16_t frequency  , uint16_t time_us );
void microbit_pwm_init(void);
/************************************************************************/

