#include "nrf_gpio.h"



//LEDs

#define LROW_0 13 
#define LROW_1 14
#define LROW_2 15


#define LCOL_0 4
#define LCOL_1 5
#define LCOL_2 6
#define LCOL_3 7
#define LCOL_4 8
#define LCOL_5 9
#define LCOL_6 10
#define LCOL_7 11
#define LCOL_8 12


#define LED0_ROW 1
#define LED1_ROW 2
#define LED2_ROW 1
#define LED3_ROW 2
#define LED4_ROW 1
#define LED5_ROW 3
#define LED6_ROW 3
#define LED7_ROW 3
#define LED8_ROW 3
#define LED9_ROW 3
#define LED10_ROW 2
#define LED11_ROW 1
#define LED12_ROW 2
#define LED13_ROW 3
#define LED14_ROW 2
#define LED15_ROW 1
#define LED16_ROW 1
#define LED17_ROW 1
#define LED18_ROW 1
#define LED19_ROW 1
#define LED20_ROW 3
#define LED21_ROW 2
#define LED22_ROW 3
#define LED23_ROW 2
#define LED24_ROW 3



#define LED0_COL 1
#define LED1_COL 4
#define LED2_COL 2
#define LED3_COL 5
#define LED4_COL 3
#define LED5_COL 4
#define LED6_COL 5
#define LED7_COL 6
#define LED8_COL 7
#define LED9_COL 8
#define LED10_COL 2
#define LED11_COL 9
#define LED12_COL 3
#define LED13_COL 9
#define LED14_COL 1
#define LED15_COL 8
#define LED16_COL 7
#define LED17_COL 6
#define LED18_COL 5
#define LED19_COL 4
#define LED20_COL 3
#define LED21_COL 7
#define LED22_COL 1
#define LED23_COL 6
#define LED24_COL 2


#define ROW_1_Value 1
#define ROW_2_Value 2
#define ROW_3_Value 3



#define LEDS_ROWS {LED0_ROW  ,LED1_ROW ,LED2_ROW,LED3_ROW,LED4_ROW, \
									 LED5_ROW  ,LED6_ROW ,LED7_ROW,LED8_ROW,LED9_ROW, \
									 LED10_ROW ,LED11_ROW , LED12_ROW, LED13_ROW,LED14_ROW, \
									 LED15_ROW ,LED16_ROW , LED17_ROW,LED18_ROW, LED19_ROW, \
									 LED20_ROW ,LED21_ROW , LED22_ROW,LED23_ROW,LED24_ROW}	

									 
#define LEDS_COLUMNS {LED0_COL  ,LED1_COL ,LED2_COL,LED3_COL,LED4_COL, \
											LED5_COL  ,LED6_COL ,LED7_COL,LED8_COL,LED9_COL, \
											LED10_COL ,LED11_COL , LED12_COL,LED13_COL,LED14_COL, \
                      LED15_COL ,LED16_COL , LED17_COL, LED18_COL,LED19_COL, \
                      LED20_COL ,LED21_COL , LED22_COL, LED23_COL,LED24_COL}

							
#define ALL_LED { LCOL_0 , LCOL_1 , LCOL_2 , LCOL_3 , LCOL_4 , LCOL_5 , LCOL_6 , LCOL_7 , LCOL_8 , LROW_0 , LROW_1 , LROW_2 }

#define TOTAL_RC 12 
#define TOTAL_R 3
#define TOTAL_C 9

#define TOTAL_LEDS 25
//Button

#define BUTTON_A  17
#define BUTTON_B  26

#define NO_BUTTONS 2

//Accelerometer
#define SDA_ACCEL 23
#define SCL_ACCEL 30


//Magentometer
#define SDA_MAG 23
#define SCL_MAG 30



//UART
#define RX_MICRO 22
#define TX_MICRO 21

//Bluetooth

//Microbit I/O's
#define PIN_MICROBIT_0  3
#define PIN_MICROBIT_1  2
#define PIN_MICROBIT_2  1


//Humming Bit Peripherals
#define BUZZER_PIN 3


//LEDs PWM
#define LED2_PIN 		1 
#define LED3_PIN 		18

