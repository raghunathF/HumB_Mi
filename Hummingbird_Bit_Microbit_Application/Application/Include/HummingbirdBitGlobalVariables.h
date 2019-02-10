/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/************************************************************************/
/******************     Globals       **********************************/
/************************************************************************/
#define TWI_INSTANCE_ID     1
APP_TIMER_DEF(transmit_timer_id);

uint8_t INITIAL_NAME[2] = {'M','B'};
uint8_t input_micro_packet[20];

ble_nus_t                        m_nus; 

char 	DEVICE_NAME [20];

bool led_change = false;
bool fake_firware_send = false;
bool app_selected = false;

bool start_advertising_flashing  = false;
bool stop_advertising_flashing   = false;

bool LED_array_enable = true;

bool flash_mb = true;


static uint8_t index = 0;

volatile bool broadcast_flag = false;
volatile bool UART_interrupt_flag = false;
volatile bool spi_xfer_done = true;



static bool first_byte = true;
static uint8_t length_receive = 0 ;
static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
volatile  uint32_t LED_value = 0;

uint8_t    sensor_outputs[20];
uint8_t    currentConnectionMode = ADVERTISING_MODE;


const char possible_names[] = {'F','N','H','M','F','L','H','B','B','2','C','3'};   //Just to be safe used A1,B2,C3

volatile uint8_t   UARTIndex = 0;

int mood_bit = 0;
uint8_t temp_p_data[20];
uint8_t temp_p_sensor_data[20];

//BLE buffer
uint8_t input_micro_packet[20];
uint8_t buffer_data[255];
volatile uint8_t tail_pointer = 0;
volatile uint8_t head_pointer = 0;

//UART Buffer
uint8_t UARTRingBuffer[255];
volatile uint8_t UARTTailPointer = 0;
volatile uint8_t UARTHeadPointer = 0; 


uint8_t sound_effect =  0;
uint8_t initials_name[4];
uint8_t prev_state = 0;
bool buzzer_progress = false;

nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

uint8_t read_axis_mag[LENGTH_AXIS_DATA] ;

uint8_t calibrate_feedback = 0 ; 
uint8_t accl_mag_chip = 0;

/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;
volatile bool    calibrationFlag = false;
/************************************************************************/
