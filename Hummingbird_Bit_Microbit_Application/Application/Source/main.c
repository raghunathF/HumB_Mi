/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*|File Description	|					This file contains the source code for initializing all the modules.		|
*|                  |         Does a regular check for presence of data through BLE or UART. 					|
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/* Please reffer to the License folder in the repository of the source folder */
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include	<stdint.h>
#include	<string.h>
#include	"nordic_common.h"
#include	"nrf.h"
#include	"ble_hci.h"
#include	"ble_advdata.h"
#include	"ble_advertising.h"
#include	"ble_conn_params.h"
#include	"softdevice_handler.h"
#include	"app_timer.h"
#include	"app_button.h"
#include	"ble_nus.h"
#include	"app_uart.h"
#include	"app_util_platform.h"
#include	"app_pwm.h"
#include	"nrf_clock.h"
#include	"nrf_delay.h"
#include	"fstorage.h"
#include  "nrf_drv_twi.h"
#include	"sdk_config.h"


#include	"HummingbirdBitPinout.h"
#include	"HummingbirdBitBLEControl.h"
#include	"HummingbirdBitLEDS.h"
#include	"HummingbirdBitLEDArray.h"
#include	"HummingbirdBitSensors.h"
#include	"HummingbirdBitBuzzer.h"
#include	"HummingbirdBitLEDMapping.h"
#include	"HummingbirdBitRudeWords.h"
#include	"HummingbirdBitFlash.h"
#include	"HummingbirdBitUART.h"
#include	"HummingbirdBitUARTControl.h"
#include	"HummingbirdBitGlobal.h"
#include	"HummingbirdBitBSP.h"
#include	"HummingbirdBitSound.h"
#include	"HummingbirdBitSPIMaster.h"
#include	"HummingbirdBitBLEInit.h"
#include	"HummingbirdBitBLECallbacks.h"
#include	"HummingbirdBitGlobalVariables.h"
#include	"HummingbirdBitNaming.h"

/************************************************************************/


/************************************************************************/
int main(void)
{
   	uint32_t err_code;
	  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		
	
		//Initialize the buzzer
	  init_timer_buzzer();

		//Intializing micro bit LED array
	  init_micro_LEDs();
	
	  //Initlializing the timer to broadcast values 
	  init_broadcast_timer();
	  init_broadcast_uart_timer();
		
		//Initialize UART 
		UARTTXRXInit();
	
		//Initializing the I2C sensors on the microbit
		init_microbit_sensors();
	
		//Initialize SPI
		SPI_init(); 
		
		//Initialize PWM module
		LEDS_PWM_init();
		
		
		//Check if the device is a HummingBirdBit with microbit or just a microbit  
		check_update_name();
		
		
		//Initialize the Bluetooth stack and configure BLE parameters
		BLEInit();
		
		// Get the initials of the microbit and see if there are any badwords in it
		getInitials_fancyName();
		
		//Check if calibration has to be done 
		start_check_update_calibrate();
		
		//Start LED three letter flashing
		start_LEDarray_advertising();
		
		//BLE advertising
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
		
    // Enter main loop.    
		for (;;)
    {
			//nrf_gpio_pin_set(LED2_TEST);
			//Look for values recevied through UART service in bluetooth .
			uart_spi_bridge();
			//Check the connection status based on a flag in the interrupt and upadte the flashing on the LED screen
			check_flashing();
			//Check the connection status and make a appropriatme sound.
			check_sound();
			//Look for values received through UART.
			check_UART();
			//nrf_gpio_pin_clear(LED2_TEST);
    }
}


