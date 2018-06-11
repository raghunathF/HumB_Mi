/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_pwm.h"



#include "bsp.h"
#include "bsp_btn_ble.h"
#include "finch_pin.h"
#include "finch_bsp.h"
#include "nrf_delay.h"
#include "SPI_master.h"
#include "UART_SPI.h"
#include "HM_LEDS.h"
#include "microbit_LEDS.h"
#include "micro_bit_sensors.h"
#include "buzzer.h"
#include "sound_buzzer.h"
#include "ledarray_map.h"
#include "fstorage.h"

#include  "f_save.h"

uint8_t INITIAL_NAME[2] = {'M','B'};


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define TEST_PIN_1 											18
#define TEST_PIN_2 											1

//#define INITIAL_LENGTH 									4

ble_nus_t                               m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static uint8_t index = 0;

volatile bool broadcast_flag = false;
volatile bool UART_interrupt_flag = false;
volatile bool spi_xfer_done = true;

uint16_t transmit_length = 0;


APP_TIMER_DEF(transmit_timer_id);

static bool first_byte = true;
static uint8_t length_receive = 0 ;
static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
volatile  uint32_t LED_value = 0;

uint8_t    sensor_outputs[20];


const char possible_names[] = {'F','N','H','M','F','L','H','B','B','2','C','3'};   //Just to be safe used A1,B2,C3
char 	DEVICE_NAME [20];



int mood_bit = 0;
uint8_t temp_p_data[20];
uint8_t temp_p_sensor_data[20];
volatile uint8_t rgb_value[3];
uint8_t input_micro_packet[20];


bool led_change = false;
bool fake_firware_send = false;
bool app_selected = false;

bool start_advertising_flashing  = false;
bool stop_advertising_flashing   = false;

bool LED_array_enable = true;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
char convert_ascii(char input)
{
	char output;
	if(input >= 0 & input <=9)
	{
		output = input + 0x30;
	}
	else
	{
		output = input + 0x37;
	}
	return output;
}



void set_devicename_array()
{
		ble_gap_addr_t 					mac;
    sd_ble_gap_address_get(&mac);
		
	  DEVICE_NAME[0] = INITIAL_NAME[0];
		DEVICE_NAME[1] = INITIAL_NAME[1];
	  DEVICE_NAME[2] = convert_ascii((mac.addr[2]&0x0F));
	
	
		DEVICE_NAME[3] = (mac.addr[1]&0xF0);
		DEVICE_NAME[3] = convert_ascii(DEVICE_NAME[3]>>4);
		
	
		DEVICE_NAME[4] = convert_ascii(mac.addr[1]&0x0F);
		DEVICE_NAME[5] = (mac.addr[0]& 0xF0);
		DEVICE_NAME[5] = convert_ascii(DEVICE_NAME[5]>>4);
		
		DEVICE_NAME[6] = convert_ascii(mac.addr[0]&0x0F);
		DEVICE_NAME[7] = '\0';
     
}




/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.This is where initial GAPNAME is set
 */
static void gap_params_init(void)
{
    uint32_t                	err_code;
    ble_gap_conn_params_t   	gap_conn_params;
    ble_gap_conn_sec_mode_t 	sec_mode;	
		
		uint8_t 									addr_byte	 = 0;
	
		set_devicename_array();
	  
	  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
	  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));	
    
	  APP_ERROR_CHECK(err_code);

																					
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

																					
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */

void change_led()
{
	
	 uint32_t temp1 = 0;
	 uint32_t temp2 = 0;
	 uint32_t temp3 = 0;
	 

	 /*
	 if(temp_p_data[0] == 'A')
		{
			
				 temp1 = temp_p_data[7] ;
				 temp2 = temp_p_data[8] ; 
				 temp2 = temp2 << 8; 
				 temp3 = temp_p_data[9] ; 
				 temp3 = temp3 << 16;
				 temp1 = temp1 + temp2 + temp3;
				 LED_value = temp1;
				 led_change = true;
				 rgb_value[0]= temp_p_data[1];
				 rgb_value[1]= temp_p_data[2];
				 rgb_value[2]= temp_p_data[3];
				 
		}
		*/
		
}

static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{	 	
			uint32_t i =0;
			/*
			for ( i = 0; i < length; i++)
			{
					
					while (app_uart_put(p_data[i]) != NRF_SUCCESS);
			}
			*/
	   // nrf_gpio_pin_set(TEST_PIN_2);
			UART_interrupt_flag = true;
	    transmit_length = length; 
			for ( i = 0; i < length; i++)
			{
					
					temp_p_data[i] = p_data[i];
			}
			//nrf_gpio_pin_clear(TEST_PIN_2);
			//change_led();
			//while (app_uart_put('\r') != NRF_SUCCESS);
			//while (app_uart_put('\n') != NRF_SUCCESS);
	  //}
}

/**@snippet [Handling the data received over BLE] */
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

uint8_t sound_effect =  0;

#define SOUND_STARTUP         1 
#define SOUND_CONNECTION      2
#define SOUND_DISCONNECTION   3


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    sound_effect  = SOUND_CONNECTION;
				    stop_advertising_flashing = true;
				    //stop_flashing_timer();
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				    sound_effect  = SOUND_DISCONNECTION;
				    start_advertising_flashing = true;
				    //start_LEDarray_advertising();
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    //bsp_btn_ble_on_ble_evt(p_ble_evt);
}



/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
		fs_sys_event_handler(sys_evt);
}


		

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		
		// Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}
#define SENSORS_LENGTH  5


// Timeout handler for the single shot timer
static void transmit_handler(void * p_context)
{
	uint32_t err_code;
	err_code = ble_nus_string_send(&m_nus, data_array, index+1);
	
	first_byte = true;
	index = 0;
	if (err_code != NRF_ERROR_INVALID_STATE)
	{
			APP_ERROR_CHECK(err_code);
	}
}

static void create_timers()
{   
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&transmit_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                transmit_handler);
    APP_ERROR_CHECK(err_code);
}


//
void start_timer()
{
	uint32_t error_code_1=0;
	error_code_1 = app_timer_start(transmit_timer_id, APP_TIMER_TICKS(4, APP_TIMER_PRESCALER), NULL);
  APP_ERROR_CHECK(error_code_1);
}



/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    
    volatile uint32_t       err_code;
		static uint8_t tmpByte = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
        		app_uart_get(&tmpByte);
						if(first_byte == true)
						{
							start_timer();
							first_byte = false;
							index = 0;
							data_array[index] = tmpByte;
							
						}
						else
						{
							index++;
							data_array[index] = tmpByte;							
						}
	
            break;

        case APP_UART_COMMUNICATION_ERROR:
					  err_code = p_event->data.error_code;
						app_uart_flush();
            break;

        case APP_UART_FIFO_ERROR:
					  err_code = p_event->data.error_code;
						app_uart_flush();
            break;

        default:
            break;
    }
}


/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_MICRO,
        TX_MICRO,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
/*
static int* init_LED_array()
{
	int LRC_array[12];
	LRC_array[0] = LROW_0;
	LRC_array[1] = LROW_1;
	LRC_array[2] = LROW_2;
  LRC_array[3] = LCOL_0;
	LRC_array[4] = LCOL_1;
	LRC_array[5] = LCOL_2;
	LRC_array[6] = LCOL_3;
	LRC_array[7] = LCOL_4;
	LRC_array[8] = LCOL_5;
	LRC_array[9] = LCOL_6;
	LRC_array[10] = LCOL_7;
	LRC_array[11] = LCOL_8;
	return LRC_array;
}
*/
/*
static void leds_init(void)
{
	  int* LRC_array = NULL;
		int i =0;	
	  //LRC_array = init_LED_array();
	
	  //for(i=0; i<= TOTAL_RC ;i++)
	  //{
		 // nrf_gpio_cfg_output(LRC_array[i]);
			//nrf_gpio_pin_clear(LRC_array[i]);
	  //}
		
	  nrf_gpio_cfg_output(LROW_0);
	  nrf_gpio_cfg_output(LCOL_0);
		nrf_gpio_cfg_input(BUTTON_A,NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(BUTTON_B,NRF_GPIO_PIN_PULLUP);

	  
	  
		//nrf_gpio_cfg_input(ADVERTISE_PIN,NRF_GPIO_PIN_PULLUP);	
		
	//	nrf_gpio_cfg(ADDR0_PIN, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	//	nrf_gpio_cfg(ADDR1_PIN, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	//	nrf_gpio_cfg(ADDR2_PIN, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	
		//nrf_gpio_cfg_input(ADDR0_PIN,NRF_GPIO_PIN_PULLUP);		
		//nrf_gpio_cfg_input(ADDR1_PIN,NRF_GPIO_PIN_PULLUP);		
		//nrf_gpio_cfg_input(ADDR2_PIN,NRF_GPIO_PIN_PULLUP);		
}
*/



/**@brief Application main function.
 */

/*
void check_BLE_data()
{
	uint32_t temp =0;
	temp = LED_value;
	set_all_led_once(temp);
	if(led_change == true)
	{
		led_change = false;
		LED_control(rgb_value[0],rgb_value[1],rgb_value[2]);
	}
}
*/

void init_temp_p_data()
{
	
		temp_p_data[0]  = 0xCA;
	  temp_p_data[1]  = 0xC1;
	  temp_p_data[2]  = 0xC2;
	  temp_p_data[3]  = 0xC3;
		temp_p_data[4]  = 0xC4;
	  temp_p_data[5]  = 0xC5;
	  temp_p_data[6]  = 0xC6;
	  temp_p_data[7]  = 0xC7;
		temp_p_data[8]  = 0xC8;
	  temp_p_data[9]  = 0xC9;
	  temp_p_data[10] = 0xCA;
	  temp_p_data[11] = 0xCB;
		temp_p_data[12] = 0xCC;
	  temp_p_data[13] = 0xCD;
	  temp_p_data[14] = 0xCE;
	  temp_p_data[15] = 0xCF;
		temp_p_data[16] = 0xD0;
		temp_p_data[17] = 0xD1;
		temp_p_data[18] = 0xD2;
}


void check_sound()
{
	if( sound_effect > 0)
	{
		
		switch (sound_effect)
		{ 
			case SOUND_STARTUP:
				buzzer_start_up();
				break;
			case SOUND_CONNECTION:
				buzzer_bluetooth_connection();
				break;
			case SOUND_DISCONNECTION:
				buzzer_bluetooth_disconnection();
				break;
			default: 
				break;
	
		}
	}
	sound_effect = 0;
}

uint8_t initials_name[3];



void getInitials_fancyName()
{
	  ble_gap_addr_t 					mac;
    sd_ble_gap_address_get(&mac);
		volatile uint32_t temp = 0;
	  uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0; 	
    temp  |= (uint32_t)(mac.addr[2]&0x0F) << 16;	
	  temp  |= (uint32_t)mac.addr[1] << 8;
	  temp  |= (uint32_t)mac.addr[0];
		mod16  =  temp%16;
	  top8   =  temp%256;
	  mid6   =  (temp/256)%64;
    bot6   =	(temp/256)/64;
		initials_name[0] = name_first[top8 + mod16];
    initials_name[1] = name_second[mid6 + mod16];
	  initials_name[2] = name_third[bot6 + mod16];
}


void check_flashing()
{
	if(start_advertising_flashing == true)
	{
		start_LEDarray_advertising();
		start_advertising_flashing = false;
	}
	else if(stop_advertising_flashing == true)
	{
		stop_LEDarray_advertising();
		stop_advertising_flashing = false;
	}
}

void test_LED_init()
{
	nrf_gpio_cfg_output(TEST_PIN_1);
	nrf_gpio_pin_clear(TEST_PIN_1);
	nrf_gpio_cfg_output(TEST_PIN_2);
	nrf_gpio_pin_clear(TEST_PIN_2);
}


int main(void)
{
	
   	volatile uint32_t err_code;
    bool erase_bonds;
	  static bool first_time = false;
		static uint16_t count_broadcast = 0;
		LED_value = 0x0055555555;
		
	  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	  init_buzzer();
	  
		//test_LED_init();
	  //buzzer_HB_control(5);
	  LEDS_PWM_init();
	
	  init_temp_p_data();
	  init_micro_LEDs();
			
	  init_microbit_sensors();
		
	  //init_finch_LED();
		//init_HM_LEDS();
	  
		SPI_init();  

		//Send Initial 
		check_update_name();


	  // Initialize.
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

		getInitials_fancyName();
		
		
		start_check_update_calibrate();
		//calibrate_compass();
		//calibrate_compass();
		//calibrate_compass();
		
		start_LEDarray_advertising();

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
    // Enter main loop.    
		for (;;)
    {
			uart_spi_bridge();
			check_flashing();
			check_sound();
    }
	
}


/**
 * @}
 */
