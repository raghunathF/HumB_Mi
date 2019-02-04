/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************     Includes       **********************************/
/************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
/************************************************************************/

/************************************************************************/
/******************     Prototypes     **********************************/
/************************************************************************/
void on_adv_evt(ble_adv_evt_t ble_adv_evt);
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
void sys_evt_dispatch(uint32_t sys_evt);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void on_ble_evt(ble_evt_t * p_ble_evt);
void conn_params_error_handler(uint32_t nrf_error);
void ble_evt_dispatch(ble_evt_t * p_ble_evt);
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
void on_ble_evt(ble_evt_t * p_ble_evt);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
/************************************************************************/

