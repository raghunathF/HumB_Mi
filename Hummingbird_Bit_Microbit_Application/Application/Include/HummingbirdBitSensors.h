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

/************************************************************************/
/******************     Defines        **********************************/
/************************************************************************/
//Pin Out
#define MICRO_SCL_PIN				0
#define MICRO_SDA_PIN				30
#define BUTTONA_PIN			    17
#define BUTTONB_PIN			    26

/* TWI instance ID. */
#define TWI_INSTANCE_ID     1

/* Common addresses definition for temperature sensor. */
#define MMA_ADDR          (0x3AU >> 1)
#define MAG_ADDR          (0x1CU >> 1)
#define LS_MAG_ADDR       (0x3CU >> 1)
#define LS_MMA_ADDR       (0x32U >> 1)


#define MAG_WHO_AM_I_VALUE			0xC4
#define LS_MAG_WHO_AM_I_VALUE		0x40


#define MMA_REG_CTRL1  				 0x2A
#define MMA_REG_WHO_AM_I       0x0D
#define MAG_REG_WHO_AM_I       0x07
#define MMA_REG_X      				 0x01U
#define MMA_REG_Y      				 0x03U
#define MMA_REG_Z      				 0x04U


/* Mode for LM75B. */
#define NORMAL_MODE 0x21

//

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
#define STANDBY_MODE 					0
#define ACTIVE_MODE						0x21

#define CORRECT_LEDARRAY						0x022a200
#define WRONG_LEDARRAY				  		0x1151151
#define MAG_REG_OFFSET_X_MSB				0x09
#define MAG_REG_OFFSET_X_MSB_LS		  0x09
#define LENGTH_OFFSET_DATA          6

#define MIN_THRESHOLD_CALIB         700
#define MIN_THRESHOLD_CALIB_LS      100

#define EXT_REG_HW_LSB 							0x00
#define EXT_ADDR       							0x49


#define TIMEOUT_I2C                 0x03
#define BRIGHTNESS                  0x55

//
#define MMA_REG_CTRL2					0x2B
#define CTRL2_VALUE						0x10
#define MMA_REG_CTRL3					0x2C
#define CTRL3_VALUE						0x00
#define MMA_REG_CTRL4					0x2D
#define CTRL4_VALUE						0x00
#define MMA_REG_CTRL5					0x2E
#define CTRL5_VALUE						0x00
#define MMA_REG_XYZ						0x0E
#define XYZ_VALUE							0x00
#define MMA_REG_SHAKE					0x15
#define SHAKE_VALUE			  		0x78
#define MMA_REG_SHAKE_THRES  	0x17
#define SHAKE_THRES_VALUE  		0x1F
#define MMA_REG_STATUS				0x00
#define MMA_REG_SHAKE_STATUS  0x16
#define MMA_REG_SHAKE_COUNT   0x18
#define SHAKE_COUNT_VALUE	    0x00

#define MAG_REG_CTRL1					0x10
#define MAG_REG_CTRL2					0x11

#define STANDBY_MODE_MAG 			0x00
#define MAG_CTRL2_VALUE				0x80
#define MAG_CTRL2_VALUE_2			0xB0
#define ACTIVE_MODE_MAG				0x61

#define LENGTH_AXIS_DATA 			7
#define LENGTH_SHAKE_DATA 		1
#define LENGTH_READ_WHO_AM_I  1

#define MAG_REG_STATUS 				0x00
#define MAG_REG_TEMP 					0x0F
#define LENGTH_TEMP_DATA 			1

#define MAG_INT_1_PIN	  			27
#define ACCL_INT_1_PIN				28
#define ACCL_INT_2_PIN				29

#define MAG_INT_1_PIN_LS	  	29
#define ACCL_INT_1_PIN_LS			28

#define MAG_CFG_REG_A_LS      0x60
#define MAG_CFG_REG_B_LS			0x61
#define MAG_CFG_REG_C_LS			0x62

#define MMA_REG_CTRL1_LS      0x20
#define MMA_REG_CTRL4_LS      0x23
#define MMA_REG_SHAKE_THS_LS  0x32

#define STANDBY_MODE_LS       0x00
#define ACTIVE_MODE_LS        0x27
#define SHAKE_TH_LS           0x50
#define CTRL4_VALUE_LS        0x80

#define MMA_REG_STATUS_LS     		0xA8
#define MMA_REG_SHAKE_STATUS_LS		0x31


#define MMA_REG_SHAKE_CONFIG_LS   0x30
#define XYZ_SHAKE_ENABLE_LS				0x2A

#define CFG_A_VALUE_S         0x00
#define CFG_B_VALUE_S         0x02
#define CFG_C_VALUE_S         0x08

#define MAG_REG_STATUS_LS 				0x67
#define MAG_REG_OFFSET_X_LSB_LS		0x45

//LS
#define LS_MAG_REG_WHO_AM_I       0x4F 
#define LS_MMA_REG_WHO_AM_I       0x0F 
/************************************************************************/

/************************************************************************/
/******************   Variables        **********************************/
/************************************************************************/
/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;
extern uint8_t input_micro_packet[20];
/************************************************************************/


/************************************************************************/
/******************   Prototypes       **********************************/
/************************************************************************/
void read_data_packet(void);
void init_microbit_sensors(void);
void check_read_sensor_data_mag(void);
/************************************************************************/

