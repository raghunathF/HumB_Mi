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

#define MMA_REG_CTRL1  				 0x2A
#define MMA_REG_WHO_AM_I       0x0D
#define MAG_REG_WHO_AM_I       0x07
#define MMA_REG_X      				 0x01U
#define MMA_REG_Y      				 0x03U
#define MMA_REG_Z      				 0x04U


/* Mode for LM75B. */
#define NORMAL_MODE 0x21

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;


/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;



//

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
#define STANDBY_MODE 					0
#define ACTIVE_MODE						0x21


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


#define MAG_REG_STATUS 				0x00
#define MAG_REG_TEMP 					0x0F
#define LENGTH_TEMP_DATA 			1


#define MAG_INT_1_PIN	  			27
#define ACCL_INT_1_PIN				28
#define ACCL_INT_2_PIN				29


extern uint8_t input_micro_packet[20];

void read_verify_offset_mag(void);
void read_data_packet();
void init_microbit_sensors(void);
void calibrate_compass();
void write_offset(uint8_t* offset);