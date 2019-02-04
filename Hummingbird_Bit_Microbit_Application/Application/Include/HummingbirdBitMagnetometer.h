/*
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*|Author						|					Raghunath J	  																													|
*|Last Edit      		|					10/31/2018																															|	
*||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/
/************************************************************************/
/******************     Prototypes     **********************************/
/************************************************************************/
void read_sensor_data_mag(void);
void read_sensor_data_mag_ls(void);
void MAG_set_mode_ls(void);
void MAG_set_mode(void);
void send_I2C_mag_ls(uint8_t const* data,uint8_t length);
void send_I2C_mag(uint8_t const* data,uint8_t length);
void read_verify_offset_mag_ls(void);
void read_verify_offset_mag(void);
void convert_axis_scale(void);
void write_offset_ls(uint8_t* offset);
void write_offset(uint8_t* offset);
uint8_t find_version(void);
void check_write_offset(uint8_t* offset);
void read_mag_packet(void);
void read_mag_packet_ls(void);
/************************************************************************/
