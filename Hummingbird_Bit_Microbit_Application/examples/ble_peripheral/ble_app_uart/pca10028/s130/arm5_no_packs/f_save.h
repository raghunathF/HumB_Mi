#include <stdbool.h>
#include <stdint.h>
#include <string.h>


void write_calib_values( volatile uint8_t* offset);
uint16_t* read_calib_values();
void start_check_update_calibrate();