//#ifndef BP_LIBRARY_H__
//#define BP_LIBRARY_H__

#include <stdint.h>
#include <string.h>

/**@brief Function to get CRC8 */
uint8_t crc8(const void * data, size_t size);

/**@brief Function for removing first chars */
char * remove_first_char(char * data);

/**@brief Function for mirroring SPI TX frame */
void mirror_data(uint8_t * data, size_t size, uint8_t * buffer);
