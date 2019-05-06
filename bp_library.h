//#ifndef BP_LIBRARY_H__
//#define BP_LIBRARY_H__

#include <stdint.h>
#include <string.h>

#define RMS_DATA  		(0x2E)
#define PERIOD  		(0x48)
#define PHASE_ANGLE  	(0x4E)
#define ACTIVE_POWER  	(0x5C)
#define FUND_POWER  	(0x5E)
#define REACTIVE_POWER  (0x60)
#define RMS_POWER  		(0x62)
#define APPARENT_POWER (0x64)
/**@brief Function to get CRC8 */
uint8_t crc8(const void * data, size_t size);

/**@brief Function for removing first chars */
char * remove_first_char(char * data);

/**@brief Function for mirroring SPI TX frame */
void mirror_data(uint8_t * data, size_t size, uint8_t * buffer);
