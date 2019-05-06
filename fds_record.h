#ifndef FDS_EXAMPLE_H__
#define FDS_EXAMPLE_H__

#include <stdint.h>

/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0xF010)
#define CONFIG_REC_KEY  (0x7010)

/* A dummy structure to save in flash. */
typedef struct
{
    uint32_t timestamp;
    uint32_t period;
    uint32_t rms_data;
    uint32_t phase_angle;
    uint32_t active_power;
    uint32_t fund_power;
    uint32_t reactive_power;
    uint32_t rms_power;
    uint32_t apparent_power;
    uint32_t crc;
} configuration_t;


void delete_all_begin(void);
bool record_delete_next(void);


#endif
