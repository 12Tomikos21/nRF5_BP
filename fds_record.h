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
    uint32_t sign_active_power;
    uint32_t sign_reactive_power;
    uint32_t overflow_active_energy;
    uint32_t overflow_reactive_energy;
    uint32_t data1;
    uint32_t crc;
} configuration_t;


void delete_all_begin(void);
bool record_delete_next(void);


#endif
