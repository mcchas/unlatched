#ifndef NVS_HELPERS_H
#define NVS_HELPERS_H

#include <iohc_packet.h>

bool nvs_init();

bool nvs_read_system_key(uint8_t key[16]);

#endif  // NVS_HELPERS_H
