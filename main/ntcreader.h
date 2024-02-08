#ifndef __NTCREADER__
#define __NTCREADER__

#include "thermostat.h"
#include "mqtt_client.h"


bool ntc_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);
void ntc_init(uint8_t *chip);
void ntc_close(void);

#endif