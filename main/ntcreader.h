#ifndef __NTCREADER__
#define __NTCREADER__

#include "thermostat.h"
#include "mqtt_client.h"


bool ntc_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);
void ntc_sendcurrent(void);
float ntc_get_temperature(void);
void ntc_init(uint8_t *chip, int intervalMs);
void ntc_close(void);

#endif