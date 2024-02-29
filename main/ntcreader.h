#ifndef __NTCREADER__
#define __NTCREADER__

#include "thermostat.h"
#include "mqtt_client.h"


extern void ntc_set_calibr_low(float temp);
extern void ntc_set_calibr_high(float temp);
extern bool  ntc_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);
extern void  ntc_sendcurrent(void);
extern float ntc_get_temperature(void);
extern void  ntc_save_calibrations(void);
extern void  ntc_init(uint8_t *chip, int intervalMs);
extern void  ntc_close(void);


#endif