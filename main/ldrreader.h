#ifndef __LDRREADER__
#define __LDRREADER__

#include "homeapp.h"
#include "mqtt_client.h"
#include "esp_adc/adc_oneshot.h"


extern bool  ldr_publish(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);
extern void  ldr_sendcurrent(void);
extern float ldr_get_brightness(void);
extern bool  ldr_init(uint8_t *chip, adc_oneshot_unit_handle_t adc_handle, int intervalMs, int resolution);
extern void  ldr_close(void);


#endif