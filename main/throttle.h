#ifndef __THROTTLE_H__
#define __THROTTLE_H__

#include "mqtt_client.h"

extern void throttle_init(uint8_t *chip);
extern void throttle_setup(float limit, int stepsPerC);
extern int throttle_check(float temperature, int tune);
extern void throttle_publish(char *prefix, char *appname, esp_mqtt_client_handle_t client);

#endif