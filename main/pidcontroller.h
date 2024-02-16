#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include "thermostat.h"
#include "mqtt_client.h"

void pidcontroller_init(uint8_t *chip, int interval, int max, float diff, float tDiverge, float sDiverge);
void pidcontroller_target(float newTarget);
int pidcontroller_tune(float pv);
bool pidcontroller_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);

#endif 