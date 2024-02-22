#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include "thermostat.h"
#include "mqtt_client.h"

extern void pidcontroller_init(uint8_t *chip, int interval, int max, float diff, float tDiverge, float sDiverge, float maxs);
extern void pidcontroller_reinit(int interval, int max, float diff, float tDiverge, float sDiverge, float maxs);
extern void pidcontroller_target(float newTarget);
extern int  pidcontroller_tune(float pv);
extern bool pidcontroller_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client);

#endif 