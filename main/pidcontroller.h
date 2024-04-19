#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include "thermostat.h"
#include "mqtt_client.h"

extern void pidcontroller_init(char *prefix, uint8_t *chip, int max, int interval, float kp, float ki, float kd);
extern void pidcontroller_adjust(int interval, float kp, float ki, float kd);
extern void pidcontroller_target(float newTarget);
extern int  pidcontroller_tune(float measurement);
extern bool pidcontroller_send(struct measurement *data, esp_mqtt_client_handle_t client);
extern void pidcontroller_send_currenttune(void);

#endif
