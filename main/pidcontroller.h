#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

#include "homeapp.h"
#include "mqtt_client.h"


typedef struct {
    // setup variables
    int interval;
    float maxTemp;
    float target;
    float pgain;
    float igain;
    float dgain;
    int maxTune;

    // internal variables used while running
    float prevValue;
    int prevTune;
    int tuneValue;
    float integral;
    time_t prevMeasTs;
    uint8_t *chipid;
    char topic[64];
} PID;


extern void pidcontroller_init  (PID *p, char *prefix, uint8_t *chip, float maxTemp, int max, int interval, float kp, float ki, float kd);
extern void pidcontroller_adjust(PID *p, float maxTemp, int interval, float kp, float ki, float kd);
extern void pidcontroller_target(PID *p, float newTarget);
extern int  pidcontroller_tune  (PID *p, float measurement);
extern bool pidcontroller_publish  (PID *p, struct measurement *data, esp_mqtt_client_handle_t client);
extern void pidcontroller_send_tune(PID *p, int newTune, bool forced);
extern void pidcontroller_send_last(PID *p);

#endif
