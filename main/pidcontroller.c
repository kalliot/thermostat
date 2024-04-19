#include <time.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"
#include "pidcontroller.h"


struct pid {
    int interval;
    float target;
    float pgain;
    float igain;
    float dgain;
    float prevValue;
    int maxTune;
    int prevTune;
    int tuneValue;
    time_t prevMeasTs;
    uint8_t *chipid;
    char topic[64];
} pidCtl = {
    15,
    .target = 27,
    5,
    1,
    3,
    0,
    0,
    0,
    0,
    0,
    NULL,
    "nonetopic"
};


static void send_changes(int newTune, bool forced)
{
    if (pidCtl.prevTune != newTune || forced)
    {

        struct measurement meas;
        meas.id = HEATER;
        meas.gpio = 0;
        meas.data.count = newTune;
        xQueueSend(evt_queue, &meas, 0);
        pidCtl.prevTune = newTune;
    }
}


void pidcontroller_send_currenttune(void)
{
    send_changes(pidCtl.tuneValue, true);
}


void pidcontroller_adjust(int interval, float kp, float ki, float kd)
{
    pidCtl.interval = interval;
    pidCtl.pgain = kp;
    pidCtl.igain = ki;
    pidCtl.dgain = kd;
}


void pidcontroller_init(char *prefix, uint8_t *chip, int max, int interval, float kp, float ki, float kd)
{
    pidCtl.chipid = chip;
    sprintf(pidCtl.topic,"%s/thermostat/%x%x%x/parameters/level", prefix, pidCtl.chipid[3], pidCtl.chipid[4], pidCtl.chipid[5]);
    time(&pidCtl.prevMeasTs);
    pidCtl.maxTune = max;
    pidcontroller_adjust(interval, kp, ki, kd);
    return;
}


// changing the target while running.
void pidcontroller_target(float newTarget)
{
    if (newTarget != pidCtl.target)
    {
        pidCtl.target = newTarget;
        pidcontroller_tune(pidCtl.prevValue);
        send_changes(pidCtl.tuneValue, true);
    }    
}


int pidcontroller_tune(float measurement)
{
    float integral, differential;
    int tuneval, elapsed;
    float error = pidCtl.target - measurement;
    float correction = 1;
    time_t now;
    
    
    time(&now);
    elapsed = now - pidCtl.prevMeasTs;

    if (elapsed != 0 && elapsed < 10 * pidCtl.interval)   // first round and after system has refreshed network time
    {
        correction = ((float) pidCtl.interval) / ((float) elapsed);
        printf("elapse %d seconds, correction for interval is %f", elapsed, correction);
    }
    

    integral = correction * pidCtl.igain * error;
    if (integral > pidCtl.maxTune)
        integral = pidCtl.maxTune;
    if (integral < 0)
        integral = 0;

    differential = measurement - pidCtl.prevValue;
    tuneval = pidCtl.pgain * error + integral - pidCtl.dgain / correction * differential;
    if (tuneval > pidCtl.maxTune)
        tuneval = pidCtl.maxTune;
    if (tuneval < 0)
        tuneval = 0;

    pidCtl.prevValue = measurement;
    pidCtl.prevMeasTs = now;
    send_changes(tuneval, false);
    return tuneval;
}


bool pidcontroller_send(struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"thermostat\",\"value\":%d,\"ts\":%jd}";
    sprintf(jsondata, datafmt,
                pidCtl.chipid[3],pidCtl.chipid[4],pidCtl.chipid[5],
                data->data.count,
                now);
    esp_mqtt_client_publish(client, pidCtl.topic, jsondata , 0, 0, 1);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}
