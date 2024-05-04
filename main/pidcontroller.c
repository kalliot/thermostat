#include <time.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"
#include "pidcontroller.h"

static const char *TAG = "pidcontroller";

static void insert_queue(int tune)
{
    struct measurement meas;
    meas.id = HEATER;
    meas.gpio = 0;
    meas.data.count = tune;
    xQueueSend(evt_queue, &meas, 0);
}

void pidcontroller_send_last(PID *p)
{
    insert_queue(p->prevTune);
}

void pidcontroller_send_tune(PID *p, int newTune, bool forced)
{
    if (p->prevTune != newTune || forced)
    {
        insert_queue(newTune);
        p->prevTune = newTune;
    }
}


void pidcontroller_adjust(PID *p, float maxTemp, int interval, float kp, float ki, float kd)
{
    p->interval = interval;
    p->pgain = kp;
    p->igain = ki;
    p->dgain = kd;
    p->maxTemp = maxTemp;
}


void pidcontroller_init(PID *p, char *prefix, uint8_t *chip, float maxTemp, int max, int interval, float kp, float ki, float kd)
{
    p->chipid = chip;
    sprintf(p->topic,"%s/thermostat/%x%x%x/parameters/level", prefix, p->chipid[3], p->chipid[4], p->chipid[5]);
    time(&p->prevMeasTs);
    p->maxTune = max;
    p->integral = 0;
    pidcontroller_adjust(p, maxTemp, interval, kp, ki, kd);
    return;
}


// changing the target while running.
void pidcontroller_target(PID *p, float newTarget)
{
    if (newTarget > p->maxTemp) newTarget = p->maxTemp;
    if (newTarget != p->target)
    {
        p->target = newTarget;
        pidcontroller_send_tune(p, pidcontroller_tune(p, p->prevValue), true);
    }    
}


int pidcontroller_tune(PID *p, float measurement)
{
    float differential;
    int tuneval, elapsed;
    float error = p->target - measurement;
    float correction = 1;
    time_t now;
    
    
    time(&now);
    elapsed = now - p->prevMeasTs;

    if (elapsed >= p->interval && elapsed < 10 * p->interval)   // first round and after system has refreshed network time
    {                                                           // elapsed is huge, just after ntc time sync, so reject it.
        correction = ((float) p->interval) / ((float) elapsed);
        p->integral += p->igain * error * correction;
        if (p->integral > p->maxTune)
            p->integral = p->maxTune;
        if (p->integral < 0)
            p->integral = 0;
    }

    differential = p->dgain * (measurement - p->prevValue);
    float ft = p->pgain * error + p->integral - differential / correction;
    tuneval = ft + 0.5; // correct rounding from float to int.

    ESP_LOGI(TAG,"%d sec, corr=%.3f, pc=%.04f, ic=%.4f, dc=%.04f, tune = %.2f",
        elapsed, correction,
        p->pgain * error, p->integral, differential / correction, ft);

    if (measurement > p->maxTemp)
    {
        tuneval = 0;
    }
    else
    {
        if (tuneval > p->maxTune)
            tuneval = p->maxTune;
        if (tuneval < 0)
            tuneval = 0;
    }
    p->prevValue = measurement;
    p->prevMeasTs = now;
    return tuneval;
}


bool pidcontroller_publish(PID *p, struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static const char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"thermostat\",\"value\":%d,\"ts\":%jd}";
    sprintf(jsondata, datafmt,
                p->chipid[3],p->chipid[4],p->chipid[5],
                data->data.count,
                now);
    esp_mqtt_client_publish(client, p->topic, jsondata , 0, 0, 1);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}
