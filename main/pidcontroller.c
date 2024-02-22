#include <time.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "pidcontroller.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"


static uint8_t *chipid;


static float target = 20.0;      // what is the needed temperature, can be changed while running with pidcontroller_target()
static float startDiff = 2.0;    // if measured temperature is target minus startDiff, put heating to full
static int maxTune = 1;          // how many power levels we want to have.
static int checkInterval = 60;   // how often should we compare, has the temperature changed. In seconds
static float tempDiverge = 0.05; // If we are this close or less to the target, we don't change the power level
static float speedDiverge = 6.0;  // temperature should change at least this much in a hour to consider speed has changed
static float maxSpeed = 30.0;     // max speed increase Celsius per hour.
static time_t prevCheck;
static float prevValue = 0.0;
static int tuneValue = 0;

static char thermostatTopic[64];

void pidcontroller_reinit(int interval, int max, float diff, float tDiverge, float sDiverge, float maxs)
{
    checkInterval   = interval;
    startDiff       = diff;
    maxTune         = max;
    tempDiverge     = tDiverge;
    speedDiverge    = sDiverge;
    maxSpeed        = maxs;
    return;
}


void pidcontroller_init(uint8_t *chip, int interval, int max, float diff, float tDiverge, float sDiverge, float maxs)
{
    chipid = chip;
    time(&prevCheck);
    pidcontroller_reinit(interval,max,diff,tDiverge,sDiverge, maxs);
    return;
}

// changing the target while running.
void pidcontroller_target(float newTarget)
{
    if (newTarget != target)
    {
        target = newTarget;

        if (prevValue == 0.0) return;

        float degrPerStep = startDiff / (float) maxTune;
        printf("degrPerStep = %f", degrPerStep);

        float diff = target - prevValue;
        tuneValue += (int) (diff / degrPerStep);
    }    
    if (tuneValue >= maxTune) tuneValue = maxTune - 1;
    if (tuneValue < 0) tuneValue = 0;
    return;
}

static void send_changes(int newTune)
{
    static int prevTune = -1;

    if (prevTune != newTune)
    {
        struct measurement meas;
        meas.id = HEATER;
        meas.gpio = 0;
        meas.data.count = newTune;
        xQueueSend(evt_queue, &meas, 0);
        prevTune = newTune;
    }
}

// we have got a new temperature measurement.
// let's see if we have to do something.
int pidcontroller_tune(float measurement)
{
    time_t now;
    float speed = 0.001;

    float diff = target - measurement;

    time(&now);
    if (prevValue == 0.0) // at startup
    {
        // TODO: do this with same algorithm as in pidcontrol_target()
        if (diff > startDiff) tuneValue = maxTune-1;
        else if (diff > (startDiff / 2)) tuneValue = maxTune / 2;
        prevCheck = now;
        prevValue = measurement;
        send_changes(tuneValue);
        return tuneValue;
    }    

    if ((now - prevCheck) < checkInterval)
        return tuneValue; // no changes

    // speed is degrees per hour
    speed = 3600 * (measurement - prevValue) / (now - prevCheck);
    prevValue = measurement;
    prevCheck = now;

    if (fabs(diff) < tempDiverge) 
    {   // trying to stay near target without changing tune too much
        if (speed >= speedDiverge) tuneValue--;
        else if (speed <= (speedDiverge * -1.0)) tuneValue++;
        else return tuneValue; // we are quite close, and speed is slow, don't change tune.
    }    
    else {
        if (speed > maxSpeed)
        {                    // don't care are we under target.
            tuneValue--;     // Speed is too high, even we may be under target
            if (diff < 0.0)  // and if over target, decrease more.
                tuneValue--;
        }
        else
        {
            if (diff > 0.0) // under target, but too slow -> increase heating
            {
                if (speed <= speedDiverge) tuneValue++; // needs to accelerate
            }
            if (diff < 0.0) // over target
            {
                if (speed >= speedDiverge * -1.0) tuneValue--; // because we still have speed.
            }
        }
    }
    // don't overflow or underflow
    if (tuneValue >= maxTune) tuneValue = maxTune-1;
    if (tuneValue < 0) tuneValue = 0;
    send_changes(tuneValue);
    return tuneValue;
}

bool pidcontroller_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"thermostat\",\"value\":%d,\"ts\":%jd}";
    sprintf(thermostatTopic,"%s/thermostat/%x%x%x/parameters/level", prefix, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                data->data.count,
                now);
    esp_mqtt_client_publish(client, thermostatTopic, jsondata , 0, 0, 1);

    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}
