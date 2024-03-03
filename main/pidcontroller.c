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
static float tempDiverge = 0.05; // If we are this close or less to the target, we don't change the power level
static float speedDiverge = 6.0;  // temperature should change at least this much in a hour to consider speed has changed
static float maxSpeed = 30.0;     // max speed increase Celsius per hour.
static time_t prevCheck;
static float prevValue = 0.0;
static float prevSpeed = 0.0;
static int tuneValue = 0;
static bool overHeatPossibility = false;

static char thermostatTopic[64];

static void calctune(void)
{
    float degrPerStep = startDiff / (float) maxTune;
    printf("--------> degrPerStep = %f", degrPerStep);

    float diff = target - prevValue;
    tuneValue += (int) (diff / degrPerStep);
    if (tuneValue >= (maxTune -3)) overHeatPossibility = true;
    if (tuneValue >= maxTune) tuneValue = maxTune - 1;
    if (tuneValue < 0) tuneValue = 0;
    printf("calctune, tune is %d\n", tuneValue);
}

static void send_changes(int newTune, bool forced)
{
    static int prevTune = -1;

    if (prevTune != newTune || forced)
    {

        struct measurement meas;
        meas.id = HEATER;
        meas.gpio = 0;
        meas.data.count = newTune;
        xQueueSend(evt_queue, &meas, 0);
        prevTune = newTune;
    }
}

void pidcontroller_send_currenttune(void)
{
    send_changes(tuneValue, true);
}

void pidcontroller_reinit(int max, float diff, float tDiverge, float sDiverge, float maxs)
{
    startDiff       = diff;
    maxTune         = max;
    tempDiverge     = tDiverge;
    speedDiverge    = sDiverge;
    maxSpeed        = maxs;

    calctune();
    send_changes(tuneValue, true);
    return;
}


void pidcontroller_init(uint8_t *chip, int max, float diff, float tDiverge, float sDiverge, float maxs)
{
    chipid = chip;
    time(&prevCheck);
    pidcontroller_reinit(max,diff,tDiverge,sDiverge, maxs);
    return;
}



// changing the target while running.
void pidcontroller_target(float newTarget)
{
    if (newTarget != target)
    {
        target = newTarget;

        calctune();
        send_changes(tuneValue, true);
    }    
}


// we have got a new temperature measurement.
// let's see if we have to do something.
int pidcontroller_tune(float measurement)
{
    time_t now;
    float speed = 0.001;
    float acceleration = 0.0;
    char *strtime;
    static bool sendTune = true;

    float diff = target - measurement;

    time(&now);
    if (prevValue == 0.0) // at startup
    {
        prevCheck = now;
        prevValue = measurement;
        return tuneValue;
    }    


    // speed is degrees per hour
    int secondsPassed = now - prevCheck;
    strtime=asctime(localtime(&now));
    strtime[strlen(strtime)-1] = 0;

    speed = 3600 * (measurement - prevValue) / secondsPassed;
    prevValue = measurement;
    prevCheck = now;

    if (prevSpeed == 0.0) prevSpeed = 0.001;

    if (secondsPassed)
    {
        acceleration = speed / (speed - prevSpeed);
        if (acceleration) acceleration = 3600 / secondsPassed * acceleration;
    }

    printf("%s> Temperature = %f Speed = %f, PrevSpeed = %f Accleration = %f\n", strtime, measurement, speed, prevSpeed, acceleration);
    prevSpeed = speed;

    if (fabs(diff) < tempDiverge)
    {   // trying to stay near target without changing tune too much
        printf("\tIn peaceful area.\n");
        if (speed >= speedDiverge) {
            tuneValue--;
            printf("\tspeed value was too big %f compared to %f. Turned tune down to %d\n", speed, speedDiverge, tuneValue);
        }
        else if (speed <= (speedDiverge * -1.0))
        {
            tuneValue++;
            printf("\tspeed value was too low %f compared to %f. Turned tune up to %d\n", speed, speedDiverge * -1.0, tuneValue);
        }
        else
        {
            printf("\tspeed is under limits %f compared to %f. Keeping the tune %d\n", speed, speedDiverge, tuneValue);
            return tuneValue; // we are quite close, and speed is slow, don't change tune.
        }
    }
    else {
        if (speed > maxSpeed)
        {                    // don't care are we under target.
            tuneValue--;     // Speed is too high, even we may be under target
            printf("\tSpeed is too high, %f compared to maxspeed %f ", speed, maxSpeed);
            if (diff < 0.0)  // and if over target, decrease more.
            {
                if (overHeatPossibility)
                {
                    printf("Overheat possibility. ");
                    tuneValue = 0;
                    overHeatPossibility = false;
                }
                else tuneValue--;
                printf("Tunevalue is %d", tuneValue);
            }
            printf("\n");
        }
        else
        {
            // under target
            if (diff > 0.0)
            {
                printf("%s> Under target and acceleration = %f\n", strtime, acceleration);
                if (speed < 0.0)  // going down
                {
                    if (acceleration < -70.0) // but decelerating
                    {
                        tuneValue--; // less heating, temp is already turning
                        printf("\tspeed is going down BUT decelerating, less power %d\n", tuneValue);
                    }
                    else
                    {
                        tuneValue++;                    // dropping continues
                        printf("\tspeed is going down and accelerating, increase power %d\n", tuneValue);
                    }
                }
            }
            // over target
            if (diff < 0.0)
            {
                printf("\ttemperature is over target, decreasing power.");
                tuneValue--; // over target, decrease heating
                if (speed > 0) {
                    tuneValue--; // over target and still increasing.
                    printf("And is increasing.");
                }
                printf("Tunevalue = %d\n", tuneValue);
            }
        }
    }
    // don't overflow or underflow
    if (tuneValue >= maxTune) tuneValue = maxTune-1;
    if (tuneValue < 0) tuneValue = 0;
    send_changes(tuneValue, sendTune);
    sendTune = false;
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
