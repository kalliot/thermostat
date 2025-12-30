#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "throttle.h"
#include "homeapp.h"
#include "statistics/statistics.h"

static float limitC = 33.0;
static int stepsPerC = 10;
static uint8_t *chipid;

static const char *TAG = "throttle";
static char *idlimit = "thr.limit";
static char *idsteps = "thr.steps";
static char *throttleTopic;

static void sendcurrent(int curr);

void throttle_init(uint8_t *chip)
{
    chipid = chip;

    throttleTopic = (char *) malloc(64);
    if (throttleTopic == NULL) return;

    limitC    = flash_read_float(setup_flash, idlimit, limitC);
    stepsPerC = flash_read(setup_flash, idsteps, stepsPerC);
}

void throttle_setup(float limit, int stepsPerC)
{
    if (limit <= 40)  // dont allow user to give too big limit
        limitC = limit;
    stepsPerC = stepsPerC;
}

// decrease tune if temperature is above limit
// return new tune value
int throttle_check(float temperature, int tune)
{
    float diff;
    int ret = tune;
    int throttling = 0;

    diff = temperature - limitC;
    throttling = (int) (diff * stepsPerC);
    ret = tune - throttling;
    if (ret < 0)
        ret = 0;
    sendcurrent(throttling);
    return ret;
}

void throttle_saveSettings(void)
{
    ESP_LOGI(TAG,"saving settings to flash");
    flash_write(setup_flash, idsteps, stepsPerC);
    flash_write_float(setup_flash, idlimit, limitC);
    flash_commitchanges(setup_flash);
}

static void sendcurrent(int throttle)
{
    static int prev = -1;
    struct measurement meas;

    if (throttle < 0) throttle = 0;
    if (throttle != prev)
    {
        meas.id = THROTTLE;
        meas.gpio = 0;
        meas.data.count = throttle;
        xQueueSend(evt_queue, &meas, 0);
        prev = throttle;
    }
}

void throttle_pubvalue(char *prefix, char *appname, struct measurement *meas, esp_mqtt_client_handle_t client)
{
    time_t now;
    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"throttling\",\"value\":%d,\"ts\":%jd}";
    int retain = 1;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    if (now < MIN_EPOCH)
    {
        now = 0;
        retain = 0;
    }

    sprintf(throttleTopic,"%s/%s/%x%x%x/parameters/throttling", prefix, appname, chipid[3], chipid[4], chipid[5]);
    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                meas->data.count, now);

    esp_mqtt_client_publish(client, throttleTopic, jsondata , 0, 0, retain);
    statistics_getptr()->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return;
}

void throttle_publish(char *prefix, char *appname, esp_mqtt_client_handle_t client)
{
    time_t now;
    int retain = 1;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    if (now < MIN_EPOCH)
    {
        now = 0;
        retain = 0;
    }

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"throttle\",\"limit\":%.02f,\"steps\":%d,\"ts\":%jd}";
    sprintf(throttleTopic,"%s/%s/%x%x%x/setup/throttle", prefix, appname, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                limitC,
                stepsPerC, now);
    esp_mqtt_client_publish(client, throttleTopic, jsondata , 0, 0, retain);
    statistics_getptr()->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
}
