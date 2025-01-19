//ldrreader.c


#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "flashmem.h"
#include "statistics/statistics.h"
#include "homeapp.h"

static adc_oneshot_unit_handle_t adc1_handle;
static uint8_t *chipid;
static char ldrTopic[64];
static int sampleInterval = 1000;
static uint8_t brightness;
static uint8_t prev_brightness = 0;
static int prev_rawvalue = 1;
static SemaphoreHandle_t mutex;
static int divider = 1;
static int minrawdiff = 1;

static const char *TAG = "ldrreader";


static int convert(int raw)
{
    if (raw == 0) return 0;
    return raw / divider;
}

static int ldr_read(void)
{
    int adc_raw;

    adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_raw);
    return adc_raw;
}

static void queue_measurement(uint8_t luminance)
{
    struct measurement meas;
    meas.id = LDR;
    meas.gpio = 39;
    meas.data.count = luminance;
    xQueueSend(evt_queue, &meas, 0);
}

float ldr_get_brightness(void)
{
    for (int i=0; i < 3; i++)
    {
        if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
        {
            prev_brightness = brightness;
            xSemaphoreGive(mutex);
            break;
        }
    }
    return prev_brightness;
}

static void ldr_reader(void* arg)
{
    for(;;)
    {
        int raw = ldr_read();
        int rawdiff = 1;

        if (xSemaphoreTake(mutex, (TickType_t) 1000) == pdTRUE)
        {
            brightness = convert(raw);
            rawdiff = abs(prev_rawvalue - raw);
            //ESP_LOGI(TAG, "rawdiff = %d, minrawdiff = %d", rawdiff, minrawdiff);
            if (rawdiff > minrawdiff && brightness != prev_brightness)
            {
                prev_brightness = brightness;
                prev_rawvalue = raw;
                queue_measurement(brightness);
            }   
            xSemaphoreGive(mutex); 
        }
        vTaskDelay(sampleInterval / portTICK_PERIOD_MS);
    }
}


void ldr_sendcurrent(void)
{
    if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
    {
        queue_measurement(brightness);
        xSemaphoreGive(mutex);
    }
}

bool ldr_publish(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client)
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

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"sensor\":\"ldr\",\"id\":\"brightness\",\"value\":%d,\"ts\":%jd}";
    sprintf(ldrTopic,"%s/thermostat/%x%x%x/parameters/brightness", prefix, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                data->data.count,
                now);
    esp_mqtt_client_publish(client, ldrTopic, jsondata , 0, 0, retain);
    statistics_getptr()->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}



bool ldr_init(uint8_t *chip, adc_oneshot_unit_handle_t adc_handle, int intervalMs, int resolution)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG,"failed to create mutex");
        return false;
    }

    chipid = chip;
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_6
    };
    adc1_handle = adc_handle;
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &config);
    // mutex is here not needed, we are not yet threading.
    divider = 4096 / resolution;
    minrawdiff = divider / 10;
    brightness = convert(ldr_read());
    ESP_LOGI(TAG,"ldr_init, first brightness read is %f", brightness);
    sampleInterval = intervalMs;
    xTaskCreate(ldr_reader, "ldr reader", 2048, NULL, 10, NULL);
    return true;
}

void ldr_close(void)
{
    return;
}

