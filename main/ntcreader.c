
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"


#define MEASURES_PER_SAMPLE 10

static adc_oneshot_unit_handle_t adc1_handle;
static uint8_t *chipid;
static char temperatureTopic[64];
static int sampleInterval = 1000;
static float temperature;
float prev = 0.0;

/* TODO: calibrations should be stored in flash */
static struct {
    int raw;
    float temp;
} calibr[] = {
    {3269, 22.22},
    {3811, 29.00}
};


static float convert(int raw)
{
                                                   // interpolate                extrapolate, under cal minimum
    float rdiff = calibr[1].raw - calibr[0].raw;   // 516                        516
    float cdiff = calibr[1].temp - calibr[0].temp; // 6.58                       6.58
    float d = raw - calibr[0].raw;                 // 7                          -7
    float x = d / rdiff;                           // 7 / 516 = 0.013566         -7 / 516 = -0.013566
    return x * cdiff + calibr[0].temp;             // 0.013566 * 6.58 + 22.22    -0.013566 * 6.58 + 22.22
}

static int ntc_read(void)
{
    int adc_raw;

    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw);
    return adc_raw;
}

static void queue_measurement(float tempval)
{
    struct measurement meas;
    meas.id = NTC;
    meas.gpio = 36;
    meas.data.temperature = tempval;
    xQueueSend(evt_queue, &meas, 0);
}

float ntc_get_temperature(void)
{
    prev = temperature;
    return temperature;
}

void ntc_sendcurrent(void)
{
    // TODO: mutex around temperature use. Every functions. See heater.c
    queue_measurement(temperature);
}


static void ntc_reader(void* arg)
{
    int cnt = 0;
    int sum = 0;
    int minraw =  0xffff; // dont count on smallest
    int maxraw = -0xffff; // biggest samples

    for(;;) 
    {
        int raw = ntc_read();

        sum += raw;
        if (raw < minraw) minraw = raw;
        if (raw > maxraw) maxraw = raw;
        if (++cnt == MEASURES_PER_SAMPLE)
        {
            int avg = (sum - minraw - maxraw) / (MEASURES_PER_SAMPLE - 2);
            cnt = 0;
            sum = 0;
            temperature = convert(avg);
            float diff = fabs(prev - temperature);
            if (diff >= 0.08)
            {
                prev = temperature;
                queue_measurement(temperature);
            }
            minraw =  0xffff;
            maxraw = -0xffff;
        }
        vTaskDelay(sampleInterval / portTICK_PERIOD_MS);
    }
}


bool ntc_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;
    
    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"sensor\":\"ntc\",\"id\":\"temperature\",\"value\":%.02f,\"ts\":%jd,\"unit\":\"C\"}";
    sprintf(temperatureTopic,"%s/thermostat/%x%x%x/parameters/temperature/ntc", prefix, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                data->data.temperature,
                now);
    esp_mqtt_client_publish(client, temperatureTopic, jsondata , 0, 0, 1);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}


void ntc_init(uint8_t *chip, int intervalMs)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    chipid = chip;
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_6
    };
    // mutex is here not needed, we are not yet threading.
    temperature = convert(ntc_read());
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);
    sampleInterval = intervalMs / MEASURES_PER_SAMPLE;
    xTaskCreate(ntc_reader, "ntc reader", 2048, NULL, 10, NULL);
    return;
}

void ntc_close(void)
{
    return;
}

