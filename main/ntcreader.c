
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"

static adc_oneshot_unit_handle_t adc1_handle;
static uint8_t *chipid;
static char temperatureTopic[64];

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


static void ntc_reader(void* arg)
{
    int cnt = 0;
    int sum = 0;
    float prev = 0.0;
    float temperature;

    for(;;) 
    {
        int raw = ntc_read();
        sum += raw;
        if (++cnt == 10)
        {
            int avg = sum / 10;
            cnt = 0;
            sum = 0;
            temperature = convert(avg);
            float diff = fabs(prev - temperature);
            if (diff >= 0.10)
            {
                prev = temperature;
                struct measurement meas;
                meas.id = NTC;
                meas.gpio = 36;
                meas.data.temperature = temperature;
                xQueueSend(evt_queue, &meas, 0);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


bool ntc_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;
    
    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"sensor\":\"ntc\",\"id\":\"temperature\",\"value\":%.02f,\"ts\":%jd,\"unit\":\"C\"}";
    sprintf(temperatureTopic,"%s%x%x%x/parameters/temperature/ntc", prefix, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                data->data.temperature,
                now);
    esp_mqtt_client_publish(client, temperatureTopic, jsondata , 0, 0, 1);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}


void ntc_init(uint8_t *chip)
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
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);
    xTaskCreate(ntc_reader, "ntc reader", 2048, NULL, 10, NULL);
    return;
}

void ntc_close(void)
{
    return;
}

