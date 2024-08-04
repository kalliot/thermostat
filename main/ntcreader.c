
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
#include "homeapp.h"



static adc_oneshot_unit_handle_t adc1_handle;
static uint8_t *chipid;
static char temperatureTopic[64];
static int sampleInterval = 1000;
static float temperature;
static int samplecnt = 10;
static int lastRaw = 0;
static float prevTemp = 0.0;
static SemaphoreHandle_t mutex;

static const char *TAG = "ntcreader";

enum {
    CAL_MIN,
    CAL_MAX
};


static struct {
    char *rawname;
    char *tempname;
    int raw;
    float temp;
} calibr[] = {
    { "cal.minraw","cal.mintemp",3269, 22.22},  // default values.
    { "cal.maxraw","cal.maxtemp",3811, 29.00}
};


static float convert(int raw)
{
                                                               // interpolate                extrapolate, under cal minimum
    float rdiff = calibr[CAL_MAX].raw - calibr[CAL_MIN].raw;   // 516                        516
    float cdiff = calibr[CAL_MAX].temp - calibr[CAL_MIN].temp; // 6.58                       6.58
    float d = raw - calibr[CAL_MIN].raw;                       // 7                          -7
    float x = d / rdiff;                                       // 7 / 516 = 0.013566         -7 / 516 = -0.013566
    return x * cdiff + calibr[CAL_MIN].temp;                   // 0.013566 * 6.58 + 22.22    -0.013566 * 6.58 + 22.22
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


void ntc_set_calibr_low(float temp, int raw)
{
    float temperature = temp;
    int rawvalue = raw;
    
    if (xSemaphoreTake(mutex, (TickType_t) 1000) == pdTRUE)
    {
        if (raw == -1) // get last measured value.
        {
            rawvalue  = lastRaw;
        }
        printf("got calibration low, raw=%d, measured temperature is %f\n", rawvalue, temp);
        calibr[CAL_MIN].raw  = rawvalue;
        calibr[CAL_MIN].temp = temperature;
        xSemaphoreGive(mutex);
    }
}


void ntc_set_calibr_high(float temp, int raw)
{
    float temperature = temp;
    int rawvalue = raw;
    
    if (xSemaphoreTake(mutex, (TickType_t) 1000) == pdTRUE)
    {
        if (raw == -1) // get last measured value.
        {
            rawvalue  = lastRaw;
        }
        ESP_LOGI(TAG,"got calibration high, raw=%d, measured temperature is %f", rawvalue, temp);
        calibr[CAL_MAX].raw  = rawvalue;
        calibr[CAL_MAX].temp = temperature;
        xSemaphoreGive(mutex);
    }
}

int ntc_get_calibr_low(float *temp)
{
    *temp = calibr[CAL_MIN].temp;
    return calibr[CAL_MIN].raw;
}

int ntc_get_calibr_high(float *temp)
{
    *temp = calibr[CAL_MAX].temp;
    return calibr[CAL_MAX].raw;
}


bool ntc_save_calibrations(void)
{
    ESP_LOGI(TAG,"saving calibrations to flash");
    if (calibr[CAL_MAX].raw  < calibr[CAL_MIN].raw)
    {
        ESP_LOGE(TAG,"Error: calibration maxraw is lower than minraw");
        return false;
    }
    if (calibr[CAL_MAX].temp  < calibr[CAL_MIN].temp)
    {
        ESP_LOGI(TAG,"Error: calibration maxtemp is lower than mintemp");
        return false;
    }
    flash_write(calibr[CAL_MAX].rawname, calibr[CAL_MAX].raw);
    flash_write(calibr[CAL_MIN].rawname, calibr[CAL_MIN].raw);
    flash_write_float(calibr[CAL_MAX].tempname, calibr[CAL_MAX].temp);
    flash_write_float(calibr[CAL_MIN].tempname, calibr[CAL_MIN].temp);
    queue_measurement(convert(ntc_read()));
    return true;
}


float ntc_get_temperature(void)
{
    for (int i=0; i < 3; i++)
    {
        if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
        {
            prevTemp = temperature;
            xSemaphoreGive(mutex);
            break;
        }
    }
    return prevTemp;
}

void ntc_sendcurrent(void)
{
    if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
    {
        queue_measurement(temperature);
        xSemaphoreGive(mutex);
    }
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
        if (++cnt == samplecnt)
        {
            int avg = (sum - minraw - maxraw) / (samplecnt - 2);
            cnt = 0;
            sum = 0;

            if (xSemaphoreTake(mutex, (TickType_t) 1000) == pdTRUE)
            {
                lastRaw = avg; // lastraw is needed for calibraions;
                temperature = convert(avg);
                float diff = fabs(prevTemp - temperature);
                time_t now;
                time(&now);
                if (diff >= 0.04)
                {
                    prevTemp = temperature;
                    queue_measurement(temperature);
                }
                xSemaphoreGive(mutex);
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
    int retain = 1;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    if (now < MIN_EPOCH)
    {
        now = 0;
        retain = 0;
    }

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"sensor\":\"ntc\",\"id\":\"temperature\",\"value\":%.02f,\"ts\":%jd,\"unit\":\"C\"}";
    sprintf(temperatureTopic,"%s/thermostat/%x%x%x/parameters/temperature/ntc", prefix, chipid[3], chipid[4], chipid[5]);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                data->data.temperature,
                now);
    esp_mqtt_client_publish(client, temperatureTopic, jsondata , 0, 0, retain);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}


bool ntc_init(uint8_t *chip, int intervalMs, int cnt)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG,"failed to create mutex");
        return false;
    }
    samplecnt = cnt;
    calibr[CAL_MIN].raw   = flash_read(calibr[CAL_MIN].rawname,  calibr[CAL_MIN].raw);
    calibr[CAL_MIN].temp  = flash_read_float(calibr[CAL_MIN].tempname, calibr[CAL_MIN].temp);
    calibr[CAL_MAX].raw   = flash_read(calibr[CAL_MAX].rawname,  calibr[CAL_MAX].raw);
    calibr[CAL_MAX].temp  = flash_read_float(calibr[CAL_MAX].tempname, calibr[CAL_MAX].temp);

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
    // mutex is here not needed, we are not yet threading.
    temperature = convert(ntc_read());
    ESP_LOGI(TAG,"ntc_init, first temperature read is %f", temperature);
    sampleInterval = intervalMs / samplecnt;
    xTaskCreate(ntc_reader, "ntc reader", 2048, NULL, 10, NULL);
    return true;
}

void ntc_close(void)
{
    return;
}

