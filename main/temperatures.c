#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "ds18b20.h"
#include "mqtt_client.h"
#include "thermostat.h"

#define MAX_SENSORS 4
#define SENSOR_NAMELEN 17
#define NO_CHANGE_INTERVAL 900

static int tempSensorCnt;
static uint8_t *chipid;
static DeviceAddress tempSensors[MAX_SENSORS];
static char temperatureTopic[64];
static char *temperatureInfo;
static char *noInfo = "\0";

static struct oneWireSensor {
    float prev;
    time_t prevsend;
    char sensorname[SENSOR_NAMELEN];
    DeviceAddress *addr;
} *sensors;            


static bool isDuplicate(DeviceAddress addr, int currentCnt)
{
    for (int i = 0; i < currentCnt; i++)
    {
        if (!memcmp(tempSensors[i],addr,sizeof(DeviceAddress)))
        {
            return true;
        }
    }
    return false;
}


static int temp_getaddresses(DeviceAddress *tempSensorAddresses) {
	unsigned int numberFound = 0;
    
    reset_search();
    for (int i = 0; i < MAX_SENSORS * 3; i++) // average 3 retries for each sensor.
    {
        gpio_set_level(BLINK_GPIO, true);
        printf("searching address %d ", numberFound);
        if (search(tempSensorAddresses[numberFound], true))
        {
            if (numberFound > 0 && isDuplicate(tempSensorAddresses[numberFound], numberFound))
            {
                printf("duplicate address, rejecting\n");
            }
            else
            {
                printf("found\n");
                numberFound++;
            }
        }
        gpio_set_level(BLINK_GPIO, false);
        if (numberFound == MAX_SENSORS)
        {
            return numberFound;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");
    return numberFound;
}

char *temperatures_info()
{
    if (!tempSensorCnt) return noInfo;
    if (temperatureInfo == NULL)
    {
        temperatureInfo = (char *) malloc((SENSOR_NAMELEN + 3) * tempSensorCnt);
        if (temperatureInfo)
            temperatureInfo[0] = 0;
        else
            return noInfo;
        for (int i = 0; i < tempSensorCnt; i++)
        {
            strcat(temperatureInfo,"\"");
            strcat(temperatureInfo,sensors[i].sensorname);
            strcat(temperatureInfo,"\"");
            strcat(temperatureInfo,",");
        }
        temperatureInfo[strlen(temperatureInfo)-1] = 0;
    }
    return temperatureInfo;
}


bool temperature_send(char *prefix, struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;
    
    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"sensor\":\"%s\",\"id\":\"temperature\",\"value\":%.02f,\"ts\":%jd,\"unit\":\"C\"}";
    sprintf(temperatureTopic,"%s/thermostat/%x%x%x/parameters/temperature/%s", prefix, chipid[3], chipid[4], chipid[5], sensors[data->gpio].sensorname);

    sprintf(jsondata, datafmt,
                chipid[3],chipid[4],chipid[5],
                sensors[data->gpio].sensorname,
                data->data.temperature,
                now);
    esp_mqtt_client_publish(client, temperatureTopic, jsondata , 0, 0, 1);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}

#define DELAY_BETWEEN_SENSORS 2000

static void getFirstTemperatures()
{
    float temperature;
    int success_cnt = 0;

    for (int k=0; k < 3; k++)
    {
        ds18b20_requestTemperatures();
        for (int i=0; i < tempSensorCnt; ) {
            if (sensors[i].prev != 0.0) continue;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            temperature = ds18b20_getTempC((DeviceAddress *) sensors[i].addr) + 1.0;
            if (temperature < -10.0 || temperature > 85.0) {
                printf("%s failed with initial value %f, reading again\n", sensors[i].sensorname, temperature);
            }
            else {
                sensors[i].prev = temperature;
                i++;
                success_cnt++;
            }
        }
        if (success_cnt == tempSensorCnt) return;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void temp_reader(void* arg)
{
    int delay = 10000 - (tempSensorCnt) * DELAY_BETWEEN_SENSORS;
    float temperature;
    time_t now;

    for(time_t now = 0; now < MIN_EPOCH; time(&now))
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    for(;;) {
        time(&now);
        //printf("reading temperatures, temp sensor count=%d, delay between reads=%d\n", tempSensorCnt, delay);
        ds18b20_requestTemperatures();
        for (int i=0; i < tempSensorCnt; i++) {
            vTaskDelay(DELAY_BETWEEN_SENSORS / portTICK_PERIOD_MS); 
            temperature = ds18b20_getTempC((DeviceAddress *) sensors[i].addr);
            float diff = fabs(sensors[i].prev - temperature);
            //printf("temperature in sensor index %d, name %s is %f\n", i, sensors[i].sensorname, temperature);
            if (temperature < -10.0 || temperature > 85.0 || diff > 20.0)
            {
                sensorerrors++;
            }
            else
            {
                int age = now - sensors[i].prevsend;
                //printf("Measurement in sensor %d age is %d seconds\n", i, age);
                if ((diff) >= 0.2 || (age > NO_CHANGE_INTERVAL))
                {
                    struct measurement meas;
                    meas.id = TEMPERATURE;
                    meas.gpio = i;
                    meas.data.temperature = temperature;
                    xQueueSend(evt_queue, &meas, 0);
                    sensors[i].prev = temperature;
                    sensors[i].prevsend = now;
                }
            }
        }    
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

bool temperatures_init(int gpio, uint8_t *chip)
{
    char buff[3];

    chipid = chip;
    ds18b20_init(gpio);
    tempSensorCnt = temp_getaddresses(tempSensors);
    if (!tempSensorCnt) return false;

    sensors = malloc(sizeof(struct oneWireSensor) * tempSensorCnt);
    if (sensors == NULL) {
        printf("malloc failed when allocating sensors\n");
        return false;
    }
    printf("\nfound %d temperature sensors\n", tempSensorCnt);
    for (int i=0;i<tempSensorCnt;i++) {
        sensors[i].addr = &tempSensors[i]; 
        sensors[i].prev = 0.0;
        sensors[i].prevsend = 0;
        sensors[i].sensorname[0]='\0';
        for (int j = 0; j < 8; j++) {
            sprintf(buff,"%x",tempSensors[i][j]);
            strcat(sensors[i].sensorname, buff);
        }
        printf("sensorname %s done\n", sensors[i].sensorname);
    }
    getFirstTemperatures();
    if (tempSensorCnt)
    {
        xTaskCreate(temp_reader, "temperature reader", 2048, NULL, 10, NULL);
    }
    return true;
}