
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "thermostat.h"



static int currentLevel = 0;
static int fullRound = 1000;
static int levelCnt = 10;
static int slotLen = 1000;
SemaphoreHandle_t mutex;

static void activator(void* arg)
{
    int slot=0;
    int level;

    for(;;) 
    {
        if(xSemaphoreTake(mutex, (TickType_t) 100) == pdTRUE)
        {
            level = currentLevel;
            xSemaphoreGive(mutex);
        }
        else {
            printf("failed to take mutex\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if (level == 0)
            gpio_set_level(HEATER_GPIO, false);
        else
        {
            if (slot <= level)
                gpio_set_level(HEATER_GPIO, true);
            else    
                gpio_set_level(HEATER_GPIO, false);
        }
        vTaskDelay(slotLen / portTICK_PERIOD_MS);
        slot++;
        if (slot == levelCnt)
            slot = 0;
    }
}

void heater_init(int fullRoundMs, int levels)
{
    fullRound = fullRoundMs;
    levelCnt = levels;
    slotLen = fullRoundMs / levels;

    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        printf("%s failed to create mutex",__FILE__);
        return;
    }
    gpio_reset_pin(HEATER_GPIO);
    gpio_set_direction(HEATER_GPIO, GPIO_MODE_OUTPUT);
    xTaskCreate(activator, "activator", 2048, NULL, 10, NULL);
    return;
}

void heater_close(void)
{
    if (mutex != NULL) vSemaphoreDelete(mutex);
}

void heater_setlevel(int level)
{
    if (level >= 0 && level < levelCnt)
    {
        if( xSemaphoreTake(mutex, (TickType_t) 100) == pdTRUE)
        {
            if (currentLevel != level)
            {
                currentLevel = level;
            }
            xSemaphoreGive(mutex);
        }
    }
    return;
}

int heater_getlevel(void)
{
    int ret = -1;
    if (xSemaphoreTake(mutex, (TickType_t ) 100) == pdTRUE)
    {
        ret = currentLevel;
        xSemaphoreGive(mutex);
    }
    return ret;
}