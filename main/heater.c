
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
static int levelCnt = 10;
static int slotLen = 1000;
SemaphoreHandle_t mutex;

static void activator(void* arg)
{
    int slot=0;
    int level;

    for(;;)
    {
        if(xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
        {
            level = currentLevel;
            xSemaphoreGive(mutex);
        }
        else
        {
            printf("failed to take mutex\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        if (level == 0)
            gpio_set_level(HEATER_GPIO, true);
        else
        {
            if (slot <= level)
                gpio_set_level(HEATER_GPIO, false);
            else
                gpio_set_level(HEATER_GPIO, true);
        }
        vTaskDelay(slotLen / portTICK_PERIOD_MS);
        slot++;
        if (slot == levelCnt)
            slot = 0;
    }
}


void heater_reconfig(int fullRoundSec, int levels)
{
    if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
    {
        // wait enough activator to go to mutex wait.
        vTaskDelay(slotLen / portTICK_PERIOD_MS);
        levelCnt = levels;
        slotLen = 1000 * fullRoundSec / levels;
        xSemaphoreGive(mutex);
    }
}

void heater_init(int fullRoundSec, int levels)
{
    levelCnt = levels;
    slotLen = 1000 * fullRoundSec / levels;

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
        if( xSemaphoreTake(mutex, (TickType_t) 1000) == pdTRUE)
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
    if (xSemaphoreTake(mutex, (TickType_t ) 1000) == pdTRUE)
    {
        ret = currentLevel;
        xSemaphoreGive(mutex);
    }
    return ret;
}