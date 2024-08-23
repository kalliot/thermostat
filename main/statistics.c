
#include <time.h>
#include <stdint.h>
#include <malloc.h>
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "homeapp.h"
#include "driver/gpio.h"
#include "statistics.h"


static struct statistics stats = {0, 0, 0, 0, 0, 0, NULL, NULL};
static const char *TAG = "STATISTICS";

struct statistics *statistics;

static int getWifiStrength(void)
{
    wifi_ap_record_t ap;

    if (!esp_wifi_sta_get_ap_info(&ap))
        return ap.rssi;
    return 0;
}


struct statistics *statistics_init(const char *prefix, const char *appname, uint8_t *chip)
{
    char tmpTopic[128];

    stats.chipid = chip;
    sprintf(tmpTopic, "%s/%s/%x%x%x/statistics", prefix, appname, chip[3],chip[4],chip[5]);
    stats.statisticsTopic = malloc(strlen(tmpTopic) + 1);
    if (stats.statisticsTopic)
    {
        strcpy(stats.statisticsTopic,tmpTopic);
        return &stats;
    }
    return NULL;
}


void statistics_send(esp_mqtt_client_handle_t client, struct statistics *s)
{
    time_t now;

    if (s==NULL) return;
    time(&now);

    gpio_set_level(BLINK_GPIO, true);
    static const char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"statistics\",\"connectcnt\":%d,\"disconnectcnt\":%d,\"sendcnt\":%d,\"sensorerrors\":%d, \"max_queued\":%d,\"ts\":%jd,\"started\":%jd,\"rssi\":%d}";
    
    sprintf(jsondata, datafmt, 
                s->chipid[3],s->chipid[4],s->chipid[5],
                s->connectcnt,
                s->disconnectcnt,
                s->sendcnt,
                s->sensorerrors,
                s->maxQElements,
                now,
                s->started,
                getWifiStrength());
    ESP_LOGI(TAG, "sending statistics to topic %s", s->statisticsTopic);
    esp_mqtt_client_publish(client, s->statisticsTopic, jsondata , 0, 0, 1);
    s->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
}


void statistics_close(void)
{
    if (stats.statisticsTopic) free(stats.statisticsTopic);
}