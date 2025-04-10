#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <stdlib.h>

#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_wifi_types.h"
#include "freertos/event_groups.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "homeapp.h"
#include "temperature/temperatures.h"
#include "flashmem.h"
#include "display.h"
#include "ntcreader.h"
#include "ldrreader.h"
#include "heater.h"
#include "pidcontroller.h"
#include "ota/ota.h"
#include "device/device.h"
#include "mqtt_client.h"
#include "apwebserver/server.h"
#include "factoryreset.h"
#include "statistics/statistics.h"

#define TEMP_BUS              25
#define STATISTICS_INTERVAL 1800
#define ESP_INTR_FLAG_DEFAULT  0
#define HEATER_POWERLEVELS    30
#define NTC_INTERVAL_MS     5000

#define SETUP_ALL       0xff
#define SETUP_PID       1
#define SETUP_NTC       2
#define SETUP_HEAT      4
#define SETUP_DISPLAY   8
#define SETUP_CALIBR    16
#define SETUP_NAMES     32
#define SETUP_BOOSTWD   64
#define SETUP_BOOSTWE   128


#if CONFIG_EXAMPLE_WIFI_SCAN_METHOD_FAST
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_FAST_SCAN
#elif CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#endif

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#endif

#define WIFI_RECONNECT_RETRYCNT 50

#define HEALTHYFLAGS_WIFI 1
#define HEALTHYFLAGS_MQTT 2
#define HEALTHYFLAGS_TEMP 4
#define HEALTHYFLAGS_NTP  8

enum PriceState {
    PRICE_HIGH,
    PRICE_LOW,
    PRICE_NORMAL
};

struct netinfo {
    char *ssid;
    char *password;
    char *mqtt_server;
    char *mqtt_port;
    char *mqtt_prefix;
};

struct {
    // ntc
    int interval;
    int samples;
    int tzoffset;

    // pidcontroller
    float max;
    float kp;
    float ki;
    float kd;

    // heater
    int pwmlen;
    float target;
    float hiboost;
    float lodeduct;
    float drop;
    float expensive;

    // boosthours, as flags
    uint32_t wkdhours;
    uint32_t wendhours;

    // alteration may be positive or negative
    float wkd_alteration;
    float wend_alteration;
} setup = { 15, 10, 7200,
            35, 5 , 1, 3,
            15, 24, 1, 1, 0.5, 8.0,
            0, 0, 1.0, 1.0};


PID pidCtl = {
    .interval   = 15,
    .target     = 27,
    .pgain      = 5,
    .igain      = 1,
    .dgain      = 3,
    .maxTune    = 0,
    .prevValue  = 0,
    .prevTune   = 0,
    .tuneValue  = 0,
    .prevMeasTs = 0,
    .chipid = NULL,
    .topic[0]   = 0
};


// globals
struct netinfo *comminfo;
QueueHandle_t evt_queue = NULL;
char jsondata[512];
char setupJson[128];

static const char *TAG = "THERMOSTAT";
static bool isConnected = false;
static uint8_t healthyflags = 0;

static char setupTopic[64];
static char setSetupTopic[64];
static char elpriceTopic[64];
static char otaUpdateTopic[64];
static char tzoffsetTopic[32];
static int retry_num = 0;
static float currentTarget = 0.0;
static char *program_version = ""; 
static char appname[20];
static float elprice = 8.00; // cents / kwh
static enum PriceState priceState = PRICE_NORMAL;
nvs_handle setup_flash;
SemaphoreHandle_t mqttBuffMtx;


static void sendSetup(esp_mqtt_client_handle_t client, uint8_t *chipid, uint8_t flags);
static void sendInfo(esp_mqtt_client_handle_t client, uint8_t *chipid);

static char *getJsonStr(cJSON *js, char *name)
{
    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsString(item))
        {
            return item->valuestring;
        }
        else ESP_LOGI(TAG, "%s is not a string", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return "\0";
}


static bool getJsonInt(cJSON *js, char *name, int *val)
{
    bool ret = false;

    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsNumber(item))
        {
            if (item->valueint != *val)
            {
                ret = true;
                *val = item->valueint;
            }
            else ESP_LOGI(TAG,"%s is not changed", name);
        }
        else ESP_LOGI(TAG,"%s is not a number", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return ret;
}


static bool getJsonFloat(cJSON *js, char *name, float *val)
{
    bool ret = false;

    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsNumber(item))
        {
            if (item->valuedouble != *val)
            {
                ret = true;
                *val = item->valuedouble;
                ESP_LOGI(TAG,"received variable %s:%2.2f", name, item->valuedouble);
            }
            ESP_LOGI(TAG,"%s is not changed", name);
        }
        else ESP_LOGI(TAG,"%s is not a number", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return ret;
}

bool isflag_active(uint32_t flags, int pos)
{
    return (flags & (1 << pos));
}

void refreshDisplay(void)
{
    struct measurement meas;
    meas.id = REFRESH_DISPLAY;
    meas.gpio = 0;
    meas.data.count = 0;
    xQueueSend(evt_queue, &meas, 0);
}

float chkHourlyBoost(void)
{
    time_t now;
    struct tm current_time;

    time(&now);
    if (now > MIN_EPOCH)
    {
        now += setup.tzoffset;
        localtime_r(&now, &current_time);
        if (current_time.tm_wday == 0 || current_time.tm_wday == 6)
        {
            if (isflag_active(setup.wendhours, current_time.tm_hour))
            {
                return setup.wend_alteration;
            }
        }
        else
        {
            if (isflag_active(setup.wkdhours, current_time.tm_hour))
            {
                return setup.wkd_alteration;
            }
        }
    }
    return 0.0;
}


// pidcontroller config
static void readPidSetupJson(cJSON *root)
{
    bool reinit_needed = false;

    if (getJsonFloat(root, "max", &setup.max))
    {
        flash_write_float(setup_flash, "pidmax", setup.max);
        reinit_needed = true;
    }
    if (getJsonFloat(root, "pidkp", &setup.kp))
    {
        flash_write_float(setup_flash, "pidkp", setup.kp);
        reinit_needed = true;
    }
    if (getJsonFloat(root, "pidki", &setup.ki))
    {
        flash_write_float(setup_flash, "pidki", setup.ki);
        reinit_needed = true;
    }
    if (getJsonFloat(root, "pidkd", &setup.kd))
    {
        flash_write_float(setup_flash, "pidkd", setup.kd);
        reinit_needed = true;
    }
    if (reinit_needed) 
    {
        pidcontroller_adjust(&pidCtl, setup.max, setup.interval,setup.kp, setup.ki, setup.kd);
        ntc_sendcurrent(); // this causes pid recalculation
    }
    flash_commitchanges(setup_flash);
}

static void sensorFriendlyName(cJSON *root)
{
    char *sensorname;
    char *friendlyname;

    sensorname   = getJsonStr(root, "sensor");
    friendlyname = getJsonStr(root, "name");
    if (temperature_set_friendlyname(sensorname, friendlyname))
    {
        ESP_LOGD(TAG, "writing sensor %s, friendlyname %s to flash",sensorname, friendlyname);
        flash_write_str(setup_flash, sensorname,friendlyname);
        flash_commitchanges(setup_flash);
    }
}


bool sendTargetInfo(esp_mqtt_client_handle_t client, uint8_t *chipid, float target, time_t now)
{
    gpio_set_level(BLINK_GPIO, true);
    char targetTopic[60];
    int retain=1;
    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"targettemp\",\"value\":%.02f,\"ts\":%jd,\"unit\":\"C\"}";

    sprintf(targetTopic,"%s/%s/%x%x%x/parameters/targettemp", comminfo->mqtt_prefix, appname, chipid[3], chipid[4], chipid[5]);

    if (now < MIN_EPOCH)
    {
        retain = 0;
        now = 0;
    }
    sprintf(jsondata, datafmt,
                    chipid[3],chipid[4],chipid[5],
                    target,
                    now);
    esp_mqtt_client_publish(client, targetTopic, jsondata , 0, 0, retain);
    statistics_getptr()->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
    return true;
}

void getPriceInfo(cJSON *root)
{
    char *stateStr = getJsonStr(root,"pricestate");

    if (!strcmp(stateStr,"low"))
    {
        ESP_LOGI(TAG,"Lo price, BIGGER TEMP IS ON!");
        priceState = PRICE_HIGH;
    }
    else if (!strcmp(stateStr,"high"))
    {
        ESP_LOGI(TAG,"Hi price, COOLER TEMP IS ON!");
        priceState = PRICE_LOW;
    }
    else
    {
        ESP_LOGI(TAG,"normal temperature is on");
        priceState = PRICE_NORMAL;
    }
    getJsonFloat(root,"price", &elprice);
}

// target = hourlyboost + setup.hiboost || setup.lodeduct - setup.drop
bool calcTargetTemperature(void)
{
    float ret = false;
    float target = setup.target + chkHourlyBoost();

    switch (priceState)
    {
        case PRICE_HIGH:
            target += setup.hiboost;
        break;

        case PRICE_LOW:
            target -= setup.lodeduct;
        break;

        default:
        break;
    }
    if (elprice > setup.expensive)
    {
        target -= setup.drop;
    }
    if (target != currentTarget)
    {
        currentTarget = target;
        ESP_LOGI(TAG,"changing target to %f", target);
        ret = true;
    }
    return ret;
}


void flags2str(uint32_t flags, char *str)
{
    for (int i=0; i < 24; i++)
    {
        if (flags & (1 << i))
        {
            str[i] = '1';
        }
        else
        {
            str[i] = '0';
        }
    }
    str[24]=0;
}

uint32_t str2flags(char *str)
{
    uint32_t flags = 0;
    int bitlen = strlen(str);

    if (bitlen != 24)
    {
        ESP_LOGI(TAG, "wrong size of bitstr %d\n", bitlen);
    }
    for (int i=0; i < 24; i++)
    {
        if (str[i]=='1')
        {
            flags |= 1 << i;
        }
    }
    ESP_LOGI(TAG,"flags = %d\n",flags);
    return flags;
}

static void chkChanges(esp_mqtt_client_handle_t client, uint8_t *chipid, time_t now)
{
    if (calcTargetTemperature())
    {
        pidcontroller_target(&pidCtl, currentTarget);
        sendTargetInfo(client, chipid, currentTarget, now);
    }
    int tune = pidcontroller_tune(&pidCtl, ntc_get_temperature());
    heater_setlevel(tune);
    refreshDisplay();
}


static uint8_t handleJson(esp_mqtt_event_handle_t event, uint8_t *chipid)
{
    cJSON *root = cJSON_Parse(event->data);
    uint8_t ret = 0;
    char id[20];
    char flagstr[33];
    time_t now;
    bool recalc = false;

    time(&now);
    if (root != NULL)
    {
        strcpy(id,getJsonStr(root,"id"));

        if (!strcmp(id,"pidsetup"))
        {
            ESP_LOGI(TAG,"got pid setup");
            readPidSetupJson(root);
            ret |= SETUP_PID;
        }
        if (!strcmp(id,"workdayboost"))
        {
            strcpy(flagstr,getJsonStr(root,"hours"));
            setup.wkdhours = str2flags(flagstr);
            flash_write32(setup_flash, "wkdhrs", setup.wkdhours);
            if (getJsonFloat(root, "change", &setup.wkd_alteration))
            {
                flash_write_float(setup_flash, "wkdalt", setup.wkd_alteration);
            }
            recalc = true;
            ret |= SETUP_BOOSTWD;
        }
        if (!strcmp(id,"weekendboost"))
        {
            strcpy(flagstr,getJsonStr(root,"hours"));
            setup.wendhours = str2flags(flagstr);
            flash_write32(setup_flash, "wendhrs", setup.wendhours);
            if (getJsonFloat(root, "change", &setup.wend_alteration))
            {
                flash_write_float(setup_flash, "wendalt", setup.wend_alteration);
            }
            recalc = true;
            ret |= SETUP_BOOSTWE;
        }
        if (!strcmp(id,"ntcreader"))
        {
            if (getJsonInt(root, "interval", &setup.interval))
            {
                flash_write(setup_flash, "interval", setup.interval);
            }
            if (getJsonInt(root, "samples", &setup.samples))
            {
                flash_write(setup_flash, "samples", setup.samples);
            }
            ret |= SETUP_NTC;
        }
        else if (!strcmp(id,"calibratelow"))
        {
            float deflow = 20;
            if (getJsonFloat(root, "temperature", &deflow))
            {
                int raw = -1;
                getJsonInt(root,"raw", &raw);
                ESP_LOGI(TAG,"got calibration low %f, raw %d", deflow, raw);
                ntc_set_calibr_low(deflow, raw);
                ret |= SETUP_CALIBR;
            }
        }
        else if (!strcmp(id,"calibratehigh"))
        {
            float defhigh = 30;
            if (getJsonFloat(root, "temperature", &defhigh))
            {
                int raw = -1;
                getJsonInt(root,"raw", &raw);
                ESP_LOGI(TAG,"got calibration high %f, raw %d", defhigh, raw);
                ntc_set_calibr_high(defhigh, raw);
                ret |= SETUP_CALIBR;
            }
        }
        else if (!strcmp(id,"calibratesave"))
        {
            ntc_save_calibrations();
        }
        else if (!strcmp(id,"tzoffset"))
        {
            getJsonInt(root,"value", &setup.tzoffset);
            ESP_LOGI(TAG,"got tz offset %d", setup.tzoffset);
        }
        else if (!strcmp(id,"heatsetup"))
        {
            ESP_LOGI(TAG,"got heatsetup");
            if (getJsonInt(root,"pwmlen", &setup.pwmlen))
            {
                heater_reconfig(setup.pwmlen, HEATER_POWERLEVELS);
                flash_write(setup_flash, "pwmlen", setup.pwmlen);
            }
            if (getJsonFloat(root, "drop", &setup.drop))
            {
                flash_write_float(setup_flash, "drop", setup.drop);
                recalc = true;
            }
            if (getJsonFloat(root, "expensive", &setup.expensive))
            {
                flash_write_float(setup_flash, "expensive", setup.expensive);
                recalc = true;
            }
            if (getJsonFloat(root, "target", &setup.target))
            {
                flash_write_float(setup_flash, "target", setup.target);
                recalc = true;
            }
            if (getJsonFloat(root, "hiboost", &setup.hiboost))
            {
                flash_write_float(setup_flash, "hiboost", setup.hiboost);
                recalc = true;
            }
            if (getJsonFloat(root, "lodeduct", &setup.lodeduct))
            {
                flash_write_float(setup_flash, "lodeduct", setup.lodeduct);
                recalc = true;
            }
            ret |= SETUP_HEAT;
        }
        else if (!strcmp(id,"sensorfriendlyname"))
        {
            sensorFriendlyName(root);
            ret |= SETUP_NAMES;
        }
        // hour has changed
        else if (!strcmp(id,"elprice"))
        {
            char *topicpostfix = &event->topic[event->topic_len - 7];
            if (!memcmp(topicpostfix,"current",7))
            {
                getPriceInfo(root);
                chkChanges(event->client, chipid, now);
            }
        }
        else if (!strcmp(id,"otaupdate"))
        {
            char *fname = getJsonStr(root,"file");
            if (strlen(fname) > 5)
            {
                ota_start(fname);
            }    
        }
        cJSON_Delete(root);
        if (recalc)
        {
            chkChanges(event->client, chipid, now);
        }
    }
    return ret;
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            ESP_LOGI(TAG,"subscribing topics");

            msg_id = esp_mqtt_client_subscribe(client, setSetupTopic , 0);
            ESP_LOGI(TAG, "sent subscribe %s succesful, msg_id=%d", setSetupTopic, msg_id);

            msg_id = esp_mqtt_client_subscribe(client, elpriceTopic , 0);
            ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", elpriceTopic, msg_id);

            msg_id = esp_mqtt_client_subscribe(client, otaUpdateTopic , 0);
            ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", otaUpdateTopic, msg_id);

            msg_id = esp_mqtt_client_subscribe(client, tzoffsetTopic , 0);
            ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", tzoffsetTopic, msg_id);

            gpio_set_level(MQTTSTATUS_GPIO, false);
            device_sendstatus(client, comminfo->mqtt_prefix, appname, (uint8_t *) handler_args);
            sendInfo(client, (uint8_t *) handler_args);
            sendSetup(client, (uint8_t *) handler_args, SETUP_ALL);
            ntc_sendcurrent();
            pidcontroller_send_last(&pidCtl);
            isConnected = true;
            statistics_getptr()->connectcnt++;
            refreshDisplay();
            healthyflags |= HEALTHYFLAGS_MQTT;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        statistics_getptr()->disconnectcnt++;
        isConnected = false;
        gpio_set_level(MQTTSTATUS_GPIO, true);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        {
            uint8_t flags = handleJson(event, (uint8_t *) handler_args);
            if (flags) sendSetup(client, (uint8_t *) handler_args, flags);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void sntp_callback(struct timeval *tv)
{
    (void) tv;
    static bool firstSyncDone = false;

    if (!firstSyncDone)
    {
        pidcontroller_send_last(&pidCtl);
        firstSyncDone = true;
        healthyflags |= HEALTHYFLAGS_NTP;
    }
}

static void sntp_start()
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    sntp_set_time_sync_notification_cb(sntp_callback);
}


static void sendInfo(esp_mqtt_client_handle_t client, uint8_t *chipid)
{
    gpio_set_level(BLINK_GPIO, true);

    char infoTopic[42];

    sprintf(infoTopic,"%s/%s/%x%x%x/info",
         comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
    sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"info\",\"memfree\":%d,\"idfversion\":\"%s\",\"progversion\":\"%s\"}",
                chipid[3],chipid[4],chipid[5],
                esp_get_free_heap_size(),
                esp_get_idf_version(),
                program_version);
    esp_mqtt_client_publish(client, infoTopic, jsondata , 0, 0, 1);
    statistics_getptr()->sendcnt++;
    ESP_LOGI(TAG,"sending info");
    gpio_set_level(BLINK_GPIO, false);
}


static char *mkSetupTopic(char *item, char *buff, uint8_t *chipid, int id)
{
    if (id != -1)
    {
        sprintf(buff,"%s/%s/%x%x%x/setup/%s/%d",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5], item, id);
    }
    else
    {
        sprintf(buff,"%s/%s/%x%x%x/setup/%s",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5], item);
    }
    return buff;
}

static void show_clock(void)
{
    time_t now;
    struct tm current_time;
    char buff[6];

    time(&now);
    if (now > MIN_EPOCH)
    {
        now += setup.tzoffset;
        localtime_r(&now, &current_time);
        sprintf(buff,"%02d:%02d", current_time.tm_hour, current_time.tm_min);
        ESP_LOGI(TAG,"localtime is %s", buff);
    }
}


static void sendSetup(esp_mqtt_client_handle_t client, uint8_t *chipid, uint8_t flags)
{
    gpio_set_level(BLINK_GPIO, true);
    time_t now;
    char tmpTopic[64];

    time(&now);
    if (flags & SETUP_PID)
    {
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"pidsetup\","
                        "\"max\":%2.2f,\"pidkp\":%2.2f,\"pidki\":%2.2f,\"pidkd\":%2.2f}",
                    chipid[3],chipid[4],chipid[5],
                    setup.max, setup.kp, setup.ki, setup.kd);
        esp_mqtt_client_publish(client, mkSetupTopic("pid",tmpTopic, chipid,-1), setupJson , 0, 0, 1);
        flags &= ~SETUP_PID;
        statistics_getptr()->sendcnt++;
    }

    if (flags & SETUP_NTC)
    {
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"ntcreader\",\"interval\":%d,\"samples\":%d}",
                    chipid[3],chipid[4],chipid[5],
                    setup.interval, setup.samples);
        esp_mqtt_client_publish(client, mkSetupTopic("ntc",tmpTopic, chipid,-1), setupJson , 0, 0, 1);
        flags &= ~SETUP_NTC;
        statistics_getptr()->sendcnt++;
    }

    if (flags & SETUP_BOOSTWD)
    {
        char strflags[25];

        flags2str(setup.wkdhours,strflags);
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"workdayboost\",\"hours\":\"%s\",\"change\":%.2f}",
                    chipid[3],chipid[4],chipid[5],
                    strflags, setup.wkd_alteration);
        esp_mqtt_client_publish(client, mkSetupTopic("workdayboost",tmpTopic, chipid,-1), setupJson , 0, 0, 1);
        flags &= ~SETUP_BOOSTWD;
    }

    if (flags & SETUP_BOOSTWE)
    {
        char strflags[25];

        flags2str(setup.wendhours,strflags);
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"weekendboost\",\"hours\":\"%s\",\"change\":%.2f}",
                    chipid[3],chipid[4],chipid[5],
                    strflags, setup.wend_alteration);
        esp_mqtt_client_publish(client, mkSetupTopic("weekendboost",tmpTopic, chipid,-1), setupJson , 0, 0, 1);

        flags &= ~SETUP_BOOSTWE;
        statistics_getptr()->sendcnt += 2;
    }

    if (flags & SETUP_HEAT)
    {
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"heatsetup\","
                        "\"pwmlen\":%d,\"target\":%2.2f,"
                        "\"drop\":%2.2f,\"expensive\":%2.2f,"
                        "\"hiboost\":%2.2f,\"lodeduct\":%2.2f}",
                    chipid[3],chipid[4],chipid[5],
                    setup.pwmlen, setup.target,
                    setup.drop, setup.expensive,
                    setup.hiboost, setup.lodeduct);
        esp_mqtt_client_publish(client, mkSetupTopic("heat",tmpTopic, chipid,-1), setupJson , 0, 0, 1);
        flags &= ~SETUP_HEAT;
        statistics_getptr()->sendcnt++;
    }

    if (flags & SETUP_CALIBR)
    {
        float temperature;
        int raw;

        raw = ntc_get_calibr_high(&temperature);
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"calibratehigh\",\"temperature\":%2.2f,\"raw\":%d}",
                    chipid[3],chipid[4],chipid[5],
                    temperature, raw);
        esp_mqtt_client_publish(client, mkSetupTopic("calibratehigh",setupTopic, chipid,-1), setupJson , 0, 0, 1);
        statistics_getptr()->sendcnt++;

        raw = ntc_get_calibr_low(&temperature);
        sprintf(setupJson, "{\"dev\":\"%x%x%x\",\"id\":\"calibratelow\",\"temperature\":%2.2f,\"raw\":%d}",
                    chipid[3],chipid[4],chipid[5],
                    temperature, raw);
        esp_mqtt_client_publish(client, mkSetupTopic("calibratelow",setupTopic, chipid,-1), setupJson , 0, 0, 1);
        flags &= ~SETUP_CALIBR;
        statistics_getptr()->sendcnt++;
    }

    if (flags & SETUP_NAMES)
    {
        sprintf(setupTopic,"%s/%s/%x%x%x/tempsensors",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"tempsensors\",\"names\":[",
            chipid[3],chipid[4],chipid[5]);

        char sensorname[40];

        for (int i = 0; ; i++)
        {
            char *sensoraddr = temperature_getsensor(i);

            if (sensoraddr == NULL) break;
            sprintf(sensorname,"{\"addr\":\"%s\",\"name\":\"%s\"},",
                sensoraddr, temperature_get_friendlyname(i));
            strcat(jsondata,sensorname);
        }
        jsondata[strlen(jsondata)-1] = 0; // cut last comma
        strcat(jsondata,"]}");
        esp_mqtt_client_publish(client, setupTopic, jsondata , 0, 0, 1);
        statistics_getptr()->sendcnt++;
    }
    gpio_set_level(BLINK_GPIO, false);
}

static esp_mqtt_client_handle_t mqtt_app_start(uint8_t *chipid)
{
    char client_id[128];
    char uri[64];
    char deviceTopic[42];
    
    sprintf(client_id,"client_id=%s%x%x%x",
        comminfo->mqtt_prefix ,chipid[3],chipid[4],chipid[5]);
    sprintf(uri,"mqtt://%s:%s",comminfo->mqtt_server, comminfo->mqtt_port);

    ESP_LOGI(TAG,"built client id=[%s]",client_id);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .credentials.client_id = client_id,
        .session.last_will.topic = device_topic(comminfo->mqtt_prefix, deviceTopic, chipid),
        .session.last_will.msg = device_data(jsondata, chipid, appname, 0),
        .session.last_will.msg_len = strlen(jsondata),
        .session.last_will.qos = 0,
        .session.last_will.retain = 1
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, chipid);
    esp_mqtt_client_start(client);
    return client;
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data)
{
    switch (event_id)
    {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG,"WIFI CONNECTING");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG,"WiFi CONNECTED");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG,"WiFi lost connection");
            gpio_set_level(WLANSTATUS_GPIO, true);
            if(retry_num < WIFI_RECONNECT_RETRYCNT){
                esp_wifi_connect();
                retry_num++;
                ESP_LOGI(TAG,"Retrying to Connect...");
            }
            break;

        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG,"Wifi got IP");
            gpio_set_level(WLANSTATUS_GPIO, false);
            retry_num = 0;
            healthyflags |= HEALTHYFLAGS_WIFI;
            break;
    }
}


void wifi_connect(char *ssid, char *password)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        }
    };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, password);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
}


struct netinfo *get_networkinfo()
{
    static struct netinfo ni;
    char *default_ssid = "XXXXXXXX";

    nvs_handle wifi_flash = flash_open("wifisetup");
    if (wifi_flash == -1) return NULL;

    ni.ssid = flash_read_str(wifi_flash, "ssid",default_ssid, 20);
    if (!strcmp(ni.ssid,"XXXXXXXX"))
        return NULL;

    ni.password    = flash_read_str(wifi_flash, "password","pass", 20);
    ni.mqtt_server = flash_read_str(wifi_flash, "mqtt_server","test.mosquitto.org", 20);
    ni.mqtt_port   = flash_read_str(wifi_flash, "mqtt_port","1883", 6);
    ni.mqtt_prefix = flash_read_str(wifi_flash, "mqtt_prefix","home/esp", 20);
    return &ni;
}

static void get_appname(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    strncpy(appname,app_desc->project_name,20);
}


void app_main(void)
{
    uint8_t chipid[8];
    time_t now, prevStatsTs;
    esp_efuse_mac_get_default(chipid);
    adc_oneshot_unit_handle_t adc_handle;
    int prevBrightness = 0;
    int tune;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_reset_pin(BLINK_GPIO);
    gpio_reset_pin(WLANSTATUS_GPIO);
    gpio_reset_pin(SETUP_GPIO);
    gpio_reset_pin(MQTTSTATUS_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(WLANSTATUS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SETUP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MQTTSTATUS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SETUP_GPIO, true);
    gpio_set_level(WLANSTATUS_GPIO, true);
    gpio_set_level(MQTTSTATUS_GPIO, true);
    gpio_set_level(BLINK_GPIO, true);

    display_init();
    display_clear();
    display_show(88, 88.88);
    get_appname();

    comminfo = get_networkinfo();

    if (comminfo == NULL)
    {
        display_text(" setup ");
        gpio_set_level(WLANSTATUS_GPIO, false);
        gpio_set_level(MQTTSTATUS_GPIO, false);
        gpio_set_level(BLINK_GPIO, false);
        server_init(); // starting ap webserver
    }
    else
    {
        setup_flash = flash_open("storage");
        sntp_start();
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        factoryreset_init();
        wifi_connect(comminfo->ssid, comminfo->password);
        esp_wifi_set_ps(WIFI_PS_NONE);
        evt_queue = xQueueCreate(10, sizeof(struct measurement));


        setup.pwmlen       = flash_read(setup_flash, "pwmlen", setup.pwmlen);
        setup.interval     = flash_read(setup_flash, "interval", setup.interval);

        setup.max          = flash_read_float(setup_flash, "pidmax", setup.max);
        setup.kp           = flash_read_float(setup_flash, "pidkp", setup.kp);
        setup.ki           = flash_read_float(setup_flash, "pidki", setup.ki);
        setup.kd           = flash_read_float(setup_flash, "pidkd", setup.kd);

        setup.samples      = flash_read(setup_flash, "samples", setup.samples);
        setup.target       = flash_read_float(setup_flash, "target", setup.target);
        setup.drop         = flash_read_float(setup_flash, "drop", setup.drop);
        setup.expensive    = flash_read_float(setup_flash, "expensive", setup.expensive);
        setup.hiboost      = flash_read_float(setup_flash, "hiboost", setup.hiboost);
        setup.lodeduct     = flash_read_float(setup_flash, "lodeduct", setup.lodeduct);

        setup.wkdhours     = flash_read32(setup_flash, "wkdhrs", setup.wkdhours);
        setup.wendhours    = flash_read32(setup_flash, "wendhrs", setup.wendhours);
        setup.wkd_alteration = flash_read_float(setup_flash, "wkdalt", setup.wkd_alteration);
        setup.wend_alteration= flash_read_float(setup_flash, "wendalt", setup.wend_alteration);


        int sensorcnt = temperature_init(TEMP_BUS, appname, chipid, 4);
        if (sensorcnt)
        {
            char *sensoraddr;
            char *friendlyname;

            for (int i = 0; i < sensorcnt; i++)
            {
                sensoraddr   = temperature_getsensor(i);
                if (sensoraddr == NULL) break;
                friendlyname = flash_read_str(setup_flash, sensoraddr, sensoraddr, 20);
                if (strcmp(friendlyname, sensoraddr))
                {
                    if (!temperature_set_friendlyname(sensoraddr, friendlyname))
                    {
                        ESP_LOGD(TAG, "Set friendlyname for %s failed", sensoraddr);
                    }
                    free(friendlyname); // flash_read_str does dynamic allocation
                }
            }
        }

        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        adc_oneshot_new_unit(&init_config1, &adc_handle);

        while (1)
        {
            if (ntc_init(chipid, adc_handle, setup.interval * 1000, setup.samples))
                break;
            else
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        }    
        ldr_init(chipid, adc_handle, 1000, 15); // display has 15 brightness values
        program_version = ota_init(comminfo->mqtt_prefix, appname, chipid);
        heater_init(setup.pwmlen, HEATER_POWERLEVELS);
        pidcontroller_init(&pidCtl, comminfo->mqtt_prefix, chipid, setup.max, HEATER_POWERLEVELS-1, setup.interval, setup.kp, setup.ki, setup.kd);
        pidcontroller_target(&pidCtl, currentTarget);
        esp_mqtt_client_handle_t client = mqtt_app_start(chipid);

        ESP_LOGI(TAG, "[APP] All init done, app_main, last line.");

        if (!statistics_init(comminfo->mqtt_prefix, appname, chipid))
        {
            ESP_LOGE(TAG,"failed in statistics init");
        }

        sprintf(setSetupTopic,"%s/%s/%x%x%x/setsetup",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);

        sprintf(otaUpdateTopic,"%s/%s/%x%x%x/otaupdate",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(elpriceTopic,"%s/elprice/#", comminfo->mqtt_prefix);
        sprintf(tzoffsetTopic,"%s/tzoffset", comminfo->mqtt_prefix);

        prevStatsTs = 0;

        ESP_LOGI(TAG,"gpios: mqtt=%d wlan=%d", MQTTSTATUS_GPIO, WLANSTATUS_GPIO);

        float ntc = ntc_get_temperature();
        float internalTemp = 10.0;
        tune = pidcontroller_tune(&pidCtl, ntc);
        heater_setlevel(tune);
        display_show(tune, ntc);

        gpio_set_level(SETUP_GPIO, false);
        throttle_setup(30.0, 10);
        while (1)
        {
            struct measurement meas;

            time(&now);
            if ((now - statistics_getptr()->started > 20) &&
                (healthyflags == (HEALTHYFLAGS_WIFI | HEALTHYFLAGS_MQTT | HEALTHYFLAGS_NTP | HEALTHYFLAGS_TEMP)))
            {
                ota_cancel_rollback();
            }

            if (now > MIN_EPOCH)
            {
                if (statistics_getptr()->started < MIN_EPOCH)
                {
                    statistics_getptr()->started = now;
                }
                if (now - prevStatsTs >= STATISTICS_INTERVAL)
                {
                    ESP_LOGI(TAG,"%d seconds since last statistics is gone", now - prevStatsTs);
                    if (isConnected)
                    {
                        statistics_send(client);
                        prevStatsTs = now;
                    }
                }
            }

            if(xQueueReceive(evt_queue, &meas, 4 * 1000 * setup.interval / portTICK_PERIOD_MS))
            {
                uint16_t qcnt = uxQueueMessagesWaiting(evt_queue);
                if (qcnt > statistics_getptr()->maxQElements)
                {
                    statistics_getptr()->maxQElements = qcnt;
                }
                switch (meas.id) {
                    case REFRESH_DISPLAY:
                        tune = pidCtl.tuneValue;
                        display_show(tune, ntc);
                    break;

                    case LDR:
                        ESP_LOGI(TAG,"got brightness %d", meas.data.count);
                        ldr_publish(comminfo->mqtt_prefix, &meas, client);
                        display_brightness(meas.data.count);
                        if (prevBrightness == 0)
                        {
                            tune = pidCtl.tuneValue;
                            display_show(tune, ntc);
                        }
                        prevBrightness = meas.data.count;
                        show_clock();
                    break;

                    case NTC:
                        ntc = meas.data.temperature;
                        if (isConnected) ntc_send(comminfo->mqtt_prefix, &meas, client);
                        tune = pidcontroller_tune(&pidCtl, ntc);
                        tune = throttle_check(internalTemp, tune);
                        heater_setlevel(tune);
                        display_show(tune, ntc);
                        pidcontroller_send_tune(&pidCtl, tune, false);
                        ESP_LOGI(TAG,"--> tune=%d", tune);
                    break;

                    case TEMPERATURE:
                        healthyflags |= HEALTHYFLAGS_TEMP;
                        internalTemp = meas.data.temperature;
                        if (isConnected) temperature_send(comminfo->mqtt_prefix, &meas, client);
                        tune = throttle_check(internalTemp, pidCtl.tuneValue);
                        heater_setlevel(tune);
                        display_show(tune, ntc);
                        pidcontroller_send_tune(&pidCtl, tune, false);
                        ESP_LOGI(TAG,"--> tune=%d", tune);
                    break;

                    case HEATER:
                        if (isConnected) pidcontroller_publish(&pidCtl, &meas, client);
                    break;

                    case OTA:
                        ota_status_publish(&meas, client);
                    break;

                    default:
                        ESP_LOGI(TAG,"unknown data type" );
                }
            }
            else
            { 
                ESP_LOGI(TAG,"timeout");
                ntc_sendcurrent();
            }
        }
    }
    display_close();
    heater_close();
    ntc_close();
    statistics_close();
}
