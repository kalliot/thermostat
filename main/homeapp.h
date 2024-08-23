#ifndef __THERMOSTAT__
#define __THERMOSTAT__

#include "freertos/queue.h"

enum meastype
{
    NTC,
    HEATER,
    TEMPERATURE,
    OTA
};

struct measurement {
    enum meastype id;
    int gpio;
    union {
        int count;
        bool state;
        float temperature;
    } data;
};

extern QueueHandle_t evt_queue;
extern char jsondata[];
extern uint16_t sendcnt;
extern uint16_t sensorerrors;
extern struct statistics *statistics;

#define BLINK_GPIO         CONFIG_SENDING_GPIO
#define SETUP_GPIO         CONFIG_SETUPLED_GPIO
#define WLANSTATUS_GPIO    CONFIG_WLANSTATUS_GPIO
#define MQTTSTATUS_GPIO    CONFIG_MQTTSTATUS_GPIO
#define HEATER_GPIO        CONFIG_HEATER_GPIO
#define MIN_EPOCH   1650000000

#endif