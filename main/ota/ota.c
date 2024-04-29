#include <stdio.h>
#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"


#include "thermostat.h"
#include "driver/gpio.h"
#include "ota.h"


static char topic[64];
static uint8_t *chipid;
static bool otaIsActive = false;
static const char *TAG = "ota_updater";
static char *otaPath;

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256


static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) 
    {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}


static void insert_queue(int bytes)
{
    struct measurement meas;
    meas.id = OTA;
    meas.gpio = 0;
    meas.data.count = bytes;
    xQueueSend(evt_queue, &meas, 0);
    if (bytes == 0) otaIsActive = false;
}


static void ota_task(void *pvParameter)
{
    char *fname = (char *) pvParameter;
    int bytesReceived = 0;

    ESP_LOGI(TAG, "Starting OTA, file=[%s]", fname);

    esp_err_t ota_finish_err = ESP_OK;
    esp_http_client_config_t config = {
        .url = CONFIG_FIRMWARE_UPGRADE_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = CONFIG_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
        .skip_cert_common_name_check = true
    };

    esp_https_ota_config_t ota_config = 
    {
        .http_config = &config,
        .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
#ifdef CONFIG_ENABLE_PARTIAL_HTTP_DOWNLOAD
        .partial_http_download = true,
        .max_http_request_size = CONFIG_HTTP_REQUEST_SIZE,
#endif
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
        if (otaPath != NULL) free(otaPath);
        insert_queue(0);
        vTaskDelete(NULL);
    }

    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
        goto ota_end;
    }
    err = validate_image_header(&app_desc);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "image header verification failed");
        goto ota_end;
    }


    while (1) 
    {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        int len = esp_https_ota_get_image_len_read(https_ota_handle);
        ESP_LOGD(TAG, "Image bytes read: %d", len);
        insert_queue(bytesReceived);
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) 
    {
        // the OTA image was not completely received and user can customise the response to this situation.
        ESP_LOGE(TAG, "Complete data was not received.");
    } 
    else 
    {
        ota_finish_err = esp_https_ota_finish(https_ota_handle);
        if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
            ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
            insert_queue(0); // done, no more data.
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } 
        else 
        {
            if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) 
            {
                ESP_LOGE(TAG, "Image validation failed, image is corrupted");
            }
            ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
            if (otaPath != NULL) free(otaPath);
            vTaskDelete(NULL);
        }
    }

ota_end:
    insert_queue(0); // done, no more data.
    esp_https_ota_abort(https_ota_handle);
    ESP_LOGE(TAG, "ESP_HTTPS_OTA upgrade failed");
    if (otaPath != NULL) free(otaPath);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}


void ota_status_publish(struct measurement *data, esp_mqtt_client_handle_t client)
{
    time_t now;

    time(&now);
    gpio_set_level(BLINK_GPIO, true);

    static char *datafmt = "{\"dev\":\"%x%x%x\",\"id\":\"otastatus\",\"value\":%d,\"ts\":%jd}";
    sprintf(jsondata, datafmt,
                chipid[3], chipid[4], chipid[5],
                data->data.count,
                now);
    esp_mqtt_client_publish(client, topic, jsondata , 0, 0, 0);
    sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
}


void ota_init(char *prefix, uint8_t *chip)
{
    chipid = chip;
    sprintf(topic,"%s/thermostat/%x%x%x/otaupdate", prefix, chipid[3], chipid[4], chipid[5]);
}


void ota_start(char *fname)
{
    if (otaIsActive)
    {
        ESP_LOGI(TAG, "Ota update already running, nothing done\n");        
    }
    else
    {
        otaIsActive = true;
        otaPath = (char *) malloc(OTA_URL_SIZE);
        sprintf(otaPath,"%s/%s", CONFIG_FIRMWARE_UPGRADE_URL, fname);
        xTaskCreate(&ota_task, "ota_task", 1024 * 8, otaPath, 5, NULL);
    }    
    return;
}

void ota_cancel_rollback(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                ESP_LOGI(TAG, "App is valid, rollback cancelled successfully");
            } else {
                ESP_LOGE(TAG, "Failed to cancel rollback");
            }
        }
        else
        {
            printf("No pending OTA verifications.\n");
        }
    }
}
