#include <stdio.h>
#include "nvs_flash.h"
#include "flashmem.h"


static nvs_handle nvsh;


void flash_open(char *name)
{
    esp_err_t err;

    ESP_ERROR_CHECK(nvs_flash_init());
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open(name, NVS_READWRITE, &nvsh);
    if (err != ESP_OK) {
        printf("Error (%d) opening NVS handle!\n", err);
    } else {
        printf("Done\n");
    }    
}

void flash_erase_all(void)
{
    esp_err_t err;

    printf("Erasing flash partition...");
    err = nvs_erase_all(nvsh);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}

char *flash_read_str(char *name, char *def, int len)
{
    esp_err_t err;
    unsigned int readlen = len;
    char *ret;

    printf("Reading %s from NVS ... ", name);
    ret = (char *) malloc(len);
    err = nvs_get_str(nvsh, name , ret, &readlen);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("%s = %s\n", name, ret);
        break;

        case ESP_ERR_NVS_NOT_FOUND:
            printf("%s is not initialized yet!\n");
            free(ret);
            ret = def;
        break;

        default :
            printf("Error (%d) reading!\n", err);
            free(ret);
            ret = def;
    }
    return ret;
}

void flash_write_str(char *name, char *value)
{
    esp_err_t err;

    printf("Updating %s in NVS ... ", name);
    err = nvs_set_str(nvsh, name, value);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}


uint16_t flash_read(char *name, uint16_t def)
{
    esp_err_t err;
    uint16_t ret;

    printf("Reading %s from NVS ... ", name);
    err = nvs_get_u16(nvsh, name , &ret);
    switch (err) {
        case ESP_OK:
            printf("Done\n");
            printf("%s = %d\n", name, ret);
        break;

        case ESP_ERR_NVS_NOT_FOUND:
            printf("%s is not initialized yet!\n");
            ret = def;
        break;

        default :
            printf("Error (%d) reading!\n", err);
            ret = def;
    }
    return ret;
}

void flash_write(char *name, uint16_t value)
{
    esp_err_t err;

    printf("Updating %s in NVS ... ", name);
    err = nvs_set_u16(nvsh, name, value);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}

void flash_commitchanges(void)
{
    esp_err_t err;

    printf("NVS commit...");
    err = nvs_commit(nvsh);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}
