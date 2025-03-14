#include "max7219.h"
#include <stdio.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define HOST    HSPI_HOST
#else
#define HOST    SPI2_HOST
#endif

    // Configure device
static max7219_t dev = {
    .cascade_size = 1,
    .digits = 8,
    .mirrored = true
};

static int brightness = 1;

void display_init(void)
{
    // Configure SPI bus
    spi_bus_config_t cfg = {
       .mosi_io_num = CONFIG_PIN_NUM_MOSI,
       .miso_io_num = -1,
       .sclk_io_num = CONFIG_PIN_NUM_CLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    ESP_ERROR_CHECK(max7219_init_desc(&dev, HOST, MAX7219_MAX_CLOCK_SPEED_HZ, CONFIG_PIN_NUM_CS));

    ESP_ERROR_CHECK(max7219_init(&dev));
    max7219_clear(&dev);
}


void display_clear(void)
{
    max7219_clear(&dev);
}


void display_brightness(int b)
{
    brightness = b;
    if (b == 0) max7219_clear(&dev);
    max7219_set_brightness(&dev, b - 1);
}


void display_text(char *t)
{
    if (brightness) max7219_draw_text_7seg(&dev, 0, t);    
}

void display_show(int f1, float f2)
{
    char buff[6];

    if (brightness)
    {
        sprintf(buff,"%02d  ",f1);
        max7219_draw_text_7seg(&dev, 4, buff);    
        sprintf(buff,"%02.02f",f2);
        max7219_draw_text_7seg(&dev, 0, buff);
    }
}

void display_close(void)
{
    return;
}