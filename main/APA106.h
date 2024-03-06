#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_DIRECT

#include <driver/gpio.h>
#ifndef SPI_DIRECT
#include <driver/spi_master.h>

typedef spi_device_handle_t apa106_handle_t;

esp_err_t apa106_init(gpio_num_t apa_pin, apa106_handle_t *apa_handle);
void apa106_deinit(apa106_handle_t *apa_handle);
esp_err_t apa106_rgb(apa106_handle_t apa_handle, uint8_t r, uint8_t g, uint8_t b);
#else
void apa106_init(gpio_num_t apa_pin);
void apa106_deinit();
void apa106_rgb(uint8_t r, uint8_t g, uint8_t b);
#endif

#ifdef __cplusplus
}
#endif
