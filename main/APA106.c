#include <string.h>
#include "APA106.h"
#ifdef SPI_DIRECT
#include <esp_private/periph_ctrl.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_periph.h>
#include <soc/spi_periph.h>
#include <hal/spi_types.h>
#endif

#ifndef SPI_DIRECT
esp_err_t apa106_init(gpio_num_t pin, apa106_handle_t *apa_handle) {
    esp_err_t result;

    const spi_bus_config_t cfg = {
        .mosi_io_num = pin,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    result = spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_DISABLED); // SPI_DMA_CH_AUTO
    if (result == ESP_OK) {
        const spi_device_interface_config_t cfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .clock_source = SPI_CLK_SRC_DEFAULT,
            .duty_cycle_pos = 256, // 100%
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = 2857143,
            .input_delay_ns = 0,
            .spics_io_num = -1,
            .flags = 0,
            .queue_size = 1
        };

        result = spi_bus_add_device(SPI2_HOST, &cfg, apa_handle);
        if (result == ESP_OK) {
            result = spi_device_acquire_bus(*apa_handle, portMAX_DELAY);
            if (result == ESP_OK)
                return result;
            spi_bus_remove_device(*apa_handle);
        }
        spi_bus_free(SPI2_HOST);
    }
    *apa_handle = NULL;
    return result;
}

void apa106_deinit(apa106_handle_t *apa_handle) {
    if (apa_handle) {
        spi_device_release_bus(*apa_handle);
        spi_bus_remove_device(*apa_handle);
        spi_bus_free(SPI2_HOST);
        *apa_handle = NULL;
    }
}

esp_err_t IRAM_ATTR apa106_rgb(apa106_handle_t apa_handle, uint8_t r, uint8_t g, uint8_t b) {
    const uint8_t ZERO = 0x10; // 0B00010000
    const uint8_t ONE = 0x1E; // 0B00011110

    if (apa_handle) {
        uint8_t apa_data[18 + 15];

        const spi_transaction_t trans = {
            .flags = 0,
            .length = (18 + 15) * 8,
            .tx_buffer = &apa_data,
            .rx_buffer = NULL
        };

        uint32_t rgb = (r << 16) | (g << 8) | b;
        uint16_t pos = 0;

        memset(apa_data, 0, 18 + 15);
        for (uint8_t i = 0; i < 24; ++i) {
            uint8_t part = pos % 8;
            uint8_t bit;

            if (rgb & 0x00800000)
                bit = ONE;
            else
                bit = ZERO;
            if (part <= 3) {
                apa_data[pos / 8 + 18] |= bit << (3 - part);
            } else {
                apa_data[pos / 8 + 18] |= bit >> (part - 3);
                apa_data[pos / 8 + 18 + 1] |= bit << (11 - part);
            }
            pos += 5;
            rgb <<= 1;
        }
        return spi_device_queue_trans(apa_handle, (spi_transaction_t*)&trans, 1);
    }
    return ESP_ERR_INVALID_ARG;
}
#else

void apa106_init(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin, spi_periph_signal[SPI2_HOST].spid_out, false, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);

    periph_module_reset(PERIPH_SPI2_MODULE);
    periph_module_enable(PERIPH_SPI2_MODULE);
    WRITE_PERI_REG(SPI_CLK_GATE_REG(2), (1 << SPI_MST_CLK_SEL_S) | (1 << SPI_MST_CLK_ACTIVE_S) | (1 << SPI_CLK_EN_S));
    WRITE_PERI_REG(SPI_CLOCK_REG(2), (0 << SPI_CLK_EQU_SYSCLK_S) | (0 << SPI_CLKDIV_PRE_S) | ((14 - 1) << SPI_CLKCNT_N_S) | ((14 - 1) << SPI_CLKCNT_H_S) | ((14 - 1) << SPI_CLKCNT_L_S));
    WRITE_PERI_REG(SPI_USER_REG(2), (0 << SPI_USR_COMMAND_S) | (0 << SPI_USR_ADDR_S) | (0 << SPI_USR_DUMMY_S) | (0 << SPI_USR_MISO_S) | (1 << SPI_USR_MOSI_S) | (0 << SPI_USR_MOSI_HIGHPART_S) | (0 << SPI_CK_OUT_EDGE_S) | (1 << SPI_DOUTDIN_S));
    WRITE_PERI_REG(SPI_USER1_REG(2), 0);
    WRITE_PERI_REG(SPI_USER2_REG(2), 0);
    WRITE_PERI_REG(SPI_CTRL_REG(2), (0 << SPI_WR_BIT_ORDER_S) | (0 << SPI_D_POL_S));
    WRITE_PERI_REG(SPI_MS_DLEN_REG(2), (18 + 15) * 8 - 1);
    WRITE_PERI_REG(SPI_MISC_REG(2), (1 << SPI_CK_DIS_S) | (1 << SPI_CS5_DIS_S) | (1 << SPI_CS4_DIS_S) | (1 << SPI_CS3_DIS_S) | (1 << SPI_CS2_DIS_S) | (1 << SPI_CS1_DIS_S) | (1 << SPI_CS0_DIS_S));
    WRITE_PERI_REG(SPI_DMA_CONF_REG(2), (1 << SPI_BUF_AFIFO_RST_S) | (0 << SPI_DMA_TX_ENA_S) | (0 << SPI_DMA_RX_ENA_S));
    WRITE_PERI_REG(SPI_SLAVE_REG(2), (0 << SPI_SLAVE_MODE_S));
    WRITE_PERI_REG(SPI_CMD_REG(2), (1 << SPI_UPDATE_S));
    while (READ_PERI_REG(SPI_CMD_REG(2)) & (1 << SPI_UPDATE_S)) {}
}

void apa106_deinit() {
    WRITE_PERI_REG(SPI_CLK_GATE_REG(2), (1 << SPI_MST_CLK_SEL_S) | (0 << SPI_MST_CLK_ACTIVE_S) | (0 << SPI_CLK_EN_S));
    WRITE_PERI_REG(SPI_CMD_REG(2), (1 << SPI_UPDATE_S));
    while (READ_PERI_REG(SPI_CMD_REG(2)) & (1 << SPI_UPDATE_S)) {}
    periph_module_disable(PERIPH_SPI2_MODULE);
}

void IRAM_ATTR apa106_rgb(uint8_t r, uint8_t g, uint8_t b) {
    const uint8_t ZERO = 0x10; // 0B00010000
    const uint8_t ONE = 0x1E; // 0B00011110

    uint8_t apa_data[18 + 15];
    uint32_t rgb = (r << 16) | (g << 8) | b;
    uint16_t pos = 0;

    memset(apa_data, 0, 18 + 15);
    for (uint8_t i = 0; i < 24; ++i) {
        uint8_t part = pos % 8;
        uint8_t bit;

        if (rgb & 0x00800000)
            bit = ONE;
        else
            bit = ZERO;
        if (part <= 3) {
            apa_data[pos / 8 + 18] |= bit << (3 - part);
        } else {
            apa_data[pos / 8 + 18] |= bit >> (part - 3);
            apa_data[pos / 8 + 18 + 1] |= bit << (11 - part);
        }
        pos += 5;
        rgb <<= 1;
    }
    while (READ_PERI_REG(SPI_CMD_REG(2)) & (1 << SPI_USR_S)) {}
    for (uint8_t i = 0; i < sizeof(apa_data); i += 4) {
        WRITE_PERI_REG(SPI_W0_REG(2) + i, *(uint32_t*)&apa_data[i]);
    }
    WRITE_PERI_REG(SPI_DMA_CONF_REG(2), (1 << SPI_BUF_AFIFO_RST_S));
    WRITE_PERI_REG(SPI_CMD_REG(2), (1 << SPI_UPDATE_S));
    while (READ_PERI_REG(SPI_CMD_REG(2)) & (1 << SPI_UPDATE_S)) {}
    WRITE_PERI_REG(SPI_CMD_REG(2), (1 << SPI_USR_S));
}
#endif
