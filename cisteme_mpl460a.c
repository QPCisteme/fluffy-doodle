// Declare compatible mention
#define DT_DRV_COMPAT cisteme_mpl460a

// Include libs
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

// Include custom libs
#include "cisteme_mpl460a_regmap.h"
#include <cisteme_mpl460a.h>

// Define C function
static int mpl460a_write(const struct device *dev, uint32_t addr, uint16_t cmd,
                         uint8_t *data, uint8_t size)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    uint8_t addr_cmd[6] = {(addr) & 0xFF,       (addr >> 8) & 0xFF,
                           (addr >> 16) & 0xFF, (addr >> 24) & 0xFF,
                           (cmd) & 0xFF,        (cmd >> 8) & 0xFF};

    uint8_t tx_full[6 + size];
    memcpy(tx_full, addr_cmd, 6);

    if (size > 0)
        memcpy(tx_full + 6, data, size);

    uint8_t rx_data[4];

    struct spi_buf tx_spi_buf = {.buf = tx_full, .len = size + 6};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_bufs = {.buf = rx_data, .len = 4};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

    int ret;
    ret = spi_transceive_dt(&drv_config->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

static int mpl460a_read(const struct device *dev, uint32_t addr, uint16_t cmd,
                        uint8_t *data, uint8_t size)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    uint8_t addr_cmd[6] = {(addr) & 0xFF,       (addr >> 8) & 0xFF,
                           (addr >> 16) & 0xFF, (addr >> 24) & 0xFF,
                           (cmd) & 0xFF,        (cmd >> 8) & 0xFF};

    uint8_t rx_data[size + 6];

    struct spi_buf tx_spi_buf = {.buf = addr_cmd, .len = 6};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_bufs = {.buf = rx_data, .len = size + 6};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

    int ret;
    ret = spi_transceive_dt(&drv_config->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (ret < 0)
    {
        return ret;
    }
    memcpy(data, rx_data + 6, size);

    return 0;
}

static int mpl460a_firmware_write(const struct device *dev, uint8_t *data,
                                  uint32_t size)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    return 0;
}

static int mpl460a_firmware_check(const struct device *dev, uint8_t *data,
                                  uint32_t size)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    return 0;
}

static int mpl460a_set_nrst(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->nrst, state);
    drv_data->nrst_state = state;

    return 0;
}

static int mpl460a_set_en(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->en, state);
    drv_data->en_state = state;

    return 0;
}

static int mpl460a_unlock_boot(const struct device *dev)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    // Union reverse endianess
    union {
        uint32_t val;
        uint8_t tab[4];
    } mpl460a_conv;

    mpl460a_conv.val = PL460_BOOT_PASS_0;
    mpl460a_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    mpl460a_conv.val = PL460_BOOT_PASS_1;
    mpl460a_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    return 0;
}

// Fill API with functions
static const struct mpl460a_api api = {
    .mpl460a_write = &mpl460a_write,
    .mpl460a_read = &mpl460a_read,
    .mpl460a_firmware_write = &mpl460a_firmware_write,
    .mpl460a_firmware_check = &mpl460a_firmware_check,
    .mpl460a_set_nrst = &mpl460a_set_nrst,
    .mpl460a_set_en = &mpl460a_set_en,
    .mpl460a_unlock_boot = &mpl460a_unlock_boot,
};

// Create data structure
static struct mpl460a_data data;

// Init function (called at creation)
static int mpl460a_init(const struct device *dev)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    drv_data->nrst_state = 0;
    drv_data->en_state = 0;

    if (!gpio_is_ready_dt(&drv_config->nrst))
        return -1;

    if (!gpio_is_ready_dt(&drv_config->en))
        return -1;

    if (!spi_is_ready_dt(&drv_config->spi))
        return -1;

    int ret;

    ret = gpio_pin_configure_dt(&drv_config->nrst, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
        return ret;

    ret = gpio_pin_configure_dt(&drv_config->en, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
        return ret;

    return 0;
}

// Create driver for each DTS instance
#define MPL460A_DEFINE(inst)                                                   \
    static struct mpl460a_data mpl460a_data_##inst;                            \
    static const struct mpl460a_config mpl460a_config_##inst = {               \
        .spi = SPI_DT_SPEC_INST_GET(inst),                                     \
        .nrst = GPIO_DT_SPEC_INST_GET(inst, nrst_gpios),                       \
        .en = GPIO_DT_SPEC_INST_GET(inst, en_gpios),                           \
        .stby = GPIO_DT_SPEC_INST_GET(inst, stby_gpios),                       \
        .txen = GPIO_DT_SPEC_INST_GET(inst, txen_gpios),                       \
        .extin = GPIO_DT_SPEC_INST_GET(inst, extin_gpios),                     \
        .nthw0 = GPIO_DT_SPEC_INST_GET(inst, nthw0_gpios),                     \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, mpl460a_init, NULL, &mpl460a_data_##inst,      \
                          &mpl460a_config_##inst, POST_KERNEL, 90, &api);

DT_INST_FOREACH_STATUS_OKAY(MPL460A_DEFINE)