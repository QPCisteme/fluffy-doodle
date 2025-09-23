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
static int boot_write(const struct device *dev, uint32_t addr, uint16_t cmd,
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

static int boot_read(const struct device *dev, uint32_t addr, uint16_t cmd,
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

static int boot_wait_wip(const struct device *dev)
{
    uint8_t rx_data[4];
    uint8_t timeout = 255;
    int ret;

    do
    {
        k_usleep(100);

        // Read WIP bit
        ret = boot_read(dev, 0, PL460_RD_BOOT_STATUS, rx_data, 4);
        if (ret < 0)
            return ret;

        // Timeout conditions
        if (timeout-- == 0)
        {
            printk("FW Write Timeout\r\n");
            return -116;
        }
    } while (rx_data[0] != 0);

    return 0;
}

static int boot_write_fw(const struct device *dev, const uint8_t *data,
                         const uint32_t size)
{
    int ret;
    uint32_t write_addr = 0;
    uint8_t pkt_size;
    uint8_t pkt_data[252];

    while (size > 0)
    {
        pkt_size = (size > 252 ? 252 : size);

        ret = boot_write(dev, write_addr, PL460_MULT_WR, &data[write_addr],
                         pkt_size);
        if (ret < 0)
            return ret;

        ret = boot_wait_wip(dev);
        if (ret < 0)
            return ret;

        write_addr += pkt_size;
        size -= pkt_size;
    }

    return 0;
}

static int boot_write_fw(const struct device *dev, const uint8_t *data,
                         const uint32_t size)
{
    int ret;
    uint32_t read_addr = 0;
    uint8_t pkt_size;
    uint8_t pkt_data[252];

    while (size > 0)
    {
        pkt_size = (size > 252 ? 252 : size);

        ret = boot_read(dev, write_addr, PL460_MULT_RD, pkt_data, pkt_size);
        if (ret < 0)
            return ret;

        ret = boot_wait_wip(dev);
        if (ret < 0)
            return ret;

        if (memcmp(pkt_data, &data[read_addr], pkt_size) != 0)
            return -1;

        read_addr += pkt_size;
        size -= pkt_size;
    }

    return 0;
}

static int set_nrst(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->nrst, state);
    drv_data->nrst_state = state;

    return 0;
}

static int set_en(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->en, state);
    drv_data->en_state = state;

    return 0;
}

static int boot_unlock(const struct device *dev)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    // Union to reverse endianess
    union {
        uint32_t val;
        uint8_t tab[4];
    } mpl460a_conv;

    mpl460a_conv.val = PL460_BOOT_PASS_0;
    boot_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    mpl460a_conv.val = PL460_BOOT_PASS_1;
    boot_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    return 0;
}

static int start_fw(const struct device *dev)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    int ret;

    // Clean CPUWAIT to start program
    ret = boot_write(dev, PL460_CPUWAIT_ADDR, 0x0000, 0, 0);
    if (ret < 0)
        return ret;

    // Give MISO to M7-SPI
    ret = boot_write(dev, 0, PL460_MISO_M7_NCLK, 0, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static int fw_get_events(const struct device *dev, uint32_t *timer_ref,
                         uint32_t *event_info)
{
    struct mpl460a_data *drv_data = dev->data;
    struct mpl460a_config *drv_config = dev->config;

    uint8_t spi_tx[4] = {(uint8_t)(PL460_G3_STATUS >> 8) & 0xff,
                         (uint8_t)PL460_G3_STATUS & 0xff, 0x00, 0x04};

    uint8_t spi_rx[12];

    struct spi_buf tx_spi_buf = {.buf = spi_tx, .len = 4};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_bufs = {.buf = spi_rx, .len = 12};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

    int ret;
    ret = spi_transceive_dt(&drv_config->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (ret < 0)
    {
        return ret;
    }

    *event_info =
        (spi_rx[0] << 24) | (spi_rx[1] << 16) | (spi_rx[2] << 8) | (spi_rx[3]);
    *timer_ref =
        (spi_rx[4] << 24) | (spi_rx[5] << 16) | (spi_rx[6] << 8) | (spi_rx[7]);

    return 0;
}

// Fill API with functions
static const struct mpl460a_api api = {
    .mpl460a_boot_write = &boot_write,
    .mpl460a_boot_read = &boot_read,
    .mpl460a_boot_write_fw = &boot_write_fw,
    .mpl460a_boot_check_fw = &boot_check_fw,
    .mpl460a_set_nrst = &set_nrst,
    .mpl460a_set_en = &set_en,
    .mpl460a_boot_unlock = &boot_unlock,
    .mpl460a_get_events = &fw_get_events,
    .mpl460a_start_fw = &start_fw,
};

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
        .spi =                                                                 \
            SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
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