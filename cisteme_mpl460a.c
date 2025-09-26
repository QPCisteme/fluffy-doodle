// Declare compatible mention
#define DT_DRV_COMPAT cisteme_mpl460a

// Include libs
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

// Include custom libs
#include <cisteme_mpl460a.h>

// Define C function
static int boot_write(const struct device *dev, uint32_t addr, uint16_t cmd,
                      uint8_t *data, uint8_t size)
{
    const struct mpl460a_config *drv_config = dev->config;

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
    const struct mpl460a_config *drv_config = dev->config;

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
    uint32_t write_addr = 0, max_addr = size;
    uint8_t pkt_size;

    while (max_addr > 0)
    {
        pkt_size = (max_addr > 252 ? 252 : max_addr);

        ret = boot_write(dev, write_addr, PL460_MULT_WR, &data[write_addr],
                         pkt_size);
        if (ret < 0)
            return ret;

        ret = boot_wait_wip(dev);
        if (ret < 0)
            return ret;

        write_addr += pkt_size;
        max_addr -= pkt_size;
    }

    return 0;
}

static int boot_check_fw(const struct device *dev, const uint8_t *data,
                         const uint32_t size)
{
    int ret, err = 0;
    uint32_t read_addr = 0, max_addr = size;
    uint8_t pkt_size;
    uint8_t pkt_data[252];

    while (max_addr > 0)
    {
        pkt_size = (max_addr > 252 ? 252 : max_addr);

        ret = boot_read(dev, read_addr, PL460_MULT_RD, pkt_data, pkt_size);
        if (ret < 0)
            return ret;

        ret = boot_wait_wip(dev);
        if (ret < 0)
            return ret;

        for (int i = 0; i < pkt_size; i++)
        {
            if (pkt_data[i] != data[read_addr + i])
            {
                err++;
            }
        }

        read_addr += pkt_size;
        max_addr -= pkt_size;
    }

    return err;
}

static int set_nrst(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    const struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->nrst, state);
    drv_data->nrst_state = state;

    return 0;
}

static int set_en(const struct device *dev, uint8_t state)
{
    struct mpl460a_data *drv_data = dev->data;
    const struct mpl460a_config *drv_config = dev->config;

    gpio_pin_set_dt(&drv_config->en, state);
    drv_data->en_state = state;

    return 0;
}

static int boot_enable(const struct device *dev)
{

    // Union to reverse endianess
    union {
        uint32_t val;
        uint8_t tab[4];
    } mpl460a_conv;

    mpl460a_conv.val = PL460_BOOT_PASS_0;
    boot_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    mpl460a_conv.val = PL460_BOOT_PASS_1;
    boot_write(dev, 0, PL460_BOOT_UNLOCK, mpl460a_conv.tab, 4);

    mpl460a_conv.val = 0x01010001;
    boot_write(dev, PL460_MISCR, PL460_WR, mpl460a_conv.tab, 4);

    return 0;
}

static int boot_disable(const struct device *dev)
{

    int ret;

    // Union to reverse endianess
    union {
        uint32_t val;
        uint8_t tab[4];
    } mpl460a_conv;

    // Clean CPUWAIT to start program
    mpl460a_conv.val = 0x01010000;
    ret = boot_write(dev, PL460_MISCR, PL460_WR, mpl460a_conv.tab, 4);
    if (ret < 0)
        return ret;

    k_msleep(10);

    // Give MISO to M7-SPI
    ret = boot_write(dev, 0, PL460_MISO_M7_NCLK, 0, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static int fw_id_send(const struct device *dev, uint16_t id, uint8_t *tx,
                      uint16_t tx_size, uint8_t *rx, uint16_t rx_size,
                      bool write)
{
    if (tx_size & 0x8000)
        return -1;

    const struct mpl460a_config *drv_config = dev->config;

    uint8_t tx_data[tx_size + 4];
    uint8_t rx_data[rx_size + 4];

    // Copy ID (LE)
    sys_put_le16(id, &tx_data[0]);
    // Copy length (LE)
    sys_put_le16(tx_size >> 1, &tx_data[2]);

    // Update R/W bit
    if (write)
        tx_data[3] |= 0x80;

    // Copy payload (16-bit words in LE)
    if (tx_size > 0)
        memcpy(tx_data + 4, tx, tx_size);

    // SPI communication
    struct spi_buf tx_spi_buf_data = {.buf = tx_data, .len = tx_size + 4};
    struct spi_buf_set tx_spi_data_set = {.buffers = &tx_spi_buf_data,
                                          .count = 1};

    struct spi_buf rx_spi_buf_data = {.buf = rx_data, .len = rx_size + 4};
    struct spi_buf_set rx_spi_data_set = {.buffers = &rx_spi_buf_data,
                                          .count = 1};

    int ret =
        spi_transceive_dt(&drv_config->spi, &tx_spi_data_set, &rx_spi_data_set);
    if (ret < 0)
        return ret;

    // Check FW header
    uint16_t header = sys_get_be16(&rx_data[0]);
    if (header != PL460_FW_HEADER)
        return -2;

    // Copy RX data if present
    if (rx_size > 0)
        memcpy(rx, rx_data + 4, rx_size);

    // Return events (LE)
    int events = sys_get_le16(&rx_data[2]);
    return events;
}

static int fw_get_events(const struct device *dev, uint32_t *timer_ref,
                         uint32_t *event_info)
{
    uint8_t rx_data[8];
    uint16_t events = fw_id_send(dev, PL460_G3_STATUS, 0, 0, rx_data, 8, false);

    if (events < 0)
        return events;

    *timer_ref = (rx_data[2] << 24) | (rx_data[3] << 16) | (rx_data[0] << 8) |
                 (rx_data[1]);
    *event_info = (rx_data[6] << 24) | (rx_data[7] << 16) | (rx_data[4] << 8) |
                  (rx_data[5]);

    return events;
}

static int fw_send(const struct device *dev, uint8_t *data, uint8_t len)
{
    // Limit packet length
    if (len > 64)
        return -1;

    struct mpl460a_data *drv_data = dev->data;
    int ret;

    drv_data->params.dataLength = len + 2;
    ret = fw_id_send(dev, PL460_G3_TX_PARAM, (uint8_t *)&drv_data->params, 40,
                     0, 0, true);
    if (ret < 0)
        return ret;

    // CMD 2 : données
    ret = fw_id_send(dev, PL460_G3_TX_DATA, data, len, 0, 0, true);
    if (ret < 0)
        return ret;

    return 0;
}

static int pib_read(const struct device *dev, uint16_t addr)
{
    int ret;

    uint32_t register_id = addr + PL460_G3_BASE_PIB;
    uint8_t tx[8] = {(register_id >> 24) & 0xff,
                     (register_id >> 16) & 0xff,
                     (register_id >> 8) & 0xff,
                     (register_id) & 0xff,
                     0x01,
                     0x00,
                     0x00,
                     0x00};

    ret = fw_id_send(dev, PL460_G3_REG_INFO, tx, 8, 0, 0, true);
    if (ret < 0)
        return ret;

    uint32_t timer_ref, event_info;

    k_msleep(10);

    ret = fw_get_events(dev, &timer_ref, &event_info);
    if (ret < 0)
        return ret;

    uint16_t reg_len = (uint16_t)event_info & 0xffff;

    printk("register len : %.4x", reg_len);

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
    .mpl460a_boot_enable = &boot_enable,
    .mpl460a_get_events = &fw_get_events,
    .mpl460a_boot_disable = &boot_disable,
    .mpl460a_send = &fw_send,
    .mpl460a_pib_read = &pib_read,
};

// Init function (called at creation)
static int mpl460a_init(const struct device *dev)
{
    struct mpl460a_data *drv_data = dev->data;
    const struct mpl460a_config *drv_config = dev->config;

    drv_data->nrst_state = 0;
    drv_data->en_state = 0;

    if (!gpio_is_ready_dt(&drv_config->nrst))
        return -1;

    if (!gpio_is_ready_dt(&drv_config->en))
        return -1;

    if (!gpio_is_ready_dt(&drv_config->txen))
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

    ret = gpio_pin_configure_dt(&drv_config->txen, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
        return ret;

    drv_data->params.timeIni = 0;
    drv_data->params.dataLength = 0; // payload + FCS
    memset(drv_data->params.preemphasis, 0, 24);
    drv_data->params.toneMap[0] = 0x3F;
    memset(drv_data->params.toneMap + 1, 0, 2);
    drv_data->params.mode = 3;
    drv_data->params.attenuation = 0;
    drv_data->params.modType = 0;
    drv_data->params.modScheme = 0;
    drv_data->params.pdc = 0;
    drv_data->params.rs2Blocks = 0;
    drv_data->params.delimiterType = 0;

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