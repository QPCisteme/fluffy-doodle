// Declare compatible mention
#define DT_DRV_COMPAT cisteme_mpl460a

// Include libs
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
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
            return -116;
        }
    } while (rx_data[0] != 0);

    return 0;
}

static int boot_write_fw(const struct device *dev, uint8_t *data, uint32_t size)
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
        {
            return ret;
        }
        write_addr += pkt_size;
        max_addr -= pkt_size;
    }

    return 0;
}

static int boot_check_fw(const struct device *dev, uint8_t *data, uint32_t size)
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
    uint8_t tab[4];

    // Clean CPUWAIT to start program
    sys_put_le32(0x01010000, tab);
    ret = boot_write(dev, PL460_MISCR, PL460_WR, tab, 4);
    if (ret < 0)
        return ret;

    // Give MISO to M7-SPI
    ret = boot_write(dev, 0, PL460_MISO_M7_NCLK, 0, 0);
    if (ret < 0)
        return ret;

    return 0;
}

static int fw_id_send(const struct device *dev, uint16_t id, uint8_t *tx,
                      uint8_t tx_size, uint8_t *rx, uint8_t rx_size, bool write)
{
    const struct mpl460a_config *drv_config = dev->config;

    uint8_t tx_header[4];
    uint8_t rx_header[4];

    if (tx_size & 0x01)
        tx_size++;

    if (rx_size & 0x01)
        rx_size++;

    // Copy ID (BE)
    sys_put_be16(id, tx_header);
    // Copy length (BE)
    sys_put_be16(MAX(tx_size >> 1, rx_size >> 1), tx_header + 2);

    // Update R/W bit
    if (write)
        tx_header[2] |= 0x80;

    // SPI communication
    struct spi_buf tx_spi_buf[2] = {{.buf = tx_header, .len = 4},
                                    {.buf = tx, .len = tx_size}};
    struct spi_buf_set tx_spi_data_set = {.buffers = tx_spi_buf, .count = 2};

    // RX buffers
    struct spi_buf rx_spi_buf[2] = {{.buf = rx_header, .len = 4},
                                    {.buf = rx, .len = rx_size}};
    struct spi_buf_set rx_spi_data_set = {.buffers = rx_spi_buf, .count = 2};

    int ret =
        spi_transceive_dt(&drv_config->spi, &tx_spi_data_set, &rx_spi_data_set);
    if (ret < 0)
        return ret;

    printk("TX : ");
    for (int i = 0; i < tx_size; i++)
        printk("%.2x ", tx[i]);
    printk("\r\n");

    printk("RX : ");
    for (int i = 0; i < rx_size; i++)
        printk("%.2x ", rx[i]);
    printk("\r\n");

    // Check FW header
    uint16_t header = sys_get_be16(&rx_header[0]);
    if (header != PL460_FW_HEADER)
        return -2;

    // Return events (LE)
    int events = sys_get_be16(&rx_header[2]);

    return events;
}

static int fw_get_events(const struct device *dev, uint32_t *timer_ref,
                         uint32_t *event_info)
{
    uint8_t rx_data[8];
    uint16_t events = fw_id_send(dev, PL460_G3_STATUS, 0, 0, rx_data, 8, false);

    if (events < 0)
        return events;

    *timer_ref = (sys_get_be16(rx_data + 2) << 16) | (sys_get_be16(rx_data));

    *event_info =
        (sys_get_be16(rx_data + 6) << 16) | (sys_get_be16(rx_data + 4));

    return events;
}

void extin_IRQ(const struct device *dev, struct gpio_callback *cb,
               uint32_t pins)
{
    struct mpl460a_data *data =
        CONTAINER_OF(cb, struct mpl460a_data, extin_cb_data);

    k_work_submit(&data->get_event_work);
}

static void wq_tx_cfm(struct k_work *work)
{
    struct mpl460a_data *drv_data =
        CONTAINER_OF(work, struct mpl460a_data, tx_cfm_work);

    uint8_t rx_cfm[10];
    uint32_t t_time, rms;
    uint8_t result;
    int ret =
        fw_id_send(drv_data->dev, PL460_G3_TX_CONFIRM, 0, 0, rx_cfm, 10, false);
    if (ret < 0)
        return;

    rms = (sys_get_be16(rx_cfm + 2) << 16) | (sys_get_be16(rx_cfm));

    t_time = (sys_get_be16(rx_cfm + 6) << 16) | (sys_get_be16(rx_cfm + 4));

    result = rx_cfm[9];

    if (drv_data->tx_cb != NULL)
        drv_data->tx_cb(drv_data->dev, t_time, rms, result);

    return;
}

static void wq_rx_data(struct k_work *work)
{
    struct mpl460a_data *drv_data =
        CONTAINER_OF(work, struct mpl460a_data, rx_data_work);

    drv_data->rx_len = (drv_data->irq_events.info & 0x000000FF);

    fw_id_send(drv_data->dev, PL460_G3_RX_DATA, 0, 0, drv_data->rx_data,
               drv_data->rx_len, false);

    return;
}

static void wq_rx_param(struct k_work *work)
{
    struct mpl460a_data *drv_data =
        CONTAINER_OF(work, struct mpl460a_data, rx_param_work);

    int ret;

    uint8_t rx_params[117];

    ret = fw_id_send(drv_data->dev, PL460_G3_RX_PARAM, 0, 0, rx_params, 117,
                     false);
    if (ret < 0)
        return;

    PL460_RX_PARAM params;

    params.ul_rx_time =
        (sys_get_be16(rx_params + 2) << 16) | (sys_get_be16(rx_params));
    params.ul_frame_duration =
        (sys_get_be16(rx_params + 6) << 16) | (sys_get_be16(rx_params + 4));
    params.us_rssi = sys_get_be16(rx_params + 8);
    params.us_data_len = sys_get_be16(rx_params + 10);
    params.uc_zct_diff = *(rx_params + 12);
    params.uc_rs_corrected_errors = *(rx_params + 13);
    params.uc_mod_type = *(rx_params + 14);
    params.uc_mod_scheme = *(rx_params + 15);
    params.ul_agc_factor =
        (sys_get_be16(rx_params + 18) << 16) | (sys_get_be16(rx_params + 16));
    params.us_agc_fine = sys_get_be16(rx_params + 20);
    params.ss_agc_offset_meas = sys_get_be16(rx_params + 22);
    params.uc_agc_active = *(rx_params + 24);
    params.uc_agc_pga_value = *(rx_params + 25);
    params.ss_snr_fch = sys_get_be16(rx_params + 26);
    params.ss_snr_pay = sys_get_be16(rx_params + 28);
    params.us_payload_corrupted_carriers = sys_get_be16(rx_params + 30);
    params.us_payload_noised_symbols = sys_get_be16(rx_params + 32);
    params.uc_payload_snr_worst_carrier = *(rx_params + 34);
    params.uc_payload_snr_worst_symbol = *(rx_params + 35);
    params.uc_payload_snr_impulsive = *(rx_params + 36);
    params.uc_payload_snr_band = *(rx_params + 37);
    params.uc_payload_snr_background = *(rx_params + 38);
    params.uc_lqi = *(rx_params + 39);
    params.uc_delimiter_type = *(rx_params + 40);
    params.uc_crc_ok = *(rx_params + 41);
    memcpy(params.puc_tone_map, rx_params + 42, 3);
    memcpy(params.puc_carrier_snr, rx_params + 45, 72);

    drv_data->rx_cb(drv_data->dev, drv_data->rx_data + 2,
                    sys_get_le16(drv_data->rx_data), &params);
    return;
}

static void wq_get_event(struct k_work *work)
{
    struct mpl460a_data *drv_data =
        CONTAINER_OF(work, struct mpl460a_data, get_event_work);

    uint32_t timer_ref, event_info;
    int ret = fw_get_events(drv_data->dev, &timer_ref, &event_info);
    if (ret < 0)
        return;

    drv_data->irq_events.flag = (uint16_t)ret;
    drv_data->irq_events.tref = timer_ref;
    drv_data->irq_events.info = event_info;

    if (ret & PL460_TX_CFM_FLAG)
    {
        k_work_submit(&drv_data->tx_cfm_work);
    }

    if (ret & PL460_RX_DATA_FLAG)
    {
        k_work_submit(&drv_data->rx_data_work);
    }

    if (ret & PL460_REG_DATA_FLAG)
    {
        k_sem_give(&drv_data->isr_sem);
    }

    if (ret & PL460_RX_PARAM_FLAG)
    {
        k_work_submit(&drv_data->rx_param_work);
    }
}

static int fw_send(const struct device *dev, uint8_t *data, uint8_t len,
                   mpl460a_tx_cb_t callback)
{
    // Limit packet length
    if (len > 64)
        return -1;

    struct mpl460a_data *drv_data = dev->data;

    int ret;

    // Send TX_PARAMS
    drv_data->params.dataLength = len;
    drv_data->tx_cb = callback;

    uint8_t tx_params[40];
    uint8_t *pSrc = (uint8_t *)&drv_data->params;
    for (int i = 0; i < 40; i += 2)
    {
        tx_params[i] = pSrc[i + 1];
        tx_params[i + 1] = pSrc[i];
    }

    ret = fw_id_send(dev, PL460_G3_TX_PARAM, tx_params, 40, 0, 0, true);
    if (ret < 0)
        return ret;

    // Send TX_DATA
    ret = fw_id_send(dev, PL460_G3_TX_DATA, data, len, 0, 0, true);

    return ret;
}

static int fw_receive(const struct device *dev, mpl460a_rx_cb_t callback)
{
    struct mpl460a_data *drv_data = dev->data;

    if (callback == NULL)
        return -1;

    drv_data->rx_cb = callback;

    return 0;
}

static int set_pib(const struct device *dev, uint32_t register_id,
                   uint8_t *value, uint16_t len)
{
    int ret;

    uint16_t size = len;
    if (len & 0x01)
        size++;

    uint8_t tx_data[6 + size];

    sys_put_le16(register_id >> 16, tx_data);
    sys_put_le16(register_id & 0x0fff, tx_data + 2);
    sys_put_le16((1 << 10) | len, tx_data + 4);
    for (int i = 0; i < size; i += 2)
    {
        tx_data[6 + i] = i < len ? value[i + 1] : 0x00;
        tx_data[7 + i] = value[i];
    }

    ret = fw_id_send(dev, PL460_G3_REG_INFO, tx_data, 6 + size, 0, 0, true);
    if (ret < 0)
        return ret;

    return ret;
}

static int get_pib(const struct device *dev, uint32_t register_id,
                   uint8_t *value, uint16_t len)
{
    struct mpl460a_data *drv_data = dev->data;

    int ret;

    // Send register_id, len and dummy
    uint8_t tx_data[8];
    sys_put_le16(register_id >> 16, tx_data);
    sys_put_le16(register_id & 0x0fff, tx_data + 2);
    sys_put_le16(len, tx_data + 4);
    sys_put_le16(0x0000, tx_data + 4);

    ret = fw_id_send(dev, PL460_G3_REG_INFO, tx_data, 8, 0, 0, true);
    if (ret < 0)
        return ret;

    if (k_sem_take(&drv_data->isr_sem, K_MSEC(10)) != 0)
    {
        return PL460_TIMEOUT;
    }

    if (!(drv_data->irq_events.flag & PL460_REG_DATA_FLAG))
        return PL460_UNEXPECTED_EVENT;

    // Read PIB value
    ret = fw_id_send(dev, PL460_G3_REG_INFO, 0, 0, value,
                     drv_data->irq_events.info >> 16, false);
    if (ret < 0)
        return ret;

    return ret;
}

static int set_mod_type(const struct device *dev, PL460_MOD_TYPE mod)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.modType = mod;

    return 0;
}

static int set_tx_mode(const struct device *dev, PL460_TX_MODE mode)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.mode = mode;

    return 0;
}

static int set_mod_scheme(const struct device *dev, PL460_MOD_SCHEME scheme)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.modScheme = scheme;

    return 0;
}

static int set_delimiter(const struct device *dev, PL460_DEL_TYPE del)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.delimiterType = del;

    return 0;
}

static int set_time_ini(const struct device *dev, uint32_t timeIni)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.timeIni = timeIni;

    return 0;
}

static int set_attenuation(const struct device *dev, uint8_t atten)
{
    struct mpl460a_data *drv_data = dev->data;

    drv_data->params.attenuation = atten;

    return 0;
}

static int set_band(const struct device *dev, PL460_BAND band)
{
    struct mpl460a_data *drv_data = dev->data;

    switch (band)
    {
    case PL460_BAND_CENA:
        memcpy(drv_data->params.toneMap, (uint8_t[]){0x3F, 0x00, 0x00}, 3);
        break;

    case PL460_BAND_CENB:
        memcpy(drv_data->params.toneMap, (uint8_t[]){0x0F, 0x00, 0x00}, 3);
        break;

    case PL460_BAND_ARIB:
        memcpy(drv_data->params.toneMap, (uint8_t[]){0xFF, 0xFF, 0x03}, 3);
        break;

    case PL460_BAND_FCC:
        memcpy(drv_data->params.toneMap, (uint8_t[]){0xFF, 0xFF, 0xFF}, 3);
        break;

    default:
        return -1;
    }

    return 0;
}

static int fast_init(const struct device *dev, uint8_t *data, uint32_t size)
{
    int ret;

    ret = set_en(dev, 1);
    if (ret < 0)
        return -3;

    k_msleep(100);

    ret = set_nrst(dev, 1);
    if (ret < 0)
        return -4;

    k_msleep(100);

    ret = boot_enable(dev);
    if (ret < 0)
        return -4;

    k_msleep(100);

    ret = boot_write_fw(dev, data, size);
    if (ret < 0)
        return -1;

    k_msleep(100);

    ret = boot_check_fw(dev, data, size);
    if (ret < 0)
        return ret;

    k_msleep(100);

    ret = boot_disable(dev);
    if (ret < 0)
        return -2;

    return 0;
}

// Fill API with functions
static const struct mpl460a_api api = {
    .mpl460a_boot_write = &boot_write,
    .mpl460a_boot_read = &boot_read,

    .mpl460a_boot_write_fw = &boot_write_fw,
    .mpl460a_boot_check_fw = &boot_check_fw,
    .mpl460a_fast_init = &fast_init,

    .mpl460a_set_nrst = &set_nrst,
    .mpl460a_set_en = &set_en,

    .mpl460a_boot_enable = &boot_enable,
    .mpl460a_boot_disable = &boot_disable,

    .mpl460a_get_events = &fw_get_events,
    .mpl460a_send = &fw_send,
    .mpl460a_get_pib = &get_pib,
    .mpl460a_set_pib = &set_pib,

    .mpl460a_set_mod_type = &set_mod_type,
    .mpl460a_set_tx_mode = &set_tx_mode,
    .mpl460a_set_mod_scheme = &set_mod_scheme,
    .mpl460a_set_delimiter = &set_delimiter,
    .mpl460a_set_time_ini = &set_time_ini,
    .mpl460a_set_attenuation = &set_attenuation,
    .mpl460a_set_band = &set_band,

    .mpl460a_receive = &fw_receive,
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

    if (!gpio_is_ready_dt(&drv_config->extin))
        return -1;

    if (!gpio_is_ready_dt(&drv_config->stby))
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

    ret = gpio_pin_configure_dt(&drv_config->stby, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
        return ret;

    ret = gpio_pin_configure_dt(&drv_config->txen, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
        return ret;

    ret = gpio_pin_configure_dt(&drv_config->extin, GPIO_INPUT);
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

    drv_data->dev = dev;

    drv_data->tx_cb = NULL;
    drv_data->rx_cb = NULL;
    drv_data->rx_len = 0;

    k_sem_init(&drv_data->isr_sem, 0, 1);
    k_work_init(&drv_data->get_event_work, wq_get_event);
    k_work_init(&drv_data->rx_param_work, wq_rx_param);
    k_work_init(&drv_data->rx_data_work, wq_rx_data);
    k_work_init(&drv_data->tx_cfm_work, wq_tx_cfm);

    gpio_pin_interrupt_configure_dt(&drv_config->extin, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&drv_data->extin_cb_data, extin_IRQ,
                       BIT(drv_config->extin.pin));
    gpio_add_callback(drv_config->extin.port, &drv_data->extin_cb_data);

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