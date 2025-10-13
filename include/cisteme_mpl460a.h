// Includes
#include "cisteme_mpl460a_regmap.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

// Config structure (READ-ONLY)
struct mpl460a_config
{
    struct spi_dt_spec spi;
    struct gpio_dt_spec nrst;
    struct gpio_dt_spec en;
    struct gpio_dt_spec stby;
    struct gpio_dt_spec txen;
    struct gpio_dt_spec extin;
    struct gpio_dt_spec nthw0;
};

// Functions typedef
typedef int (*mpl460a_boot_cmd_t)(const struct device *dev, uint32_t addr,
                                  uint16_t cmd, uint8_t *data, uint8_t size);
typedef int (*mpl460a_fw_event_cmd_t)(const struct device *dev,
                                      uint32_t *timer_ref,
                                      uint32_t *event_info);
typedef int (*mpl460a_boot_fw_cmd_t)(const struct device *dev, uint8_t *data,
                                     uint32_t size);
typedef int (*mpl460a_set_cmd_t)(const struct device *dev, uint8_t state);
typedef int (*mpl460a_cmd_t)(const struct device *dev);
typedef int (*mpl460a_data_cmd_t)(const struct device *dev, uint16_t *data,
                                  uint8_t len);

typedef int (*mpl460a_pib_cmd_t)(const struct device *dev, uint32_t register_id,
                                 uint16_t *value, uint16_t len);

typedef int (*mpl460a_mod_type_t)(const struct device *dev, PL460_MOD_TYPE mod);
typedef int (*mpl460a_tx_mode_t)(const struct device *dev, PL460_TX_MODE mode);
typedef int (*mpl460a_mod_scheme_t)(const struct device *dev,
                                    PL460_MOD_SCHEME scheme);
typedef int (*mpl460a_delimiter_t)(const struct device *dev,
                                   PL460_DEL_TYPE del);
typedef int (*mpl460a_time_ini_t)(const struct device *dev, uint32_t timeIni);
typedef int (*mpl460a_attenuation_t)(const struct device *dev, uint8_t atten);
typedef int (*mpl460a_band_t)(const struct device *dev, PL460_BAND band);

typedef int (*mpl460a_tx_cb_t)(const struct device *dev, uint32_t t_time,
                               uint32_t RMS, uint8_t result);
typedef int (*mpl460a_rx_cb_t)(const struct device *dev, uint16_t data,
                               uint8_t len);

typedef int (*mpl460a_send_t)(const struct device *dev, uint16_t *data,
                              uint8_t len, mpl460a_tx_cb_t callback);
typedef int (*mpl460a_receive_t)(const struct device *dev,
                                 mpl460a_rx_cb_t callback);

// Data structure (READ-WRITE)
struct mpl460a_data
{
    const struct device *dev;

    uint8_t nrst_state;
    uint8_t en_state;

    CENA_TX_PARAM params;
    PL460_EVENT_DATA irq_events;

    struct gpio_callback extin_cb_data;
    struct k_work get_event_work;

    struct k_sem isr_sem;

    mpl460a_tx_cb_t tx_cb;
    mpl460a_rx_cb_t rx_cb;

    uint16_t *rx_data;
    uint8_t rx_len;
};

// API declaration
__subsystem struct mpl460a_api
{
    // Bootloader commands
    mpl460a_boot_cmd_t mpl460a_boot_write;
    mpl460a_boot_cmd_t mpl460a_boot_read;

    mpl460a_boot_fw_cmd_t mpl460a_boot_write_fw;
    mpl460a_boot_fw_cmd_t mpl460a_boot_check_fw;
    mpl460a_boot_fw_cmd_t mpl460a_fast_init;

    mpl460a_set_cmd_t mpl460a_set_nrst;
    mpl460a_set_cmd_t mpl460a_set_en;

    mpl460a_cmd_t mpl460a_boot_enable;
    mpl460a_cmd_t mpl460a_boot_disable;

    mpl460a_fw_event_cmd_t mpl460a_get_events;
    mpl460a_send_t mpl460a_send;

    mpl460a_pib_cmd_t mpl460a_get_pib;
    mpl460a_pib_cmd_t mpl460a_set_pib;

    mpl460a_mod_type_t mpl460a_set_mod_type;
    mpl460a_tx_mode_t mpl460a_set_tx_mode;
    mpl460a_mod_scheme_t mpl460a_set_mod_scheme;
    mpl460a_delimiter_t mpl460a_set_delimiter;
    mpl460a_time_ini_t mpl460a_set_time_ini;
    mpl460a_attenuation_t mpl460a_set_attenuation;
    mpl460a_band_t mpl460a_set_band;

    mpl460a_receive_t mpl460a_receive;
};

__syscall int mpl460a_boot_write(const struct device *dev, uint32_t addr,
                                 uint16_t cmd, uint8_t *data, uint8_t size);

static inline int z_impl_mpl460a_boot_write(const struct device *dev,
                                            uint32_t addr, uint16_t cmd,
                                            uint8_t *data, uint8_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_write == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_write(dev, addr, cmd, data, size);
}

__syscall int mpl460a_boot_read(const struct device *dev, uint32_t addr,
                                uint16_t cmd, uint8_t *data, uint8_t size);

static inline int z_impl_mpl460a_boot_read(const struct device *dev,
                                           uint32_t addr, uint16_t cmd,
                                           uint8_t *data, uint8_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_read == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_read(dev, addr, cmd, data, size);
}

__syscall int mpl460a_boot_write_fw(const struct device *dev, uint8_t *data,
                                    uint32_t size);

static inline int z_impl_mpl460a_boot_write_fw(const struct device *dev,
                                               uint8_t *data, uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_write_fw == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_write_fw(dev, data, size);
}

__syscall int mpl460a_boot_check_fw(const struct device *dev, uint8_t *data,
                                    uint32_t size);

static inline int z_impl_mpl460a_boot_check_fw(const struct device *dev,
                                               uint8_t *data, uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_check_fw == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_check_fw(dev, data, size);
}

__syscall int mpl460a_set_nrst(const struct device *dev, uint8_t state);

static inline int z_impl_mpl460a_set_nrst(const struct device *dev,
                                          uint8_t state)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_nrst == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_nrst(dev, state);
}

__syscall int mpl460a_set_en(const struct device *dev, uint8_t state);

static inline int z_impl_mpl460a_set_en(const struct device *dev, uint8_t state)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_en == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_en(dev, state);
}

__syscall int mpl460a_boot_enable(const struct device *dev);

static inline int z_impl_mpl460a_boot_enable(const struct device *dev)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_enable == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_enable(dev);
}

__syscall int mpl460a_get_events(const struct device *dev, uint32_t *timer_ref,
                                 uint32_t *events_info);

static inline int z_impl_mpl460a_get_events(const struct device *dev,
                                            uint32_t *timer_ref,
                                            uint32_t *events_info)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_get_events == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_get_events(dev, timer_ref, events_info);
}

__syscall int mpl460a_boot_disable(const struct device *dev);

static inline int z_impl_mpl460a_boot_disable(const struct device *dev)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_disable == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_disable(dev);
}

__syscall int mpl460a_send(const struct device *dev, uint16_t *data,
                           uint8_t len, mpl460a_tx_cb_t callback);

static inline int z_impl_mpl460a_send(const struct device *dev, uint16_t *data,
                                      uint8_t len, mpl460a_tx_cb_t callback)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_send == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_send(dev, data, len, callback);
}

__syscall int mpl460a_get_pib(const struct device *dev, uint32_t register_id,
                              uint16_t *value, uint16_t len);

static inline int z_impl_mpl460a_get_pib(const struct device *dev,
                                         uint32_t register_id, uint16_t *value,
                                         uint16_t len)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_get_pib == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_get_pib(dev, register_id, value, len);
}

__syscall int mpl460a_set_pib(const struct device *dev, uint32_t register_id,
                              uint16_t *value, uint16_t len);

static inline int z_impl_mpl460a_set_pib(const struct device *dev,
                                         uint32_t register_id, uint16_t *value,
                                         uint16_t len)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_pib == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_pib(dev, register_id, value, len);
}

__syscall int mpl460a_set_mod_type(const struct device *dev,
                                   PL460_MOD_TYPE mod);

static inline int z_impl_mpl460a_set_mod_type(const struct device *dev,
                                              PL460_MOD_TYPE mod)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_mod_type == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_mod_type(dev, mod);
}

__syscall int mpl460a_set_tx_mode(const struct device *dev, PL460_TX_MODE mode);

static inline int z_impl_mpl460a_set_tx_mode(const struct device *dev,
                                             PL460_TX_MODE mode)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_tx_mode == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_tx_mode(dev, mode);
}

__syscall int mpl460a_set_mod_scheme(const struct device *dev,
                                     PL460_MOD_SCHEME scheme);

static inline int z_impl_mpl460a_set_mod_scheme(const struct device *dev,
                                                PL460_MOD_SCHEME scheme)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_mod_scheme == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_mod_scheme(dev, scheme);
}

__syscall int mpl460a_set_delimiter(const struct device *dev,
                                    PL460_DEL_TYPE del);

static inline int z_impl_mpl460a_set_delimiter(const struct device *dev,
                                               PL460_DEL_TYPE del)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_delimiter == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_delimiter(dev, del);
}

__syscall int mpl460a_set_time_ini(const struct device *dev, uint32_t timeIni);

static inline int z_impl_mpl460a_set_time_ini(const struct device *dev,
                                              uint32_t timeIni)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_time_ini == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_time_ini(dev, timeIni);
}

__syscall int mpl460a_set_attenuation(const struct device *dev, uint8_t atten);

static inline int z_impl_mpl460a_set_attenuation(const struct device *dev,
                                                 uint8_t atten)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_attenuation == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_attenuation(dev, atten);
}

__syscall int mpl460a_set_band(const struct device *dev, PL460_BAND band);

static inline int z_impl_mpl460a_set_band(const struct device *dev,
                                          PL460_BAND band)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_set_band == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_set_band(dev, band);
}

__syscall int mpl460a_fast_init(const struct device *dev, uint8_t *data,
                                uint32_t size);

static inline int z_impl_mpl460a_fast_init(const struct device *dev,
                                           uint8_t *data, uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_fast_init == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_fast_init(dev, data, size);
}

__syscall int mpl460a_receive(const struct device *dev,
                              mpl460a_rx_cb_t callback);

static inline int z_impl_mpl460a_receive(const struct device *dev,
                                         mpl460a_rx_cb_t callback)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_receive == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_receive(dev, callback);
}

// Include syscall
#include <syscalls/cisteme_mpl460a.h>