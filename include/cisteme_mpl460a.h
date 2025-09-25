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

// Data structure (READ-WRITE)
struct mpl460a_data
{
    uint8_t nrst_state;
    uint8_t en_state;
};

// Functions typedef
typedef int (*mpl460a_boot_cmd_t)(const struct device *dev, uint32_t addr,
                                  uint16_t cmd, uint8_t *data, uint8_t size);
typedef int (*mpl460a_fw_event_cmd_t)(const struct device *dev,
                                      uint32_t *timer_ref,
                                      uint32_t *event_info);
typedef int (*mpl460a_boot_fw_cmd_t)(const struct device *dev,
                                     const uint8_t *data, const uint32_t size);
typedef int (*mpl460a_set_cmd_t)(const struct device *dev, uint8_t state);
typedef int (*mpl460a_cmd_t)(const struct device *dev);
typedef int (*mpl460a_data_cmd_t)(const struct device *dev, uint8_t *data,
                                  uint8_t len);
// API declaration
__subsystem struct mpl460a_api
{
    // General
    mpl460a_set_cmd_t mpl460a_set_nrst;
    mpl460a_set_cmd_t mpl460a_set_en;
    mpl460a_cmd_t mpl460a_boot_disable;

    // Bootloader commands
    mpl460a_boot_cmd_t mpl460a_boot_write;
    mpl460a_boot_cmd_t mpl460a_boot_read;
    mpl460a_boot_fw_cmd_t mpl460a_boot_write_fw;
    mpl460a_boot_fw_cmd_t mpl460a_boot_check_fw;
    mpl460a_cmd_t mpl460a_boot_enable;

    // Firmware commands
    mpl460a_fw_event_cmd_t mpl460a_get_events;
    mpl460a_data_cmd_t mpl460a_send;
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

__syscall int mpl460a_boot_write_fw(const struct device *dev,
                                    const uint8_t *data, const uint32_t size);

static inline int z_impl_mpl460a_boot_write_fw(const struct device *dev,
                                               const uint8_t *data,
                                               const uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_boot_write_fw == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_boot_write_fw(dev, data, size);
}

__syscall int mpl460a_boot_check_fw(const struct device *dev,
                                    const uint8_t *data, const uint32_t size);

static inline int z_impl_mpl460a_boot_check_fw(const struct device *dev,
                                               const uint8_t *data,
                                               const uint32_t size)
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

__syscall int mpl460a_send(const struct device *dev, uint8_t *data,
                           uint8_t len);

static inline int z_impl_mpl460a_send(const struct device *dev, uint8_t *data,
                                      uint8_t len)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_send == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_send(dev, data, len);
}

// Include syscall
#include <syscalls/cisteme_mpl460a.h>