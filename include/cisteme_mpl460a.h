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
typedef int (*mpl460a_data_cmd_t)(const struct device *dev, uint32_t addr,
                                  uint16_t cmd, uint8_t *data, uint8_t size);
typedef int (*mpl460a_firmware_cmd_t)(const struct device *dev, uint8_t *data,
                                      uint32_t size);
typedef int (*mpl460a_set_cmd_t)(const struct device *dev, uint8_t state);
typedef int (*mpl460a_cmd_t)(const struct device *dev);

// API declaration
__subsystem struct mpl460a_api
{
    mpl460a_data_cmd_t mpl460a_write;
    mpl460a_data_cmd_t mpl460a_read;
    mpl460a_firmware_cmd_t mpl460a_firmware_write;
    mpl460a_firmware_cmd_t mpl460a_firmware_check;
    mpl460a_set_cmd_t mpl460a_set_nrst;
    mpl460a_set_cmd_t mpl460a_set_en;
    mpl460a_cmd_t mpl460a_unlock_boot;
};

__syscall int mpl460a_write(const struct device *dev, uint32_t addr,
                            uint16_t cmd, uint8_t *data, uint8_t size);

static inline int z_impl_mpl460a_write(const struct device *dev, uint32_t addr,
                                       uint16_t cmd, uint8_t *data,
                                       uint8_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_write == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_write(dev, addr, cmd, data, size);
}

__syscall int mpl460a_read(const struct device *dev, uint32_t addr,
                           uint16_t cmd, uint8_t *data, uint8_t size);

static inline int z_impl_mpl460a_read(const struct device *dev, uint32_t addr,
                                      uint16_t cmd, uint8_t *data, uint8_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_read == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_read(dev, addr, cmd, data, size);
}

__syscall int mpl460a_firmware_write(const struct device *dev, uint8_t *data,
                                     uint32_t size);

static inline int z_impl_mpl460a_firmware_write(const struct device *dev,
                                                uint8_t *data, uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_firmware_write == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_firmware_write(dev, data, size);
}

__syscall int mpl460a_firmware_check(const struct device *dev, uint8_t *data,
                                     uint32_t size);

static inline int z_impl_mpl460a_firmware_check(const struct device *dev,
                                                uint8_t *data, uint32_t size)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_firmware_check == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_firmware_check(dev, data, size);
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

__syscall int mpl460a_unlock_boot(const struct device *dev);

static inline int z_impl_mpl460a_unlock_boot(const struct device *dev)
{
    const struct mpl460a_api *api = (const struct mpl460a_api *)dev->api;
    if (api->mpl460a_unlock_boot == NULL)
    {
        return -ENOSYS;
    }
    return api->mpl460a_unlock_boot(dev);
}

// Include syscall
#include <syscalls/cisteme_mpl460a.h>