#include <esp_log.h>

#include "ina228.hpp"

esp_err_t ina228::init(i2c_port_t port, uint8_t addr, gpio_num_t alert, i2c_config_t *i2c_cfg)
{
    if (i2c_cfg != nullptr) {
        auto ret = i2c_param_config(port, i2c_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C setup failed: 0x%x", ret);
            return ret;
        }
    }

    trans_buf = (uint8_t *)(heap_caps_calloc(1, TRANS_SIZE, MALLOC_CAP_SPIRAM));
    if (trans_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to prepare I2C buffer");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ina228::configure_shunt(double max_current, double r_shunt)
{
    current_lsb = max_current / (2 << 19); // 2 power of 19
    shunt_cal = (uint32_t)((double)(13107.2 * 1000000) * current_lsb * r_shunt);

    ESP_LOGI(TAG, "New Rshunt=%f ohm, max current=%.3f", r_shunt, max_current);
    ESP_LOGI(TAG, "New CURRENT_LSB=%f, SHUNT_CAL=%lu", current_lsb, shunt_cal);

    return ESP_OK;
}

esp_err_t ina228::set_adc_range(ina228_defs::adc_range range)
{
    return 0;
}

esp_err_t ina228::read_voltage(double *volt_out, TickType_t wait_ticks)
{
    return 0;
}

esp_err_t ina228::read_current(double *amps_out, TickType_t wait_ticks)
{
    return 0;
}

esp_err_t ina228::read_volt_shunt(double *volt_out, TickType_t wait_ticks)
{
    return 0;
}

esp_err_t ina228::read_power(double *power_out, TickType_t wait_ticks)
{
    return 0;
}

esp_err_t ina228::write(uint8_t cmd, const uint8_t *buf, size_t len, TickType_t wait_ticks)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(trans_buf, TRANS_SIZE);
    auto ret = i2c_master_start(handle);
    ret = ret ?: i2c_master_write_byte(handle, (addr_msb | I2C_MASTER_WRITE), true);
    ret = ret ?: i2c_master_write_byte(handle, cmd, true);
    ret = ret ?: i2c_master_write(handle, buf, len, true);
    ret = ret ?: i2c_master_stop(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare transaction: 0x%x", ret);
        return ret;
    }

    ret = i2c_master_cmd_begin(i2c_port, handle, wait_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ina228::write_u8(uint8_t cmd, uint8_t data, TickType_t wait_ticks)
{
    return write(cmd, (uint8_t *)&data, sizeof(data), wait_ticks);
}

esp_err_t ina228::write_u16(uint8_t cmd, uint16_t data, TickType_t wait_ticks)
{
    uint16_t data_send = __bswap16(data);
    return write(cmd, (uint8_t *)&data_send, sizeof(data_send), wait_ticks);
}

esp_err_t ina228::read(uint8_t cmd, uint8_t *buf_out, size_t buf_len, TickType_t wait_ticks)
{
    if(buf_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(trans_buf, TRANS_SIZE);
    auto ret = i2c_master_start(handle);
    ret = ret ?: i2c_master_write_byte(handle, (addr_msb | I2C_MASTER_WRITE), true);
    ret = ret ?: i2c_master_write_byte(handle, cmd, true);
    ret = ret ?: i2c_master_read(handle, buf_out, buf_len, I2C_MASTER_NACK);
    ret = ret ?: i2c_master_stop(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare transaction: 0x%x", ret);
        return ret;
    }

    ret = i2c_master_cmd_begin(i2c_port, handle, wait_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ina228::read_u8(uint8_t cmd, uint8_t *out, TickType_t wait_ticks)
{
    return read(cmd, out, sizeof(uint8_t), wait_ticks);
}

esp_err_t ina228::read_u16(uint8_t cmd, uint16_t *out, TickType_t wait_ticks)
{
    if (out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t data_out = 0;
    auto ret = read(cmd, (uint8_t *)&data_out, sizeof(data_out), wait_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    *out = __bswap16(data_out);
    return ESP_OK;
}

