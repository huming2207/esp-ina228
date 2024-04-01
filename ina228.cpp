#include <csignal>
#include <esp_log.h>
#include <esp_event.h>

#include "ina228.hpp"

esp_err_t ina228::init(i2c_master_bus_handle_t i2c_master, gpio_num_t alert, uint8_t addr, uint32_t freq_hz)
{
    esp_err_t ret = ESP_OK;
    uint8_t i2c_addr = addr;
    if (addr == 0) {
        for (uint8_t curr_addr = 0x40; curr_addr <= 0x4f; curr_addr += 1) {
            ret = i2c_master_probe(i2c_master, curr_addr, 250);
            if (ret != ESP_OK) {
                if (curr_addr == 0x4f) {
                    ESP_LOGE(TAG, "Address auto-detect fail!");
                    return ret;
                } else {
                    ESP_LOGW(TAG, "Auto-detect address @ 0x%02x seems invalid", curr_addr);
                }
            } else {
                ESP_LOGI(TAG, "Address @ 0x%02x seems OK", (curr_addr));
                i2c_addr = curr_addr;
                break;
            }
        }
    } else {
        ret = i2c_master_probe(i2c_master, i2c_addr, 250);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "No response at addr @ 0x%02x", i2c_addr);
        }
    }

    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i2c_addr,
            .scl_speed_hz = freq_hz,
    };
    
    ret = i2c_master_bus_add_device(i2c_master, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: 0x%x", ret);
        return ret;
    }

    ret = reset();
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t manufacturer_id = 0, dev_id = 0;
    ret = ret ?: read_u16(REG_MANUFACTURER_ID, &manufacturer_id);
    ret = ret ?: read_u16(REG_DEVICE_ID, &dev_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ID read failed: 0x%x", ret);
        return ret;
    } else {
        ESP_LOGI(TAG, "Device OK: ID 0x%04x; 0x%04x", manufacturer_id, dev_id);
    }

    gpio_config_t alert_cfg = {
            .pin_bit_mask = (1ULL << (uint32_t)alert),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
    };

    ret = gpio_config(&alert_cfg);
    gpio_install_isr_service(0);
    ret = ret ?: gpio_isr_handler_add(alert, alert_isr_handler, this);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Alert init failed: 0x%x", ret);
        return ret;
    }

    alert_evt = xEventGroupCreate();
    if (alert_evt == nullptr) {
        ESP_LOGE(TAG, "Alert event group init failed: 0x%x", ret);
        return ret;
    }

    xTaskCreate(alert_monitor_task, "ina228_alert", 3072, this, tskIDLE_PRIORITY + 1, nullptr);
    return ret;
}

esp_err_t ina228::configure_shunt(double max_current, double r_shunt)
{
    current_lsb = max_current / (2 << 19); // 2 power of 19
    shunt_cal = (uint16_t)((double)(13107.2 * 1000000) * current_lsb * r_shunt);

    ESP_LOGI(TAG, "New Rshunt=%f ohm, max current=%.3f", r_shunt, max_current);
    ESP_LOGI(TAG, "New CURRENT_LSB=%f, SHUNT_CAL=%u", current_lsb, shunt_cal);

    return write_u16(REG_SHUNT_CAL, shunt_cal);
}

esp_err_t ina228::set_adc_range(ina228::adc_range _range)
{
    uint16_t cfg = 0;
    auto ret = read_u16(REG_CONFIG, &cfg);

    cfg &= ~((uint16_t)BIT(4));
    cfg |= (uint16_t)(_range == ina228::ADC_RANGE_0 ? 0 : BIT(4));
    ret = ret ?: write_u16(REG_CONFIG, cfg);

    range = _range;
    return ret;
}

esp_err_t ina228::read_voltage(double *volt_out, int32_t wait_ms)
{
    if (unlikely(volt_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t volt_reading = 0;
    auto ret = read(REG_VBUS, (uint8_t *)&volt_reading, 3, wait_ms);

    bool sign = volt_reading & 0x80;
    volt_reading = __bswap32(volt_reading & 0xffffff) >> 4;
    if (sign) volt_reading *= -1;

    *volt_out = (volt_reading) * VBUS_LSB;

    return ret;
}

esp_err_t ina228::read_current(double *amps_out, int32_t wait_ms)
{
    if (unlikely(amps_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t amps_reading = 0;
    auto ret = read(REG_VBUS, (uint8_t *)&amps_reading, 3, wait_ms);

    bool sign = amps_reading & 0x80;
    amps_reading = __bswap32(amps_reading & 0xffffff) >> 4;
    if (sign) amps_reading *= -1;
    *amps_out = (amps_reading) * current_lsb;

    return ret;
}

esp_err_t ina228::read_die_temp(double *temp, int32_t wait_ms)
{
    if (unlikely(temp == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t temp_reading = 0;
    auto ret = read_u16(REG_DIETEMP, &temp_reading, wait_ms);
    *temp = (double)temp_reading * DIE_TEMP_LSB;

    return ret;
}


esp_err_t ina228::read_volt_shunt(double *volt_out, int32_t wait_ms)
{
    if (unlikely(volt_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t volt_reading = 0;
    auto ret = read(REG_VSHUNT, (uint8_t *)&volt_reading, 3, wait_ms);

    bool sign = volt_reading & 0x80;
    volt_reading = __bswap32(volt_reading & 0xffffff) >> 4;
    if (sign) volt_reading *= -1;
    *volt_out = (volt_reading) * (range == ADC_RANGE_0 ? V_SHUNT_LSB_RANGE0 : V_SHUNT_LSB_RANGE1);

    return ret;
}

esp_err_t ina228::read_power(double *power_out, int32_t wait_ms)
{
    if (unlikely(power_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t power_reading = 0;
    auto ret = read(REG_POWER, (uint8_t *)&power_reading, 3, wait_ms);

    power_reading = __bswap32(power_reading & 0xffffff) >> 8;
    *power_out = 3.2 * current_lsb * power_reading;

    return ret;
}

esp_err_t ina228::read_energy(double *joules_out, int32_t wait_ms)
{
    if (unlikely(joules_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t joules_reading = 0; // Only 40 bits used
    auto ret = read(REG_VSHUNT, (uint8_t *)&joules_reading, 5, wait_ms);

    joules_reading = __bswap64(joules_reading & 0xffffffffffULL);
    *joules_out = 16 * 3.2 * current_lsb * (double)joules_reading;

    return ret;
}

esp_err_t ina228::clear_energy_counter()
{
    uint16_t cfg = 0;
    auto ret = read_u16(REG_CONFIG, &cfg);

    cfg |= (uint16_t)BIT(14); // CONFIG->RSTACC
    ret = ret ?: write_u16(REG_CONFIG, cfg);

    return ret;
}

esp_err_t ina228::read_alert_flag(uint16_t *flag_out)
{
    if (flag_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t buf = 0;
    auto ret = read_u16(REG_DIAG_ALRT, &buf);
    if (ret != ESP_OK) {
        return ret;
    }

    *flag_out = buf;
    return ret;
}

esp_err_t ina228::write_alert_flag(uint16_t flag)
{
    return write_u16(REG_DIAG_ALRT, flag);
}

esp_err_t ina228::write(uint8_t cmd, const uint8_t *buf, size_t len, int32_t wait_ms)
{
    if (buf == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    auto ret = i2c_master_transmit(i2c_dev, &cmd, sizeof(cmd), wait_ms);
    ret = ret ?: i2c_master_transmit(i2c_dev, buf, len, wait_ms);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ina228::write_u8(uint8_t cmd, uint8_t data, int32_t wait_ms)
{
    return write(cmd, (uint8_t *)&data, sizeof(data), wait_ms);
}

esp_err_t ina228::write_u16(uint8_t cmd, uint16_t data, int32_t wait_ms)
{
    uint16_t data_send = __bswap16(data);
    return write(cmd, (uint8_t *)&data_send, sizeof(data_send), wait_ms);
}

esp_err_t ina228::read(uint8_t cmd, uint8_t *buf_out, size_t buf_len, int32_t wait_ms)
{
    if(buf_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    auto ret = i2c_master_transmit_receive(i2c_dev, &cmd, sizeof(cmd), buf_out, buf_len, wait_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}

esp_err_t ina228::read_u8(uint8_t cmd, uint8_t *out, int32_t wait_ms)
{
    return read(cmd, out, sizeof(uint8_t), wait_ms);
}

esp_err_t ina228::read_u16(uint8_t cmd, uint16_t *out, int32_t wait_ms)
{
    if (out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t data_out = 0;
    auto ret = read(cmd, (uint8_t *)&data_out, sizeof(data_out), wait_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    *out = __bswap16(data_out);
    return ESP_OK;
}

void ina228::alert_monitor_task(void *_ctx)
{
    auto *ctx = (ina228 *)_ctx;
    while (true) {
        xEventGroupWaitBits(ctx->alert_evt, ina228::ALERT_INTR_TRIG, pdTRUE, pdFALSE, portMAX_DELAY);
        uint16_t alert_flag = 0;
        if (ctx->read_alert_flag(&alert_flag) == ESP_OK) {
            if (ctx->alert_cb != nullptr) {
                ctx->alert_cb->handle_alert(alert_flag);
            }
        }
    }
}

void ina228::alert_isr_handler(void *_ctx)
{
    auto *ctx = (ina228 *)_ctx;
    xEventGroupSetBits(ctx->alert_evt, ina228::ALERT_INTR_TRIG);
}

esp_err_t ina228::reset()
{
    auto ret = write_u16(REG_CONFIG, 0x8000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed, check comm? 0x%x", ret);
        return ret;
    }

    return ret;
}

