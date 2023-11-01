#include <esp_log.h>
#include <esp_event.h>

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

    auto ret = write_u16(REG_CONFIG, 0x8000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed, check comm? 0x%x", ret);
        return ret;
    }

    uint16_t manufacturer_id = 0, dev_id = 0;
    ret = ret ?: read_u16(REG_MANUFACTURER_ID, &manufacturer_id);
    ret = ret ?: read_u16(REG_DEVICE_ID, &dev_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ID read failed: 0x%x", ret);
        return ret;
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

esp_err_t ina228::read_voltage(double *volt_out, TickType_t wait_ticks)
{
    if (unlikely(volt_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t volt_reading = 0;
    auto ret = read(REG_VBUS, (uint8_t *)&volt_reading, 3, wait_ticks);

    bool sign = volt_reading & 0x80;
    volt_reading = __bswap32(volt_reading & 0xffffff) >> 4;
    if (sign) volt_reading *= -1;

    *volt_out = (volt_reading) * VBUS_LSB;

    return ret;
}

esp_err_t ina228::read_current(double *amps_out, TickType_t wait_ticks)
{
    if (unlikely(amps_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t amps_reading = 0;
    auto ret = read(REG_VBUS, (uint8_t *)&amps_reading, 3, wait_ticks);

    bool sign = amps_reading & 0x80;
    amps_reading = __bswap32(amps_reading & 0xffffff) >> 4;
    if (sign) amps_reading *= -1;
    *amps_out = (amps_reading) * current_lsb;

    return ret;
}

esp_err_t ina228::read_die_temp(double *temp, TickType_t wait_ticks)
{
    if (unlikely(temp == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t temp_reading = 0;
    auto ret = read_u16(REG_DIETEMP, &temp_reading, wait_ticks);
    *temp = (double)temp_reading * DIE_TEMP_LSB;

    return ret;
}


esp_err_t ina228::read_volt_shunt(double *volt_out, TickType_t wait_ticks)
{
    if (unlikely(volt_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t volt_reading = 0;
    auto ret = read(REG_VSHUNT, (uint8_t *)&volt_reading, 3, wait_ticks);

    bool sign = volt_reading & 0x80;
    volt_reading = __bswap32(volt_reading & 0xffffff) >> 4;
    if (sign) volt_reading *= -1;
    *volt_out = (volt_reading) * (range == ADC_RANGE_0 ? V_SHUNT_LSB_RANGE0 : V_SHUNT_LSB_RANGE1);

    return ret;
}

esp_err_t ina228::read_power(double *power_out, TickType_t wait_ticks)
{
    if (unlikely(power_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t power_reading = 0;
    auto ret = read(REG_POWER, (uint8_t *)&power_reading, 3, wait_ticks);

    power_reading = __bswap32(power_reading & 0xffffff) >> 8;
    *power_out = 3.2 * current_lsb * power_reading;

    return ret;
}

esp_err_t ina228::read_energy(double *joules_out, TickType_t wait_ticks)
{
    if (unlikely(joules_out == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t joules_reading = 0; // Only 40 bits used
    auto ret = read(REG_VSHUNT, (uint8_t *)&joules_reading, 5, wait_ticks);

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
