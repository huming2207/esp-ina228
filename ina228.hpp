#pragma once

#include <esp_err.h>
#include <driver/i2c.h>

namespace ina228_defs
{
    enum adc_range : uint8_t {
        ADC_RANGE_0 = 0,
        ADC_RANGE_1 = 1,
    };

    enum adc_speed : uint8_t {
        ADC_SPEED_50US = 0,
        ADC_SPEED_84US = 1,
        ADC_SPEED_150US = 2,
        ADC_SPEED_280US = 3,
        ADC_SPEED_540US = 4,
        ADC_SPEED_1052US = 5,
        ADC_SPEED_2074US = 6,
        ADC_SPEED_4120US = 7,
    };

    static const constexpr uint8_t CONFIG =			0x00;
    static const constexpr uint8_t ADC_CONFIG =		0x01;
    static const constexpr uint8_t SHUNT_CAL =		0x02;
    static const constexpr uint8_t SHUNT_TEMPCO =		0x03;
    static const constexpr uint8_t VSHUNT =			0x04;
    static const constexpr uint8_t VBUS = 			0x05;
    static const constexpr uint8_t DIETEMP =			0x06;
    static const constexpr uint8_t CURRENT =			0x07;
    static const constexpr uint8_t POWER =			0x08;
    static const constexpr uint8_t ENERGY =			0x09;
    static const constexpr uint8_t CHARGE =			0x0A;
    static const constexpr uint8_t DIAG_ALRT =		0x0B;
    static const constexpr uint8_t SOVL	=		0x0C;
    static const constexpr uint8_t SUVL =			0x0D;
    static const constexpr uint8_t BOVL	=		0x0E;
    static const constexpr uint8_t BUVL	=		0x0F;
    static const constexpr uint8_t TEMP_LIMIT =		0x10;
    static const constexpr uint8_t PWR_LIMIT =		0x11;
    static const constexpr uint8_t MANUFACTURER_ID =	0x3E;
    static const constexpr uint8_t DEVICE_ID =		0x3F;
}

class ina228
{
public:
    /**
     * Initialise INA228 (and I2C if needed)
     * @param port I2C Port number
     * @param addr INA228 address
     * @param alert INA228 interrupt pin
     * @param i2c_cfg Leave null if this I2C port has been set up elsewhere
     * @return ESP_OK if success
     */
    esp_err_t init(i2c_port_t port, uint8_t addr, gpio_num_t alert, i2c_config_t *i2c_cfg = nullptr);

    /**
     * Configure shunt setting
     * @param max_current Maximum current in amps
     * @param r_shunt Shunt resistor in ohms
     */
    esp_err_t configure_shunt(double max_current, double r_shunt);

    /**
     * Set INA228's ADC range (0 or 1)
     * @param range 0 or 1 (just RTFM)
     * @return ESP_OK if success
     */
    esp_err_t set_adc_range(ina228_defs::adc_range range);

    /**
     * Read voltage
     * @param volt_out Voltage output
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_voltage(double *volt_out, TickType_t wait_ticks = portMAX_DELAY);

    /**
     * Read current
     * @param amps_out Current output
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_current(double *amps_out, TickType_t wait_ticks = portMAX_DELAY);

    /**
     * Read Vshut
     * @param volt_out Shunt voltage output
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_volt_shunt(double *volt_out, TickType_t wait_ticks = portMAX_DELAY);

    /**
     * Read power output
     * @param power_out Power output
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_power(double *power_out, TickType_t wait_ticks = portMAX_DELAY);


private:
    esp_err_t write(uint8_t cmd, const uint8_t *buf, size_t len, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t write_u8(uint8_t cmd, uint8_t data, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t write_u16(uint8_t cmd, uint16_t data, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read(uint8_t cmd, uint8_t *buf_out, size_t buf_len, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read_u8(uint8_t cmd, uint8_t *out, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read_u16(uint8_t cmd, uint16_t *out, TickType_t wait_ticks = portMAX_DELAY);

private:
    uint8_t addr_msb = 0;
    i2c_port_t i2c_port = I2C_NUM_MAX;
    uint8_t *trans_buf = nullptr;
    uint16_t shunt_cal = 0;
    double current_lsb = 0;

private:
    static const constexpr char TAG[] = "ina228";
    static const constexpr size_t TRANS_SIZE = I2C_LINK_RECOMMENDED_SIZE(16); // Maybe 8 is enough...
};