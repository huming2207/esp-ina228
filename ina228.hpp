#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_err.h>
#include <driver/i2c.h>

class ina228_alert_cb
{
public:
    virtual void handle_alert(uint16_t flags) = 0;
};

class ina228
{
public:
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

    enum adc_sample : uint8_t {
        ADC_SAMPLE_1 = 0,
        ADC_SAMPLE_4 = 1,
        ADC_SAMPLE_16 = 2,
        ADC_SAMPLE_64 = 3,
        ADC_SAMPLE_128 = 4,
        ADC_SAMPLE_256 = 5,
        ADC_SAMPLE_512 = 6,
        ADC_SAMPLE_1024 = 7,
    };

    enum alert_flags : uint16_t {
        ALERT_FLAG_ALATCH = BIT(15), // Latched alert or not
        ALERT_FLAG_CNVR = BIT(14), // Conversion ready alert enable/disable
        ALERT_FLAG_SLOWALERT = BIT(13), // Alert on averaged or not
        ALERT_FLAG_APOL = BIT(12), // Alert polarity (0 = active-low, 1 = active-high)
        ALERT_ENERGYOF = BIT(11), // Energy overflow flag
        ALERT_CHARGEOF = BIT(10), // Charge overflow flag
        ALERT_MATHOF = BIT(9), // Math overflow flag
        ALERT_TMPOL = BIT(7), // Over-temperature flag
        ALERT_SHUNTOL = BIT(6), // Shunt over-limit flag
        ALERT_SHUNTUL = BIT(5), // Shunt under-limit flag
        ALERT_BUSOL = BIT(4), // Bus over-limit flag
        ALERT_BUSUL = BIT(3), // Bus under-limit flag
        ALERT_POL = BIT(2), // Power over-limit flag
        ALERT_CNVRF = BIT(1), // Conversion complete flag
        ALERT_MEMSTAT = BIT(0), // Memory checksum status flag (1 = OK, 0 = gg)
    };

    enum alert_intr_flags : uint32_t {
        ALERT_INTR_TRIG = BIT(16),
    };

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
    esp_err_t set_adc_range(ina228::adc_range _range);

    /**
     * Read voltage
     * @param volt_out Voltage output
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_voltage(double *volt_out, TickType_t wait_ticks = portMAX_DELAY);

    esp_err_t read_die_temp(double *temp, TickType_t wait_ticks = portMAX_DELAY);

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

    /**
     * Read energy
     * @param joules_out Energy output in joules
     * @param wait_ticks Wait for ticks, or timeout if after that (default forever, never timeout)
     * @return ESP_OK if success
     */
    esp_err_t read_energy(double *joules_out, TickType_t wait_ticks = portMAX_DELAY);

    /**
     * Clear energy counter
     * @return ESP_OK if success
     */
    esp_err_t clear_energy_counter();

    /**
     * Read alert flags
     * @param flag_out Flags output (bit field)
     * @return ESP_OK if success
     */
    esp_err_t read_alert_flag(uint16_t *flag_out);

    esp_err_t write_alert_flag(uint16_t flag);

private:
    esp_err_t write(uint8_t reg, const uint8_t *buf, size_t len, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t write_u8(uint8_t reg, uint8_t data, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t write_u16(uint8_t reg, uint16_t data, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read(uint8_t reg, uint8_t *buf_out, size_t buf_len, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read_u8(uint8_t reg, uint8_t *out, TickType_t wait_ticks = portMAX_DELAY);
    esp_err_t read_u16(uint8_t reg, uint16_t *out, TickType_t wait_ticks = portMAX_DELAY);
    static void alert_monitor_task(void *_ctx);
    static IRAM_ATTR void alert_isr_handler(void *_ctx);

private:
    adc_range range = ADC_RANGE_0;
    uint8_t addr_msb = 0;
    uint16_t shunt_cal = 0;
    uint8_t *trans_buf = nullptr;
    i2c_port_t i2c_port = I2C_NUM_MAX;
    EventGroupHandle_t alert_evt = nullptr;
    ina228_alert_cb *alert_cb = nullptr;
    double current_lsb = 0;

private:
    static const constexpr char TAG[] = "ina228";
    static const constexpr size_t TRANS_SIZE = I2C_LINK_RECOMMENDED_SIZE(16); // Maybe 8 is enough...

    static const constexpr uint8_t REG_CONFIG =			0x00;
    static const constexpr uint8_t REG_ADC_CONFIG =		0x01;
    static const constexpr uint8_t REG_SHUNT_CAL =		0x02;
    static const constexpr uint8_t REG_SHUNT_TEMPCO =	0x03;
    static const constexpr uint8_t REG_VSHUNT =			0x04;
    static const constexpr uint8_t REG_VBUS = 			0x05;
    static const constexpr uint8_t REG_DIETEMP =		0x06;
    static const constexpr uint8_t REG_CURRENT =		0x07;
    static const constexpr uint8_t REG_POWER =			0x08;
    static const constexpr uint8_t REG_ENERGY =			0x09;
    static const constexpr uint8_t REG_CHARGE =			0x0A;
    static const constexpr uint8_t REG_DIAG_ALRT =		0x0B;
    static const constexpr uint8_t REG_SOVL	=		    0x0C;
    static const constexpr uint8_t REG_SUVL =			0x0D;
    static const constexpr uint8_t REG_BOVL	=		    0x0E;
    static const constexpr uint8_t REG_BUVL	=		    0x0F;
    static const constexpr uint8_t REG_TEMP_LIMIT =		0x10;
    static const constexpr uint8_t REG_PWR_LIMIT =		0x11;
    static const constexpr uint8_t REG_MANUFACTURER_ID =	0x3E;
    static const constexpr uint8_t REG_DEVICE_ID =		0x3F;

    static const constexpr double VBUS_LSB = 0.0001953125;
    static const constexpr double DIE_TEMP_LSB = 0.0078125;
    static const constexpr double V_SHUNT_LSB_RANGE0 = 0.0003125;
    static const constexpr double V_SHUNT_LSB_RANGE1 = 0.000078125;
};