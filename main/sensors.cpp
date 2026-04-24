#include "sensors.hpp"

#include <array>
#include <cstring>

// ENS160/AHT21 support is disabled.
// #include "ahtxx.h"
#include "board_config.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
// #include "ens160.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_state.hpp"

extern "C" {
#include "lps28dfw_reg.h"
}

namespace sensors
{
namespace
{

struct Lps28Context
{
    i2c_master_dev_handle_t i2c_device = nullptr;
    stmdev_ctx_t io_ctx = {};
    lps28dfw_md_t mode = {};
    std::array<float, 16> pressure_window = {};
    size_t window_index = 0;
    bool window_full = false;
    float running_sum = 0.0f;
    bool initialized = false;
};

// ENS160/AHT21 support is disabled.
// i2c_master_bus_handle_t g_environment_i2c_bus = nullptr;
i2c_master_bus_handle_t g_barometer_i2c_bus = nullptr;
Lps28Context g_lps28 = {};
// ahtxx_handle_t g_aht21 = nullptr;
// ens160_handle_t g_ens160 = nullptr;

portMUX_TYPE g_ultra_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t g_ultra_echo_start_us = 0;
volatile int64_t g_ultra_echo_end_us = 0;
volatile int64_t g_ultra_trigger_time_us = 0;
volatile bool g_ultra_measurement_active = false;
volatile bool g_ultra_sample_ready = false;

int32_t lps28_write_reg(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (handle == nullptr || data == nullptr || len == 0 || len > 63)
    {
        return -1;
    }
    auto *device = static_cast<i2c_master_dev_handle_t *>(handle);
    uint8_t tx[64] = {};
    tx[0] = reg;
    std::memcpy(&tx[1], data, len);
    return i2c_master_transmit(*device, tx, len + 1U, 100) == ESP_OK ? 0 : -1;
}

int32_t lps28_read_reg(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (handle == nullptr || data == nullptr || len == 0)
    {
        return -1;
    }
    auto *device = static_cast<i2c_master_dev_handle_t *>(handle);
    return i2c_master_transmit_receive(*device, &reg, 1, data, len, 100) == ESP_OK ? 0 : -1;
}

esp_err_t init_i2c_master_bus(i2c_port_num_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio,
                              i2c_master_bus_handle_t *out_bus)
{
    if (out_bus == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = port;
    bus_config.sda_io_num = sda_gpio;
    bus_config.scl_io_num = scl_gpio;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = 1;
    return i2c_new_master_bus(&bus_config, out_bus);
}

void IRAM_ATTR ultrasonic_echo_isr(void *)
{
    const int level = gpio_get_level(kBoardConfig.ultrasonic_echo_gpio);
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&g_ultra_mux);
    if (level == 1)
    {
        g_ultra_echo_start_us = now_us;
        g_ultra_measurement_active = true;
        g_ultra_sample_ready = false;
    }
    else
    {
        g_ultra_echo_end_us = now_us;
        g_ultra_measurement_active = false;
        g_ultra_sample_ready = true;
    }
    portEXIT_CRITICAL_ISR(&g_ultra_mux);
}

} // namespace

esp_err_t init_global_isr_service()
{
    const esp_err_t err = gpio_install_isr_service(0);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
    {
        return ESP_OK;
    }
    return err;
}

// ENS160/AHT21 support is disabled.
// esp_err_t init_environment_i2c_bus()
// {
//     return init_i2c_master_bus(I2C_NUM_0, kBoardConfig.env_i2c_sda_gpio, kBoardConfig.env_i2c_scl_gpio,
//                                &g_environment_i2c_bus);
// }

esp_err_t init_barometer_i2c_bus()
{
    return init_i2c_master_bus(I2C_NUM_1, kBoardConfig.barometer_i2c_sda_gpio,
                               kBoardConfig.barometer_i2c_scl_gpio, &g_barometer_i2c_bus);
}

esp_err_t init_ultrasonic()
{
    gpio_config_t output_config = {};
    output_config.pin_bit_mask = 1ULL << kBoardConfig.ultrasonic_trig_gpio;
    output_config.mode = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), app::kTag, "ultrasonic trigger gpio_config failed");

    gpio_config_t input_config = {};
    input_config.pin_bit_mask = 1ULL << kBoardConfig.ultrasonic_echo_gpio;
    input_config.mode = GPIO_MODE_INPUT;
    input_config.intr_type = GPIO_INTR_ANYEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&input_config), app::kTag, "ultrasonic echo gpio_config failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(kBoardConfig.ultrasonic_echo_gpio, ultrasonic_echo_isr, nullptr),
                        app::kTag, "gpio_isr_handler_add failed");
    gpio_set_level(kBoardConfig.ultrasonic_trig_gpio, 0);
    return ESP_OK;
}

esp_err_t init_barometer()
{
    i2c_device_config_t device_config = {};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = 0x5C;
    device_config.scl_speed_hz = kBoardConfig.i2c_clock_hz;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(g_barometer_i2c_bus, &device_config, &g_lps28.i2c_device), app::kTag,
                        "lps28 add device failed");

    g_lps28.io_ctx.handle = &g_lps28.i2c_device;
    g_lps28.io_ctx.read_reg = lps28_read_reg;
    g_lps28.io_ctx.write_reg = lps28_write_reg;

    lps28dfw_id_t chip_id = {};
    ESP_RETURN_ON_FALSE(lps28dfw_id_get(&g_lps28.io_ctx, &chip_id) == 0 && chip_id.whoami == LPS28DFW_ID, ESP_FAIL,
                        app::kTag, "LPS28DFW WHO_AM_I mismatch");

    g_lps28.mode.fs = static_cast<decltype(g_lps28.mode.fs)>(0);
    g_lps28.mode.odr = static_cast<decltype(g_lps28.mode.odr)>(8);
    g_lps28.mode.avg = static_cast<decltype(g_lps28.mode.avg)>(0);
    g_lps28.mode.lpf = static_cast<decltype(g_lps28.mode.lpf)>(0);
    ESP_RETURN_ON_FALSE(lps28dfw_mode_set(&g_lps28.io_ctx, &g_lps28.mode) == 0, ESP_FAIL, app::kTag,
                        "lps28dfw_mode_set failed");

    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_FALSE(lps28dfw_read_reg(&g_lps28.io_ctx, LPS28DFW_CTRL_REG2, &ctrl2, 1) == 0, ESP_FAIL, app::kTag,
                        "LPS28DFW ctrl2 read failed");
    ctrl2 |= (1U << 3U);
    ESP_RETURN_ON_FALSE(lps28dfw_write_reg(&g_lps28.io_ctx, LPS28DFW_CTRL_REG2, &ctrl2, 1) == 0, ESP_FAIL, app::kTag,
                        "LPS28DFW ctrl2 write failed");

    g_lps28.pressure_window.fill(app::kSeaLevelPressureHpa);
    g_lps28.running_sum = app::kSeaLevelPressureHpa * static_cast<float>(g_lps28.pressure_window.size());
    g_lps28.window_full = true;
    g_lps28.initialized = true;
    return ESP_OK;
}

// ENS160/AHT21 support is disabled.
// esp_err_t init_environment()
// {
//     ahtxx_config_t aht_config = I2C_AHT21_CONFIG_DEFAULT;
//     ESP_RETURN_ON_ERROR(ahtxx_init(g_environment_i2c_bus, &aht_config, &g_aht21), app::kTag, "ahtxx_init failed");
//
//     ens160_config_t ens_config = I2C_ENS160_CONFIG_DEFAULT;
//     ens_config.i2c_address = static_cast<uint16_t>(kBoardConfig.ens160_i2c_address);
//     ens_config.i2c_clock_speed = static_cast<uint32_t>(kBoardConfig.i2c_clock_hz);
//     ESP_RETURN_ON_ERROR(ens160_init(g_environment_i2c_bus, &ens_config, &g_ens160), app::kTag, "ens160_init failed");
//     return ens160_enable_standard_mode(g_ens160);
// }
//
// i2c_master_bus_handle_t environment_i2c_bus()
// {
//     return g_environment_i2c_bus;
// }

i2c_master_bus_handle_t barometer_i2c_bus()
{
    return g_barometer_i2c_bus;
}

void barometer_task(void *)
{
    while (true)
    {
        if (!g_lps28.initialized)
        {
            app::invalidate_barometer_snapshot();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        lps28dfw_data_t data = {};
        if (lps28dfw_data_get(&g_lps28.io_ctx, &g_lps28.mode, &data) == 0)
        {
            const float pressure_hpa = data.pressure.hpa;
            g_lps28.running_sum -= g_lps28.pressure_window[g_lps28.window_index];
            g_lps28.pressure_window[g_lps28.window_index] = pressure_hpa;
            g_lps28.running_sum += pressure_hpa;
            g_lps28.window_index = (g_lps28.window_index + 1U) % g_lps28.pressure_window.size();
            if (g_lps28.window_index == 0U)
            {
                g_lps28.window_full = true;
            }

            const size_t samples = g_lps28.window_full ? g_lps28.pressure_window.size() : g_lps28.window_index;
            const float filtered_pressure =
                samples == 0U ? pressure_hpa : (g_lps28.running_sum / static_cast<float>(samples));
            app::update_barometer_snapshot(filtered_pressure, data.heat.deg_c);
        }
        else
        {
            app::invalidate_barometer_snapshot();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ENS160/AHT21 support is disabled.
// void env_sensor_task(void *)
// {
//     while (true)
//     {
//         float temperature_c = 0.0f;
//         float humidity_pct = 0.0f;
//         float dewpoint_c = 0.0f;
//         ens160_air_quality_data_t air_quality = {};
//         bool valid = false;
//
//         if (g_aht21 != nullptr && ahtxx_get_measurements(g_aht21, &temperature_c, &humidity_pct, &dewpoint_c) == ESP_OK)
//         {
//             if (g_ens160 != nullptr)
//             {
//                 ens160_set_compensation_factors(g_ens160, temperature_c, humidity_pct);
//                 if (ens160_get_measurement(g_ens160, &air_quality) == ESP_OK)
//                 {
//                     valid = true;
//                 }
//             }
//         }
//
//         app::update_environment_snapshot(temperature_c, humidity_pct, dewpoint_c, air_quality.uba_aqi, air_quality.tvoc,
//                                          air_quality.eco2, valid);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void ultrasonic_task(void *)
{
    while (true)
    {
        bool measurement_active = false;
        bool sample_ready = false;
        int64_t echo_start_us = 0;
        int64_t echo_end_us = 0;
        int64_t trigger_time_us = 0;

        portENTER_CRITICAL(&g_ultra_mux);
        measurement_active = g_ultra_measurement_active;
        sample_ready = g_ultra_sample_ready;
        echo_start_us = g_ultra_echo_start_us;
        echo_end_us = g_ultra_echo_end_us;
        trigger_time_us = g_ultra_trigger_time_us;
        portEXIT_CRITICAL(&g_ultra_mux);

        const int64_t now_us = esp_timer_get_time();
        if (!measurement_active && !sample_ready)
        {
            portENTER_CRITICAL(&g_ultra_mux);
            g_ultra_echo_start_us = 0;
            g_ultra_echo_end_us = 0;
            g_ultra_trigger_time_us = now_us;
            g_ultra_measurement_active = true;
            g_ultra_sample_ready = false;
            portEXIT_CRITICAL(&g_ultra_mux);

            gpio_set_level(kBoardConfig.ultrasonic_trig_gpio, 1);
            esp_rom_delay_us(10);
            gpio_set_level(kBoardConfig.ultrasonic_trig_gpio, 0);
        }
        else if (sample_ready)
        {
            const float pulse_width_us = static_cast<float>(echo_end_us - echo_start_us);
            const float distance_cm = pulse_width_us * 0.01715f;
            app::update_ultrasonic_snapshot(distance_cm, pulse_width_us > 0.0f);

            portENTER_CRITICAL(&g_ultra_mux);
            g_ultra_sample_ready = false;
            g_ultra_measurement_active = false;
            portEXIT_CRITICAL(&g_ultra_mux);
        }
        else if (measurement_active && (now_us - trigger_time_us) > kBoardConfig.ultrasonic_timeout_us)
        {
            app::update_ultrasonic_snapshot(0.0f, false);
            portENTER_CRITICAL(&g_ultra_mux);
            g_ultra_sample_ready = false;
            g_ultra_measurement_active = false;
            portEXIT_CRITICAL(&g_ultra_mux);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace sensors
