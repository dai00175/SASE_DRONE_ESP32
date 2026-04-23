#include "imu_component.hpp"

#include <memory>

#include "BNO08x.hpp"
#include "board_config.hpp"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_state.hpp"

namespace imu
{
namespace
{

std::unique_ptr<BNO08x> g_imu;

} // namespace

esp_err_t init()
{
    bno08x_config_t config(kBoardConfig.bno_spi_host, kBoardConfig.bno_spi_gpios[0], kBoardConfig.bno_spi_gpios[1],
                           kBoardConfig.bno_spi_gpios[2], kBoardConfig.bno_cs_gpio, kBoardConfig.bno_int_gpio,
                           kBoardConfig.bno_rst_gpio, 2000000U, false);

    g_imu = std::make_unique<BNO08x>(config);
    ESP_RETURN_ON_FALSE(g_imu != nullptr, ESP_ERR_NO_MEM, app::kTag, "BNO08x allocation failed");
    ESP_RETURN_ON_FALSE(g_imu->initialize(), ESP_FAIL, app::kTag, "BNO08x initialize failed");
    ESP_RETURN_ON_FALSE(g_imu->rpt.rv_game.enable(static_cast<uint32_t>(kBoardConfig.flight_loop_period_us)), ESP_FAIL,
                        app::kTag, "rv_game enable failed");
    ESP_RETURN_ON_FALSE(g_imu->rpt.cal_gyro.enable(static_cast<uint32_t>(kBoardConfig.flight_loop_period_us)), ESP_FAIL,
                        app::kTag, "cal_gyro enable failed");

    ESP_RETURN_ON_FALSE(
        g_imu->rpt.rv_game.register_cb([]() {
            if (g_imu == nullptr)
            {
                return;
            }
            const bno08x_euler_angle_t euler = g_imu->rpt.rv_game.get_euler(true);
            app::update_imu_euler(euler.x, euler.y, euler.z);
        }),
        ESP_FAIL, app::kTag, "rv_game register_cb failed");

    ESP_RETURN_ON_FALSE(
        g_imu->rpt.cal_gyro.register_cb([]() {
            if (g_imu == nullptr)
            {
                return;
            }
            const bno08x_gyro_t gyro = g_imu->rpt.cal_gyro.get();
            app::update_imu_gyro(gyro.x * app::kRadToDeg, gyro.y * app::kRadToDeg, gyro.z * app::kRadToDeg);
        }),
        ESP_FAIL, app::kTag, "cal_gyro register_cb failed");

    uint32_t samples = 0;
    uint32_t last_sequence = 0;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    float yaw_sum = 0.0f;
    const int64_t start_us = esp_timer_get_time();

    while (samples < 20U && (esp_timer_get_time() - start_us) < 5000000)
    {
        const imu_snapshot_t imu_snapshot = app::snapshot_imu();
        if (imu_snapshot.valid && imu_snapshot.sequence != last_sequence)
        {
            roll_sum += imu_snapshot.roll_deg;
            pitch_sum += imu_snapshot.pitch_deg;
            yaw_sum += imu_snapshot.yaw_deg;
            last_sequence = imu_snapshot.sequence;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_RETURN_ON_FALSE(samples >= 20U, ESP_ERR_TIMEOUT, app::kTag, "IMU offset calibration timed out");
    app::set_imu_offsets(roll_sum / static_cast<float>(samples), pitch_sum / static_cast<float>(samples),
                         yaw_sum / static_cast<float>(samples));
    return ESP_OK;
}

} // namespace imu
