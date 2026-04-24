#include "imu_component.hpp"

#include <cinttypes>
#include <memory>

#include "BNO08x.hpp"
#include "board_config.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_state.hpp"

namespace imu
{
namespace
{

std::unique_ptr<BNO08x> g_imu;

struct OffsetCalibrationState
{
    bool active = false;
    bool complete = false;
    uint32_t samples = 0;
    uint32_t last_sequence = 0;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    float yaw_sum = 0.0f;
    int64_t start_us = 0;
};

OffsetCalibrationState g_offset_calibration = {};

constexpr uint32_t kOffsetCalibrationSamples = 20U;
constexpr int64_t kOffsetCalibrationTimeoutUs = 5000000;
constexpr int64_t kInitialReportTimeoutUs = 4000000;

void process_available_reports()
{
    if (g_imu == nullptr)
    {
        return;
    }

    if (g_imu->rpt.rv_game.has_new_data())
    {
        const bno08x_euler_angle_t euler = g_imu->rpt.rv_game.get_euler(true);
        app::update_imu_euler(euler.x, euler.y, euler.z);
    }

    if (g_imu->rpt.cal_gyro.has_new_data())
    {
        const bno08x_gyro_t gyro = g_imu->rpt.cal_gyro.get();
        app::update_imu_gyro(gyro.x * app::kRadToDeg, gyro.y * app::kRadToDeg, gyro.z * app::kRadToDeg);
    }
}

void start_offset_calibration()
{
    g_offset_calibration = {};
    g_offset_calibration.active = true;
    g_offset_calibration.start_us = esp_timer_get_time();
    ESP_LOGI(app::kTag, "Collecting IMU offset samples");
}

void update_offset_calibration()
{
    if (!g_offset_calibration.active || g_offset_calibration.complete)
    {
        return;
    }

    const imu_snapshot_t imu_snapshot = app::snapshot_imu();
    if (imu_snapshot.valid && imu_snapshot.sequence != g_offset_calibration.last_sequence)
    {
        g_offset_calibration.roll_sum += imu_snapshot.roll_deg;
        g_offset_calibration.pitch_sum += imu_snapshot.pitch_deg;
        g_offset_calibration.yaw_sum += imu_snapshot.yaw_deg;
        g_offset_calibration.last_sequence = imu_snapshot.sequence;
        g_offset_calibration.samples++;
    }

    if (g_offset_calibration.samples >= kOffsetCalibrationSamples)
    {
        const float samples = static_cast<float>(g_offset_calibration.samples);
        app::set_imu_offsets(g_offset_calibration.roll_sum / samples, g_offset_calibration.pitch_sum / samples,
                             g_offset_calibration.yaw_sum / samples);
        g_offset_calibration.complete = true;
        ESP_LOGI(app::kTag, "IMU offset calibration complete with %" PRIu32 " samples", g_offset_calibration.samples);
        return;
    }

    if ((esp_timer_get_time() - g_offset_calibration.start_us) >= kOffsetCalibrationTimeoutUs)
    {
        g_offset_calibration.complete = true;
        ESP_LOGW(app::kTag, "IMU offset calibration timed out after %lld ms; continuing with zero offsets",
                 kOffsetCalibrationTimeoutUs / 1000LL);
    }
}

bool wait_for_initial_report()
{
    const int64_t start_us = esp_timer_get_time();

    while ((esp_timer_get_time() - start_us) < kInitialReportTimeoutUs)
    {
        if (!g_imu->data_available())
        {
            continue;
        }

        process_available_reports();
        if (app::snapshot_imu().valid)
        {
            return true;
        }
    }

    return app::snapshot_imu().valid;
}

} // namespace

esp_err_t init()
{
    bno08x_config_t config(kBoardConfig.bno_spi_host, kBoardConfig.bno_spi_gpios[0], kBoardConfig.bno_spi_gpios[1],
                           kBoardConfig.bno_spi_gpios[2], kBoardConfig.bno_cs_gpio, kBoardConfig.bno_int_gpio,
                           kBoardConfig.bno_rst_gpio, 3000000U, false);

    g_imu = std::make_unique<BNO08x>(config);
    ESP_RETURN_ON_FALSE(g_imu != nullptr, ESP_ERR_NO_MEM, app::kTag, "BNO08x allocation failed");
    ESP_LOGI(app::kTag, "Starting BNO08x transport initialization");
    ESP_RETURN_ON_FALSE(g_imu->initialize(), ESP_FAIL, app::kTag, "BNO08x initialize failed");
    ESP_LOGI(app::kTag, "BNO08x transport initialized");
    ESP_RETURN_ON_FALSE(g_imu->rpt.rv_game.enable(static_cast<uint32_t>(kBoardConfig.flight_loop_period_us)), ESP_FAIL,
                        app::kTag, "rv_game enable failed");
    ESP_RETURN_ON_FALSE(g_imu->rpt.cal_gyro.enable(static_cast<uint32_t>(kBoardConfig.flight_loop_period_us)), ESP_FAIL,
                        app::kTag, "cal_gyro enable failed");
    ESP_LOGI(app::kTag, "Waiting for initial IMU report");
    ESP_RETURN_ON_FALSE(wait_for_initial_report(), ESP_ERR_TIMEOUT, app::kTag,
                        "Timed out waiting for initial IMU report");
    ESP_LOGI(app::kTag, "Initial IMU report received");
    start_offset_calibration();
    update_offset_calibration();
    return ESP_OK;
}

void imu_task(void *)
{
    while (true)
    {
        if (g_imu != nullptr && g_imu->data_available())
        {
            process_available_reports();
            update_offset_calibration();
        }
        else
        {
            update_offset_calibration();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

} // namespace imu
