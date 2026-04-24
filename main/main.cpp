#include "bluetooth_link.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "flight_control.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_component.hpp"
#include "motors.hpp"
#include "sensors.hpp"
#include "shared_state.hpp"

namespace
{

esp_err_t create_tasks()
{
    TaskHandle_t flight_task_handle = nullptr;
    BaseType_t task_result = xTaskCreatePinnedToCore(flight_control::flight_task, "flight_task", app::kFlightTaskStack,
                                                     nullptr, 24, &flight_task_handle, app::kFlightCore);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "flight_task create failed");
    app::register_flight_task_handle(flight_task_handle);

    task_result = xTaskCreatePinnedToCore(bluetooth::bluetooth_task, "bluetooth_task", app::kBackgroundTaskStack,
                                          nullptr, 7, nullptr, app::kBackgroundCore);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "bluetooth_task create failed");

    task_result = xTaskCreatePinnedToCore(imu::imu_task, "imu_task", app::kBackgroundTaskStack, nullptr, 10, nullptr,
                                          app::kBackgroundCore);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "imu_task create failed");

    task_result = xTaskCreatePinnedToCore(flight_control::command_task, "command_task", app::kBackgroundTaskStack,
                                          nullptr, 8, nullptr, app::kBackgroundCore);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "command_task create failed");

    // Barometer support is disabled; leave the task code available for easy re-enable later.
    // task_result = xTaskCreatePinnedToCore(sensors::barometer_task, "barometer_task", app::kBackgroundTaskStack,
    //                                       nullptr, 5, nullptr, app::kBackgroundCore);
    // ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "barometer_task create failed");

    // ENS160/AHT21 support is disabled; leave the task code available for easy re-enable later.
    // task_result = xTaskCreatePinnedToCore(sensors::env_sensor_task, "env_sensor_task", app::kBackgroundTaskStack,
    //                                       nullptr, 5, nullptr, app::kBackgroundCore);
    // ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "env_sensor_task create failed");

    // Ultrasonic support is disabled; leave the task code available for easy re-enable later.
    // task_result = xTaskCreatePinnedToCore(sensors::ultrasonic_task, "ultrasonic_task", app::kBackgroundTaskStack,
    //                                       nullptr, 5, nullptr, app::kBackgroundCore);
    // ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "ultrasonic_task create failed");

    task_result = xTaskCreatePinnedToCore(bluetooth::telemetry_task, "telemetry_task", app::kBackgroundTaskStack,
                                          nullptr, 6, nullptr, app::kBackgroundCore);
    ESP_RETURN_ON_FALSE(task_result == pdPASS, ESP_ERR_NO_MEM, app::kTag, "telemetry_task create failed");

    return ESP_OK;
}

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(app::kTag, "Starting Metro ESP32-S3 flight controller");

    ESP_LOGI(app::kTag, "Initializing system objects");
    if (app::init_system_objects() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "System object initialization failed");
        return;
    }

    ESP_LOGI(app::kTag, "Initializing global ISR service");
    if (sensors::init_global_isr_service() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "Global ISR service initialization failed");
        return;
    }

    ESP_LOGI(app::kTag, "Initializing motors");
    if (motors::init() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "Motor initialization failed");
        motors::apply_outputs_off();
        return;
    }

    // ENS160/AHT21 support is disabled; leave the bus init available for easy re-enable later.
    // if (sensors::init_environment_i2c_bus() != ESP_OK)
    // {
    //     ESP_LOGE(app::kTag, "Environment I2C bus initialization failed");
    //     motors::apply_outputs_off();
    //     return;
    // }

    // Barometer support is disabled; leave the bus init available for easy re-enable later.
    // if (sensors::init_barometer_i2c_bus() != ESP_OK)
    // {
    //     ESP_LOGE(app::kTag, "Barometer I2C bus initialization failed");
    //     motors::apply_outputs_off();
    //     return;
    // }

    ESP_LOGI(app::kTag, "Initializing Bluetooth UART");
    if (bluetooth::init() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "Bluetooth UART initialization failed");
        motors::apply_outputs_off();
        return;
    }

    // Ultrasonic support is disabled; leave the init path available for easy re-enable later.
    // if (sensors::init_ultrasonic() != ESP_OK)
    // {
    //     ESP_LOGE(app::kTag, "Ultrasonic initialization failed");
    //     motors::apply_outputs_off();
    //     return;
    // }

    // Barometer support is disabled; leave the init path available for easy re-enable later.
    // if (sensors::init_barometer() != ESP_OK)
    // {
    //     ESP_LOGE(app::kTag, "Barometer initialization failed");
    //     motors::apply_outputs_off();
    //     return;
    // }

    // ENS160/AHT21 support is disabled; leave the init path available for easy re-enable later.
    // if (sensors::init_environment() != ESP_OK)
    // {
    //     ESP_LOGE(app::kTag, "ENS160/AHT21 initialization failed");
    //     motors::apply_outputs_off();
    //     return;
    // }

    ESP_LOGI(app::kTag, "Initializing IMU");
    if (imu::init() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "IMU initialization failed");
        motors::apply_outputs_off();
        return;
    }

    ESP_LOGI(app::kTag, "Creating tasks");
    if (create_tasks() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "Task creation failed");
        motors::apply_outputs_off();
        return;
    }

    ESP_LOGI(app::kTag, "Starting flight timer");
    if (flight_control::init_flight_timer() != ESP_OK)
    {
        ESP_LOGE(app::kTag, "Flight timer initialization failed");
        motors::apply_outputs_off();
        return;
    }

    ESP_LOGI(app::kTag, "Flight controller initialized");
}
