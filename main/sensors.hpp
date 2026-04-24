#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

namespace sensors
{

esp_err_t init_global_isr_service();
// ENS160/AHT21 support is disabled.
// esp_err_t init_environment_i2c_bus();
esp_err_t init_barometer_i2c_bus();
esp_err_t init_ultrasonic();
esp_err_t init_barometer();
// esp_err_t init_environment();

// i2c_master_bus_handle_t environment_i2c_bus();
i2c_master_bus_handle_t barometer_i2c_bus();

void barometer_task(void *);
// void env_sensor_task(void *);
void ultrasonic_task(void *);

} // namespace sensors
