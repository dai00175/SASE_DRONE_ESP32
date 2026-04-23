#pragma once

#include "esp_err.h"

namespace sensors
{

esp_err_t init_global_isr_service();
esp_err_t init_i2c_bus();
esp_err_t init_ultrasonic();
esp_err_t init_barometer();
esp_err_t init_environment();

void barometer_task(void *);
void env_sensor_task(void *);
void ultrasonic_task(void *);

} // namespace sensors
