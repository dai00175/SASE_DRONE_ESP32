#pragma once

#include "esp_err.h"

namespace imu
{

esp_err_t init();
void imu_task(void *);

} // namespace imu
