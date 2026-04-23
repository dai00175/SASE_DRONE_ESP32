#pragma once

#include "esp_err.h"

namespace bluetooth
{

esp_err_t init();
void bluetooth_task(void *);
void telemetry_task(void *);

} // namespace bluetooth
