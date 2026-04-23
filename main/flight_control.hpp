#pragma once

#include "esp_err.h"

namespace flight_control
{

void flight_task(void *);
void command_task(void *);
esp_err_t init_flight_timer();

} // namespace flight_control
