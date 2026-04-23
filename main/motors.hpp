#pragma once

#include <array>
#include <cstddef>

#include "esp_err.h"

namespace motors
{

esp_err_t init();
esp_err_t set_pulse_us(size_t motor_index, int pulse_us);
void apply_outputs(const std::array<int32_t, 4> &outputs_us);
void apply_outputs_off();

} // namespace motors
