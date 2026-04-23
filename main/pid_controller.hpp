#pragma once

#include <array>

#include "flight_types.hpp"

namespace pid
{

struct AxisGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct Tuning
{
    AxisGains roll = {1.5f, 0.5f, 0.05f};
    AxisGains pitch = {1.5f, 0.5f, 0.05f};
    AxisGains yaw = {2.0f, 0.0f, 0.05f};
    float integrator_limit = 200.0f;
    int integrator_enable_delta_us = 50;
};

struct Output
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

const Tuning &tuning();
void set_tuning(const Tuning &new_tuning);

void reset();
Output compute(const control_setpoint_t &setpoint, const imu_snapshot_t &imu_snapshot, float dt_s);
std::array<int32_t, 4> mix_outputs(int throttle_us, const Output &pid_output);

} // namespace pid
