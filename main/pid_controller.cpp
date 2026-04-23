#include "pid_controller.hpp"

#include <cmath>

#include "board_config.hpp"
#include "shared_state.hpp"

namespace pid
{
namespace
{

struct IntegratorState
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

Tuning g_tuning = {};
IntegratorState g_integrators = {};

float compute_axis(float setpoint_deg, float measured_deg, float gyro_dps, int throttle_us, float dt_s,
                   const AxisGains &gains, float *integral)
{
    const float error = setpoint_deg - measured_deg;
    if (throttle_us >= (kBoardConfig.pwm_min_us + g_tuning.integrator_enable_delta_us))
    {
        *integral += error * dt_s;
    }

    *integral = app::clamp_value(*integral, -g_tuning.integrator_limit, g_tuning.integrator_limit);
    return (gains.kp * error) + (gains.ki * (*integral)) - (gains.kd * gyro_dps);
}

} // namespace

const Tuning &tuning()
{
    return g_tuning;
}

void set_tuning(const Tuning &new_tuning)
{
    g_tuning = new_tuning;
}

void reset()
{
    g_integrators = {};
}

Output compute(const control_setpoint_t &setpoint, const imu_snapshot_t &imu_snapshot, float dt_s)
{
    Output output = {};
    if (!imu_snapshot.valid)
    {
        return output;
    }

    output.roll = compute_axis(setpoint.roll_deg, imu_snapshot.roll_deg, imu_snapshot.gyro_x_dps, setpoint.throttle_us,
                               dt_s, g_tuning.roll, &g_integrators.roll);
    output.pitch = compute_axis(setpoint.pitch_deg, imu_snapshot.pitch_deg, imu_snapshot.gyro_y_dps, setpoint.throttle_us,
                                dt_s, g_tuning.pitch, &g_integrators.pitch);
    output.yaw = compute_axis(setpoint.yaw_deg, imu_snapshot.yaw_deg, imu_snapshot.gyro_z_dps, setpoint.throttle_us,
                              dt_s, g_tuning.yaw, &g_integrators.yaw);

    if (setpoint.throttle_us < (kBoardConfig.pwm_min_us + g_tuning.integrator_enable_delta_us))
    {
        reset();
    }

    return output;
}

std::array<int32_t, 4> mix_outputs(int throttle_us, const Output &pid_output)
{
    return {
        static_cast<int32_t>(std::lround(throttle_us + pid_output.roll - pid_output.pitch + pid_output.yaw)),
        static_cast<int32_t>(std::lround(throttle_us + pid_output.roll + pid_output.pitch - pid_output.yaw)),
        static_cast<int32_t>(std::lround(throttle_us - pid_output.roll + pid_output.pitch + pid_output.yaw)),
        static_cast<int32_t>(std::lround(throttle_us - pid_output.roll - pid_output.pitch - pid_output.yaw)),
    };
}

} // namespace pid
