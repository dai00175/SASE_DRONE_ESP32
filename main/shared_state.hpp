#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>

#include "board_config.hpp"
#include "esp_err.h"
#include "flight_types.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace app
{

inline constexpr char kTag[] = "flight_ctrl";
inline constexpr float kSeaLevelPressureHpa = 1013.25f;
inline constexpr float kRadToDeg = 57.29577951308232f;
inline constexpr size_t kBluetoothLineBufferSize = 512;
inline constexpr size_t kCommandQueueDepth = 16;
inline constexpr size_t kTelemetryQueueDepth = 1;
inline constexpr uint32_t kFlightCore = 1;
inline constexpr uint32_t kBackgroundCore = 0;
inline constexpr uint32_t kFlightTaskStack = 6144;
inline constexpr uint32_t kBackgroundTaskStack = 4096;
inline constexpr EventBits_t kEvtImuFresh = BIT0;

template <typename T>
T clamp_value(T value, T min_value, T max_value)
{
    return std::min(std::max(value, min_value), max_value);
}

const char *drone_state_to_string(DroneState state);
float pressure_to_altitude_m(float pressure_hpa);

esp_err_t init_system_objects();

QueueHandle_t command_queue();
QueueHandle_t telemetry_queue();
EventGroupHandle_t system_events();

void register_flight_task_handle(TaskHandle_t handle);
TaskHandle_t flight_task_handle();

void update_control_setpoint(const control_setpoint_t &setpoint);
control_setpoint_t snapshot_control_setpoint();

void update_barometer_snapshot(float pressure_hpa, float temperature_c);
void invalidate_barometer_snapshot();
void update_ultrasonic_snapshot(float distance_cm, bool valid);
// ENS160/AHT21 support is disabled.
// void update_environment_snapshot(float temperature_c, float humidity_pct, float dewpoint_c,
//                                  ens160_aqi_uba_indexes_t aqi, uint16_t tvoc_ppb, uint16_t eco2_ppm, bool valid);

void update_imu_euler(float roll_deg, float pitch_deg, float yaw_deg);
void update_imu_gyro(float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);
imu_snapshot_t snapshot_imu();
void set_imu_offsets(float roll_deg, float pitch_deg, float yaw_deg);

void update_loop_runtime(int64_t loop_period_us, float roll_pid, float pitch_pid, float yaw_pid);
void update_motor_runtime(const std::array<int32_t, 4> &motor_outputs_us);
flight_runtime_t snapshot_runtime();

aux_sensor_snapshot_t snapshot_aux();
telemetry_packet_t build_telemetry_packet();

int64_t get_last_bluetooth_rx_us();
bool is_bluetooth_connected();
void set_bluetooth_connected(bool connected);
void note_bluetooth_rx();

} // namespace app
