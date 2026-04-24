#include "shared_state.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

namespace app
{
namespace
{

struct ImuOffsetState
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
};

QueueHandle_t g_command_queue = nullptr;
QueueHandle_t g_telemetry_queue = nullptr;
EventGroupHandle_t g_system_events = nullptr;
TaskHandle_t g_flight_task_handle = nullptr;

portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_control_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_aux_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_runtime_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_link_mux = portMUX_INITIALIZER_UNLOCKED;

imu_snapshot_t g_imu_snapshot = {};
ImuOffsetState g_imu_offsets = {};
control_setpoint_t g_control_setpoint = {};
aux_sensor_snapshot_t g_aux_snapshot = {};
flight_runtime_t g_flight_runtime = {};

bool g_bluetooth_connected = false;
int64_t g_last_bluetooth_rx_us = 0;

} // namespace

const char *drone_state_to_string(DroneState state)
{
    switch (state)
    {
    case DroneState::IDLE:
        return "IDLE";
    case DroneState::TAKEOFF:
        return "TAKEOFF";
    case DroneState::USERCNTRL:
        return "USERCNTRL";
    case DroneState::PRGMCNTRL:
        return "PRGMCNTRL";
    case DroneState::LANDING:
        return "LANDING";
    default:
        return "UNKNOWN";
    }
}

float pressure_to_altitude_m(float pressure_hpa)
{
    if (pressure_hpa <= 0.0f)
    {
        return 0.0f;
    }
    const float ratio = pressure_hpa / kSeaLevelPressureHpa;
    return 44330.0f * (1.0f - std::pow(ratio, 0.1903f));
}

esp_err_t init_system_objects()
{
    g_command_queue = xQueueCreate(kCommandQueueDepth, sizeof(command_message_t));
    if (g_command_queue == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    g_telemetry_queue = xQueueCreate(kTelemetryQueueDepth, sizeof(telemetry_packet_t));
    if (g_telemetry_queue == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    g_system_events = xEventGroupCreate();
    if (g_system_events == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    control_setpoint_t defaults = {};
    defaults.throttle_us = kBoardConfig.pwm_min_us;
    defaults.state = DroneState::IDLE;
    defaults.command_timestamp_us = esp_timer_get_time();
    update_control_setpoint(defaults);
    return ESP_OK;
}

QueueHandle_t command_queue()
{
    return g_command_queue;
}

QueueHandle_t telemetry_queue()
{
    return g_telemetry_queue;
}

EventGroupHandle_t system_events()
{
    return g_system_events;
}

void register_flight_task_handle(TaskHandle_t handle)
{
    g_flight_task_handle = handle;
}

TaskHandle_t flight_task_handle()
{
    return g_flight_task_handle;
}

void update_control_setpoint(const control_setpoint_t &setpoint)
{
    portENTER_CRITICAL(&g_control_mux);
    g_control_setpoint = setpoint;
    portEXIT_CRITICAL(&g_control_mux);
}

control_setpoint_t snapshot_control_setpoint()
{
    portENTER_CRITICAL(&g_control_mux);
    control_setpoint_t copy = g_control_setpoint;
    portEXIT_CRITICAL(&g_control_mux);
    return copy;
}

imu_snapshot_t snapshot_imu()
{
    portENTER_CRITICAL(&g_imu_mux);
    imu_snapshot_t copy = g_imu_snapshot;
    portEXIT_CRITICAL(&g_imu_mux);
    return copy;
}

aux_sensor_snapshot_t snapshot_aux()
{
    portENTER_CRITICAL(&g_aux_mux);
    aux_sensor_snapshot_t copy = g_aux_snapshot;
    portEXIT_CRITICAL(&g_aux_mux);
    return copy;
}

flight_runtime_t snapshot_runtime()
{
    portENTER_CRITICAL(&g_runtime_mux);
    flight_runtime_t copy = g_flight_runtime;
    portEXIT_CRITICAL(&g_runtime_mux);
    return copy;
}

void update_barometer_snapshot(float pressure_hpa, float temperature_c)
{
    portENTER_CRITICAL(&g_aux_mux);
    g_aux_snapshot.barometer_pressure_hpa = pressure_hpa;
    g_aux_snapshot.barometer_altitude_m = pressure_to_altitude_m(pressure_hpa);
    g_aux_snapshot.barometer_temperature_c = temperature_c;
    g_aux_snapshot.barometer_valid = true;
    portEXIT_CRITICAL(&g_aux_mux);
}

void invalidate_barometer_snapshot()
{
    portENTER_CRITICAL(&g_aux_mux);
    g_aux_snapshot.barometer_valid = false;
    portEXIT_CRITICAL(&g_aux_mux);
}

void update_ultrasonic_snapshot(float distance_cm, bool valid)
{
    portENTER_CRITICAL(&g_aux_mux);
    g_aux_snapshot.ultrasonic_distance_cm = distance_cm;
    g_aux_snapshot.ultrasonic_valid = valid;
    portEXIT_CRITICAL(&g_aux_mux);
}

// ENS160/AHT21 support is disabled.
// void update_environment_snapshot(float temperature_c, float humidity_pct, float dewpoint_c,
//                                  ens160_aqi_uba_indexes_t aqi, uint16_t tvoc_ppb, uint16_t eco2_ppm, bool valid)
// {
//     portENTER_CRITICAL(&g_aux_mux);
//     g_aux_snapshot.env_temperature_c = temperature_c;
//     g_aux_snapshot.env_humidity_pct = humidity_pct;
//     g_aux_snapshot.env_dewpoint_c = dewpoint_c;
//     g_aux_snapshot.env_aqi = aqi;
//     g_aux_snapshot.env_tvoc_ppb = tvoc_ppb;
//     g_aux_snapshot.env_eco2_ppm = eco2_ppm;
//     g_aux_snapshot.env_valid = valid;
//     portEXIT_CRITICAL(&g_aux_mux);
// }

void update_imu_euler(float roll_deg, float pitch_deg, float yaw_deg)
{
    portENTER_CRITICAL(&g_imu_mux);
    const ImuOffsetState offsets = g_imu_offsets;
    g_imu_snapshot.roll_deg = roll_deg - offsets.roll_deg;
    g_imu_snapshot.pitch_deg = pitch_deg - offsets.pitch_deg;
    g_imu_snapshot.yaw_deg = yaw_deg - offsets.yaw_deg;
    g_imu_snapshot.timestamp_us = esp_timer_get_time();
    g_imu_snapshot.sequence++;
    g_imu_snapshot.valid = true;
    portEXIT_CRITICAL(&g_imu_mux);

    if (g_system_events != nullptr)
    {
        xEventGroupSetBits(g_system_events, kEvtImuFresh);
    }
}

void update_imu_gyro(float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
    portENTER_CRITICAL(&g_imu_mux);
    g_imu_snapshot.gyro_x_dps = gyro_x_dps;
    g_imu_snapshot.gyro_y_dps = gyro_y_dps;
    g_imu_snapshot.gyro_z_dps = gyro_z_dps;
    g_imu_snapshot.timestamp_us = esp_timer_get_time();
    g_imu_snapshot.valid = true;
    portEXIT_CRITICAL(&g_imu_mux);
}

void set_imu_offsets(float roll_deg, float pitch_deg, float yaw_deg)
{
    portENTER_CRITICAL(&g_imu_mux);
    g_imu_offsets.roll_deg = roll_deg;
    g_imu_offsets.pitch_deg = pitch_deg;
    g_imu_offsets.yaw_deg = yaw_deg;
    portEXIT_CRITICAL(&g_imu_mux);
}

void update_loop_runtime(int64_t loop_period_us, float roll_pid, float pitch_pid, float yaw_pid)
{
    portENTER_CRITICAL(&g_runtime_mux);
    g_flight_runtime.last_loop_period_us = loop_period_us;
    g_flight_runtime.roll_pid = roll_pid;
    g_flight_runtime.pitch_pid = pitch_pid;
    g_flight_runtime.yaw_pid = yaw_pid;
    if (g_flight_runtime.loop_count == 0)
    {
        g_flight_runtime.min_loop_period_us = loop_period_us;
        g_flight_runtime.max_loop_period_us = loop_period_us;
        g_flight_runtime.avg_loop_period_us = static_cast<double>(loop_period_us);
    }
    else
    {
        g_flight_runtime.min_loop_period_us = std::min(g_flight_runtime.min_loop_period_us, loop_period_us);
        g_flight_runtime.max_loop_period_us = std::max(g_flight_runtime.max_loop_period_us, loop_period_us);
        const double count = static_cast<double>(g_flight_runtime.loop_count);
        g_flight_runtime.avg_loop_period_us =
            ((g_flight_runtime.avg_loop_period_us * count) + static_cast<double>(loop_period_us)) / (count + 1.0);
    }
    g_flight_runtime.loop_count++;
    portEXIT_CRITICAL(&g_runtime_mux);
}

void update_motor_runtime(const std::array<int32_t, 4> &motor_outputs_us)
{
    portENTER_CRITICAL(&g_runtime_mux);
    g_flight_runtime.motor_outputs_us = motor_outputs_us;
    portEXIT_CRITICAL(&g_runtime_mux);
}

telemetry_packet_t build_telemetry_packet()
{
    telemetry_packet_t packet = {};
    packet.control = snapshot_control_setpoint();
    packet.imu = snapshot_imu();
    packet.aux = snapshot_aux();
    packet.runtime = snapshot_runtime();
    packet.bluetooth_connected = is_bluetooth_connected();
    return packet;
}

int64_t get_last_bluetooth_rx_us()
{
    portENTER_CRITICAL(&g_link_mux);
    const int64_t timestamp_us = g_last_bluetooth_rx_us;
    portEXIT_CRITICAL(&g_link_mux);
    return timestamp_us;
}

bool is_bluetooth_connected()
{
    portENTER_CRITICAL(&g_link_mux);
    const bool connected = g_bluetooth_connected;
    portEXIT_CRITICAL(&g_link_mux);
    return connected;
}

void set_bluetooth_connected(bool connected)
{
    portENTER_CRITICAL(&g_link_mux);
    g_bluetooth_connected = connected;
    if (connected)
    {
        g_last_bluetooth_rx_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&g_link_mux);
}

void note_bluetooth_rx()
{
    portENTER_CRITICAL(&g_link_mux);
    g_bluetooth_connected = true;
    g_last_bluetooth_rx_us = esp_timer_get_time();
    portEXIT_CRITICAL(&g_link_mux);
}

} // namespace app
