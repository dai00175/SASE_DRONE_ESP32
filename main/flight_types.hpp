#pragma once

#include <array>
#include <cstdint>

#include "board_config.hpp"
// ENS160/AHT21 support is disabled.
// #include "ens160.h"

struct imu_snapshot_t
{
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float gyro_x_dps = 0.0f;
    float gyro_y_dps = 0.0f;
    float gyro_z_dps = 0.0f;
    int64_t timestamp_us = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

struct control_setpoint_t
{
    int throttle_us = kBoardConfig.pwm_min_us;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    DroneState state = DroneState::IDLE;
    int mission_id = 0;
    int64_t command_timestamp_us = 0;
};

struct aux_sensor_snapshot_t
{
    float barometer_pressure_hpa = 0.0f;
    float barometer_altitude_m = 0.0f;
    float barometer_temperature_c = 0.0f;
    bool barometer_valid = false;

    float ultrasonic_distance_cm = 0.0f;
    bool ultrasonic_valid = false;

    // ENS160/AHT21 support is disabled.
    // float env_temperature_c = 0.0f;
    // float env_humidity_pct = 0.0f;
    // float env_dewpoint_c = 0.0f;
    // ens160_aqi_uba_indexes_t env_aqi = ENS160_AQI_UBA_INDEX_UNKNOWN;
    // uint16_t env_tvoc_ppb = 0;
    // uint16_t env_eco2_ppm = 0;
    // bool env_valid = false;
};

struct flight_runtime_t
{
    std::array<int32_t, 4> motor_outputs_us = {
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
        kBoardConfig.pwm_min_us,
    };
    float roll_pid = 0.0f;
    float pitch_pid = 0.0f;
    float yaw_pid = 0.0f;
    int64_t last_loop_period_us = 0;
    int64_t min_loop_period_us = 0;
    int64_t max_loop_period_us = 0;
    double avg_loop_period_us = 0.0;
    uint32_t loop_count = 0;
};

struct telemetry_packet_t
{
    control_setpoint_t control;
    imu_snapshot_t imu;
    aux_sensor_snapshot_t aux;
    flight_runtime_t runtime;
    bool bluetooth_connected = false;
};

enum class CommandAction : uint8_t
{
    NONE = 0,
    TAKEOFF,
    LAND,
    CANCEL,
    PROGRAM_CONTROL,
    USER_CONTROL,
    SETPOINT_UPDATE,
};

struct command_message_t
{
    CommandAction action = CommandAction::NONE;
    int mission_id = 0;
    bool has_throttle = false;
    int throttle_us = kBoardConfig.pwm_min_us;
    bool has_roll = false;
    float roll_deg = 0.0f;
    bool has_pitch = false;
    float pitch_deg = 0.0f;
    bool has_yaw = false;
    float yaw_deg = 0.0f;
};
