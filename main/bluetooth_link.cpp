#include "bluetooth_link.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "board_config.hpp"
#include "cJSON.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "shared_state.hpp"

namespace bluetooth
{
namespace
{

bool try_get_number(const cJSON *object, const char *name, double *value)
{
    if (object == nullptr || name == nullptr || value == nullptr)
    {
        return false;
    }
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (cJSON_IsNumber(item))
    {
        *value = item->valuedouble;
        return true;
    }
    return false;
}

CommandAction parse_action(const char *action)
{
    if (action == nullptr)
    {
        return CommandAction::NONE;
    }
    if (std::strcmp(action, "takeoff") == 0)
    {
        return CommandAction::TAKEOFF;
    }
    if (std::strcmp(action, "land") == 0)
    {
        return CommandAction::LAND;
    }
    if (std::strcmp(action, "cancel") == 0)
    {
        return CommandAction::CANCEL;
    }
    if (std::strcmp(action, "program_control") == 0)
    {
        return CommandAction::PROGRAM_CONTROL;
    }
    if (std::strcmp(action, "user_control") == 0)
    {
        return CommandAction::USER_CONTROL;
    }
    return CommandAction::SETPOINT_UPDATE;
}

void enqueue_command(const command_message_t &command)
{
    if (app::command_queue() == nullptr)
    {
        return;
    }
    if (xQueueSend(app::command_queue(), &command, 0) != pdTRUE)
    {
        ESP_LOGW(app::kTag, "Command queue full, dropping command");
    }
}

void process_command_json(const char *json_message)
{
    cJSON *root = cJSON_Parse(json_message);
    if (root == nullptr)
    {
        ESP_LOGW(app::kTag, "Malformed JSON command");
        return;
    }

    int mission_id = 0;
    const cJSON *mission = cJSON_GetObjectItemCaseSensitive(root, "mission_id");
    if (cJSON_IsNumber(mission))
    {
        mission_id = mission->valueint;
    }

    const cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    if (!cJSON_IsArray(commands))
    {
        cJSON_Delete(root);
        return;
    }

    cJSON *command_node = nullptr;
    cJSON_ArrayForEach(command_node, commands)
    {
        if (!cJSON_IsObject(command_node))
        {
            continue;
        }

        command_message_t command = {};
        command.mission_id = mission_id;

        const cJSON *action_item = cJSON_GetObjectItemCaseSensitive(command_node, "action");
        if (cJSON_IsString(action_item) && action_item->valuestring != nullptr)
        {
            command.action = parse_action(action_item->valuestring);
        }

        double value = 0.0;
        if (try_get_number(command_node, "throttle_us", &value) || try_get_number(command_node, "throttle", &value))
        {
            command.has_throttle = true;
            command.throttle_us = app::clamp_value(static_cast<int>(std::lround(value)), kBoardConfig.pwm_min_us,
                                                   kBoardConfig.pwm_max_us);
        }
        if (try_get_number(command_node, "roll", &value) || try_get_number(command_node, "roll_deg", &value))
        {
            command.has_roll = true;
            command.roll_deg = static_cast<float>(value);
        }
        if (try_get_number(command_node, "pitch", &value) || try_get_number(command_node, "pitch_deg", &value))
        {
            command.has_pitch = true;
            command.pitch_deg = static_cast<float>(value);
        }
        if (try_get_number(command_node, "yaw", &value) || try_get_number(command_node, "yaw_deg", &value))
        {
            command.has_yaw = true;
            command.yaw_deg = static_cast<float>(value);
        }

        if (command.action == CommandAction::NONE &&
            !(command.has_throttle || command.has_roll || command.has_pitch || command.has_yaw))
        {
            continue;
        }

        enqueue_command(command);
    }

    cJSON_Delete(root);
}

esp_err_t send_telemetry_json(const telemetry_packet_t &packet)
{
    cJSON *root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(root, "state", app::drone_state_to_string(packet.control.state));
    cJSON_AddBoolToObject(root, "bluetooth_connected", packet.bluetooth_connected);
    cJSON_AddNumberToObject(root, "mission_id", packet.control.mission_id);

    cJSON *setpoint = cJSON_AddObjectToObject(root, "setpoint");
    cJSON_AddNumberToObject(setpoint, "throttle_us", packet.control.throttle_us);
    cJSON_AddNumberToObject(setpoint, "roll_deg", packet.control.roll_deg);
    cJSON_AddNumberToObject(setpoint, "pitch_deg", packet.control.pitch_deg);
    cJSON_AddNumberToObject(setpoint, "yaw_deg", packet.control.yaw_deg);

    cJSON *imu = cJSON_AddObjectToObject(root, "imu");
    cJSON_AddBoolToObject(imu, "valid", packet.imu.valid);
    cJSON_AddNumberToObject(imu, "sequence", packet.imu.sequence);
    cJSON_AddNumberToObject(imu, "roll_deg", packet.imu.roll_deg);
    cJSON_AddNumberToObject(imu, "pitch_deg", packet.imu.pitch_deg);
    cJSON_AddNumberToObject(imu, "yaw_deg", packet.imu.yaw_deg);
    cJSON_AddNumberToObject(imu, "gyro_x_dps", packet.imu.gyro_x_dps);
    cJSON_AddNumberToObject(imu, "gyro_y_dps", packet.imu.gyro_y_dps);
    cJSON_AddNumberToObject(imu, "gyro_z_dps", packet.imu.gyro_z_dps);

    cJSON *motors = cJSON_AddObjectToObject(root, "motors");
    for (size_t i = 0; i < packet.runtime.motor_outputs_us.size(); ++i)
    {
        char key[8] = {};
        std::snprintf(key, sizeof(key), "m%u", static_cast<unsigned>(i + 1U));
        cJSON_AddNumberToObject(motors, key, packet.runtime.motor_outputs_us[i]);
    }

    cJSON *loop = cJSON_AddObjectToObject(root, "loop");
    cJSON_AddNumberToObject(loop, "last_us", static_cast<double>(packet.runtime.last_loop_period_us));
    cJSON_AddNumberToObject(loop, "min_us", static_cast<double>(packet.runtime.min_loop_period_us));
    cJSON_AddNumberToObject(loop, "max_us", static_cast<double>(packet.runtime.max_loop_period_us));
    cJSON_AddNumberToObject(loop, "avg_us", packet.runtime.avg_loop_period_us);
    cJSON_AddNumberToObject(loop, "count", packet.runtime.loop_count);

    cJSON *barometer = cJSON_AddObjectToObject(root, "barometer");
    cJSON_AddBoolToObject(barometer, "valid", packet.aux.barometer_valid);
    cJSON_AddNumberToObject(barometer, "pressure_hpa", packet.aux.barometer_pressure_hpa);
    cJSON_AddNumberToObject(barometer, "altitude_m", packet.aux.barometer_altitude_m);
    cJSON_AddNumberToObject(barometer, "temperature_c", packet.aux.barometer_temperature_c);

    cJSON *ultrasonic = cJSON_AddObjectToObject(root, "ultrasonic");
    cJSON_AddBoolToObject(ultrasonic, "valid", packet.aux.ultrasonic_valid);
    cJSON_AddNumberToObject(ultrasonic, "distance_cm", packet.aux.ultrasonic_distance_cm);

    cJSON *environment = cJSON_AddObjectToObject(root, "environment");
    cJSON_AddBoolToObject(environment, "valid", packet.aux.env_valid);
    cJSON_AddNumberToObject(environment, "temperature_c", packet.aux.env_temperature_c);
    cJSON_AddNumberToObject(environment, "humidity_pct", packet.aux.env_humidity_pct);
    cJSON_AddNumberToObject(environment, "dewpoint_c", packet.aux.env_dewpoint_c);
    cJSON_AddNumberToObject(environment, "aqi_uba", packet.aux.env_aqi);
    cJSON_AddNumberToObject(environment, "tvoc_ppb", packet.aux.env_tvoc_ppb);
    cJSON_AddNumberToObject(environment, "eco2_ppm", packet.aux.env_eco2_ppm);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    const int json_length = static_cast<int>(std::strlen(json));
    const int written = uart_write_bytes(kBoardConfig.bluetooth_uart_port, json, json_length);
    uart_write_bytes(kBoardConfig.bluetooth_uart_port, "\n", 1);
    cJSON_free(json);
    return written == json_length ? ESP_OK : ESP_FAIL;
}

} // namespace

esp_err_t init()
{
    const uart_config_t uart_config = {
        .baud_rate = kBoardConfig.bluetooth_baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .rx_glitch_filt_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(kBoardConfig.bluetooth_uart_port, 2048, 2048, 0, nullptr, 0), app::kTag,
                        "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(kBoardConfig.bluetooth_uart_port, &uart_config), app::kTag,
                        "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(kBoardConfig.bluetooth_uart_port, kBoardConfig.bluetooth_tx_gpio,
                                     kBoardConfig.bluetooth_rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        app::kTag, "uart_set_pin failed");
    app::set_bluetooth_connected(false);
    return ESP_OK;
}

void bluetooth_task(void *)
{
    std::array<char, app::kBluetoothLineBufferSize> line_buffer = {};
    size_t line_length = 0;
    telemetry_packet_t packet = {};

    while (true)
    {
        uint8_t byte = 0;
        const int bytes_read = uart_read_bytes(kBoardConfig.bluetooth_uart_port, &byte, 1, pdMS_TO_TICKS(20));
        if (bytes_read > 0)
        {
            app::note_bluetooth_rx();
            if (byte == '\r')
            {
                continue;
            }
            if (byte == '\n')
            {
                line_buffer[line_length] = '\0';
                if (line_length > 0)
                {
                    process_command_json(line_buffer.data());
                }
                line_length = 0;
                continue;
            }
            if (line_length + 1U < line_buffer.size())
            {
                line_buffer[line_length++] = static_cast<char>(byte);
            }
            else
            {
                line_length = 0;
            }
        }

        if (app::telemetry_queue() != nullptr && xQueueReceive(app::telemetry_queue(), &packet, 0) == pdTRUE)
        {
            const esp_err_t err = send_telemetry_json(packet);
            if (err != ESP_OK)
            {
                ESP_LOGW(app::kTag, "Telemetry send failed: %s", esp_err_to_name(err));
            }
        }

        const int64_t now_us = esp_timer_get_time();
        if (app::get_last_bluetooth_rx_us() != 0 &&
            (now_us - app::get_last_bluetooth_rx_us()) >
                (static_cast<int64_t>(kBoardConfig.bluetooth_timeout_ms) * 1000))
        {
            app::set_bluetooth_connected(false);
        }
    }
}

void telemetry_task(void *)
{
    while (true)
    {
        const telemetry_packet_t packet = app::build_telemetry_packet();
        if (app::telemetry_queue() != nullptr)
        {
            xQueueOverwrite(app::telemetry_queue(), &packet);
        }
        vTaskDelay(pdMS_TO_TICKS(kBoardConfig.telemetry_period_ms));
    }
}

} // namespace bluetooth
