#include "bluetooth_link.hpp"

#include <array>
#include <cctype>
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

constexpr int64_t kPartialLineTimeoutUs = 2000000LL;

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

bool ascii_is_space(char ch)
{
    return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

const char *trim_ascii_whitespace(const char *text)
{
    if (text == nullptr)
    {
        return nullptr;
    }

    while (*text != '\0' && ascii_is_space(*text))
    {
        ++text;
    }
    return text;
}

void copy_trimmed_ascii_whitespace(const char *source, char *dest, size_t dest_size)
{
    if (dest == nullptr || dest_size == 0)
    {
        return;
    }

    dest[0] = '\0';
    if (source == nullptr)
    {
        return;
    }

    const char *start = trim_ascii_whitespace(source);
    const char *end = start + std::strlen(start);
    while (end > start && ascii_is_space(*(end - 1)))
    {
        --end;
    }

    size_t length = static_cast<size_t>(end - start);
    if (length >= dest_size)
    {
        length = dest_size - 1U;
    }

    std::memcpy(dest, start, length);
    dest[length] = '\0';
}

bool ascii_equals_ignore_case(const char *lhs, const char *rhs)
{
    if (lhs == nullptr || rhs == nullptr)
    {
        return false;
    }

    while (*lhs != '\0' && *rhs != '\0')
    {
        if (std::tolower(static_cast<unsigned char>(*lhs)) != std::tolower(static_cast<unsigned char>(*rhs)))
        {
            return false;
        }
        ++lhs;
        ++rhs;
    }

    return *lhs == '\0' && *rhs == '\0';
}

bool is_valid_command_start(uint8_t byte)
{
    return byte == '{' || byte == '[' || std::isalpha(static_cast<unsigned char>(byte)) != 0 ||
           std::isspace(static_cast<unsigned char>(byte)) != 0;
}

bool is_command_line_byte(uint8_t byte)
{
    return byte == '\t' || byte == ' ' || (byte >= 0x20 && byte <= 0x7E);
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

const char *action_to_string(CommandAction action)
{
    switch (action)
    {
    case CommandAction::TAKEOFF:
        return "takeoff";
    case CommandAction::LAND:
        return "land";
    case CommandAction::CANCEL:
        return "cancel";
    case CommandAction::PROGRAM_CONTROL:
        return "program_control";
    case CommandAction::USER_CONTROL:
        return "user_control";
    case CommandAction::SETPOINT_UPDATE:
        return "setpoint_update";
    case CommandAction::NONE:
    default:
        return "none";
    }
}

void log_enqueued_command(const command_message_t &command)
{
    ESP_LOGI(app::kTag,
             "Bluetooth command accepted: action=%s mission_id=%d throttle=%s%d roll=%s%.2f pitch=%s%.2f yaw=%s%.2f",
             action_to_string(command.action), command.mission_id, command.has_throttle ? "" : "-",
             command.has_throttle ? command.throttle_us : 0, command.has_roll ? "" : "-",
             static_cast<double>(command.has_roll ? command.roll_deg : 0.0f), command.has_pitch ? "" : "-",
             static_cast<double>(command.has_pitch ? command.pitch_deg : 0.0f), command.has_yaw ? "" : "-",
             static_cast<double>(command.has_yaw ? command.yaw_deg : 0.0f));
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
        return;
    }

    log_enqueued_command(command);
}

void enqueue_land_failsafe(const char *reason)
{
    if (reason != nullptr && reason[0] != '\0')
    {
        ESP_LOGW(app::kTag, "Enqueuing LAND failsafe: %s", reason);
    }

    command_message_t command = {};
    command.action = CommandAction::LAND;
    enqueue_command(command);
}

bool process_command_json(const char *json_message)
{
    cJSON *root = cJSON_Parse(json_message);
    if (root == nullptr)
    {
        ESP_LOGW(app::kTag, "Malformed JSON command");
        return false;
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
        ESP_LOGW(app::kTag, "JSON command missing commands array");
        cJSON_Delete(root);
        return false;
    }

    bool enqueued_any = false;
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
        enqueued_any = true;
    }

    cJSON_Delete(root);
    return enqueued_any;
}

bool process_plaintext_command(const char *message)
{
    char command_text[app::kBluetoothLineBufferSize] = {};
    copy_trimmed_ascii_whitespace(message, command_text, sizeof(command_text));
    if (command_text[0] == '\0')
    {
        return false;
    }

    command_message_t command = {};
    if (ascii_equals_ignore_case(command_text, "takeoff"))
    {
        command.action = CommandAction::TAKEOFF;
    }
    else if (ascii_equals_ignore_case(command_text, "land"))
    {
        command.action = CommandAction::LAND;
    }
    else if (ascii_equals_ignore_case(command_text, "cancel"))
    {
        command.action = CommandAction::CANCEL;
    }
    else if (ascii_equals_ignore_case(command_text, "program_control"))
    {
        command.action = CommandAction::PROGRAM_CONTROL;
    }
    else if (ascii_equals_ignore_case(command_text, "user_control"))
    {
        command.action = CommandAction::USER_CONTROL;
    }
    else
    {
        return false;
    }

    enqueue_command(command);
    return true;
}

void process_command_message(const char *message)
{
    const char *trimmed = trim_ascii_whitespace(message);
    if (trimmed == nullptr || trimmed[0] == '\0')
    {
        return;
    }

    if (trimmed[0] == '{' || trimmed[0] == '[')
    {
        if (!process_command_json(trimmed))
        {
            ESP_LOGW(app::kTag, "Bluetooth JSON command was not accepted");
        }
        return;
    }

    if (!process_plaintext_command(trimmed))
    {
        ESP_LOGW(app::kTag, "Unsupported Bluetooth command format: %s", trimmed);
    }
}

void log_bluetooth_connection_established()
{
    ESP_LOGI(app::kTag, "Bluetooth connection established");
}

void log_bluetooth_connection_lost(int64_t idle_ms)
{
    ESP_LOGW(app::kTag, "Bluetooth connection lost after %lld ms without RX",
             static_cast<long long>(idle_ms));
}

void log_received_packet(const char *packet)
{
    if (packet == nullptr || packet[0] == '\0')
    {
        return;
    }

    ESP_LOGI(app::kTag, "Bluetooth RX packet: %s", packet);
}

void log_bluetooth_task_heartbeat(bool connected, size_t buffered_chars, uint32_t rx_bytes, uint32_t rx_packets,
                                  uint32_t tx_packets, int64_t last_rx_age_ms)
{
    ESP_LOGI(app::kTag,
             "Bluetooth task heartbeat: connected=%s buffered=%u rx_bytes=%u rx_packets=%u tx_packets=%u last_rx_age_ms=%lld",
             connected ? "yes" : "no", static_cast<unsigned>(buffered_chars), static_cast<unsigned>(rx_bytes),
             static_cast<unsigned>(rx_packets), static_cast<unsigned>(tx_packets), static_cast<long long>(last_rx_age_ms));
}

void log_serial_monitor(const telemetry_packet_t &packet)
{
    const int64_t last_rx_us = app::get_last_bluetooth_rx_us();
    const int64_t now_us = esp_timer_get_time();
    const int64_t last_rx_age_ms = last_rx_us == 0 ? -1LL : (now_us - last_rx_us) / 1000LL;

    ESP_LOGI(app::kTag,
             "State=%s mission=%d BT=%s last_rx_age_ms=%lld | Setpoint thr=%d roll=%.2f pitch=%.2f yaw=%.2f | IMU deg R=%.2f P=%.2f Y=%.2f | PWM us M1=%ld M2=%ld M3=%ld M4=%ld",
             app::drone_state_to_string(packet.control.state), packet.control.mission_id,
             packet.bluetooth_connected ? "yes" : "no", static_cast<long long>(last_rx_age_ms),
             packet.control.throttle_us, static_cast<double>(packet.control.roll_deg),
             static_cast<double>(packet.control.pitch_deg), static_cast<double>(packet.control.yaw_deg),
             static_cast<double>(packet.imu.roll_deg), static_cast<double>(packet.imu.pitch_deg),
             static_cast<double>(packet.imu.yaw_deg), static_cast<long>(packet.runtime.motor_outputs_us[0]),
             static_cast<long>(packet.runtime.motor_outputs_us[1]), static_cast<long>(packet.runtime.motor_outputs_us[2]),
             static_cast<long>(packet.runtime.motor_outputs_us[3]));
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

    // Barometer support is disabled.
    // cJSON *barometer = cJSON_AddObjectToObject(root, "barometer");
    // cJSON_AddBoolToObject(barometer, "valid", packet.aux.barometer_valid);
    // cJSON_AddNumberToObject(barometer, "pressure_hpa", packet.aux.barometer_pressure_hpa);
    // cJSON_AddNumberToObject(barometer, "altitude_m", packet.aux.barometer_altitude_m);
    // cJSON_AddNumberToObject(barometer, "temperature_c", packet.aux.barometer_temperature_c);

    // Ultrasonic support is disabled.
    // cJSON *ultrasonic = cJSON_AddObjectToObject(root, "ultrasonic");
    // cJSON_AddBoolToObject(ultrasonic, "valid", packet.aux.ultrasonic_valid);
    // cJSON_AddNumberToObject(ultrasonic, "distance_cm", packet.aux.ultrasonic_distance_cm);

    // ENS160/AHT21 support is disabled.
    // cJSON *environment = cJSON_AddObjectToObject(root, "environment");
    // cJSON_AddBoolToObject(environment, "valid", packet.aux.env_valid);
    // cJSON_AddNumberToObject(environment, "temperature_c", packet.aux.env_temperature_c);
    // cJSON_AddNumberToObject(environment, "humidity_pct", packet.aux.env_humidity_pct);
    // cJSON_AddNumberToObject(environment, "dewpoint_c", packet.aux.env_dewpoint_c);
    // cJSON_AddNumberToObject(environment, "aqi_uba", packet.aux.env_aqi);
    // cJSON_AddNumberToObject(environment, "tvoc_ppb", packet.aux.env_tvoc_ppb);
    // cJSON_AddNumberToObject(environment, "eco2_ppm", packet.aux.env_eco2_ppm);

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
    ESP_LOGI(app::kTag, "Bluetooth UART ready on UART%d TX=%d RX=%d baud=%d", static_cast<int>(kBoardConfig.bluetooth_uart_port),
             static_cast<int>(kBoardConfig.bluetooth_tx_gpio), static_cast<int>(kBoardConfig.bluetooth_rx_gpio),
             kBoardConfig.bluetooth_baud_rate);
    app::set_bluetooth_connected(false);
    return ESP_OK;
}

void bluetooth_task(void *)
{
    std::array<char, app::kBluetoothLineBufferSize> line_buffer = {};
    size_t line_length = 0;
    bool discarding_line = false;
    int64_t last_line_byte_us = 0;
    telemetry_packet_t packet = {};
    int64_t last_heartbeat_us = esp_timer_get_time();
    uint32_t rx_bytes_since_heartbeat = 0;
    uint32_t rx_packets_since_heartbeat = 0;
    uint32_t tx_packets_since_heartbeat = 0;

    ESP_LOGI(app::kTag, "Bluetooth task started");

    while (true)
    {
        uint8_t byte = 0;
        const int bytes_read = uart_read_bytes(kBoardConfig.bluetooth_uart_port, &byte, 1, pdMS_TO_TICKS(20));
        if (bytes_read > 0)
        {
            rx_bytes_since_heartbeat += static_cast<uint32_t>(bytes_read);
            if (!app::is_bluetooth_connected())
            {
                log_bluetooth_connection_established();
            }
            app::note_bluetooth_rx();
            if (byte == '\r')
            {
                continue;
            }
            const int64_t byte_time_us = esp_timer_get_time();
            if (line_length > 0 && last_line_byte_us != 0 && (byte_time_us - last_line_byte_us) > kPartialLineTimeoutUs)
            {
                ESP_LOGW(app::kTag, "Bluetooth RX partial line timed out after %lld ms, dropping %u buffered bytes",
                         static_cast<long long>((byte_time_us - last_line_byte_us) / 1000LL),
                         static_cast<unsigned>(line_length));
                line_length = 0;
                discarding_line = false;
            }
            last_line_byte_us = byte_time_us;
            if (byte == '\n')
            {
                if (!discarding_line)
                {
                    line_buffer[line_length] = '\0';
                    if (line_length > 0)
                    {
                        rx_packets_since_heartbeat++;
                        log_received_packet(line_buffer.data());
                        process_command_message(line_buffer.data());
                    }
                }
                line_length = 0;
                discarding_line = false;
                last_line_byte_us = 0;
                continue;
            }
            if (discarding_line)
            {
                continue;
            }
            if (line_length == 0 && !is_valid_command_start(byte))
            {
                ESP_LOGW(app::kTag, "Bluetooth RX discarded junk byte before command: 0x%02X",
                         static_cast<unsigned>(byte));
                discarding_line = true;
                continue;
            }
            if (!is_command_line_byte(byte))
            {
                ESP_LOGW(app::kTag, "Bluetooth RX discarded line with non-ASCII byte: 0x%02X",
                         static_cast<unsigned>(byte));
                line_length = 0;
                discarding_line = true;
                continue;
            }
            if (line_length + 1U < line_buffer.size())
            {
                line_buffer[line_length++] = static_cast<char>(byte);
            }
            else
            {
                ESP_LOGW(app::kTag, "Bluetooth RX line exceeded %u bytes, dropping packet",
                         static_cast<unsigned>(line_buffer.size() - 1U));
                line_length = 0;
                discarding_line = true;
            }
        }

        if (app::telemetry_queue() != nullptr && xQueueReceive(app::telemetry_queue(), &packet, 0) == pdTRUE)
        {
            const esp_err_t err = send_telemetry_json(packet);
            if (err != ESP_OK)
            {
                ESP_LOGW(app::kTag, "Telemetry send failed: %s", esp_err_to_name(err));
            }
            else
            {
                tx_packets_since_heartbeat++;
            }
        }

        const int64_t now_us = esp_timer_get_time();
        const int64_t last_rx_us = app::get_last_bluetooth_rx_us();
        const int64_t timeout_us = static_cast<int64_t>(kBoardConfig.bluetooth_timeout_ms) * 1000;
        if (app::is_bluetooth_connected() && last_rx_us != 0 && (now_us - last_rx_us) > timeout_us)
        {
            enqueue_land_failsafe("Bluetooth RX timeout");
            app::set_bluetooth_connected(false);
            log_bluetooth_connection_lost((now_us - last_rx_us) / 1000);
        }

        if ((now_us - last_heartbeat_us) >= 1000000LL)
        {
            const int64_t refreshed_last_rx_us = app::get_last_bluetooth_rx_us();
            const int64_t last_rx_age_ms =
                refreshed_last_rx_us == 0 ? -1LL : (now_us - refreshed_last_rx_us) / 1000LL;
            log_bluetooth_task_heartbeat(app::is_bluetooth_connected(), line_length, rx_bytes_since_heartbeat,
                                         rx_packets_since_heartbeat, tx_packets_since_heartbeat, last_rx_age_ms);
            rx_bytes_since_heartbeat = 0;
            rx_packets_since_heartbeat = 0;
            tx_packets_since_heartbeat = 0;
            last_heartbeat_us = now_us;
        }
    }
}

void telemetry_task(void *)
{
    int64_t last_log_us = 0;

    while (true)
    {
        const telemetry_packet_t packet = app::build_telemetry_packet();
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) >= (static_cast<int64_t>(kBoardConfig.serial_log_period_ms) * 1000))
        {
            log_serial_monitor(packet);
            last_log_us = now_us;
        }

        if (app::telemetry_queue() != nullptr)
        {
            xQueueOverwrite(app::telemetry_queue(), &packet);
        }
        vTaskDelay(pdMS_TO_TICKS(kBoardConfig.telemetry_period_ms));
    }
}

} // namespace bluetooth
